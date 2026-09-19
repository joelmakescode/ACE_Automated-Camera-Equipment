#define _POSIX_C_SOURCE 200809L

#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "geometry.h"
#include "motion.h"
#include "pins.h"
#include "stepper.h"

static volatile sig_atomic_t g_abort = 0;

static void on_signal(int sig) {
    (void)sig;
    g_abort = 1;
}

static void print_usage(const char *prog) {
    fprintf(stderr,
        "Verwendung: %s [--pattern single|all|sync|ratio|square|tension|drum]\n"
        "               [--motor 0..%d] [--steps N | --revs N] [--dir cw|ccw]\n"
        "               [--delay-us N] [--hold] [--dry-run]\n"
        "\n"
        "Muster:\n"
        "  single   ein einzelner Motor (--motor, --steps, --dir)\n"
        "  all      jeder Motor nacheinander vor und zurueck\n"
        "  sync     alle vier gleichzeitig, gleiche Schrittzahl\n"
        "  ratio    alle vier gleichzeitig, ungleiche Schrittzahl\n"
        "  square   vier Kanten, je zwei Seile auf und zwei ab\n"
        "  tension  alle vier Seile gleich weit fahren, ohne Rueckfahrt:\n"
        "           --mm -2 nimmt Spannung raus, --mm 2 zieht an\n"
        "  drum     ein Seil um --revs Umdrehungen abwickeln, zum Ausmessen\n"
        "           des tatsaechlichen Wickeldurchmessers mit dem Lineal\n"
        "\n"
        "Optionen:\n"
        "  --steps    Halbschritte, Standard %d (= eine Umdrehung)\n"
        "  --revs     Umdrehungen statt Halbschritte\n"
        "  --mm       Seillaenge in mm statt Halbschritte, Vorzeichen erlaubt\n"
        "             (positiv = aufwickeln), %.4f mm pro Halbschritt\n"
        "  --dir      cw oder ccw, nur fuer single, Standard cw\n"
        "  --delay-us Zeit pro Halbschritt, Standard %u us (Minimum %u us)\n"
        "  --hold     Spulen am Ende bestromt lassen, bis Ctrl-C\n"
        "  --dry-run  nur Phasenfolge ausgeben, GPIO nicht anfassen\n"
        "\n"
        "Verdrahtung (BCM-Nummern):\n",
        prog, ACE_MOTOR_COUNT - 1, ACE_HALFSTEPS_PER_REV, ACE_MM_PER_HALFSTEP,
        ACE_DEFAULT_STEP_DELAY_US, ACE_MIN_STEP_DELAY_US);

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        fprintf(stderr, "  Motor %d  %-14s  IN1=%2u IN2=%2u IN3=%2u IN4=%2u\n",
                i, ACE_MOTOR_NAMES[i], ACE_MOTOR_PINS[i][0], ACE_MOTOR_PINS[i][1],
                ACE_MOTOR_PINS[i][2], ACE_MOTOR_PINS[i][3]);
    }
}

static void steps_clear(long steps[ACE_MOTOR_COUNT]) {
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) steps[i] = 0;
}

static int run_move(const char *label,
                    const long steps[ACE_MOTOR_COUNT],
                    unsigned int delay_us) {
    printf("%-26s [%7ld %7ld %7ld %7ld]\n",
           label, steps[0], steps[1], steps[2], steps[3]);
    fflush(stdout);

    if (motion_start(steps, delay_us) != 0) return -1;

    for (;;) {
        int r = motion_tick();
        if (r < 0) return -1;
        if (r > 0) return 0;
        if (g_abort) {
            motion_release();
            fprintf(stderr, "Abbruch, Spulen stromlos.\n");
            return 1;
        }
        motion_wait_next();
    }
}

static int pattern_single(long steps[ACE_MOTOR_COUNT], int motor, long count,
                          int dir, unsigned int delay_us) {
    char label[64];
    snprintf(label, sizeof(label), "Motor %d (%s)", motor, ACE_MOTOR_NAMES[motor]);
    steps_clear(steps);
    steps[motor] = dir * count;
    return run_move(label, steps, delay_us);
}

static int pattern_all(long steps[ACE_MOTOR_COUNT], long count, unsigned int delay_us) {
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        char label[64];

        snprintf(label, sizeof(label), "Motor %d (%s) vor", i, ACE_MOTOR_NAMES[i]);
        steps_clear(steps);
        steps[i] = count;
        int rc = run_move(label, steps, delay_us);
        if (rc != 0) return rc;

        snprintf(label, sizeof(label), "Motor %d (%s) zurueck", i, ACE_MOTOR_NAMES[i]);
        steps_clear(steps);
        steps[i] = -count;
        rc = run_move(label, steps, delay_us);
        if (rc != 0) return rc;
    }
    return 0;
}

static int pattern_sync(long steps[ACE_MOTOR_COUNT], long count, unsigned int delay_us) {
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) steps[i] = count;
    int rc = run_move("alle vier vor", steps, delay_us);
    if (rc != 0) return rc;

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) steps[i] = -count;
    return run_move("alle vier zurueck", steps, delay_us);
}

static int pattern_ratio(long steps[ACE_MOTOR_COUNT], long count, unsigned int delay_us) {
    steps[0] = count;
    steps[1] = count * 3 / 4;
    steps[2] = count / 2;
    steps[3] = count / 4;
    int rc = run_move("ungleich vor", steps, delay_us);
    if (rc != 0) return rc;

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) steps[i] = -steps[i];
    return run_move("ungleich zurueck", steps, delay_us);
}

static int pattern_tension(long steps[ACE_MOTOR_COUNT], long count, int dir,
                           unsigned int delay_us) {
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) steps[i] = dir * count;

    return run_move(dir > 0 ? "alle vier aufwickeln" : "alle vier abwickeln",
                    steps, delay_us);
}

static int pattern_drum(long steps[ACE_MOTOR_COUNT], int motor, long count,
                        int dir, unsigned int delay_us) {
    char label[64];
    double revs = (double)count / (double)ACE_HALFSTEPS_PER_REV;

    snprintf(label, sizeof(label), "Motor %d, %.2f Umdrehungen", motor, revs);
    steps_clear(steps);
    steps[motor] = dir * count;

    int rc = run_move(label, steps, delay_us);
    if (rc != 0) return rc;

    printf("\nAbgewickeltes Seil messen, dann:\n");
    printf("  Wickeldurchmesser = gemessene Laenge in mm / (%.2f * 3.1416)\n", revs);
    printf("                    = gemessene Laenge in mm / %.3f\n", revs * ACE_PI);
    printf("Weicht der Wert von den anderen Winden ab, gehoert er einzeln in\n");
    printf("ACE_MOTOR_DRUM_LIST in geometry.h, Stelle %d (aktuell %.2f mm):\n",
           motor, ACE_MOTOR_DRUM_MM[motor]);
    printf("  #define ACE_MOTOR_DRUM_LIST { %.2f, %.2f, %.2f, %.2f }\n",
           ACE_MOTOR_DRUM_MM[0], ACE_MOTOR_DRUM_MM[1],
           ACE_MOTOR_DRUM_MM[2], ACE_MOTOR_DRUM_MM[3]);
    printf("Gilt der Wert fuer alle vier, reicht ACE_DRUM_DIAMETER_MM.\n");
    printf("Wickelt der Motor auf statt ab, nochmal mit --dir ccw.\n");
    return 0;
}

static int pattern_square(long steps[ACE_MOTOR_COUNT], long n, unsigned int delay_us) {
    const long edges[4][ACE_MOTOR_COUNT] = {
        {  n,  n, -n, -n },
        { -n,  n,  n, -n },
        { -n, -n,  n,  n },
        {  n, -n, -n,  n },
    };
    const char *labels[4] = {
        "Kante nach vorne", "Kante nach rechts",
        "Kante nach hinten", "Kante nach links"
    };

    for (int e = 0; e < 4; e++) {
        memcpy(steps, edges[e], sizeof(long) * ACE_MOTOR_COUNT);
        int rc = run_move(labels[e], steps, delay_us);
        if (rc != 0) return rc;
    }
    return 0;
}

int main(int argc, char **argv) {
    const char  *pattern  = "all";
    int          motor    = 0;
    long         count    = ACE_HALFSTEPS_PER_REV;
    int          dir      = 1;
    unsigned int delay_us = ACE_DEFAULT_STEP_DELAY_US;
    int          hold     = 0;
    int          dry_run  = 0;

    static struct option long_opts[] = {
        {"pattern",  required_argument, 0, 'p'},
        {"motor",    required_argument, 0, 'm'},
        {"steps",    required_argument, 0, 's'},
        {"revs",     required_argument, 0, 'v'},
        {"mm",       required_argument, 0, 'M'},
        {"dir",      required_argument, 0, 'r'},
        {"delay-us", required_argument, 0, 'u'},
        {"hold",     no_argument,       0, 'H'},
        {"dry-run",  no_argument,       0, 'n'},
        {"help",     no_argument,       0, '?'},
        {0, 0, 0, 0}
    };

    int opt, opt_index = 0;
    while ((opt = getopt_long(argc, argv, "p:m:s:v:M:r:u:Hn", long_opts, &opt_index)) != -1) {
        switch (opt) {
            case 'p': pattern  = optarg; break;
            case 'm': motor    = atoi(optarg); break;
            case 's': count    = atol(optarg); break;
            case 'v': count    = atol(optarg) * ACE_HALFSTEPS_PER_REV; break;
            case 'M': count    = lround(atof(optarg) / ACE_MM_PER_HALFSTEP); break;
            case 'u': delay_us = (unsigned int)strtoul(optarg, NULL, 10); break;
            case 'H': hold     = 1; break;
            case 'n': dry_run  = 1; break;
            case 'r':
                if      (strcmp(optarg, "cw")  == 0) dir =  1;
                else if (strcmp(optarg, "ccw") == 0) dir = -1;
                else {
                    fprintf(stderr, "--dir erwartet cw oder ccw\n");
                    return 1;
                }
                break;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    if (motor < 0 || motor >= ACE_MOTOR_COUNT) {
        fprintf(stderr, "--motor muss zwischen 0 und %d liegen.\n", ACE_MOTOR_COUNT - 1);
        return 1;
    }
    if (count < 0) {
        count = -count;
        dir   = -dir;
    }

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    if (dry_run) stepper_set_dry_run(1);

    if (motion_init() != 0) return 1;

    long steps[ACE_MOTOR_COUNT];
    steps_clear(steps);
    int rc;

    if (strcmp(pattern, "single") == 0) {
        rc = pattern_single(steps, motor, count, dir, delay_us);
    } else if (strcmp(pattern, "all") == 0) {
        rc = pattern_all(steps, count, delay_us);
    } else if (strcmp(pattern, "sync") == 0) {
        rc = pattern_sync(steps, count, delay_us);
    } else if (strcmp(pattern, "ratio") == 0) {
        rc = pattern_ratio(steps, count, delay_us);
    } else if (strcmp(pattern, "tension") == 0) {
        rc = pattern_tension(steps, count, dir, delay_us);
    } else if (strcmp(pattern, "drum") == 0) {
        rc = pattern_drum(steps, motor, count, dir, delay_us);
    } else if (strcmp(pattern, "square") == 0) {
        rc = pattern_square(steps, count, delay_us);
    } else {
        fprintf(stderr, "Unbekanntes Muster: %s\n", pattern);
        print_usage(argv[0]);
        motion_shutdown();
        return 1;
    }

    printf("Endposition (Halbschritte): %ld %ld %ld %ld\n",
           motion_position(0), motion_position(1),
           motion_position(2), motion_position(3));

    if (hold && rc == 0) {
        motion_hold();
        printf("Spulen bleiben bestromt. Beenden mit Ctrl-C.\n");
        while (!g_abort) pause();
        printf("\n");
    }

    motion_shutdown();
    return rc < 0 ? 1 : 0;
}
