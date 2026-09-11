#define _POSIX_C_SOURCE 200809L

#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "stepper.h"

static const unsigned int MOTOR_PINS[4] = { 22, 18, 17, 27 };

#define HALFSTEPS_PER_REV 4096

#define DEFAULT_DELAY_US  2000u
#define MIN_DELAY_US      1200u

static volatile sig_atomic_t g_abort = 0;

static void on_signal(int sig) {
    (void)sig;
    g_abort = 1;
}

static void sleep_us(unsigned int us) {
    struct timespec ts;
    ts.tv_sec  = us / 1000000u;
    ts.tv_nsec = (long)(us % 1000000u) * 1000L;
    nanosleep(&ts, NULL);
}

static void print_usage(const char *prog) {
    fprintf(stderr,
        "Verwendung: %s [--steps N | --revs N] [--dir cw|ccw]\n"
        "               [--delay-us N] [--hold] [--dry-run]\n"
        "\n"
        "  --steps    Halbschritte, Standard %d (= eine Umdrehung)\n"
        "  --revs     Umdrehungen statt Halbschritte\n"
        "  --dir      cw (vorwaerts) oder ccw (rueckwaerts), Standard cw\n"
        "  --delay-us Zeit pro Halbschritt, Standard %u us (Minimum %u us)\n"
        "  --hold     Spulen am Ende bestromt lassen, bis Ctrl-C (haelt die Last,\n"
        "             der Motor wird dabei warm)\n"
        "  --dry-run  nur Phasenfolge ausgeben, GPIO nicht anfassen\n"
        "\n"
        "  Pins (BCM): IN1=%u IN2=%u IN3=%u IN4=%u\n",
        prog, HALFSTEPS_PER_REV, DEFAULT_DELAY_US, MIN_DELAY_US,
        MOTOR_PINS[0], MOTOR_PINS[1], MOTOR_PINS[2], MOTOR_PINS[3]);
}

int main(int argc, char **argv) {
    long         steps    = HALFSTEPS_PER_REV;
    int          dir      = 1;
    unsigned int delay_us = DEFAULT_DELAY_US;
    int          hold     = 0;
    int          dry_run  = 0;

    static struct option long_opts[] = {
        {"steps",    required_argument, 0, 's'},
        {"revs",     required_argument, 0, 'v'},
        {"dir",      required_argument, 0, 'r'},
        {"delay-us", required_argument, 0, 'u'},
        {"hold",     no_argument,       0, 'H'},
        {"dry-run",  no_argument,       0, 'n'},
        {"help",     no_argument,       0, '?'},
        {0, 0, 0, 0}
    };

    int opt, opt_index = 0;
    while ((opt = getopt_long(argc, argv, "s:v:r:u:Hn", long_opts, &opt_index)) != -1) {
        switch (opt) {
            case 's': steps    = atol(optarg); break;
            case 'v': steps    = atol(optarg) * HALFSTEPS_PER_REV; break;
            case 'u': delay_us = (unsigned int)strtoul(optarg, NULL, 10); break;
            case 'H': hold     = 1; break;
            case 'n': dry_run  = 1; break;
            case 'r':
                if      (strcmp(optarg, "cw")  == 0) dir =  1;
                else if (strcmp(optarg, "ccw") == 0) dir = -1;
                else { fprintf(stderr, "--dir erwartet cw oder ccw\n"); return 1; }
                break;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    if (steps < 0) { steps = -steps; dir = -dir; }
    if (delay_us < MIN_DELAY_US) {
        fprintf(stderr, "Schrittzeit %u us zu kurz, auf %u us begrenzt.\n",
                delay_us, MIN_DELAY_US);
        delay_us = MIN_DELAY_US;
    }

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    if (dry_run) stepper_set_dry_run(1);

    Stepper *motor = stepper_create(0, MOTOR_PINS);
    if (!motor) return 1;

    printf("%ld Halbschritte %s, %u us pro Schritt (%.1f s)\n",
           steps, dir > 0 ? "vorwaerts" : "rueckwaerts", delay_us,
           (double)steps * delay_us / 1e6);

    int  rc   = 0;
    long done = 0;
    for (; done < steps && !g_abort; done++) {
        if (stepper_advance(motor, dir) != 0) {
            fprintf(stderr, "Schritt %ld fehlgeschlagen.\n", done);
            rc = 1;
            break;
        }
        sleep_us(delay_us);

        if (!dry_run && (done + 1) % 512 == 0) {
            printf("  %ld / %ld\r", done + 1, steps);
            fflush(stdout);
        }
    }

    if (g_abort) printf("\nAbbruch nach %ld Schritten.\n", done);

    printf("\nPosition: %ld Halbschritte (%.2f Umdrehungen)\n",
           stepper_position(motor),
           (double)stepper_position(motor) / HALFSTEPS_PER_REV);
    
    if (hold && !g_abort && rc == 0) {
        stepper_hold(motor);
        printf("Spulen bleiben bestromt. Beenden mit Ctrl-C.\n");
        while (!g_abort) pause();
        printf("\n");
    }

    stepper_destroy(motor);   /* schaltet die Spulen stromlos */
    return rc;
}
