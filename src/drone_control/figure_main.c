#define _POSIX_C_SOURCE 200809L

#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "figure.h"
#include "geometry.h"
#include "kinematics.h"
#include "motion.h"
#include "path.h"
#include "pins.h"
#include "stepper.h"

static volatile sig_atomic_t g_abort = 0;

static void on_signal(int sig) {
    (void)sig;
    g_abort = 1;
}

static void sleep_ms(long ms) {
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

static long ms_since(const struct timespec *then) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - then->tv_sec) * 1000L
         + (now.tv_nsec - then->tv_nsec) / 1000000L;
}

typedef struct {
    double worst_spread_mm;
    double min_tension;
    double min_height_mm;
    double max_height_mm;
    int    spread_warned;
    int    tension_warned;
    int    height_warned;
} RunStats;

/* ---------------------------------------------------------------- Ausgabe */

static void print_usage(const char *prog) {
    fprintf(stderr,
        "Verwendung: %s [--size X,Y] [--segment-mm N] [--delay-us N] [--loops N]\n"
        "               [--tension MM] [--settle-ms N] [--wait N] [--mass-g N]\n"
        "               [--plan-only] [--hold] [--dry-run] [--quiet]\n"
        "\n"
        "Faehrt ein X ohne Kamera. Die Plattform wird vorher von Hand in die\n"
        "Mitte des Ankerfelds gestellt; genau dort setzt das Programm seinen\n"
        "Nullpunkt. Danach laufen vier Speichen Mitte -> Spitze -> Mitte.\n"
        "\n"
        "  --size X,Y     halber Abstand Mitte -> Spitze in mm,\n"
        "                 Standard %.0f,%.0f, Grenze %.0f,%.0f\n"
        "  --segment-mm   Laenge eines Teilstuecks, Standard %.0f mm. Kleiner\n"
        "                 heisst genauer, aber mehr Nachrechnen unterwegs\n"
        "  --delay-us     Zeit pro Halbschritt, Standard %u us (Minimum %u us)\n"
        "  --loops        wie oft das X gefahren wird, Standard 1\n"
        "  --tension MM   vorher alle vier Seile um MM anspannen, Standard aus.\n"
        "                 Haengen die Seile durch, hilft z.B. --tension 1\n"
        "  --settle-ms    Pause an jeder Spitze, Standard %d ms\n"
        "  --wait N       Sekunden Vorlauf vor der ersten Bewegung, Standard %d\n"
        "  --mass-g N     Masse der Plattform in g, prueft die Winden dagegen\n"
        "  --plan-only    nur rechnen und ausgeben, nichts fahren\n"
        "  --hold         Spulen am Ende bestromt lassen, bis Ctrl-C\n"
        "  --dry-run      Motorphasen nur ausgeben, GPIO nicht anfassen\n"
        "  --quiet        keine laufende Statuszeile\n",
        prog,
        ACE_FIGURE_SPAN_X_MM / 2.0, ACE_FIGURE_SPAN_Y_MM / 2.0,
        ACE_REACH_LIMIT_X_MM, ACE_REACH_LIMIT_Y_MM,
        ACE_SEGMENT_MM, ACE_TRAVEL_DELAY_US, ACE_MIN_STEP_DELAY_US,
        ACE_FIGURE_SETTLE_MS, ACE_FIGURE_WAIT_S);
}

static void print_header(double half_x, double half_y, double segment_mm,
                         unsigned int delay_us, int loops) {
    printf("ACE Figurenfahrt - X, ohne Kamera\n\n");

    printf("Ankerfeld      %.0f x %.0f mm, Anker %.0f mm ueber der Flaeche\n",
           ACE_ANCHOR_SPAN_X_MM, ACE_ANCHOR_SPAN_Y_MM, ACE_RIG_HEIGHT_MM);
    printf("Plattform      %.0f mm unter den Ankern\n", ACE_HOVER_HEIGHT_MM);
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        printf("Winde %d %-13s Anker %+4.0f,%+4.0f  Wickel %.2f mm"
               "  ->  %.4f mm/Halbschritt\n",
               i, ACE_MOTOR_NAMES[i],
               ACE_MOTOR_CORNER[i][0] * ACE_ANCHOR_SPAN_X_MM / 2.0,
               ACE_MOTOR_CORNER[i][1] * ACE_ANCHOR_SPAN_Y_MM / 2.0,
               ACE_MOTOR_DRUM_MM[i], ACE_MM_PER_HALFSTEP_AT(i));
    }
    printf("Fahrgrenze     x +/-%.0f mm, y +/-%.0f mm\n",
           ACE_REACH_LIMIT_X_MM, ACE_REACH_LIMIT_Y_MM);
    printf("Figur          X, Spitzen bei +/-%.0f, +/-%.0f mm, %dx\n",
           half_x, half_y, loops);
    printf("Aufloesung     %.1f mm je Teilstueck, %u us je Halbschritt"
           "  ->  %.1f mm Seil/s\n\n",
           segment_mm, delay_us,
           ACE_MM_PER_HALFSTEP_AT(0) / (delay_us / 1000000.0));
}

static int print_waypoints(const FigurePoint *points, int count) {
    int weak = 0;

    printf("Wegpunkte - Statik im Raum\n");
    printf("  #  Punkt              x      y  |");
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) printf("  Seil%d", i);
    printf(" |  flachstes  Seilzug min..max\n");

    for (int p = 0; p < count; p++) {
        FigurePose pose;
        figure_pose(points[p].x_mm, points[p].y_mm, &pose);

        printf("  %d  %-13s %+6.1f %+6.1f  |", p, points[p].label,
               points[p].x_mm, points[p].y_mm);
        for (int i = 0; i < ACE_MOTOR_COUNT; i++)
            printf("  %5.0f", pose.length_mm[i]);
        printf(" |   %4.1f Grad   %.2f .. %.2f",
               pose.min_elevation_deg, pose.min_tension, pose.max_tension);

        if (!pose.inside_anchor_field) {
            printf("   AUSSERHALB DES ANKERFELDS");
            weak++;
        } else if (pose.min_tension < ACE_MIN_CABLE_TENSION) {
            printf("   Seil zu lose");
            weak++;
        }
        printf("\n");
    }
    printf("  Seilzug in Vielfachen des Plattformgewichts. Weil die Seile flach\n"
           "  liegen, ist der Zug groesser als die Last, die sie abnehmen.\n\n");
    return weak;
}

static void print_legs(const FigurePoint *points, int count,
                       double segment_mm, unsigned int delay_us) {
    double total_s     = 0.0;
    long   total_steps = 0;

    printf("Teilfahrten\n");
    printf("  %-3s%-13s    %-13s %6s  %4s  %10s  %6s\n",
           "#", "von", "nach", "Weg", "Stk", "Halbschr.", "Dauer");

    for (int p = 1; p < count; p++) {
        FigureSim sim;
        figure_simulate(points[p - 1].x_mm, points[p - 1].y_mm,
                        points[p].x_mm, points[p].y_mm, segment_mm, &sim);

        double dx  = points[p].x_mm - points[p - 1].x_mm;
        double dy  = points[p].y_mm - points[p - 1].y_mm;
        double way = sqrt(dx * dx + dy * dy);

        /* Die Bresenham-Kopplung taktet die fuehrende Winde, also bestimmt
         * deren Schrittzahl die Dauer, nicht die Summe ueber alle vier.
         * Welche Winde fuehrt, entscheidet sich in Halbschritten, nicht in
         * mm: bei ungleichen Wickeldurchmessern ist das nicht dasselbe. */
        double lead_steps = 0.0;
        for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
            double d = fabs(kin_cable_length(i, points[p].x_mm, points[p].y_mm)
                          - kin_cable_length(i, points[p - 1].x_mm,
                                                points[p - 1].y_mm))
                     / ACE_MM_PER_HALFSTEP_AT(i);
            if (d > lead_steps) lead_steps = d;
        }
        double secs = lead_steps * (delay_us / 1000000.0);

        printf("  %-3d%-13s -> %-13s %6.1f  %4d  %10ld  %6.1f s\n",
               p, points[p - 1].label, points[p].label,
               way, sim.segments, sim.total_halfsteps, secs);

        total_s     += secs;
        total_steps += sim.total_halfsteps;
    }
    printf("  %-3s%-13s    %-13s %6s  %4s  %10ld  %6.1f s\n\n",
           "", "Summe", "", "", "", total_steps, total_s);
}

/* Der eigentliche Punkt der Uebung: zeigen, was die 3D-Geometrie zwischen
 * zwei Wegpunkten anrichtet, wenn man sie nicht laufend nachrechnet. */
static void print_space_check(const FigurePoint *points, int count,
                              double segment_mm) {
    double worst_naive_off = 0.0, worst_naive_dz = 0.0;
    double worst_plan_off  = 0.0, worst_plan_dz  = 0.0;

    printf("Kontrolle im Raum - was zwischen den Wegpunkten passiert\n");
    printf("  Die Seillaenge ist die 3D-Strecke Anker -> Plattform und haengt\n"
           "  deshalb nicht linear an x und y. Laesst man die vier Winden stur\n"
           "  von A nach B durchlaufen, passen ihre Laengen unterwegs zu keinem\n"
           "  gemeinsamen Punkt mehr: jede fordert eine andere Hoehe. Zwei Seile\n"
           "  werden lose, die Plattform sackt und pendelt. Darum wird alle\n"
           "  %.1f mm aus der vollen 3D-Formel neu geplant.\n\n", segment_mm);

    printf("  %-3s%-13s    %-13s %9s %7s  |  %9s %7s\n",
           "#", "von", "nach", "naiv quer", "naiv dz", "gepl.quer", "gepl.dz");
    for (int p = 1; p < count; p++) {
        FigureSim naive, plan;

        figure_simulate(points[p - 1].x_mm, points[p - 1].y_mm,
                        points[p].x_mm, points[p].y_mm, 0.0, &naive);
        figure_simulate(points[p - 1].x_mm, points[p - 1].y_mm,
                        points[p].x_mm, points[p].y_mm, segment_mm, &plan);

        printf("  %-3d%-13s -> %-13s %9.3f %7.3f  |  %9.3f %7.3f\n",
               p, points[p - 1].label, points[p].label,
               naive.max_offline_mm, naive.max_height_mm,
               plan.max_offline_mm, plan.max_height_mm);

        if (naive.max_offline_mm > worst_naive_off) worst_naive_off = naive.max_offline_mm;
        if (naive.max_height_mm  > worst_naive_dz)  worst_naive_dz  = naive.max_height_mm;
        if (plan.max_offline_mm  > worst_plan_off)  worst_plan_off  = plan.max_offline_mm;
        if (plan.max_height_mm   > worst_plan_dz)   worst_plan_dz   = plan.max_height_mm;
    }

    printf("\n  quer = Abstand von der Geraden, dz = Widerspruch zwischen den vier\n"
           "  geforderten Hoehen. Beides in mm.\n");
    printf("  schlimmster Fall   naiv %.2f quer / %.2f dz"
           "   geplant %.3f quer / %.3f dz\n",
           worst_naive_off, worst_naive_dz, worst_plan_off, worst_plan_dz);

    if (worst_plan_dz > ACE_MAX_HEIGHT_SPREAD_MM) {
        printf("  Der geplante Wert liegt ueber %.1f mm. --segment-mm verkleinern.\n",
               (double)ACE_MAX_HEIGHT_SPREAD_MM);
    }
    printf("\n");
}

static void print_load(const FigurePoint *points, int count, double mass_g) {
    double max_tension = 0.0, min_tension = 0.0;
    int    max_motor = 0, max_point = 0;

    for (int p = 0; p < count; p++) {
        FigurePose pose;
        figure_pose(points[p].x_mm, points[p].y_mm, &pose);

        for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
            if (pose.tension[i] > max_tension) {
                max_tension = pose.tension[i];
                max_motor   = i;
                max_point   = p;
            }
            if ((p == 0 && i == 0) || pose.tension[i] < min_tension)
                min_tension = pose.tension[i];
        }
    }

    double weakest_n = figure_winch_force_n(0);
    for (int i = 1; i < ACE_MOTOR_COUNT; i++) {
        double f = figure_winch_force_n(i);
        if (f < weakest_n) weakest_n = f;
    }

    printf("Last und Antrieb\n");
    printf("  groesster Seilzug   %.2f x Plattformgewicht  (Winde %d %s, bei %s)\n",
           max_tension, max_motor, ACE_MOTOR_NAMES[max_motor],
           points[max_point].label);
    printf("  kleinster Seilzug   %.2f x Plattformgewicht  (Grenze %.2f)\n",
           min_tension, (double)ACE_MIN_CABLE_TENSION);
    printf("  Winde schafft       %.2f N  (%.1f Nmm auf %.1f mm Wickelradius)\n",
           weakest_n, (double)ACE_MOTOR_TORQUE_NMM, ACE_MOTOR_DRUM_MM[0] / 2.0);

    if (max_tension > 0.0) {
        /* Gewichtskraft in N, die eine Winde bei diesem Zugverhaeltnis noch
         * halten kann; daraus die Masse in Gramm. */
        double max_weight_n = weakest_n / max_tension;
        printf("  tragbare Plattform  bis %.0f g\n",
               max_weight_n / (ACE_GRAVITY_MM_S2 / 1000.0) * 1000.0);
    }

    if (mass_g > 0.0) {
        double weight_n = (mass_g / 1000.0) * (ACE_GRAVITY_MM_S2 / 1000.0);
        double need_n   = weight_n * max_tension;
        const char *verdict =
            (need_n > weakest_n)         ? "ZU VIEL, die Winde verliert Schritte"
          : (need_n > weakest_n * 0.6)   ? "knapp, unter 40 % Reserve"
                                         : "in Ordnung";
        printf("  bei %.0f g gebraucht %.2f N von %.2f N  ->  %s\n",
               mass_g, need_n, weakest_n, verdict);
    }
    printf("  Das Haltemoment gilt beim 28BYJ-48 nur im Kriechgang. Bei\n"
           "  Betriebsdrehzahl bleibt rund die Haelfte, also Reserve lassen.\n\n");
}

/* ------------------------------------------------------------------ Fahrt */

static void track(RunStats *st, double spread, double height,
                  const FigurePose *pose) {
    if (spread > st->worst_spread_mm) st->worst_spread_mm = spread;
    if (pose->min_tension < st->min_tension) st->min_tension = pose->min_tension;
    if (height < st->min_height_mm) st->min_height_mm = height;
    if (height > st->max_height_mm) st->max_height_mm = height;

    if (!st->height_warned &&
        fabs(height - ACE_HOVER_HEIGHT_MM) > ACE_HEIGHT_DRIFT_WARN_MM) {
        st->height_warned = 1;
        fprintf(stderr,
                "\nDie Plattform haengt auf %.1f mm statt %.0f mm. Geplant wird "
                "auf der\ngemessenen Hoehe, die Fahrt bleibt also waagerecht - "
                "aber die Ursache\nbleibt: Seildehnung, Schlupf oder ein falscher "
                "Wickeldurchmesser.\n",
                height, ACE_HOVER_HEIGHT_MM);
    }

    if (!st->spread_warned && spread > ACE_MAX_HEIGHT_SPREAD_MM) {
        st->spread_warned = 1;
        fprintf(stderr, "\nDie vier Seile fordern %.2f mm verschiedene Hoehen "
                        "und ziehen gegeneinander.\n--segment-mm verkleinern.\n",
                spread);
    }
    if (!st->tension_warned && pose->min_tension < ACE_MIN_CABLE_TENSION) {
        st->tension_warned = 1;
        fprintf(stderr, "\nEin Seil traegt nur noch %.2f des Plattformgewichts "
                        "und wird lose.\nSein Schrittzaehler beschreibt die Lage "
                        "dann nicht mehr: --size verkleinern.\n",
                pose->min_tension);
    }
}

static int drive_leg(double from_x, double from_y, const FigurePoint *to,
                     unsigned int delay_us, int quiet, RunStats *st) {
    if (path_start(to->x_mm, to->y_mm, delay_us) != 0) {
        fprintf(stderr, "Teilfahrt nach %s konnte nicht gestartet werden.\n",
                to->label);
        return -1;
    }

    double dx  = to->x_mm - from_x;
    double dy  = to->y_mm - from_y;
    double leg = sqrt(dx * dx + dy * dy);

    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    long last_print = -1000;

    while (path_busy() && !g_abort) {
        long   steps[ACE_MOTOR_COUNT];
        double lengths[ACE_MOTOR_COUNT];
        double x, y;

        motion_positions(steps);
        kin_lengths(steps, lengths);
        kin_position_from_lengths(lengths, &x, &y);

        /* Laufende 3D-Kontrolle: die vier koppelnavigierten Laengen muessen
         * auch zwischen den Wegpunkten zu einem einzigen Punkt im Raum
         * passen, sonst ziehen die Winden gegeneinander. Dazu die Hoehe
         * selbst - sie ist keine Konstante, sondern folgt aus den Laengen. */
        double     spread = kin_height_spread(lengths, x, y);
        double     height = kin_height(lengths, x, y);
        FigurePose pose;
        figure_pose(x, y, &pose);
        track(st, spread, height, &pose);

        if (!quiet) {
            long now = ms_since(&t0);
            if (now - last_print >= 200) {
                last_print = now;

                double done = (leg > 1e-9)
                    ? sqrt((x - from_x) * (x - from_x)
                         + (y - from_y) * (y - from_y)) / leg * 100.0
                    : 100.0;
                if (done > 100.0) done = 100.0;
                if (done < 0.0)   done = 0.0;

                printf("\r  -> %-13s %+7.1f,%+7.1f mm  h %5.1f  Seil",
                       to->label, x, y, height);
                for (int i = 0; i < ACE_MOTOR_COUNT; i++)
                    printf(" %5.0f", lengths[i]);
                printf("  dz %5.3f  Zug %4.2f  %3.0f %%   ",
                       spread, pose.min_tension, done);
                fflush(stdout);
            }
        }
        sleep_ms(40);
    }

    if (g_abort) {
        path_abort();
        return 1;
    }
    if (!quiet) printf("\n");
    return 0;
}

static int apply_tension(double mm, unsigned int delay_us) {
    long steps[ACE_MOTOR_COUNT];

    for (int i = 0; i < ACE_MOTOR_COUNT; i++)
        steps[i] = lround(mm / ACE_MM_PER_HALFSTEP_AT(i));

    printf("Spannen: alle vier Seile %+.1f mm, %+ld Halbschritte je Winde.\n",
           mm, steps[0]);

    if (motion_start(steps, delay_us) != 0) return -1;
    while (motion_busy() && !g_abort) sleep_ms(20);

    return g_abort ? 1 : 0;
}

static void print_result(const RunStats *st) {
    long   steps[ACE_MOTOR_COUNT];
    double lengths[ACE_MOTOR_COUNT];
    double x, y;

    motion_positions(steps);
    kin_lengths(steps, lengths);
    kin_position_from_lengths(lengths, &x, &y);

    printf("\nErgebnis\n");
    printf("  Endlage laut Schrittzaehlern  x=%+.2f  y=%+.2f mm"
           "   (Soll 0,0, daneben %.2f mm)\n", x, y, sqrt(x * x + y * y));
    printf("  Hoehe unter den Ankern        %.2f mm"
           "   (Nennmass %.0f, unterwegs %.1f .. %.1f)\n",
           kin_height(lengths, x, y), ACE_HOVER_HEIGHT_MM,
           st->min_height_mm, st->max_height_mm);
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        double diff = lengths[i] - kin_cable_length(i, 0.0, 0.0);
        printf("  Winde %d %-13s %+7.3f mm gegenueber der Mitte"
               "  (%+ld Halbschritte)\n",
               i, ACE_MOTOR_NAMES[i], diff,
               lround(diff / ACE_MM_PER_HALFSTEP_AT(i)));
    }
    printf("  groesster Hoehenwiderspruch unterwegs  %.3f mm\n",
           st->worst_spread_mm);
    printf("  kleinster Seilzug unterwegs            %.2f x Gewicht\n",
           st->min_tension);
    printf("\n  Diese Zahlen kommen aus den Schrittzaehlern, nicht aus einer\n"
           "  Messung. Ob die Plattform wirklich wieder in der Mitte steht,\n"
           "  zeigt nur das Lineal: die Winden koennen Schritte verloren haben.\n");
}

/* ------------------------------------------------------------------- main */

int main(int argc, char **argv) {
    double       half_x     = ACE_FIGURE_SPAN_X_MM / 2.0;
    double       half_y     = ACE_FIGURE_SPAN_Y_MM / 2.0;
    double       segment_mm = ACE_SEGMENT_MM;
    unsigned int delay_us   = ACE_TRAVEL_DELAY_US;
    int          loops      = 1;
    double       tension_mm = 0.0;
    long         settle_ms  = ACE_FIGURE_SETTLE_MS;
    int          wait_s     = ACE_FIGURE_WAIT_S;
    double       mass_g     = 0.0;
    int          plan_only  = 0;
    int          hold       = 0;
    int          dry_run    = 0;
    int          quiet      = 0;

    static struct option long_opts[] = {
        {"size",       required_argument, 0, 's'},
        {"segment-mm", required_argument, 0, 'g'},
        {"delay-us",   required_argument, 0, 'u'},
        {"loops",      required_argument, 0, 'l'},
        {"tension",    required_argument, 0, 'T'},
        {"settle-ms",  required_argument, 0, 'S'},
        {"wait",       required_argument, 0, 'w'},
        {"mass-g",     required_argument, 0, 'm'},
        {"plan-only",  no_argument,       0, 'p'},
        {"hold",       no_argument,       0, 'H'},
        {"dry-run",    no_argument,       0, 'n'},
        {"quiet",      no_argument,       0, 'q'},
        {"help",       no_argument,       0,  1 },
        {0, 0, 0, 0}
    };

    int opt, opt_index = 0;
    while ((opt = getopt_long(argc, argv, "s:g:u:l:T:S:w:m:pHnq",
                              long_opts, &opt_index)) != -1) {
        switch (opt) {
            case 'g': segment_mm = atof(optarg); break;
            case 'u': delay_us   = (unsigned int)strtoul(optarg, NULL, 10); break;
            case 'l': loops      = atoi(optarg); break;
            case 'T': tension_mm = atof(optarg); break;
            case 'S': settle_ms  = atol(optarg); break;
            case 'w': wait_s     = atoi(optarg); break;
            case 'm': mass_g     = atof(optarg); break;
            case 'p': plan_only  = 1; break;
            case 'H': hold       = 1; break;
            case 'n': dry_run    = 1; break;
            case 'q': quiet      = 1; break;
            case 's':
                if (sscanf(optarg, "%lf,%lf", &half_x, &half_y) != 2) {
                    fprintf(stderr, "--size erwartet X,Y in mm, z.B. --size 120,90\n");
                    return 1;
                }
                break;
            case 1:
                print_usage(argv[0]);
                return 0;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    if (half_x <= 0.0 || half_y <= 0.0) {
        fprintf(stderr, "--size braucht positive Werte.\n");
        return 1;
    }
    if (half_x > ACE_REACH_LIMIT_X_MM || half_y > ACE_REACH_LIMIT_Y_MM) {
        fprintf(stderr,
                "X zu gross: %.0f,%.0f mm liegt ausserhalb der Fahrgrenze "
                "%.0f,%.0f mm.\nkin_plan wuerde beschneiden und das X waere "
                "verzogen. --size kleiner waehlen oder\nACE_REACH_LIMIT_* in "
                "geometry.h anheben.\n",
                half_x, half_y, ACE_REACH_LIMIT_X_MM, ACE_REACH_LIMIT_Y_MM);
        return 1;
    }
    if (loops < 1)           loops      = 1;
    if (segment_mm <= 0.0)   segment_mm = ACE_SEGMENT_MM;
    if (wait_s < 0)          wait_s     = 0;
    if (settle_ms < 0)       settle_ms  = 0;

    /* Hier einmal begrenzen statt in motion_start bei jedem der vielen
     * Teilstuecke, und damit stimmen auch die Dauern in der Planung. */
    if (delay_us < ACE_MIN_STEP_DELAY_US) {
        fprintf(stderr, "Schrittzeit %u us zu kurz, auf %u us begrenzt.\n\n",
                delay_us, ACE_MIN_STEP_DELAY_US);
        delay_us = ACE_MIN_STEP_DELAY_US;
    }

    FigurePoint points[ACE_FIGURE_MAX_POINTS];
    int         count = figure_x_points(half_x, half_y, points);

    print_header(half_x, half_y, segment_mm, delay_us, loops);
    int weak = print_waypoints(points, count);
    print_legs(points, count, segment_mm, delay_us);
    print_space_check(points, count, segment_mm);
    print_load(points, count, mass_g);

    if (weak > 0) {
        fprintf(stderr, "%d Wegpunkt(e) sind statisch heikel, siehe oben. "
                        "--size verkleinern.\n\n", weak);
    }
    if (plan_only) return 0;

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    if (dry_run) stepper_set_dry_run(1);

    if (motion_init() != 0) return 1;
    if (motion_thread_start() != 0) {
        motion_shutdown();
        return 1;
    }

    printf("Die Plattform muss jetzt von Hand in der Mitte des Ankerfelds stehen.\n");
    for (int s = wait_s; s > 0 && !g_abort; s--) {
        printf("\rStart in %d s ... ", s);
        fflush(stdout);
        sleep_ms(1000);
    }
    printf("\r                      \r");

    int rc       = 0;
    int zero_set = 0;

    if (!g_abort && tension_mm != 0.0) {
        if (apply_tension(tension_mm, delay_us) < 0) rc = 1;
    }

    if (!g_abort && rc == 0) {
        /* Erst jetzt den Nullpunkt setzen: nach dem Spannen stehen die
         * Schrittzaehler auf dem Stand, der zur Mitte gehoert. */
        long start_steps[ACE_MOTOR_COUNT];
        motion_positions(start_steps);
        kin_reset(0.0, 0.0, start_steps);
        zero_set = 1;
        printf("Nullpunkt gesetzt: hier ist x=0, y=0.\n\n");
    }

    RunStats st = { 0.0, 1e9, 1e9, -1e9, 0, 0, 0 };

    for (int loop = 0; loop < loops && rc == 0 && !g_abort; loop++) {
        if (loops > 1) printf("Durchgang %d von %d\n", loop + 1, loops);

        for (int p = 1; p < count && rc == 0 && !g_abort; p++) {
            int r = drive_leg(points[p - 1].x_mm, points[p - 1].y_mm,
                              &points[p], delay_us, quiet, &st);
            if (r < 0) rc = 1;
            if (r != 0) break;

            if (settle_ms > 0 && !g_abort) sleep_ms(settle_ms);
        }
    }

    if (g_abort) {
        path_abort();
        printf("\nAbbruch.\n");
    }

    /* Ohne gesetzten Nullpunkt hat kin_* keine Referenz, die Zahlen waeren
     * Zufall. Dann lieber gar nichts behaupten. */
    if (zero_set) {
        if (st.min_tension > 1e8) st.min_tension = 0.0;
        print_result(&st);
    }

    if (hold && rc == 0 && !g_abort) {
        motion_hold();
        printf("\nSpulen bleiben bestromt. Beenden mit Ctrl-C.\n");
        while (!g_abort) pause();
        printf("\n");
    }

    /* Erst die Bahn stilllegen: sonst startet der Idle-Hook von path.c im
     * Motor-Thread noch ein Teilstueck, nachdem die Spulen stromlos sind. */
    path_abort();
    motion_release();
    motion_shutdown();
    return rc;
}
