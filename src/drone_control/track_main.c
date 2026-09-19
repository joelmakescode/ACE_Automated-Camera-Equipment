#define _POSIX_C_SOURCE 200809L

#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "ball_detector.h"
#include "figure.h"
#include "geometry.h"
#include "kinematics.h"
#include "motion.h"
#include "path.h"
#include "pins.h"
#include "stepper.h"
#include "visual.h"

static volatile sig_atomic_t g_abort = 0;

static void on_signal(int sig) {
    (void)sig;
    g_abort = 1;
}

static void sleep_ms(long ms) {
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/* ------------------------------------------------------------- Zustand */

typedef struct {
    BallDetector *cam;
    HsvRange      range;
    int           width, height;

    VisualModel   vis;
    unsigned int  delay_us;
    double        gain;
    double        tolerance_px;
    double        max_step_mm;
    double        probe_mm;
    long          settle_ms;
    int           quiet;

    int    weak;                /* Winde, die Schritte verliert, -1 = keine */
    double slip_total_mm;       /* seit dem letzten Referenzieren           */
    int    slip_events;
    double slip_worst_mm;

    double height_min_mm;
    double height_max_mm;
    int    height_warned;

    long   cycles;
    int    relearns;
    int    lost;
} Tracker;

/* Lage des Objekts relativ zur Bildmitte, gemittelt ueber mehrere Treffer.
 * Nebenbei wird dabei die Kamerapipe leergelesen - wird waehrend einer
 * Fahrt nicht gelesen, staut sich rpicam-vid auf und das naechste Bild ist
 * Sekunden alt. */
static int measure(Tracker *t, double *err_u, double *err_v, double *radius) {
    double sum_u = 0.0, sum_v = 0.0, sum_r = 0.0;
    int    hits  = 0;

    for (int f = 0; f < ACE_TRACK_SAMPLE_FRAMES && !g_abort; f++) {
        DetectionResult r;
        if (bd_detect(t->cam, &t->range, &r) != 0) return -1;

        if (r.found) {
            sum_u += r.x - t->width  / 2.0;
            sum_v += r.y - t->height / 2.0;
            sum_r += r.radius;
            if (++hits >= ACE_TRACK_SAMPLES) break;
        }
    }

    if (hits < ACE_TRACK_SAMPLES) return 1;      /* Objekt nicht sicher da */

    if (err_u)  *err_u  = sum_u / hits;
    if (err_v)  *err_v  = sum_v / hits;
    if (radius) *radius = sum_r / hits;
    return 0;
}

static void read_state(double *x_mm, double *y_mm, long steps[ACE_MOTOR_COUNT]) {
    motion_positions(steps);
    kin_position(steps, x_mm, y_mm);
}

/* Die Hoehe ist keine Konstante: die Plattform haengt, und ihre Hoehe folgt
 * aus den Seillaengen. Fuer x und y ist das folgenlos, weil sich z in der
 * Paarformel herauskuerzt - fuer die Planung nicht, darum wird sie
 * mitgefuehrt und hier ueberwacht. */
static double read_height(Tracker *t) {
    long   steps[ACE_MOTOR_COUNT];
    double x, y, h;

    motion_positions(steps);
    kin_pose(steps, &x, &y, &h);

    if (h < t->height_min_mm) t->height_min_mm = h;
    if (h > t->height_max_mm) t->height_max_mm = h;

    if (!t->height_warned &&
        fabs(h - ACE_HOVER_HEIGHT_MM) > ACE_HEIGHT_DRIFT_WARN_MM) {
        t->height_warned = 1;
        fprintf(stderr,
                "\nDie Plattform haengt auf %.1f mm statt %.0f mm unter den "
                "Ankern.\nGeplant wird auf der gemessenen Hoehe, die Fahrt "
                "bleibt also waagerecht.\nDie Ursache bleibt: Seildehnung, "
                "Schlupf oder falscher Wickeldurchmesser.\n",
                h, ACE_HOVER_HEIGHT_MM);
    }
    return h;
}

/* Fahrt abwarten und dabei weiter Bilder holen, damit die Pipe nicht
 * volllaeuft und die Messung danach aktuell ist. */
static int drive_and_drain(Tracker *t) {
    while (path_busy() && !g_abort) {
        DetectionResult ignored;
        if (bd_detect(t->cam, &t->range, &ignored) != 0) return -1;
    }
    if (g_abort) { path_abort(); return 1; }

    if (t->settle_ms > 0) {
        struct timespec t0;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        for (;;) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            long ms = (now.tv_sec - t0.tv_sec) * 1000L
                    + (now.tv_nsec - t0.tv_nsec) / 1000000L;
            if (ms >= t->settle_ms || g_abort) break;

            DetectionResult ignored;
            if (bd_detect(t->cam, &t->range, &ignored) != 0) return -1;
        }
    }
    return 0;
}

/* ---------------------------------------------------- schwache Winde */

/* Der Schlupf ist messbar, weil die Lage aus den uebrigen drei Winden
 * vollstaendig bestimmt ist: die Laenge, die diese Lage fuer die schwache
 * Winde fordert, gegen die Laenge, die ihr Schrittzaehler behauptet. Das
 * braucht keine Kamera und keine Annahme ueber die Ursache. */
static void check_slip(Tracker *t) {
    if (t->weak < 0) return;

    long steps[ACE_MOTOR_COUNT];
    motion_positions(steps);

    double slip = kin_length_residual(t->weak, steps);
    double mag  = fabs(slip);

    if (mag > t->slip_worst_mm) t->slip_worst_mm = mag;
    t->slip_total_mm = slip;

    if (mag < ACE_SLIP_WARN_MM) return;

    double x, y;
    kin_position(steps, &x, &y);

    if (mag >= ACE_SLIP_RECOVER_MM) {
        kin_reset_motor(t->weak, x, y, steps);
        t->slip_events++;
        fprintf(stderr,
                "\nWinde %d (%s) ist um %+.2f mm durchgerutscht, neu referenziert "
                "(%d. Mal).\n", t->weak, ACE_MOTOR_NAMES[t->weak], slip,
                t->slip_events);
    } else if (!t->quiet) {
        fprintf(stderr, "\nWinde %d (%s): %+.2f mm Schlupf.\n",
                t->weak, ACE_MOTOR_NAMES[t->weak], slip);
    }
}

/* ------------------------------------------------------------ Lernphase */

/* Vier feste Probefahrten, deren Bildantwort das Modell bestimmt. Der
 * Ausreisserfilter bleibt dabei offen: er soll bewegte Objekte abwehren,
 * wuerde hier aber genau die Messungen abweisen, die ein grob falsches
 * Modell geraderuecken. Das Objekt muss in dieser Phase stillliegen. */
static int learn(Tracker *t) {
    static const double dir[4][2] = { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } };

    double probe_dx[4], probe_dy[4], drop_u[4], drop_v[4];
    int    good = 0;

    vis_relearn_begin(&t->vis);

    for (int k = 0; k < 4 && !g_abort; k++) {
        double eu0, ev0, eu1, ev1;
        double x0, y0, x1, y1;
        long   steps[ACE_MOTOR_COUNT];

        int m = measure(t, &eu0, &ev0, NULL);
        if (m != 0) {
            fprintf(stderr, "Lernphase: Objekt nicht sichtbar. HSV-Bereich oder "
                            "Beleuchtung pruefen.\n");
            return -1;
        }
        read_state(&x0, &y0, steps);

        double tx = x0 + dir[k][0] * t->probe_mm;
        double ty = y0 + dir[k][1] * t->probe_mm;
        kin_clamp(&tx, &ty);

        if (path_start(tx, ty, t->delay_us) != 0) return -1;
        if (drive_and_drain(t) != 0) return -1;

        m = measure(t, &eu1, &ev1, NULL);
        if (m != 0) {
            fprintf(stderr, "Lernphase: Objekt nach der Probefahrt verloren. "
                            "--probe-mm verkleinern.\n");
            return -1;
        }
        read_state(&x1, &y1, steps);

        /* Der tatsaechlich gefahrene Weg aus den vertrauenswuerdigen Winden,
         * nicht der kommandierte: so faelscht ein Schlupf das Bildmodell
         * nicht. */
        probe_dx[good] = x1 - x0;
        probe_dy[good] = y1 - y0;
        drop_u[good]   = eu0 - eu1;
        drop_v[good]   = ev0 - ev1;

        vis_update(&t->vis, probe_dx[good], probe_dy[good],
                   drop_u[good], drop_v[good], 1e9, 1e9);
        good++;

        check_slip(t);
    }

    if (g_abort) return 1;
    if (good < 2) {
        fprintf(stderr, "Lernphase: zu wenige brauchbare Probefahrten.\n");
        return -1;
    }

    /* Die Probefahrten gegen das fertige Modell zurueckrechnen. Passen sie
     * nicht zueinander, hat sich das Objekt bewegt oder eine Winde ist
     * gerutscht - dann taugt das Modell nichts. */
    double worst = 0.0;
    for (int k = 0; k < good; k++) {
        double pu, pv;
        vis_predict(&t->vis, probe_dx[k], probe_dy[k], &pu, &pv);
        double r = sqrt((drop_u[k] - pu) * (drop_u[k] - pu)
                      + (drop_v[k] - pv) * (drop_v[k] - pv));
        if (r > worst) worst = r;
    }

    printf("Bildmodell   %.3f px/mm, Kamera %+.1f Grad verdreht"
           "   (Sichtbreite %.0f mm)\n",
           vis_scale_px_per_mm(&t->vis), vis_angle_deg(&t->vis),
           vis_view_width_mm(&t->vis, t->width));
    printf("             groesste Abweichung der Probefahrten %.1f px\n", worst);

    if (worst > 0.25 * vis_scale_px_per_mm(&t->vis) * t->probe_mm) {
        printf("             Das ist viel. Lag das Objekt still? Eine Winde, die\n"
               "             waehrend der Probefahrt rutscht, sieht genauso aus.\n");
    }
    return 0;
}

/* ------------------------------------------------------------- Regelung */

static int follow(Tracker *t, long max_cycles) {
    double prev_err = -1.0;
    int    stall    = 0;
    int    lost_run = 0;

    while (!g_abort && (max_cycles <= 0 || t->cycles < max_cycles)) {
        double eu, ev, radius;

        int m = measure(t, &eu, &ev, &radius);
        if (m < 0) return -1;
        if (m > 0) {
            /* Ohne Objekt wird nicht geraten. Stehenbleiben ist hier die
             * sichere Antwort, erst recht mit einer Winde, die rutscht. */
            if (lost_run == 0) {
                printf("\nObjekt nicht im Bild, Plattform bleibt stehen.\n");
            }
            lost_run++;
            t->lost++;
            continue;
        }
        if (lost_run > 0) {
            printf("Objekt wieder da.\n");
            lost_run = 0;
        }

        double err = sqrt(eu * eu + ev * ev);
        t->cycles++;

        if (err <= t->tolerance_px) {
            double h = read_height(t);
            if (!t->quiet) {
                printf("\rzentriert   Bildfehler %5.1f px   r=%4.0f px"
                       "   Kamera %+6.1f Grad   h %5.1f mm   Schlupf %+5.2f mm   ",
                       err, radius, vis_angle_deg(&t->vis), h, t->slip_total_mm);
                fflush(stdout);
            }
            prev_err = err;
            check_slip(t);
            continue;
        }

        /* Klingt der Fehler nicht ab, liegt der geschaetzte Kamerawinkel
         * weiter daneben, als der Regler von sich aus einfangen kann. */
        if (prev_err > 0.0 && err > prev_err * ACE_TRACK_STALL_RATIO) {
            if (++stall >= ACE_TRACK_STALL_LIMIT) {
                printf("\nBildfehler klingt nicht ab (%.0f -> %.0f px). "
                       "Kamera hat sich gedreht,\nBildmodell wird neu gelernt.\n",
                       prev_err, err);
                stall = 0;
                t->relearns++;
                if (learn(t) < 0) return -1;
                prev_err = -1.0;
                continue;
            }
        } else {
            stall = 0;
        }

        double dx, dy;
        vis_required_move(&t->vis, eu, ev, &dx, &dy);
        dx *= t->gain;
        dy *= t->gain;

        double step = sqrt(dx * dx + dy * dy);
        if (step > t->max_step_mm) {
            dx *= t->max_step_mm / step;
            dy *= t->max_step_mm / step;
        }

        double x0, y0;
        long   steps[ACE_MOTOR_COUNT];
        read_state(&x0, &y0, steps);

        double tx = x0 + dx;
        double ty = y0 + dy;
        kin_clamp(&tx, &ty);

        if (!t->quiet) {
            printf("\rZug %-4ld Bildfehler %5.1f px  ->  %+6.1f,%+6.1f mm"
                   "   Lage %+6.1f,%+6.1f  h %5.1f   Kamera %+6.1f Grad   ",
                   t->cycles, err, dx, dy, tx, ty, read_height(t),
                   vis_angle_deg(&t->vis));
            fflush(stdout);
        }

        if (path_start(tx, ty, t->delay_us) != 0) return -1;
        int r = drive_and_drain(t);
        if (r < 0) return -1;
        if (r > 0) break;

        double x1, y1;
        read_state(&x1, &y1, steps);

        double eu1, ev1;
        if (measure(t, &eu1, &ev1, NULL) == 0) {
            vis_update(&t->vis, x1 - x0, y1 - y0, eu - eu1, ev - ev1,
                       ACE_VIS_GATE_REL, ACE_VIS_GATE_PX);
        }

        check_slip(t);
        prev_err = err;
    }
    return 0;
}

/* --------------------------------------------------------------- Bericht */

static void print_setup(const Tracker *t) {
    printf("ACE Objektverfolgung mit Kamera\n\n");
    printf("Bild           %d x %d px, Mitte bei %d,%d\n",
           t->width, t->height, t->width / 2, t->height / 2);
    printf("Ziel           Objekt innerhalb %.0f px um die Bildmitte\n",
           t->tolerance_px);
    printf("Regler         Gain %.2f, hoechstens %.0f mm je Zug\n",
           t->gain, t->max_step_mm);
    printf("               stabil bis %.0f Grad Fehler im Kamerawinkel\n",
           acos(t->gain / 2.0 > 1.0 ? 1.0 : t->gain / 2.0) * 180.0 / ACE_PI);

    if (t->weak >= 0) {
        printf("Schwache Winde %d (%s): aus der Lageschaetzung genommen.\n",
               t->weak, ACE_MOTOR_NAMES[t->weak]);
        printf("               Die Lage kommt aus den anderen drei, dadurch ist\n"
               "               der Schlupf dieser Winde messbar. Warnung ab\n"
               "               %.1f mm, neu referenziert ab %.1f mm.\n",
               (double)ACE_SLIP_WARN_MM, (double)ACE_SLIP_RECOVER_MM);
        printf("               Vorspannung %.1f mm, damit ihr Seil straff bleibt.\n",
               (double)ACE_WEAK_PRELOAD_MM);

        double margin = figure_margin_without(t->weak, 0.0, 0.0);
        printf("Arbeitsraum    Faellt sie ganz aus, tragen drei Seile nur noch\n"
               "               ueber ihrem Dreieck. In der Mitte betraegt der\n"
               "               Abstand zu dessen Kante %+.0f mm.\n", margin);
        if (margin <= 1.0) {
            printf("               Das heisst: eine Haelfte der Flaeche haengt an\n"
                   "               dieser Winde. Faellt sie aus, sackt die Plattform\n"
                   "               dort weg. Die Vorspannung ist genau dagegen da.\n");
        }
    } else {
        printf("Schwache Winde keine, alle vier gehen in die Lage ein.\n");
    }
    printf("\n");
}

static void print_result(const Tracker *t) {
    printf("\n\nErgebnis\n");
    printf("  Zuege                        %ld\n", t->cycles);
    printf("  Bildmodell neu gelernt       %d x\n", t->relearns);
    printf("  Kamera zuletzt               %+.1f Grad, %.3f px/mm"
           "  (Sichtbreite %.0f mm)\n",
           vis_angle_deg(&t->vis), vis_scale_px_per_mm(&t->vis),
           vis_view_width_mm(&t->vis, t->width));
    printf("  Messungen uebernommen        %d, verworfen %d\n",
           t->vis.samples, t->vis.rejected);
    printf("  Bilder ohne Objekt           %d\n", t->lost);

    if (t->height_max_mm > t->height_min_mm) {
        printf("  Hoehe unter den Ankern       %.1f .. %.1f mm"
               "  (Nennmass %.0f)\n",
               t->height_min_mm, t->height_max_mm, ACE_HOVER_HEIGHT_MM);
        printf("                               geplant wurde zuletzt auf %.1f mm\n",
               kin_planned_height());
    }

    if (t->weak >= 0) {
        printf("  Winde %d neu referenziert     %d x\n", t->weak, t->slip_events);
        printf("  groesster Schlupf            %.2f mm"
               "  (%ld Halbschritte)\n",
               t->slip_worst_mm,
               lround(t->slip_worst_mm / ACE_MM_PER_HALFSTEP_AT(t->weak)));
        if (t->slip_events > 0) {
            printf("\n  Der Schlupf wurde jedes Mal herausgerechnet, die Lage der\n"
                   "  Plattform blieb dadurch richtig. Haeufen sich die Ereignisse,\n"
                   "  hilft nur die Mechanik: Getriebe, Seilfuehrung, Wickelreibung.\n");
        }
    }
    printf("\n  Die Lage stammt aus den Schrittzaehlern der gesunden Winden.\n"
           "  Ob das Objekt wirklich mittig steht, sagt allein das Bild - und\n"
           "  das steht oben als Bildfehler.\n");
}

/* ------------------------------------------------------------------ main */

static void print_usage(const char *prog) {
    HsvRange d = bd_default_hsv_range();

    fprintf(stderr,
        "Verwendung: %s [--device X] [--width N] [--height N]\n"
        "               [--h-min N] [--h-max N] [--s-min N] [--s-max N]\n"
        "               [--v-min N] [--v-max N] [--stream[=port]]\n"
        "               [--weak-motor N] [--preload MM] [--no-learn]\n"
        "               [--gain G] [--tolerance PX] [--max-step MM]\n"
        "               [--probe-mm MM] [--delay-us N] [--cycles N]\n"
        "               [--tension MM] [--wait N] [--dry-run] [--quiet]\n"
        "\n"
        "Haelt ein farbiges Objekt in der Bildmitte. Die Plattform steht zu\n"
        "Beginn von Hand in der Mitte des Ankerfelds; dort liegt der Nullpunkt.\n"
        "\n"
        "  --weak-motor N Winde, die Schritte verliert, Standard %d. Sie wird\n"
        "                 aus der Lageschaetzung genommen, wodurch ihr Schlupf\n"
        "                 messbar wird. -1 schaltet das ab\n"
        "  --preload MM   Vorspannung auf dieser Winde, Standard %.1f mm\n"
        "  --no-learn     ohne Lernphase starten, mit den Werten aus geometry.h\n"
        "  --probe-mm MM  Laenge einer Probefahrt der Lernphase, Standard %.0f\n"
        "  --gain G       Daempfung, Standard %.2f. Kleiner = mehr Reserve\n"
        "                 gegen eine verdrehte Kamera, aber langsamer\n"
        "  --tolerance PX Objekt gilt innerhalb davon als zentriert, Std. %.0f\n"
        "  --max-step MM  laengster Fahrweg je Zug, Standard %.0f\n"
        "  --cycles N     nach N Zuegen beenden, Standard 0 = endlos\n"
        "  --tension MM   vor dem Start alle vier Seile anspannen\n"
        "  --stream       Livebild mit Markierung auf Port 8080\n"
        "\n"
        "Farbe: Standard Rot, h %d-%d s %d-%d v %d-%d. Ist h-min groesser als\n"
        "       h-max, wird ueber den Nullpunkt der Hue-Skala hinweg gesucht.\n",
        prog, ACE_WEAK_MOTOR, (double)ACE_WEAK_PRELOAD_MM,
        (double)ACE_TRACK_PROBE_MM, (double)ACE_TRACK_GAIN,
        (double)ACE_TRACK_TOLERANCE_PX, (double)ACE_TRACK_MAX_STEP_MM,
        d.h_min, d.h_max, d.s_min, d.s_max, d.v_min, d.v_max);
}

int main(int argc, char **argv) {
    const char *device      = "/dev/video0";
    int         width       = 1280;
    int         height      = 720;
    int         stream_on   = 0;
    int         stream_port = 8080;
    int         do_learn    = 1;
    int         dry_run     = 0;
    int         wait_s      = ACE_FIGURE_WAIT_S;
    double      tension_mm  = 0.0;
    double      preload_mm  = ACE_WEAK_PRELOAD_MM;
    long        max_cycles  = 0;

    Tracker t;
    t.range        = bd_default_hsv_range();
    t.delay_us     = ACE_TRAVEL_DELAY_US;
    t.gain         = ACE_TRACK_GAIN;
    t.tolerance_px = ACE_TRACK_TOLERANCE_PX;
    t.max_step_mm  = ACE_TRACK_MAX_STEP_MM;
    t.probe_mm     = ACE_TRACK_PROBE_MM;
    t.settle_ms    = ACE_TRACK_SETTLE_MS;
    t.quiet        = 0;
    t.weak         = ACE_WEAK_MOTOR;
    t.slip_total_mm = 0.0;
    t.slip_events  = 0;
    t.slip_worst_mm = 0.0;
    t.height_min_mm = 1e9;
    t.height_max_mm = -1e9;
    t.height_warned = 0;
    t.cycles       = 0;
    t.relearns     = 0;
    t.lost         = 0;
    t.cam          = NULL;

    static struct option lo[] = {
        {"device",      required_argument, 0, 'd'},
        {"width",       required_argument, 0, 'w'},
        {"height",      required_argument, 0, 'H'},
        {"stream",      optional_argument, 0, 't'},
        {"weak-motor",  required_argument, 0, 'W'},
        {"preload",     required_argument, 0, 'P'},
        {"no-learn",    no_argument,       0, 'L'},
        {"gain",        required_argument, 0, 'g'},
        {"tolerance",   required_argument, 0, 'e'},
        {"max-step",    required_argument, 0, 'M'},
        {"probe-mm",    required_argument, 0, 'b'},
        {"delay-us",    required_argument, 0, 'u'},
        {"cycles",      required_argument, 0, 'c'},
        {"tension",     required_argument, 0, 'T'},
        {"settle-ms",   required_argument, 0, 'S'},
        {"wait",        required_argument, 0, 'a'},
        {"dry-run",     no_argument,       0, 'n'},
        {"quiet",       no_argument,       0, 'q'},
        {"h-min",       required_argument, 0, 1},
        {"h-max",       required_argument, 0, 2},
        {"s-min",       required_argument, 0, 3},
        {"s-max",       required_argument, 0, 4},
        {"v-min",       required_argument, 0, 5},
        {"v-max",       required_argument, 0, 6},
        {"help",        no_argument,       0, 7},
        {0, 0, 0, 0}
    };

    int opt, oi = 0;
    while ((opt = getopt_long(argc, argv, "d:w:H:t::W:P:Lg:e:M:b:u:c:T:S:a:nq",
                              lo, &oi)) != -1) {
        switch (opt) {
            case 'd': device         = optarg; break;
            case 'w': width          = atoi(optarg); break;
            case 'H': height         = atoi(optarg); break;
            case 'W': t.weak         = atoi(optarg); break;
            case 'P': preload_mm     = atof(optarg); break;
            case 'L': do_learn       = 0; break;
            case 'g': t.gain         = atof(optarg); break;
            case 'e': t.tolerance_px = atof(optarg); break;
            case 'M': t.max_step_mm  = atof(optarg); break;
            case 'b': t.probe_mm     = atof(optarg); break;
            case 'u': t.delay_us     = (unsigned int)strtoul(optarg, NULL, 10); break;
            case 'c': max_cycles     = atol(optarg); break;
            case 'T': tension_mm     = atof(optarg); break;
            case 'S': t.settle_ms    = atol(optarg); break;
            case 'a': wait_s         = atoi(optarg); break;
            case 'n': dry_run        = 1; break;
            case 'q': t.quiet        = 1; break;
            case 't':
                stream_on = 1;
                if (optarg) stream_port = atoi(optarg);
                break;
            case 1: t.range.h_min = atoi(optarg); break;
            case 2: t.range.h_max = atoi(optarg); break;
            case 3: t.range.s_min = atoi(optarg); break;
            case 4: t.range.s_max = atoi(optarg); break;
            case 5: t.range.v_min = atoi(optarg); break;
            case 6: t.range.v_max = atoi(optarg); break;
            case 7: print_usage(argv[0]); return 0;
            default: print_usage(argv[0]); return 1;
        }
    }

    if (width <= 0 || height <= 0) {
        fprintf(stderr, "--width und --height muessen positiv sein.\n");
        return 1;
    }
    if (t.gain <= 0.0 || t.gain > 1.5) {
        fprintf(stderr, "--gain sollte zwischen 0 und 1.5 liegen.\n");
        return 1;
    }
    if (t.weak >= ACE_MOTOR_COUNT) {
        fprintf(stderr, "--weak-motor muss zwischen -1 und %d liegen.\n",
                ACE_MOTOR_COUNT - 1);
        return 1;
    }
    if (t.weak < 0) t.weak = -1;
    if (t.delay_us < ACE_MIN_STEP_DELAY_US) {
        fprintf(stderr, "Schrittzeit %u us zu kurz, auf %u us begrenzt.\n",
                t.delay_us, ACE_MIN_STEP_DELAY_US);
        t.delay_us = ACE_MIN_STEP_DELAY_US;
    }
    if (wait_s < 0)       wait_s    = 0;
    if (t.settle_ms < 0)  t.settle_ms = 0;

    t.width  = width;
    t.height = height;

    print_setup(&t);

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    if (dry_run) stepper_set_dry_run(1);

    t.cam = bd_create_camera(device, width, height);
    if (!t.cam) {
        fprintf(stderr, "Konnte Kamera nicht initialisieren.\n");
        return 1;
    }

    if (motion_init() != 0) {
        bd_release(t.cam);
        return 1;
    }
    if (motion_thread_start() != 0) {
        motion_shutdown();
        bd_release(t.cam);
        return 1;
    }

    if (stream_on) {
        if (bd_stream_start(stream_port) != 0) {
            fprintf(stderr, "Stream-Server auf Port %d fehlgeschlagen.\n",
                    stream_port);
        } else {
            fprintf(stderr, "Livebild unter http://<pi-ip>:%d/\n", stream_port);
        }
    }

    printf("Die Plattform muss jetzt von Hand in der Mitte stehen.\n");
    for (int s = wait_s; s > 0 && !g_abort; s--) {
        printf("\rStart in %d s ... ", s);
        fflush(stdout);
        sleep_ms(1000);
    }
    printf("\r                      \r");

    int rc = 0;

    if (!g_abort && tension_mm != 0.0) {
        long steps[ACE_MOTOR_COUNT];
        for (int i = 0; i < ACE_MOTOR_COUNT; i++)
            steps[i] = lround(tension_mm / ACE_MM_PER_HALFSTEP_AT(i));
        printf("Spannen: alle vier Seile %+.1f mm.\n", tension_mm);
        if (motion_start(steps, t.delay_us) == 0) {
            while (motion_busy() && !g_abort) sleep_ms(20);
        }
    }

    if (!g_abort) {
        long steps[ACE_MOTOR_COUNT];
        motion_positions(steps);
        kin_reset(0.0, 0.0, steps);

        /* Erst nach dem Nullpunkt: sonst wuerde die Vorspannung schon in die
         * Referenzlaenge einfliessen und sich damit selbst aufheben. */
        kin_set_untrusted(t.weak);
        if (t.weak >= 0) kin_set_trim(t.weak, -preload_mm);

        printf("Nullpunkt gesetzt.\n\n");

        double px_per_mm = (double)width / ACE_VIEW_WIDTH_MM;
        vis_init(&t.vis, px_per_mm, 0.0, ACE_VIS_LAMBDA);

        if (do_learn) {
            printf("Lernphase: vier Probefahrten ueber %.0f mm. Objekt bitte\n"
                   "still liegen lassen.\n", t.probe_mm);
            int lr = learn(&t);
            if (lr < 0) rc = 1;
            printf("\n");
        } else {
            printf("Ohne Lernphase, Startwerte aus geometry.h: %.3f px/mm, "
                   "0 Grad.\n\n", px_per_mm);
        }

        if (rc == 0 && !g_abort) {
            if (follow(&t, max_cycles) < 0) rc = 1;
        }
    }

    if (g_abort) {
        path_abort();
        printf("\nAbbruch.\n");
    }

    print_result(&t);

    if (stream_on) bd_stream_stop();

    path_abort();
    motion_release();
    motion_shutdown();
    bd_release(t.cam);
    return rc;
}
