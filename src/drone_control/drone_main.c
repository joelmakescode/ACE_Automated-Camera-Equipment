#define _POSIX_C_SOURCE 200809L

#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>

#include "ball_detector.h"
#include "calibrate.h"
#include "geometry.h"
#include "kinematics.h"
#include "motion.h"
#include "navigator.h"
#include "pins.h"
#include "stepper.h"

static volatile sig_atomic_t g_abort = 0;

static void on_signal(int sig) {
    (void)sig;
    g_abort = 1;
}

static void print_usage(const char *prog) {
    HsvRange def = bd_default_hsv_range();

    fprintf(stderr,
        "Verwendung: %s [--device /dev/video0] [--width N] [--height N]\n"
        "               [--h-min N] [--h-max N] [--s-min N] [--s-max N]\n"
        "               [--v-min N] [--v-max N] [--stream [port]]\n"
        "               [--delay-us N] [--dry-run] [--quiet]\n"
        "               [--calibrate | --center] [--at X,Y] [--calib-mm N]\n"
        "\n"
        "  --calibrate faehrt %.0f mm in x und y, misst am Bild die Verschiebung\n"
        "              des Objekts, kommt in die Mitte zurueck und spannt an\n"
        "  --center    nur in die Mitte fahren und anspannen, ohne Messung\n"
        "  --goto X,Y  auf diese Position fahren und dort anspannen; zeigt vorher\n"
        "              je Winde, ob das Seil kuerzer oder laenger werden soll\n"
        "  --at X,Y    wo die Plattform gerade haengt, Standard 0,0 (Mitte)\n"
        "  --calib-mm  Kalibrierweg in mm, Standard %.0f\n"
        "  --object-mm Durchmesser des Kalibrierobjekts in mm (rundes farbiges\n"
        "              Ding, kein Ball noetig). Ohne diese Angabe kann nur die\n"
        "              Sichtbreite bestimmt werden, nicht der Wickeldurchmesser\n"
        "  --delay-us  Zeit pro Halbschritt, Standard %u us (Minimum %u us)\n"
        "  --dry-run   Motorphasen nur ausgeben, GPIO nicht anfassen\n"
        "  --quiet     keine Statuszeile pro Frame\n"
        "\n"
        "Farbe: Standard ist Rot auf hellem Grund, h %d-%d s %d-%d v %d-%d.\n"
        "       Rot liegt an beiden Enden der Hue-Skala. Ist h-min groesser als\n"
        "       h-max, sucht der Detektor darum in beiden Bereichen, also\n"
        "       h-min..179 und 0..h-max. Die Saettigung trennt das Objekt vom\n"
        "       hellen Hintergrund: Weiss und Grau haben kaum Saettigung.\n"
        "       Spricht nichts an, zuerst s-min senken, dann v-min.\n",
        prog, ACE_CALIBRATION_DISTANCE_MM, ACE_CALIBRATION_DISTANCE_MM,
        ACE_TRAVEL_DELAY_US, ACE_MIN_STEP_DELAY_US,
        def.h_min, def.h_max, def.s_min, def.s_max, def.v_min, def.v_max);
}

static void print_geometry(void) {
    double half_diagonal = 0.5 * sqrt(ACE_ANCHOR_SPAN_X_MM * ACE_ANCHOR_SPAN_X_MM
                                    + ACE_ANCHOR_SPAN_Y_MM * ACE_ANCHOR_SPAN_Y_MM);
    double cable_angle = atan2(ACE_HOVER_HEIGHT_MM, half_diagonal) * 180.0 / ACE_PI;

    printf("Ankerfeld      %.0f x %.0f mm, Anker %.0f mm ueber der Flaeche\n",
           ACE_ANCHOR_SPAN_X_MM, ACE_ANCHOR_SPAN_Y_MM, ACE_RIG_HEIGHT_MM);
    printf("Plattform      %.0f mm unter den Ankern  ->  Kamera %.0f mm ueber der Flaeche\n",
           ACE_HOVER_HEIGHT_MM, ACE_CAMERA_HEIGHT_MM);
    printf("Seil in Mitte  %.1f mm lang, %.1f Grad zur Waagerechten\n",
           kin_cable_length(0, 0.0, 0.0), cable_angle);
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        printf("Winde %d %-13s Wickel %.2f mm -> %.4f mm/Halbschritt, Trim %+.1f mm\n",
               i, ACE_MOTOR_NAMES[i], ACE_MOTOR_DRUM_MM[i],
               ACE_MM_PER_HALFSTEP_AT(i), ACE_MOTOR_TRIM_MM[i]);
    }
    printf("Suchfahrt X    %.0f x %.0f mm\n",
           ACE_PATROL_SPAN_X_MM, ACE_PATROL_SPAN_Y_MM);
    printf("Fahrgrenze     x +/-%.0f mm, y +/-%.0f mm\n",
           ACE_REACH_LIMIT_X_MM, ACE_REACH_LIMIT_Y_MM);
    printf("Sichtbreite    %.0f mm\n", ACE_VIEW_WIDTH_MM);
    printf("Startposition  x=%.0f y=%.0f mm\n", ACE_START_X_MM, ACE_START_Y_MM);
    printf("Bahnaufloesung %.0f mm je Teilstueck\n\n", ACE_SEGMENT_MM);
}

int main(int argc, char **argv) {
    const char  *device      = "/dev/video0";
    int          width       = 1280;
    int          height      = 720;
    int          stream_on   = 0;
    int          stream_port = 8080;
    unsigned int delay_us    = ACE_TRAVEL_DELAY_US;
    int          dry_run     = 0;
    int          quiet       = 0;
    int          mode_cal    = 0;
    int          mode_center = 0;
    double       at_x        = 0.0;
    double       at_y        = 0.0;
    double       calib_mm    = ACE_CALIBRATION_DISTANCE_MM;
    double       object_mm   = ACE_OBJECT_DIAMETER_MM;
    int          mode_goto   = 0;
    double       goto_x      = 0.0;
    double       goto_y      = 0.0;

    HsvRange range = bd_default_hsv_range();

    static struct option long_opts[] = {
        {"device",   required_argument, 0, 'd'},
        {"width",    required_argument, 0, 'w'},
        {"height",   required_argument, 0, 'h'},
        {"stream",   optional_argument, 0, 't'},
        {"delay-us", required_argument, 0, 'u'},
        {"dry-run",  no_argument,       0, 'n'},
        {"quiet",     no_argument,       0, 'q'},
        {"calibrate", no_argument,       0, 'c'},
        {"center",    no_argument,       0, 'C'},
        {"at",        required_argument, 0, 'a'},
        {"calib-mm",  required_argument, 0, 'D'},
        {"object-mm", required_argument, 0, 'O'},
        {"goto",      required_argument, 0, 'g'},
        {"h-min",    required_argument, 0, 1},
        {"h-max",    required_argument, 0, 2},
        {"s-min",    required_argument, 0, 3},
        {"s-max",    required_argument, 0, 4},
        {"v-min",    required_argument, 0, 5},
        {"v-max",    required_argument, 0, 6},
        {"help",     no_argument,       0, '?'},
        {0, 0, 0, 0}
    };

    int opt, opt_index = 0;
    while ((opt = getopt_long(argc, argv, "d:w:h:t::u:nqcCa:D:O:g:", long_opts, &opt_index)) != -1) {
        switch (opt) {
            case 'd': device   = optarg; break;
            case 'w': width    = atoi(optarg); break;
            case 'h': height   = atoi(optarg); break;
            case 'u': delay_us = (unsigned int)strtoul(optarg, NULL, 10); break;
            case 'n': dry_run  = 1; break;
            case 'q': quiet    = 1; break;
            case 'c': mode_cal    = 1; break;
            case 'C': mode_center = 1; break;
            case 'D': calib_mm    = atof(optarg); break;
            case 'O': object_mm   = atof(optarg); break;
            case 'g':
                if (sscanf(optarg, "%lf,%lf", &goto_x, &goto_y) != 2) {
                    fprintf(stderr, "--goto erwartet X,Y in mm, z.B. --goto 50,0\n");
                    return 1;
                }
                mode_goto = 1;
                break;
            case 'a':
                if (sscanf(optarg, "%lf,%lf", &at_x, &at_y) != 2) {
                    fprintf(stderr, "--at erwartet X,Y in mm, z.B. --at 80,-40\n");
                    return 1;
                }
                break;
            case 't':
                stream_on = 1;
                if (optarg) stream_port = atoi(optarg);
                break;
            case 1: range.h_min = atoi(optarg); break;
            case 2: range.h_max = atoi(optarg); break;
            case 3: range.s_min = atoi(optarg); break;
            case 4: range.s_max = atoi(optarg); break;
            case 5: range.v_min = atoi(optarg); break;
            case 6: range.v_max = atoi(optarg); break;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    print_geometry();

    if (dry_run) stepper_set_dry_run(1);

    BallDetector *detector = bd_create_camera(device, width, height);
    if (!detector) {
        fprintf(stderr, "Konnte Kamera nicht initialisieren.\n");
        return 1;
    }

    if (motion_init() != 0) {
        bd_release(detector);
        return 1;
    }

    if (motion_thread_start() != 0) {
        motion_shutdown();
        bd_release(detector);
        return 1;
    }

    if (mode_cal || mode_center || mode_goto) {
        int cal_rc;
        if (mode_cal) {
            cal_rc = cal_run(detector, &range, width, at_x, at_y,
                             calib_mm, object_mm, delay_us);
        } else if (mode_goto) {
            cal_rc = cal_goto(detector, &range, at_x, at_y, goto_x, goto_y, delay_us);
        } else {
            cal_rc = cal_center(detector, &range, at_x, at_y, delay_us);
        }

        if (!g_abort) {
            printf("Spannung wird gehalten. Beenden mit Ctrl-C.\n");
            while (!g_abort) pause();
            printf("\n");
        }

        motion_release();
        motion_shutdown();
        bd_release(detector);
        return cal_rc == 0 ? 0 : 1;
    }

    if (nav_init(width, height, delay_us) != 0) {
        motion_shutdown();
        bd_release(detector);
        return 1;
    }

    if (stream_on) {
        if (bd_stream_start(stream_port) != 0) {
            fprintf(stderr, "Konnte Stream-Server auf Port %d nicht starten.\n",
                    stream_port);
            motion_shutdown();
            bd_release(detector);
            return 1;
        }
        fprintf(stderr, "Live-Stream unter http://<pi-ip>:%d/\n", stream_port);
    }

    long frame_num = 0;
    int  rc = 0;

    while (!g_abort) {
        DetectionResult result;
        if (bd_detect(detector, &range, &result) != 0) {
            fprintf(stderr, "Frame-Erfassung fehlgeschlagen, breche ab.\n");
            rc = 1;
            break;
        }

        nav_update(&result);

        if (stream_on) bd_stream_push(detector, &result);

        if (!quiet) {
            double x_mm, y_mm;
            nav_target(&x_mm, &y_mm);
            printf("frame=%ld %-9s found=%d px=%.0f,%.0f pos=%.0f,%.0f mm\n",
                   frame_num, nav_state_name(nav_state()), result.found ? 1 : 0,
                   result.x, result.y, x_mm, y_mm);
            fflush(stdout);
        }

        frame_num++;
    }

    if (g_abort) printf("\nAbbruch.\n");

    if (stream_on) bd_stream_stop();

    motion_release();
    motion_shutdown();
    bd_release(detector);
    return rc;
}
