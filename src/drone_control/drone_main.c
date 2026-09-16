#define _POSIX_C_SOURCE 200809L

#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include "ball_detector.h"
#include "geometry.h"
#include "kinematics.h"
#include "motion.h"
#include "navigator.h"
#include "stepper.h"

static volatile sig_atomic_t g_abort = 0;

static void on_signal(int sig) {
    (void)sig;
    g_abort = 1;
}

static void print_usage(const char *prog) {
    fprintf(stderr,
        "Verwendung: %s [--device /dev/video0] [--width N] [--height N]\n"
        "               [--h-min N] [--h-max N] [--s-min N] [--s-max N]\n"
        "               [--v-min N] [--v-max N] [--stream [port]]\n"
        "               [--delay-us N] [--dry-run] [--quiet]\n"
        "\n"
        "  --delay-us Zeit pro Halbschritt, Standard %u us (Minimum %u us)\n"
        "  --dry-run  Motorphasen nur ausgeben, GPIO nicht anfassen\n"
        "  --quiet    keine Statuszeile pro Frame\n",
        prog, ACE_TRAVEL_DELAY_US, ACE_MIN_STEP_DELAY_US);
}

static void print_geometry(void) {
    printf("Ankerquadrat   %.0f x %.0f mm\n",
           ACE_ANCHOR_SPAN_X_MM, ACE_ANCHOR_SPAN_Y_MM);
    printf("Schwebehoehe   %.0f mm\n", ACE_HOVER_HEIGHT_MM);
    printf("Wickeldurchm.  %.1f mm  ->  %.4f mm pro Halbschritt\n",
           ACE_DRUM_DIAMETER_MM, ACE_MM_PER_HALFSTEP);
    printf("Suchfahrt X    %.0f x %.0f mm\n",
           ACE_PATROL_SPAN_MM, ACE_PATROL_SPAN_MM);
    printf("Sichtbreite    %.0f mm\n", ACE_VIEW_WIDTH_MM);
    printf("Startposition  x=%.0f y=%.0f mm\n\n",
           ACE_START_X_MM, ACE_START_Y_MM);
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

    HsvRange range = bd_default_hsv_range();

    static struct option long_opts[] = {
        {"device",   required_argument, 0, 'd'},
        {"width",    required_argument, 0, 'w'},
        {"height",   required_argument, 0, 'h'},
        {"stream",   optional_argument, 0, 't'},
        {"delay-us", required_argument, 0, 'u'},
        {"dry-run",  no_argument,       0, 'n'},
        {"quiet",    no_argument,       0, 'q'},
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
    while ((opt = getopt_long(argc, argv, "d:w:h:t::u:nq", long_opts, &opt_index)) != -1) {
        switch (opt) {
            case 'd': device   = optarg; break;
            case 'w': width    = atoi(optarg); break;
            case 'h': height   = atoi(optarg); break;
            case 'u': delay_us = (unsigned int)strtoul(optarg, NULL, 10); break;
            case 'n': dry_run  = 1; break;
            case 'q': quiet    = 1; break;
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

    if (nav_init(width, height, delay_us) != 0) {
        motion_shutdown();
        bd_release(detector);
        return 1;
    }

    if (motion_thread_start() != 0) {
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
