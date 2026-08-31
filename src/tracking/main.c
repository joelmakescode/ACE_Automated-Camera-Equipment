#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

#include "ball_detector.h"

static void print_usage(const char *prog) {
    fprintf(stderr,
        "Verwendung: %s [--device /dev/video0 | --image pfad.jpg]\n"
        "               [--width N] [--height N] [--frames N]\n"
        "               [--h-min N] [--h-max N] [--s-min N] [--s-max N]\n"
        "               [--v-min N] [--v-max N] [--save-frame pfad.jpg]\n",
        prog);
}

int main(int argc, char **argv) {
    const char *device = "/dev/video0";
    const char *image_path = NULL;
    const char *save_frame_path = NULL;
    int width = 1280;
    int height = 720;
    long frames = 0;

    HsvRange range = bd_default_hsv_range();

    static struct option long_opts[] = {
        {"device",     required_argument, 0, 'd'},
        {"image",      required_argument, 0, 'i'},
        {"width",      required_argument, 0, 'w'},
        {"height",     required_argument, 0, 'h'},
        {"frames",     required_argument, 0, 'f'},
        {"save-frame", required_argument, 0, 's'},
        {"h-min",      required_argument, 0, 1},
        {"h-max",      required_argument, 0, 2},
        {"s-min",      required_argument, 0, 3},
        {"s-max",      required_argument, 0, 4},
        {"v-min",      required_argument, 0, 5},
        {"v-max",      required_argument, 0, 6},
        {"help",       no_argument,       0, '?'},
        {0, 0, 0, 0}
    };

    int opt, opt_index = 0;
    while ((opt = getopt_long(argc, argv, "d:i:w:h:f:s:", long_opts, &opt_index)) != -1) {
        switch (opt) {
            case 'd': device = optarg; break;
            case 'i': image_path = optarg; break;
            case 'w': width = atoi(optarg); break;
            case 'h': height = atoi(optarg); break;
            case 'f': frames = atol(optarg); break;
            case 's': save_frame_path = optarg; break;
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

    BallDetector *detector = image_path
        ? bd_create_from_image(image_path)
        : bd_create_camera(device, width, height);

    if (!detector) {
        fprintf(stderr, "Konnte Detector nicht initialisieren.\n");
        return 1;
    }

    long frame_num = 0;
    for (;;) {
        DetectionResult result;
        if (bd_detect(detector, &range, &result) != 0) {
            fprintf(stderr, "Frame-Erfassung fehlgeschlagen, breche ab.\n");
            bd_release(detector);
            return 1;
        }

        printf("frame=%ld found=%d x=%.1f y=%.1f radius=%.1f\n",
               frame_num, result.found ? 1 : 0, result.x, result.y, result.radius);

        if (save_frame_path && bd_save_annotated(detector, &result, save_frame_path) != 0) {
            fprintf(stderr, "Konnte Frame nicht nach %s schreiben.\n", save_frame_path);
        }

        frame_num++;

        if (image_path) break;
        if (frames > 0 && frame_num >= frames) break;
    }

    bd_release(detector);
    return 0;
}