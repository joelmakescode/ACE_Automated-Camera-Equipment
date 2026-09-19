#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

#include "ball_detector.h"

static void print_usage(const char *prog) {
    HsvRange def = bd_default_hsv_range();

    fprintf(stderr,
        "Verwendung: %s [--device /dev/video0 | --image pfad.jpg]\n"
        "               [--width N] [--height N] [--frames N]\n"
        "               [--h-min N] [--h-max N] [--s-min N] [--s-max N]\n"
        "               [--v-min N] [--v-max N] [--save-frame pfad.jpg]\n"
        "               [--stream [port]] [--probe] [--mask]\n"
        "\n"
        "  --probe  misst, welche HSV-Werte in der Bildmitte tatsaechlich\n"
        "           ankommen, und schlaegt daraus einen Bereich vor. Objekt\n"
        "           dafuer mittig vor die Kamera halten.\n"
        "  --mask   zeigt in --stream und --save-frame die Maske statt des\n"
        "           Bildes: weiss ist, was der Bereich gerade durchlaesst.\n"
        "\n"
        "Farbe: Standard ist Rot auf hellem Grund, h %d-%d s %d-%d v %d-%d.\n"
        "       Ist h-min groesser als h-max, wird ueber den Nullpunkt hinweg\n"
        "       gesucht, also h-min..179 und 0..h-max. Genau so wird Rot\n"
        "       erfasst, das an beiden Enden der Hue-Skala liegt.\n"
        "       Mit --stream laesst sich der Bereich am Livebild pruefen.\n",
        prog, def.h_min, def.h_max, def.s_min, def.s_max, def.v_min, def.v_max);
}

static void print_probe(const ProbeResult *pr, const HsvRange *range) {
    printf("  Bildmitte   H %3d (%d..%d)  S %3d (%d..%d)  V %3d (%d..%d)\n",
           pr->h_med, pr->h_lo, pr->h_hi,
           pr->s_med, pr->s_lo, pr->s_hi,
           pr->v_med, pr->v_lo, pr->v_hi);
    printf("  BGR-Mittel  B %3d  G %3d  R %3d   %s\n",
           pr->b_mean, pr->g_mean, pr->r_mean,
           pr->r_mean > pr->b_mean + 30 ? "(rotlastig, plausibel)"
         : pr->b_mean > pr->r_mean + 30 ? "(BLAULASTIG - R und B vertauscht?)"
         : "(weder rot noch blau dominant)");
    printf("  Maske       %ld von %ld Pixeln (%.2f %%), groesste Flaeche %.0f px\n",
           pr->mask_pixels, pr->frame_pixels,
           100.0 * (double)pr->mask_pixels / (double)pr->frame_pixels,
           pr->best_area);
    printf("  aktuell     h %d-%d  s %d-%d  v %d-%d\n",
           range->h_min, range->h_max, range->s_min, range->s_max,
           range->v_min, range->v_max);

    if (pr->s_med < 60) {
        printf("  -> Saettigung %d ist sehr niedrig. Das Objekt kommt grau an:\n"
               "     zu wenig Licht, Weissabgleich, oder die Farbe ist blass.\n",
               pr->s_med);
    }
    if (pr->hue_wraps) {
        printf("  -> Hue liegt an beiden Enden, also Rot. Vorschlag:\n"
               "     --h-min 170 --h-max 10 --s-min %d --v-min %d\n",
               pr->s_lo > 40 ? pr->s_lo - 20 : 20,
               pr->v_lo > 40 ? pr->v_lo - 20 : 20);
    } else {
        int lo = pr->h_lo > 5 ? pr->h_lo - 5 : 0;
        int hi = pr->h_hi < 174 ? pr->h_hi + 5 : 179;
        printf("  -> Vorschlag: --h-min %d --h-max %d --s-min %d --v-min %d\n",
               lo, hi,
               pr->s_lo > 40 ? pr->s_lo - 20 : 20,
               pr->v_lo > 40 ? pr->v_lo - 20 : 20);
    }
}

int main(int argc, char **argv) {
    const char *device = "/dev/video0";
    const char *image_path = NULL;
    const char *save_frame_path = NULL;
    int width = 1280;
    int height = 720;
    long frames = 0;
    int stream_enabled = 0;
    int stream_port = 8080;
    int probe = 0;
    int view_mask = 0;

    HsvRange range = bd_default_hsv_range();

    static struct option long_opts[] = {
        {"device",     required_argument, 0, 'd'},
        {"image",      required_argument, 0, 'i'},
        {"width",      required_argument, 0, 'w'},
        {"height",     required_argument, 0, 'h'},
        {"frames",     required_argument, 0, 'f'},
        {"save-frame", required_argument, 0, 's'},
        {"stream",     optional_argument, 0, 't'},
        {"probe",      no_argument,       0, 'p'},
        {"mask",       no_argument,       0, 'm'},
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
    while ((opt = getopt_long(argc, argv, "d:i:w:h:f:s:t::pm", long_opts, &opt_index)) != -1) {
        switch (opt) {
            case 'd': device = optarg; break;
            case 'i': image_path = optarg; break;
            case 'w': width = atoi(optarg); break;
            case 'h': height = atoi(optarg); break;
            case 'f': frames = atol(optarg); break;
            case 'p': probe = 1; break;
            case 'm': view_mask = 1; break;
            case 's': save_frame_path = optarg; break;
            case 't':
                stream_enabled = 1;
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

    BallDetector *detector = image_path
        ? bd_create_from_image(image_path)
        : bd_create_camera(device, width, height);

    if (!detector) {
        fprintf(stderr, "Konnte Detector nicht initialisieren.\n");
        return 1;
    }

    if (view_mask) bd_set_view_mask(1);

    if (stream_enabled) {
        if (bd_stream_start(stream_port) != 0) {
            fprintf(stderr, "Konnte Stream-Server auf Port %d nicht starten.\n", stream_port);
            bd_release(detector);
            return 1;
        }
        fprintf(stderr, "Live-Stream unter http://<pi-ip>:%d/\n", stream_port);
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

        if (probe) {
            ProbeResult pr;
            if (bd_probe(detector, &range, 80, &pr) == 0) print_probe(&pr, &range);
        }

        if (save_frame_path && bd_save_annotated(detector, &result, save_frame_path) != 0) {
            fprintf(stderr, "Konnte Frame nicht nach %s schreiben.\n", save_frame_path);
        }

        if (stream_enabled) {
            bd_stream_push(detector, &result);
        }

        frame_num++;

        if (image_path) break;
        if (frames > 0 && frame_num >= frames) break;
    }

    if (stream_enabled) {
        bd_stream_stop();
    }

    bd_release(detector);
    return 0;
}