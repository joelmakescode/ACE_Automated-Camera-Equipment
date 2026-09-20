#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

#include "ball_detector.h"
#include "geometry.h"   /* nur fuer ACE_CAMERA_HEIGHT_MM */

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
        "  --lens N Linsenposition in Dioptrien, also 1/Abstand[m].\n"
        "           Standard %.2f, passend zu %.0f mm aus geometry.h.\n"
        "           Fuer eine Fokusreihe mehrere Werte durchprobieren.\n"
        "  --focus  manual, continuous, auto oder default. Standard manual:\n"
        "           der Abstand steht fest, ein pumpender Autofokus bringt\n"
        "           nur Unschaerfe und einen wandernden Massstab.\n"
        "  --jpeg Q Guete des Streams 1..100, Standard 90. Zum Beurteilen\n"
        "           der Schaerfe hoch setzen, nicht herunter.\n"
        "\n"
        "Farbe: Standard ist Rot auf hellem Grund, h %d-%d s %d-%d v %d-%d.\n"
        "       Ist h-min groesser als h-max, wird ueber den Nullpunkt hinweg\n"
        "       gesucht, also h-min..179 und 0..h-max. Genau so wird Rot\n"
        "       erfasst, das an beiden Enden der Hue-Skala liegt.\n"
        "       Mit --stream laesst sich der Bereich am Livebild pruefen.\n",
        prog,
        1000.0 / ACE_CAMERA_HEIGHT_MM, (double)ACE_CAMERA_HEIGHT_MM,
        def.h_min, def.h_max, def.s_min, def.s_max, def.v_min, def.v_max);
}

static const char *channel_hint(const PatchStats *p) {
    if (p->r_mean > p->b_mean + 30) return "rotlastig";
    if (p->b_mean > p->r_mean + 30) return "BLAULASTIG - R und B vertauscht?";
    return "weder rot noch blau dominant";
}

static void print_patch(const char *label, const PatchStats *p) {
    printf("  %-11s H %3d (%3d..%3d)  S %3d (%3d..%3d)  V %3d (%3d..%3d)"
           "   B%3d G%3d R%3d  %s\n",
           label,
           p->h_med, p->h_lo, p->h_hi,
           p->s_med, p->s_lo, p->s_hi,
           p->v_med, p->v_lo, p->v_hi,
           p->b_mean, p->g_mean, p->r_mean, channel_hint(p));
}

static int looks_red(const PatchStats *p) {
    return p->hue_wraps || p->h_med >= 160 || p->h_med <= 12;
}

static void suggest(const PatchStats *p) {
    int s_min = p->s_lo > 40 ? p->s_lo - 20 : 20;
    int v_min = p->v_lo > 40 ? p->v_lo - 20 : 20;

    if (looks_red(p)) {
        printf("  -> Rot (Hue liegt am Rand der Skala). Vorschlag:\n"
               "     --h-min 170 --h-max 10 --s-min %d --v-min %d\n", s_min, v_min);
    } else {
        printf("  -> Vorschlag: --h-min %d --h-max %d --s-min %d --v-min %d\n",
               p->h_lo > 5 ? p->h_lo - 5 : 0,
               p->h_hi < 174 ? p->h_hi + 5 : 179,
               s_min, v_min);
    }
}

static void print_probe(const ProbeResult *pr, const HsvRange *range) {
    printf("  Maske       %ld von %ld Pixeln (%.2f %%), groesste Flaeche %.0f px"
           "   [aktuell h %d-%d s %d-%d v %d-%d]\n",
           pr->mask_pixels, pr->frame_pixels,
           100.0 * (double)pr->mask_pixels / (double)pr->frame_pixels,
           pr->best_area,
           range->h_min, range->h_max, range->s_min, range->s_max,
           range->v_min, range->v_max);

    if (pr->blob_found) {
        printf("  Fund bei    x=%.0f y=%.0f r=%.0f\n",
               pr->blob_x, pr->blob_y, pr->blob_radius);
        print_patch("im Fund", &pr->blob);
    }
    print_patch("Bildmitte", &pr->centre);

    if (!pr->blob_found) {
        printf("  -> Nichts gefunden. Die Werte unten stammen aus der Bildmitte,\n"
               "     also nur brauchbar, wenn das Objekt dort liegt.\n");
        if (pr->centre.s_med < 60) {
            printf("  -> Saettigung %d ist sehr niedrig: das Objekt kommt grau an.\n"
                   "     Zu wenig Licht, Weissabgleich, oder die Farbe ist blass.\n",
                   pr->centre.s_med);
        }
        suggest(&pr->centre);
        return;
    }

    if (pr->blob.s_med < 60) {
        printf("  -> Der Fund ist kaum gesaettigt (S %d). Das ist vermutlich\n"
               "     Hintergrund, kein farbiges Objekt.\n", pr->blob.s_med);
    } else if (looks_red(&pr->blob)) {
        printf("  -> Der Fund ist rot und klar gesaettigt. Passt.\n");
    }
    suggest(&pr->blob);
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
    const char *focus_mode = "manual";
    double lens_position = 1000.0 / ACE_CAMERA_HEIGHT_MM;
    int jpeg_quality = 90;
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
        {"focus",      required_argument, 0, 'F'},
        {"lens",       required_argument, 0, 'L'},
        {"jpeg",       required_argument, 0, 'J'},
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
    while ((opt = getopt_long(argc, argv, "d:i:w:h:f:s:t::pmF:L:J:", long_opts, &opt_index)) != -1) {
        switch (opt) {
            case 'd': device = optarg; break;
            case 'i': image_path = optarg; break;
            case 'w': width = atoi(optarg); break;
            case 'h': height = atoi(optarg); break;
            case 'f': frames = atol(optarg); break;
            case 'F': focus_mode    = optarg; break;
            case 'L': lens_position = atof(optarg); break;
            case 'J': jpeg_quality  = atoi(optarg); break;
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

    /* Vor bd_create_camera: der Fokus geht in die Kommandozeile von
     * rpicam-vid ein. Fester Abstand, also fester Fokus. */
    bd_set_focus(focus_mode, lens_position);

    BallDetector *detector = image_path
        ? bd_create_from_image(image_path)
        : bd_create_camera(device, width, height);

    if (!detector) {
        fprintf(stderr, "Konnte Detector nicht initialisieren.\n");
        return 1;
    }

    if (view_mask) bd_set_view_mask(1);
    bd_set_stream_quality(jpeg_quality);

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

        printf("frame=%ld found=%d x=%.1f y=%.1f radius=%.1f%s\n",
               frame_num, result.found ? 1 : 0, result.x, result.y, result.radius,
               result.clipped ? "  AM BILDRAND ANGESCHNITTEN" : "");

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