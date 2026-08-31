#ifndef ACE_TRACKING_BALL_DETECTOR_H
#define ACE_TRACKING_BALL_DETECTOR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int h_min, s_min, v_min;
    int h_max, s_max, v_max;
} HsvRange;

    HsvRange bd_default_hsv_range(void);

    typedef struct {
        bool found;
        double x;
        double y;
        double radius;
    } DetectionResult;

    typedef struct BallDetector BallDetector;
    BallDetector *bd_create_camera(const char *device_path, int width, int height);
    BallDetector *bd_create_from_image(const char *image_path);

    int bd_detect(BallDetector *detector, const HsvRange *range, DetectionResult *result);

    int bd_save_annotated(BallDetector *detector, const DetectionResult *result, const char *out_path);

    /* Startet einen einfachen MJPEG-HTTP-Server (multipart/x-mixed-replace)
     * auf dem angegebenen Port, fuer Live-Ansicht im Browser/ffplay unter
     * http://<host>:<port>/. 0 bei Erfolg, -1 bei Fehler (z.B. Port belegt). */
    int bd_stream_start(int port);

    /* Kodiert den zuletzt erfassten Frame (mit Overlay falls result->found)
     * als JPEG und macht ihn fuer alle verbundenen Stream-Clients verfuegbar. */
    int bd_stream_push(BallDetector *detector, const DetectionResult *result);

    /* Stoppt den Stream-Server und schliesst alle Verbindungen. */
    void bd_stream_stop(void);

    void bd_release(BallDetector *detector);

#ifdef __cplusplus
}
#endif
#endif