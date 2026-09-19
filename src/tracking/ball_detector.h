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
        bool clipped;
        double x;
        double y;
        double radius;
    } DetectionResult;

    typedef struct {
        int  h_med, s_med, v_med;
        int  h_lo, h_hi;
        int  s_lo, s_hi;
        int  v_lo, v_hi;
        int  b_mean, g_mean, r_mean;
        bool hue_wraps;
    } PatchStats;

    typedef struct {
        PatchStats centre;
        PatchStats blob;
        bool   blob_found;
        double blob_x, blob_y, blob_radius, best_area;
        long   mask_pixels;
        long   frame_pixels;
    } ProbeResult;

    typedef struct BallDetector BallDetector;
    BallDetector *bd_create_camera(const char *device_path, int width, int height);
    BallDetector *bd_create_from_image(const char *image_path);

    int bd_detect(BallDetector *detector, const HsvRange *range, DetectionResult *result);

    int bd_probe(BallDetector *detector, const HsvRange *range, int window_px,
                 ProbeResult *result);

    void bd_set_view_mask(int enabled);

    int bd_save_annotated(BallDetector *detector, const DetectionResult *result, const char *out_path);

    int bd_stream_start(int port);

    int bd_stream_push(BallDetector *detector, const DetectionResult *result);

    void bd_stream_stop(void);

    void bd_release(BallDetector *detector);

#ifdef __cplusplus
}
#endif
#endif