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
    void bd_release(BallDetector *detector);

#ifdef __cplusplus
}
#endif
#endif