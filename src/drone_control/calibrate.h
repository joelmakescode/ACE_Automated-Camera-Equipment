#ifndef ACE_DRONE_CONTROL_CALIBRATE_H
#define ACE_DRONE_CONTROL_CALIBRATE_H

#include "ball_detector.h"

#ifdef __cplusplus
extern "C" {
#endif

int cal_center(BallDetector *detector, const HsvRange *range,
               double start_x_mm, double start_y_mm,
               unsigned int step_delay_us);

int cal_goto(BallDetector *detector, const HsvRange *range,
             double start_x_mm, double start_y_mm,
             double target_x_mm, double target_y_mm,
             unsigned int step_delay_us);

int cal_run(BallDetector *detector, const HsvRange *range,
            int frame_width, double start_x_mm, double start_y_mm,
            double distance_mm, double object_diameter_mm,
            unsigned int step_delay_us);

#ifdef __cplusplus
}
#endif
#endif
