#ifndef ACE_DRONE_CONTROL_NAVIGATOR_H
#define ACE_DRONE_CONTROL_NAVIGATOR_H

#include "ball_detector.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NAV_PATROL,
    NAV_APPROACH,
    NAV_HOVER
} NavState;

int  nav_init(int frame_width, int frame_height, unsigned int step_delay_us);
void nav_update(const DetectionResult *result);

NavState    nav_state(void);
const char *nav_state_name(NavState state);
void        nav_target(double *x_mm, double *y_mm);
void        nav_command(double *shift_x_mm, double *shift_y_mm);

#ifdef __cplusplus
}
#endif
#endif
