#ifndef ACE_DRONE_CONTROL_PATH_H
#define ACE_DRONE_CONTROL_PATH_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

int  path_start(double target_x_mm, double target_y_mm,
                unsigned int step_delay_us);
void path_abort(void);
bool path_busy(void);

void path_target(double *x_mm, double *y_mm);

#ifdef __cplusplus
}
#endif
#endif
