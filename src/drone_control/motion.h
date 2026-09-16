#ifndef ACE_DRONE_CONTROL_MOTION_H
#define ACE_DRONE_CONTROL_MOTION_H

#include <stdbool.h>

#include "pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ACE_DEFAULT_STEP_DELAY_US 2000u
#define ACE_MIN_STEP_DELAY_US     1200u

int  motion_init(void);
void motion_shutdown(void);

int  motion_start(const long steps[ACE_MOTOR_COUNT], unsigned int step_delay_us);
int  motion_tick(void);
void motion_wait_next(void);
int  motion_run(const long steps[ACE_MOTOR_COUNT], unsigned int step_delay_us);
void motion_abort(void);

int  motion_thread_start(void);
void motion_thread_stop(void);

bool motion_busy(void);
long motion_position(int motor);

void motion_hold(void);
void motion_release(void);

#ifdef __cplusplus
}
#endif
#endif
