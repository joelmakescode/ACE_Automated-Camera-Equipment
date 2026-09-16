#ifndef ACE_DRONE_CONTROL_KINEMATICS_H
#define ACE_DRONE_CONTROL_KINEMATICS_H

#include "geometry.h"
#include "pins.h"

#ifdef __cplusplus
extern "C" {
#endif

void kin_reset(double x_mm, double y_mm, const long motor_steps[ACE_MOTOR_COUNT]);

void kin_position(const long motor_steps[ACE_MOTOR_COUNT],
                  double *x_mm, double *y_mm);

void kin_plan(const long motor_steps[ACE_MOTOR_COUNT],
              double target_x_mm, double target_y_mm,
              long steps[ACE_MOTOR_COUNT]);

void kin_clamp(double *x_mm, double *y_mm);

double kin_cable_length(int motor, double x_mm, double y_mm);

#ifdef __cplusplus
}
#endif
#endif
