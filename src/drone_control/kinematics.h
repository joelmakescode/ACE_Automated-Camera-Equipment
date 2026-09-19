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

/* Die koppelnavigierten Seillaengen zu einem Schrittzaehlerstand. */
void kin_lengths(const long motor_steps[ACE_MOTOR_COUNT],
                 double lengths[ACE_MOTOR_COUNT]);

/* Vorwaertsloesung direkt aus Seillaengen statt aus Schrittzaehlern. */
void kin_position_from_lengths(const double lengths[ACE_MOTOR_COUNT],
                               double *x_mm, double *y_mm);

/* Hoehe unter den Ankern, die Seil <motor> bei dieser Laenge und dieser
 * XY-Lage erzwingt. Nur wenn alle vier Seile dieselbe Hoehe fordern, ist
 * der Satz Laengen mit einem einzigen Punkt im Raum vertraeglich. */
double kin_implied_height(int motor, double x_mm, double y_mm, double length_mm);

/* Spanne der vier geforderten Hoehen: 0 heisst raeumlich widerspruchsfrei. */
double kin_height_spread(const double lengths[ACE_MOTOR_COUNT],
                         double x_mm, double y_mm);

#ifdef __cplusplus
}
#endif
#endif
