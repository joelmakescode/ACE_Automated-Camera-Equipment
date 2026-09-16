#ifndef ACE_DRONE_CONTROL_STEPPER_H
#define ACE_DRONE_CONTROL_STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Stepper Stepper;

void stepper_set_dry_run(int enabled);

Stepper *stepper_create(int id, const unsigned int pins[4]);

int stepper_advance(Stepper *s, int dir);
int stepper_hold(Stepper *s);
int stepper_release(Stepper *s);

long stepper_position(const Stepper *s);

void stepper_destroy(Stepper *s);

#ifdef __cplusplus
}
#endif
#endif
