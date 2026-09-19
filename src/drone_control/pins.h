#ifndef ACE_DRONE_CONTROL_PINS_H
#define ACE_DRONE_CONTROL_PINS_H

#define ACE_MOTOR_COUNT 4
#define ACE_HALFSTEPS_PER_REV 4096

static const unsigned int ACE_MOTOR_PINS[ACE_MOTOR_COUNT][4] = {
    { 17, 18, 27, 22 },
    { 23, 24, 25,  4 },
    {  5,  6, 12, 13 },
    { 16, 19, 20, 21 },
};

static const int ACE_MOTOR_DIRECTION[ACE_MOTOR_COUNT] = { -1, 1, -1, 1 };

static const char *const ACE_MOTOR_NAMES[ACE_MOTOR_COUNT] = {
    "vorne links",
    "vorne rechts",
    "hinten rechts",
    "hinten links",
};

#endif
