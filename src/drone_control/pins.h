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

/* Lage der Winden am Ankerfeld, von oben auf das Brett gesehen:
 * x zeigt nach rechts, y nach oben.
 *
 *        y
 *        ^
 *   1 ---+--- 2        1 oben links     2 oben rechts
 *        |             0 unten links    3 unten rechts
 *   0 ---+--- 3  > x
 */
static const int ACE_MOTOR_CORNER[ACE_MOTOR_COUNT][2] = {
    { -1, -1 },   /* 0 unten links  */
    { -1,  1 },   /* 1 oben links   */
    {  1,  1 },   /* 2 oben rechts  */
    {  1, -1 },   /* 3 unten rechts */
};

static const char *const ACE_MOTOR_NAMES[ACE_MOTOR_COUNT] = {
    "unten links",
    "oben links",
    "oben rechts",
    "unten rechts",
};

#endif
