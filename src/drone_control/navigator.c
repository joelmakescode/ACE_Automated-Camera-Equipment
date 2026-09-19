#define _POSIX_C_SOURCE 200809L

#include "navigator.h"

#include "geometry.h"
#include "kinematics.h"
#include "motion.h"

#include <math.h>
#include <stddef.h>
#include <time.h>

#define PATROL_HALF_X (ACE_PATROL_SPAN_X_MM / 2.0)
#define PATROL_HALF_Y (ACE_PATROL_SPAN_Y_MM / 2.0)

static const double PATROL_PATH[][2] = {
    { -PATROL_HALF_X, -PATROL_HALF_Y },
    {  PATROL_HALF_X,  PATROL_HALF_Y },
    {  0.0,            0.0           },
    {  PATROL_HALF_X, -PATROL_HALF_Y },
    { -PATROL_HALF_X,  PATROL_HALF_Y },
    {  0.0,            0.0           },
};

#define PATROL_POINTS ((int)(sizeof(PATROL_PATH) / sizeof(PATROL_PATH[0])))

static NavState     g_state = NAV_PATROL;
static int          g_frame_width   = 1280;
static int          g_frame_height  = 720;
static unsigned int g_step_delay_us = ACE_TRAVEL_DELAY_US;

static int g_patrol_index = 0;

static struct timespec g_last_seen;
static int             g_ever_seen = 0;

static void read_motors(long steps[ACE_MOTOR_COUNT]) {
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) steps[i] = motion_position(i);
}

static long ms_since(const struct timespec *then) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - then->tv_sec) * 1000L
         + (now.tv_nsec - then->tv_nsec) / 1000000L;
}

static double clamp_abs(double value, double limit) {
    if (value >  limit) return  limit;
    if (value < -limit) return -limit;
    return value;
}

static int start_move(const long motor_steps[ACE_MOTOR_COUNT],
                      double x_mm, double y_mm) {
    long steps[ACE_MOTOR_COUNT];

    kin_plan(motor_steps, x_mm, y_mm, steps);
    return motion_start(steps, g_step_delay_us);
}

int nav_init(int frame_width, int frame_height, unsigned int step_delay_us) {
    if (frame_width <= 0 || frame_height <= 0) return -1;

    g_frame_width   = frame_width;
    g_frame_height  = frame_height;
    g_step_delay_us = step_delay_us;

    g_state        = NAV_PATROL;
    g_patrol_index = 0;
    g_ever_seen    = 0;

    long motor_steps[ACE_MOTOR_COUNT];
    read_motors(motor_steps);
    kin_reset(ACE_START_X_MM, ACE_START_Y_MM, motor_steps);

    clock_gettime(CLOCK_MONOTONIC, &g_last_seen);
    return 0;
}

static void follow_object(const DetectionResult *result,
                          const long motor_steps[ACE_MOTOR_COUNT]) {
    double error_x = result->x - g_frame_width  / 2.0;
    double error_y = result->y - g_frame_height / 2.0;

    int centered = fabs(error_x) <= ACE_CENTER_TOLERANCE_PX &&
                   fabs(error_y) <= ACE_CENTER_TOLERANCE_PX;

    if (g_state == NAV_PATROL) motion_abort();

    if (centered) {
        motion_abort();
        g_state = NAV_HOVER;
        return;
    }

    g_state = NAV_APPROACH;
    if (motion_busy()) return;

    double mm_per_pixel = ACE_VIEW_WIDTH_MM / (double)g_frame_width;
    double shift_x = clamp_abs(error_x * mm_per_pixel * ACE_CORRECTION_GAIN
                               * ACE_IMAGE_TO_FIELD_X, ACE_MAX_CORRECTION_MM);
    double shift_y = clamp_abs(error_y * mm_per_pixel * ACE_CORRECTION_GAIN
                               * ACE_IMAGE_TO_FIELD_Y, ACE_MAX_CORRECTION_MM);

    double x_mm, y_mm;
    kin_position(motor_steps, &x_mm, &y_mm);
    start_move(motor_steps, x_mm + shift_x, y_mm + shift_y);
}

static void continue_patrol(const long motor_steps[ACE_MOTOR_COUNT]) {
    if (motion_busy()) return;

    start_move(motor_steps, PATROL_PATH[g_patrol_index][0],
               PATROL_PATH[g_patrol_index][1]);
    g_patrol_index = (g_patrol_index + 1) % PATROL_POINTS;
}

void nav_update(const DetectionResult *result) {
    long motor_steps[ACE_MOTOR_COUNT];
    read_motors(motor_steps);

    if (result && result->found) {
        clock_gettime(CLOCK_MONOTONIC, &g_last_seen);
        g_ever_seen = 1;
        follow_object(result, motor_steps);
        return;
    }

    if (g_state != NAV_PATROL && g_ever_seen) {
        if (ms_since(&g_last_seen) < ACE_LOST_GRACE_MS) return;
        g_state        = NAV_PATROL;
        g_patrol_index = 0;
    }

    continue_patrol(motor_steps);
}

NavState nav_state(void) {
    return g_state;
}

const char *nav_state_name(NavState state) {
    switch (state) {
        case NAV_PATROL:   return "SUCHFAHRT";
        case NAV_APPROACH: return "ANFAHRT";
        case NAV_HOVER:    return "SCHWEBEN";
        default:           return "?";
    }
}

void nav_target(double *x_mm, double *y_mm) {
    long motor_steps[ACE_MOTOR_COUNT];
    read_motors(motor_steps);
    kin_position(motor_steps, x_mm, y_mm);
}
