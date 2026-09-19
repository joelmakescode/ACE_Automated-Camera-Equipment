#include "path.h"

#include "geometry.h"
#include "kinematics.h"
#include "motion.h"
#include "pins.h"

#include <math.h>
#include <pthread.h>
#include <stddef.h>

#define PATH_MAX_SEGMENTS 512

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

static double       g_from_x = 0.0, g_from_y = 0.0;
static double       g_to_x   = 0.0, g_to_y   = 0.0;
static int          g_segments = 0;
static int          g_started  = 0;
static unsigned int g_step_delay_us = ACE_TRAVEL_DELAY_US;
static int          g_active = 0;

static void read_motors(long steps[ACE_MOTOR_COUNT]) {
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) steps[i] = motion_position(i);
}

static void waypoint(int k, double *x_mm, double *y_mm) {
    double t = (double)k / (double)g_segments;
    *x_mm = g_from_x + (g_to_x - g_from_x) * t;
    *y_mm = g_from_y + (g_to_y - g_from_y) * t;
}

static int start_next_segment(void) {
    long   motors[ACE_MOTOR_COUNT];
    long   steps[ACE_MOTOR_COUNT];
    double x_mm, y_mm;

    g_started++;
    waypoint(g_started, &x_mm, &y_mm);

    read_motors(motors);
    kin_plan(motors, x_mm, y_mm, steps);

    if (g_started >= g_segments) g_active = 0;

    if (motion_start(steps, g_step_delay_us) != 0) {
        g_active = 0;
        return -1;
    }
    return 0;
}

static void pump(void) {
    pthread_mutex_lock(&g_lock);
    if (g_active && !motion_busy()) start_next_segment();
    pthread_mutex_unlock(&g_lock);
}

int path_start(double target_x_mm, double target_y_mm,
               unsigned int step_delay_us) {
    long motors[ACE_MOTOR_COUNT];
    int  rc;

    pthread_mutex_lock(&g_lock);

    motion_set_idle_hook(pump);

    g_active = 0;
    motion_abort();

    read_motors(motors);
    kin_position(motors, &g_from_x, &g_from_y);

    g_to_x = target_x_mm;
    g_to_y = target_y_mm;
    kin_clamp(&g_to_x, &g_to_y);

    double dx   = g_to_x - g_from_x;
    double dy   = g_to_y - g_from_y;
    double dist = sqrt(dx * dx + dy * dy);

    g_segments = (int)ceil(dist / ACE_SEGMENT_MM);
    if (g_segments < 1)                 g_segments = 1;
    if (g_segments > PATH_MAX_SEGMENTS) g_segments = PATH_MAX_SEGMENTS;

    g_started       = 0;
    g_step_delay_us = step_delay_us;
    g_active        = 1;

    rc = start_next_segment();

    pthread_mutex_unlock(&g_lock);
    return rc;
}

void path_abort(void) {
    pthread_mutex_lock(&g_lock);
    g_active = 0;
    motion_abort();
    pthread_mutex_unlock(&g_lock);
}

bool path_busy(void) {
    pthread_mutex_lock(&g_lock);
    int pending = g_active;
    pthread_mutex_unlock(&g_lock);

    return pending || motion_busy();
}

void path_target(double *x_mm, double *y_mm) {
    pthread_mutex_lock(&g_lock);
    if (x_mm) *x_mm = g_to_x;
    if (y_mm) *y_mm = g_to_y;
    pthread_mutex_unlock(&g_lock);
}
