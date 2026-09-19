#define _POSIX_C_SOURCE 200809L

#include "navigator.h"

#include "geometry.h"
#include "kinematics.h"
#include "motion.h"
#include "path.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
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

static struct timespec g_last_cmd;
static int    g_have_last_cmd = 0;
static double g_cmd_err_x = 0.0;
static double g_cmd_err_y = 0.0;
static int    g_worse_x = 0;
static int    g_worse_y = 0;
static int    g_warned_x = 0;
static int    g_warned_y = 0;
static double g_shift_x = 0.0;
static double g_shift_y = 0.0;
static double g_img_x = ACE_IMAGE_TO_FIELD_X;
static double g_img_y = ACE_IMAGE_TO_FIELD_Y;

static void forget_last_command(void) {
    g_have_last_cmd = 0;
    g_worse_x = g_worse_y = 0;
    g_shift_x = g_shift_y = 0.0;
}

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

static int start_move(double x_mm, double y_mm) {
    return path_start(x_mm, y_mm, g_step_delay_us);
}

int nav_init(int frame_width, int frame_height, unsigned int step_delay_us) {
    if (frame_width <= 0 || frame_height <= 0) return -1;

    g_frame_width   = frame_width;
    g_frame_height  = frame_height;
    g_step_delay_us = step_delay_us;

    g_state        = NAV_PATROL;
    g_patrol_index = 0;
    g_ever_seen    = 0;
    g_warned_x     = 0;
    g_warned_y     = 0;

    forget_last_command();
    path_abort();

    long motor_steps[ACE_MOTOR_COUNT];
    read_motors(motor_steps);
    kin_reset(ACE_START_X_MM, ACE_START_Y_MM, motor_steps);

    clock_gettime(CLOCK_MONOTONIC, &g_last_seen);
    return 0;
}

static void check_direction(double error_x, double error_y) {
    if (!g_have_last_cmd) return;

    g_worse_x = (fabs(error_x) > fabs(g_cmd_err_x) + 3.0) ? g_worse_x + 1 : 0;
    g_worse_y = (fabs(error_y) > fabs(g_cmd_err_y) + 3.0) ? g_worse_y + 1 : 0;

    if (g_worse_x >= ACE_WRONG_WAY_STRIKES && !g_warned_x) {
        fprintf(stderr,
                "\nDie Abweichung in x waechst seit %d Korrekturen (%.0f -> %.0f px).\n"
                "Die Plattform faehrt vom Objekt weg. ACE_IMAGE_TO_FIELD_X in\n"
                "geometry.h steht auf %+.1f und muesste %+.1f sein.\n"
                "Sofort probieren: --image-to-field %+.0f,%+.0f\n\n",
                g_worse_x, fabs(g_cmd_err_x), fabs(error_x),
                g_img_x, -g_img_x, -g_img_x, g_img_y);
        g_warned_x = 1;
    }
    if (g_worse_y >= ACE_WRONG_WAY_STRIKES && !g_warned_y) {
        fprintf(stderr,
                "\nDie Abweichung in y waechst seit %d Korrekturen (%.0f -> %.0f px).\n"
                "Die Plattform faehrt vom Objekt weg. ACE_IMAGE_TO_FIELD_Y in\n"
                "geometry.h steht auf %+.1f und muesste %+.1f sein.\n"
                "Sofort probieren: --image-to-field %+.0f,%+.0f\n\n",
                g_worse_y, fabs(g_cmd_err_y), fabs(error_y),
                g_img_y, -g_img_y, g_img_x, -g_img_y);
        g_warned_y = 1;
    }
}

static void follow_object(const DetectionResult *result,
                          const long motor_steps[ACE_MOTOR_COUNT]) {
    double error_x = result->x - g_frame_width  / 2.0;
    double error_y = result->y - g_frame_height / 2.0;

    double limit = (g_state == NAV_HOVER) ? ACE_HOVER_RELEASE_PX
                                          : ACE_CENTER_TOLERANCE_PX;
    int centered = fabs(error_x) <= limit && fabs(error_y) <= limit;

    if (centered) {
        if (g_state != NAV_HOVER) {
            path_abort();
            g_state = NAV_HOVER;
            forget_last_command();
        }
        return;
    }

    if (g_state == NAV_PATROL) {
        path_abort();
        forget_last_command();
    }
    g_state = NAV_APPROACH;

    if (g_have_last_cmd && ms_since(&g_last_cmd) < ACE_REAIM_MS) return;

    check_direction(error_x, error_y);

    double mm_per_pixel = ACE_VIEW_WIDTH_MM / (double)g_frame_width;
    double shift_x = clamp_abs(error_x * mm_per_pixel * ACE_CORRECTION_GAIN
                               * g_img_x, ACE_MAX_CORRECTION_MM);
    double shift_y = clamp_abs(error_y * mm_per_pixel * ACE_CORRECTION_GAIN
                               * g_img_y, ACE_MAX_CORRECTION_MM);

    double x_mm, y_mm;
    kin_position(motor_steps, &x_mm, &y_mm);
    start_move(x_mm + shift_x, y_mm + shift_y);

    g_shift_x       = shift_x;
    g_shift_y       = shift_y;
    g_cmd_err_x     = error_x;
    g_cmd_err_y     = error_y;
    g_have_last_cmd = 1;
    clock_gettime(CLOCK_MONOTONIC, &g_last_cmd);
}

static void continue_patrol(void) {
    if (path_busy()) return;

    start_move(PATROL_PATH[g_patrol_index][0], PATROL_PATH[g_patrol_index][1]);
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
        forget_last_command();
    }

    continue_patrol();
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

void nav_command(double *shift_x_mm, double *shift_y_mm) {
    if (shift_x_mm) *shift_x_mm = g_shift_x;
    if (shift_y_mm) *shift_y_mm = g_shift_y;
}

void nav_set_image_to_field(double x, double y) {
    g_img_x = x;
    g_img_y = y;
}

void nav_image_to_field(double *x, double *y) {
    if (x) *x = g_img_x;
    if (y) *y = g_img_y;
}
