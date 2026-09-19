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
static int    g_wrong_x = 0;
static int    g_wrong_y = 0;
static int    g_warned_clip = 0;

typedef struct {
    long   n;
    double sp, se, spe, sp2;
    double pmin, pmax;
} Fit;

static Fit g_fit_x;
static Fit g_fit_y;
static int g_fit_reported = 0;

static void fit_reset(Fit *f) {
    f->n = 0;
    f->sp = f->se = f->spe = f->sp2 = 0.0;
    f->pmin = f->pmax = 0.0;
}

static void fit_add(Fit *f, double p, double e) {
    if (f->n == 0) { f->pmin = p; f->pmax = p; }
    if (p < f->pmin) f->pmin = p;
    if (p > f->pmax) f->pmax = p;
    f->n++;
    f->sp  += p;
    f->se  += e;
    f->spe += p * e;
    f->sp2 += p * p;
}

static int fit_slope(const Fit *f, double *slope) {
    if (f->n < 30 || (f->pmax - f->pmin) < 15.0) return 0;

    double n   = (double)f->n;
    double den = f->sp2 - f->sp * f->sp / n;
    if (fabs(den) < 1e-6) return 0;

    *slope = (f->spe - f->sp * f->se / n) / den;
    return 1;
}
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
    g_wrong_x      = 0;
    g_wrong_y      = 0;
    g_warned_clip  = 0;
    g_fit_reported = 0;
    fit_reset(&g_fit_x);
    fit_reset(&g_fit_y);

    forget_last_command();
    path_abort();

    long motor_steps[ACE_MOTOR_COUNT];
    read_motors(motor_steps);
    kin_reset(ACE_START_X_MM, ACE_START_Y_MM, motor_steps);

    clock_gettime(CLOCK_MONOTONIC, &g_last_seen);
    return 0;
}

static void report_wrong_way(const char *axis, int strikes,
                             double was_px, double now_px) {
    double nx = g_wrong_x ? -g_img_x : g_img_x;
    double ny = g_wrong_y ? -g_img_y : g_img_y;

    fprintf(stderr,
            "\nDie Abweichung in %s waechst seit %d Korrekturen (%.0f -> %.0f px).\n"
            "Die Plattform faehrt vom Objekt weg.\n",
            axis, strikes, was_px, now_px);

    if (g_wrong_x && g_wrong_y) {
        fprintf(stderr, "Beide Achsen laufen falsch herum.\n");
    }
    fprintf(stderr,
            "Sofort probieren: --image-to-field %+.0f,%+.0f\n"
            "Passt es, gehoert es dauerhaft nach geometry.h:\n"
            "  #define ACE_IMAGE_TO_FIELD_X  %+.1f\n"
            "  #define ACE_IMAGE_TO_FIELD_Y  %+.1f\n\n",
            nx, ny, nx, ny);
}

static void check_direction(double error_x, double error_y, int clipped) {
    if (!g_have_last_cmd || clipped) return;

    g_worse_x = (fabs(error_x) > fabs(g_cmd_err_x) + 3.0) ? g_worse_x + 1 : 0;
    g_worse_y = (fabs(error_y) > fabs(g_cmd_err_y) + 3.0) ? g_worse_y + 1 : 0;

    if (g_worse_x >= ACE_WRONG_WAY_STRIKES && !g_wrong_x) {
        g_wrong_x = 1;
        report_wrong_way("x", g_worse_x, fabs(g_cmd_err_x), fabs(error_x));
    }
    if (g_worse_y >= ACE_WRONG_WAY_STRIKES && !g_wrong_y) {
        g_wrong_y = 1;
        report_wrong_way("y", g_worse_y, fabs(g_cmd_err_y), fabs(error_y));
    }
}

static void follow_object(const DetectionResult *result,
                          const long motor_steps[ACE_MOTOR_COUNT]) {
    double error_x = result->x - g_frame_width  / 2.0;
    double error_y = result->y - g_frame_height / 2.0;

    double limit = (g_state == NAV_HOVER) ? ACE_HOVER_RELEASE_PX
                                          : ACE_CENTER_TOLERANCE_PX;
    int centered = !result->clipped &&
                   fabs(error_x) <= limit && fabs(error_y) <= limit;

    if (result->clipped && !g_warned_clip) {
        fprintf(stderr,
                "\nDas Objekt beruehrt den Bildrand. Der gemessene Mittelpunkt\n"
                "klebt dann an der Kante und die Abweichung laesst sich nicht mehr\n"
                "ausregeln. Objekt weiter in die Bildmitte legen oder %s\n"
                "vergroessern.\n\n",
                "ACE_CAMERA_HEIGHT_MM");
        g_warned_clip = 1;
    }

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

    if (!result->clipped) {
        double mx, my;
        kin_position(motor_steps, &mx, &my);
        fit_add(&g_fit_x, mx, result->x);
        fit_add(&g_fit_y, my, result->y);
        if (!g_fit_reported) {
            double a, b;
            if (fit_slope(&g_fit_x, &a) && fit_slope(&g_fit_y, &b)) {
                nav_print_scale();
                g_fit_reported = 1;
            }
        }
    }

    if (g_have_last_cmd && ms_since(&g_last_cmd) < ACE_REAIM_MS) return;

    check_direction(error_x, error_y, result->clipped);

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

void nav_print_scale(void) {
    double sx, sy;
    int ok_x = fit_slope(&g_fit_x, &sx);
    int ok_y = fit_slope(&g_fit_y, &sy);

    printf("\nGemessener Zusammenhang Plattformweg -> Bildposition\n");
    if (!ok_x && !ok_y) {
        printf("  noch zu wenig Daten (x %ld Frames ueber %.0f mm, "
               "y %ld ueber %.0f mm; noetig 30 Frames ueber 15 mm,\n"
               "  und das Objekt darf den Bildrand nicht beruehren)\n\n",
               g_fit_x.n, g_fit_x.pmax - g_fit_x.pmin,
               g_fit_y.n, g_fit_y.pmax - g_fit_y.pmin);
        return;
    }

    if (ok_x) {
        printf("  x: %+6.2f px/mm aus %ld Frames ueber %.0f mm"
               "  ->  ACE_IMAGE_TO_FIELD_X %+.1f, Sichtbreite %.0f mm\n",
               sx, g_fit_x.n, g_fit_x.pmax - g_fit_x.pmin,
               (sx > 0.0) ? -1.0 : 1.0, (double)g_frame_width / fabs(sx));
    }
    if (ok_y) {
        printf("  y: %+6.2f px/mm aus %ld Frames ueber %.0f mm"
               "  ->  ACE_IMAGE_TO_FIELD_Y %+.1f\n",
               sy, g_fit_y.n, g_fit_y.pmax - g_fit_y.pmin,
               (sy > 0.0) ? -1.0 : 1.0);
    }
    printf("  aktiv sind x %+.1f, y %+.1f\n\n", g_img_x, g_img_y);
}

void nav_set_image_to_field(double x, double y) {
    g_img_x = x;
    g_img_y = y;
}

void nav_image_to_field(double *x, double *y) {
    if (x) *x = g_img_x;
    if (y) *y = g_img_y;
}
