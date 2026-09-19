#include "kinematics.h"

#include <math.h>

static double anchor_x(int motor) {
    return ACE_MOTOR_CORNER[motor][0] * ACE_ANCHOR_SPAN_X_MM / 2.0;
}

static double anchor_y(int motor) {
    return ACE_MOTOR_CORNER[motor][1] * ACE_ANCHOR_SPAN_Y_MM / 2.0;
}

static void find_opposite_pairs(int *x_neg, int *x_pos, int *y_neg, int *y_pos) {
    *x_neg = *x_pos = *y_neg = *y_pos = -1;

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        for (int j = 0; j < ACE_MOTOR_COUNT; j++) {
            if (*x_neg < 0 &&
                ACE_MOTOR_CORNER[i][1] == ACE_MOTOR_CORNER[j][1] &&
                ACE_MOTOR_CORNER[i][0] < 0 && ACE_MOTOR_CORNER[j][0] > 0) {
                *x_neg = i;
                *x_pos = j;
            }
            if (*y_neg < 0 &&
                ACE_MOTOR_CORNER[i][0] == ACE_MOTOR_CORNER[j][0] &&
                ACE_MOTOR_CORNER[i][1] < 0 && ACE_MOTOR_CORNER[j][1] > 0) {
                *y_neg = i;
                *y_pos = j;
            }
        }
    }
}

static double g_reference_length[ACE_MOTOR_COUNT];
static long   g_reference_steps[ACE_MOTOR_COUNT];

double kin_cable_length(int motor, double x_mm, double y_mm) {
    if (motor < 0 || motor >= ACE_MOTOR_COUNT) return 0.0;

    double dx = x_mm - anchor_x(motor);
    double dy = y_mm - anchor_y(motor);
    return sqrt(dx * dx + dy * dy + ACE_HOVER_HEIGHT_MM * ACE_HOVER_HEIGHT_MM);
}

void kin_reset(double x_mm, double y_mm, const long motor_steps[ACE_MOTOR_COUNT]) {
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        g_reference_length[i] = kin_cable_length(i, x_mm, y_mm);
        g_reference_steps[i]  = motor_steps ? motor_steps[i] : 0;
    }
}

static double current_length(int motor, const long motor_steps[ACE_MOTOR_COUNT]) {
    long wound = motor_steps[motor] - g_reference_steps[motor];
    return g_reference_length[motor] - (double)wound * ACE_MM_PER_HALFSTEP;
}

void kin_position(const long motor_steps[ACE_MOTOR_COUNT],
                  double *x_mm, double *y_mm) {
    int x_neg, x_pos, y_neg, y_pos;
    find_opposite_pairs(&x_neg, &x_pos, &y_neg, &y_pos);

    if (x_mm) {
        double a = current_length(x_neg, motor_steps);
        double b = current_length(x_pos, motor_steps);
        *x_mm = (a * a - b * b) / (2.0 * ACE_ANCHOR_SPAN_X_MM);
    }
    if (y_mm) {
        double a = current_length(y_neg, motor_steps);
        double b = current_length(y_pos, motor_steps);
        *y_mm = (a * a - b * b) / (2.0 * ACE_ANCHOR_SPAN_Y_MM);
    }
}

void kin_clamp(double *x_mm, double *y_mm) {
    if (x_mm) {
        if (*x_mm >  ACE_REACH_LIMIT_X_MM) *x_mm =  ACE_REACH_LIMIT_X_MM;
        if (*x_mm < -ACE_REACH_LIMIT_X_MM) *x_mm = -ACE_REACH_LIMIT_X_MM;
    }
    if (y_mm) {
        if (*y_mm >  ACE_REACH_LIMIT_Y_MM) *y_mm =  ACE_REACH_LIMIT_Y_MM;
        if (*y_mm < -ACE_REACH_LIMIT_Y_MM) *y_mm = -ACE_REACH_LIMIT_Y_MM;
    }
}

void kin_plan(const long motor_steps[ACE_MOTOR_COUNT],
              double target_x_mm, double target_y_mm,
              long steps[ACE_MOTOR_COUNT]) {
    kin_clamp(&target_x_mm, &target_y_mm);

    double from_centre = sqrt(target_x_mm * target_x_mm + target_y_mm * target_y_mm);
    double slack = ACE_SLACK_PER_100MM * from_centre / 100.0;

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        double now    = current_length(i, motor_steps);
        double wanted = kin_cable_length(i, target_x_mm, target_y_mm) + slack;
        steps[i] = lround((now - wanted) / ACE_MM_PER_HALFSTEP);
    }
}
