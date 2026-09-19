#include "kinematics.h"

#include <math.h>

static double anchor_x(int motor) {
    return ACE_MOTOR_CORNER[motor][0] * ACE_ANCHOR_SPAN_X_MM / 2.0;
}

static double anchor_y(int motor) {
    return ACE_MOTOR_CORNER[motor][1] * ACE_ANCHOR_SPAN_Y_MM / 2.0;
}

static int find_axis_pairs(int axis, int pairs[][2]) {
    int other = 1 - axis;
    int count = 0;

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        for (int j = 0; j < ACE_MOTOR_COUNT; j++) {
            if (ACE_MOTOR_CORNER[i][other] == ACE_MOTOR_CORNER[j][other] &&
                ACE_MOTOR_CORNER[i][axis] < 0 && ACE_MOTOR_CORNER[j][axis] > 0) {
                pairs[count][0] = i;
                pairs[count][1] = j;
                if (++count >= ACE_MOTOR_COUNT) return count;
            }
        }
    }
    return count;
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

static double axis_position(int axis, double span,
                            const long motor_steps[ACE_MOTOR_COUNT]) {
    int pairs[ACE_MOTOR_COUNT][2];
    int count = find_axis_pairs(axis, pairs);
    if (count == 0) return 0.0;

    double sum = 0.0;
    for (int p = 0; p < count; p++) {
        double a = current_length(pairs[p][0], motor_steps);
        double b = current_length(pairs[p][1], motor_steps);
        sum += (a * a - b * b) / (2.0 * span);
    }
    return sum / (double)count;
}

void kin_position(const long motor_steps[ACE_MOTOR_COUNT],
                  double *x_mm, double *y_mm) {
    if (x_mm) *x_mm = axis_position(0, ACE_ANCHOR_SPAN_X_MM, motor_steps);
    if (y_mm) *y_mm = axis_position(1, ACE_ANCHOR_SPAN_Y_MM, motor_steps);
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
