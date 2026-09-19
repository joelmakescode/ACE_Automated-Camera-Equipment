#include "kinematics.h"

#include <math.h>

static const double ANCHORS[ACE_MOTOR_COUNT][2] = {
    { -ACE_ANCHOR_SPAN_X_MM / 2.0,  ACE_ANCHOR_SPAN_Y_MM / 2.0 },
    {  ACE_ANCHOR_SPAN_X_MM / 2.0,  ACE_ANCHOR_SPAN_Y_MM / 2.0 },
    {  ACE_ANCHOR_SPAN_X_MM / 2.0, -ACE_ANCHOR_SPAN_Y_MM / 2.0 },
    { -ACE_ANCHOR_SPAN_X_MM / 2.0, -ACE_ANCHOR_SPAN_Y_MM / 2.0 },
};

static double g_reference_length[ACE_MOTOR_COUNT];
static long   g_reference_steps[ACE_MOTOR_COUNT];

double kin_cable_length(int motor, double x_mm, double y_mm) {
    if (motor < 0 || motor >= ACE_MOTOR_COUNT) return 0.0;

    double dx = x_mm - ANCHORS[motor][0];
    double dy = y_mm - ANCHORS[motor][1];
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
    double l0 = current_length(0, motor_steps);
    double l1 = current_length(1, motor_steps);
    double l3 = current_length(3, motor_steps);

    if (x_mm) *x_mm = (l0 * l0 - l1 * l1) / (2.0 * ACE_ANCHOR_SPAN_X_MM);
    if (y_mm) *y_mm = (l3 * l3 - l0 * l0) / (2.0 * ACE_ANCHOR_SPAN_Y_MM);
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

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        double now    = current_length(i, motor_steps);
        double wanted = kin_cable_length(i, target_x_mm, target_y_mm);
        steps[i] = lround((now - wanted) / ACE_MM_PER_HALFSTEP);
    }
}
