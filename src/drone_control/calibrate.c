#define _POSIX_C_SOURCE 200809L

#include "calibrate.h"

#include "geometry.h"
#include "kinematics.h"
#include "motion.h"
#include "pins.h"

#include <math.h>
#include <stdio.h>
#include <time.h>

static void read_motors(long steps[ACE_MOTOR_COUNT]) {
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) steps[i] = motion_position(i);
}

static int drain_until_idle(BallDetector *detector, const HsvRange *range) {
    while (motion_busy()) {
        DetectionResult ignored;
        if (bd_detect(detector, range, &ignored) != 0) return -1;
    }
    return 0;
}

static int move_to(BallDetector *detector, const HsvRange *range,
                   double x_mm, double y_mm, unsigned int step_delay_us) {
    long motors[ACE_MOTOR_COUNT];
    long steps[ACE_MOTOR_COUNT];

    read_motors(motors);
    kin_plan(motors, x_mm, y_mm, steps);

    if (motion_start(steps, step_delay_us) != 0) return -1;
    return drain_until_idle(detector, range);
}

static int sample_object(BallDetector *detector, const HsvRange *range,
                         double *px, double *py) {
    int    hits = 0;
    double sum_x = 0.0;
    double sum_y = 0.0;

    for (int frame = 0; frame < ACE_CALIBRATION_MAX_FRAMES; frame++) {
        DetectionResult result;
        if (bd_detect(detector, range, &result) != 0) return -1;

        if (result.found) {
            sum_x += result.x;
            sum_y += result.y;
            hits++;
            if (hits >= ACE_CALIBRATION_SAMPLES) break;
        }
    }

    if (hits < ACE_CALIBRATION_SAMPLES) return -1;

    *px = sum_x / hits;
    *py = sum_y / hits;
    return 0;
}

static int apply_tension(BallDetector *detector, const HsvRange *range,
                         unsigned int step_delay_us) {
    long steps[ACE_MOTOR_COUNT];
    long wind = lround(ACE_TENSION_MM / ACE_MM_PER_HALFSTEP);

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) steps[i] = wind;

    if (motion_start(steps, step_delay_us) != 0) return -1;
    if (drain_until_idle(detector, range) != 0) return -1;

    long motors[ACE_MOTOR_COUNT];
    read_motors(motors);
    kin_reset(0.0, 0.0, motors);

    printf("Leichte Spannung: alle vier Seile %.1f mm aufgewickelt (%ld Halbschritte),\n"
           "Spulen bleiben bestromt, Mitte neu referenziert.\n",
           ACE_TENSION_MM, wind);
    return 0;
}

int cal_center(BallDetector *detector, const HsvRange *range,
               double start_x_mm, double start_y_mm,
               unsigned int step_delay_us) {
    long motors[ACE_MOTOR_COUNT];
    read_motors(motors);
    kin_reset(start_x_mm, start_y_mm, motors);

    printf("Start bei x=%.0f y=%.0f mm, fahre in die Mitte.\n",
           start_x_mm, start_y_mm);

    if (move_to(detector, range, 0.0, 0.0, step_delay_us) != 0) return -1;
    return apply_tension(detector, range, step_delay_us);
}

int cal_run(BallDetector *detector, const HsvRange *range,
            int frame_width, double start_x_mm, double start_y_mm,
            double distance_mm, unsigned int step_delay_us) {
    long motors[ACE_MOTOR_COUNT];
    read_motors(motors);
    kin_reset(start_x_mm, start_y_mm, motors);

    if (start_x_mm != 0.0 || start_y_mm != 0.0) {
        printf("Start bei x=%.0f y=%.0f mm, fahre zuerst in die Mitte.\n",
               start_x_mm, start_y_mm);
        if (move_to(detector, range, 0.0, 0.0, step_delay_us) != 0) return -1;
    }

    printf("Kalibrierung: Objekt ins Bild legen und liegen lassen.\n");

    double base_x, base_y;
    if (sample_object(detector, range, &base_x, &base_y) != 0) {
        fprintf(stderr,
                "Kein Objekt erkannt. Kalibrierung uebersprungen, HSV-Bereich pruefen "
                "(--h-min/--h-max ...).\n");
        if (move_to(detector, range, 0.0, 0.0, step_delay_us) != 0) return -1;
        apply_tension(detector, range, step_delay_us);
        return -1;
    }

    printf("Objekt bei %.1f, %.1f px\n", base_x, base_y);

    if (move_to(detector, range, distance_mm, 0.0, step_delay_us) != 0) return -1;

    double shifted_x, shifted_y;
    if (sample_object(detector, range, &shifted_x, &shifted_y) != 0) {
        fprintf(stderr, "Objekt nach der x-Fahrt verloren, Abstand verkleinern "
                        "(--calib-mm).\n");
        move_to(detector, range, 0.0, 0.0, step_delay_us);
        apply_tension(detector, range, step_delay_us);
        return -1;
    }

    double shift_x_px = shifted_x - base_x;

    if (move_to(detector, range, 0.0, 0.0, step_delay_us) != 0) return -1;
    if (move_to(detector, range, 0.0, distance_mm, step_delay_us) != 0) return -1;

    if (sample_object(detector, range, &shifted_x, &shifted_y) != 0) {
        fprintf(stderr, "Objekt nach der y-Fahrt verloren, Abstand verkleinern "
                        "(--calib-mm).\n");
        move_to(detector, range, 0.0, 0.0, step_delay_us);
        apply_tension(detector, range, step_delay_us);
        return -1;
    }

    double shift_y_px = shifted_y - base_y;

    if (move_to(detector, range, 0.0, 0.0, step_delay_us) != 0) return -1;

    if (fabs(shift_x_px) < 5.0 || fabs(shift_y_px) < 5.0) {
        fprintf(stderr,
                "Bildverschiebung zu klein (%.1f / %.1f px). Entweder hat sich die "
                "Plattform nicht bewegt (Wickeldurchmesser falsch, Seil rutscht) oder "
                "der Fahrweg ist zu kurz.\n", shift_x_px, shift_y_px);
        apply_tension(detector, range, step_delay_us);
        return -1;
    }

    double mm_per_px_x = distance_mm / fabs(shift_x_px);
    double mm_per_px_y = distance_mm / fabs(shift_y_px);
    double view_width  = mm_per_px_x * (double)frame_width;
    double sign_x = (shift_x_px < 0.0) ? 1.0 : -1.0;
    double sign_y = (shift_y_px < 0.0) ? 1.0 : -1.0;
    double spread = fabs(mm_per_px_x - mm_per_px_y)
                  / ((mm_per_px_x + mm_per_px_y) / 2.0) * 100.0;

    printf("\nx: Plattform +%.0f mm  ->  Objekt %+.1f px  =  %.4f mm/px\n",
           distance_mm, shift_x_px, mm_per_px_x);
    printf("y: Plattform +%.0f mm  ->  Objekt %+.1f px  =  %.4f mm/px\n",
           distance_mm, shift_y_px, mm_per_px_y);
    printf("Abweichung zwischen den Achsen: %.1f %%\n", spread);

    if (spread > 15.0) {
        printf("Ueber 15 %%: Kamera vermutlich verdreht montiert oder das Objekt "
               "hat sich bewegt.\n");
    }

    printf("\nIn geometry.h eintragen:\n");
    printf("  #define ACE_VIEW_WIDTH_MM     %.0f\n", view_width);
    printf("  #define ACE_IMAGE_TO_FIELD_X  %.1f\n", sign_x);
    printf("  #define ACE_IMAGE_TO_FIELD_Y  %.1f\n\n", sign_y);

    return apply_tension(detector, range, step_delay_us);
}
