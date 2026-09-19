#define _POSIX_C_SOURCE 200809L

#include "calibrate.h"

#include "geometry.h"
#include "kinematics.h"
#include "motion.h"
#include "path.h"
#include "pins.h"

#include <math.h>
#include <stdio.h>
#include <time.h>

static void read_motors(long steps[ACE_MOTOR_COUNT]) {
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) steps[i] = motion_position(i);
}

static int drain_until_idle(BallDetector *detector, const HsvRange *range) {
    while (path_busy()) {
        DetectionResult ignored;
        if (bd_detect(detector, range, &ignored) != 0) return -1;
    }
    return 0;
}

static int move_to(BallDetector *detector, const HsvRange *range,
                   double x_mm, double y_mm, unsigned int step_delay_us) {
    if (path_start(x_mm, y_mm, step_delay_us) != 0) return -1;
    return drain_until_idle(detector, range);
}

static int sample_object(BallDetector *detector, const HsvRange *range,
                         double *px, double *py, double *radius_px) {
    int    hits = 0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_r = 0.0;

    for (int frame = 0; frame < ACE_CALIBRATION_MAX_FRAMES; frame++) {
        DetectionResult result;
        if (bd_detect(detector, range, &result) != 0) return -1;

        if (result.found) {
            sum_x += result.x;
            sum_y += result.y;
            sum_r += result.radius;
            hits++;
            if (hits >= ACE_CALIBRATION_SAMPLES) break;
        }
    }

    if (hits < ACE_CALIBRATION_SAMPLES) return -1;

    *px = sum_x / hits;
    *py = sum_y / hits;
    if (radius_px) *radius_px = sum_r / hits;
    return 0;
}

static int apply_tension(BallDetector *detector, const HsvRange *range,
                         double at_x_mm, double at_y_mm,
                         unsigned int step_delay_us) {
    long steps[ACE_MOTOR_COUNT];
    long wind = lround(ACE_TENSION_MM / ACE_MM_PER_HALFSTEP);

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) steps[i] = wind;

    if (motion_start(steps, step_delay_us) != 0) return -1;
    if (drain_until_idle(detector, range) != 0) return -1;

    long motors[ACE_MOTOR_COUNT];
    read_motors(motors);
    kin_reset(at_x_mm, at_y_mm, motors);

    printf("Leichte Spannung: alle vier Seile %.1f mm aufgewickelt (%ld Halbschritte),\n"
           "Spulen bleiben bestromt, Position x=%.0f y=%.0f neu referenziert.\n",
           ACE_TENSION_MM, wind, at_x_mm, at_y_mm);
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
    return apply_tension(detector, range, 0.0, 0.0, step_delay_us);
}

int cal_goto(BallDetector *detector, const HsvRange *range,
             double start_x_mm, double start_y_mm,
             double target_x_mm, double target_y_mm,
             unsigned int step_delay_us) {
    long motors[ACE_MOTOR_COUNT];
    read_motors(motors);
    kin_reset(start_x_mm, start_y_mm, motors);

    long planned[ACE_MOTOR_COUNT];
    kin_plan(motors, target_x_mm, target_y_mm, planned);

    printf("Fahrt von x=%.0f y=%.0f nach x=%.0f y=%.0f\n",
           start_x_mm, start_y_mm, target_x_mm, target_y_mm);
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        printf("  Motor %d (%-13s) %+7ld Halbschritte = %+7.1f mm Seil  %s\n",
               i, ACE_MOTOR_NAMES[i], planned[i],
               planned[i] * ACE_MM_PER_HALFSTEP,
               planned[i] > 0 ? "kuerzer" : (planned[i] < 0 ? "laenger" : "-"));
    }

    if (move_to(detector, range, target_x_mm, target_y_mm, step_delay_us) != 0) return -1;
    return apply_tension(detector, range, target_x_mm, target_y_mm, step_delay_us);
}

int cal_run(BallDetector *detector, const HsvRange *range,
            int frame_width, double start_x_mm, double start_y_mm,
            double distance_mm, double object_diameter_mm,
            unsigned int step_delay_us) {
    long motors[ACE_MOTOR_COUNT];
    read_motors(motors);
    kin_reset(start_x_mm, start_y_mm, motors);

    if (start_x_mm != 0.0 || start_y_mm != 0.0) {
        printf("Start bei x=%.0f y=%.0f mm, fahre zuerst in die Mitte.\n",
               start_x_mm, start_y_mm);
        if (move_to(detector, range, 0.0, 0.0, step_delay_us) != 0) return -1;
    }

    printf("Kalibrierung: Objekt ins Bild legen und liegen lassen.\n");

    double base_x, base_y, base_r;
    if (sample_object(detector, range, &base_x, &base_y, &base_r) != 0) {
        fprintf(stderr,
                "Kein Objekt erkannt. Kalibrierung uebersprungen, HSV-Bereich pruefen "
                "(--h-min/--h-max ...).\n");
        if (move_to(detector, range, 0.0, 0.0, step_delay_us) != 0) return -1;
        apply_tension(detector, range, 0.0, 0.0, step_delay_us);
        return -1;
    }

    printf("Objekt bei %.1f, %.1f px, Radius %.1f px\n", base_x, base_y, base_r);

    double scale_mm_per_px = 0.0;
    if (object_diameter_mm > 0.0) {
        if (base_r < 3.0) {
            fprintf(stderr, "Radius %.1f px zu klein fuer einen Massstab.\n", base_r);
        } else {
            scale_mm_per_px = object_diameter_mm / (2.0 * base_r);
            printf("Massstab aus Objektdurchmesser %.1f mm: %.4f mm/px\n",
                   object_diameter_mm, scale_mm_per_px);
        }
    }

    if (move_to(detector, range, distance_mm, 0.0, step_delay_us) != 0) return -1;

    double shifted_x, shifted_y;
    if (sample_object(detector, range, &shifted_x, &shifted_y, NULL) != 0) {
        fprintf(stderr, "Objekt nach der x-Fahrt verloren, Abstand verkleinern "
                        "(--calib-mm).\n");
        move_to(detector, range, 0.0, 0.0, step_delay_us);
        apply_tension(detector, range, 0.0, 0.0, step_delay_us);
        return -1;
    }

    double shift_x_px = shifted_x - base_x;

    if (move_to(detector, range, 0.0, 0.0, step_delay_us) != 0) return -1;
    if (move_to(detector, range, 0.0, distance_mm, step_delay_us) != 0) return -1;

    if (sample_object(detector, range, &shifted_x, &shifted_y, NULL) != 0) {
        fprintf(stderr, "Objekt nach der y-Fahrt verloren, Abstand verkleinern "
                        "(--calib-mm).\n");
        move_to(detector, range, 0.0, 0.0, step_delay_us);
        apply_tension(detector, range, 0.0, 0.0, step_delay_us);
        return -1;
    }

    double shift_y_px = shifted_y - base_y;

    if (move_to(detector, range, 0.0, 0.0, step_delay_us) != 0) return -1;

    if (fabs(shift_x_px) < 5.0 || fabs(shift_y_px) < 5.0) {
        fprintf(stderr,
                "Bildverschiebung zu klein (%.1f / %.1f px). Entweder hat sich die "
                "Plattform nicht bewegt (Seil rutscht, Motor verliert Schritte) oder "
                "der Fahrweg ist zu kurz.\n", shift_x_px, shift_y_px);
        apply_tension(detector, range, 0.0, 0.0, step_delay_us);
        return -1;
    }

    double sign_x = (shift_x_px < 0.0) ? 1.0 : -1.0;
    double sign_y = (shift_y_px < 0.0) ? 1.0 : -1.0;

    printf("\nx: Plattform soll +%.0f mm  ->  Objekt %+.1f px\n",
           distance_mm, shift_x_px);
    printf("y: Plattform soll +%.0f mm  ->  Objekt %+.1f px\n",
           distance_mm, shift_y_px);

    if (scale_mm_per_px <= 0.0) {
        double mm_per_px_x = distance_mm / fabs(shift_x_px);
        double mm_per_px_y = distance_mm / fabs(shift_y_px);
        double spread = fabs(mm_per_px_x - mm_per_px_y)
                      / ((mm_per_px_x + mm_per_px_y) / 2.0) * 100.0;

        printf("Ohne --object-mm wird angenommen, dass %.1f mm auch wirklich "
               "gefahren wurden.\n", distance_mm);
        printf("  x: %.4f mm/px, y: %.4f mm/px, Abweichung %.1f %%\n",
               mm_per_px_x, mm_per_px_y, spread);
        printf("\nIn geometry.h eintragen:\n");
        printf("  #define ACE_VIEW_WIDTH_MM     %.0f\n",
               mm_per_px_x * (double)frame_width);
        printf("  #define ACE_IMAGE_TO_FIELD_X  %.1f\n", sign_x);
        printf("  #define ACE_IMAGE_TO_FIELD_Y  %.1f\n\n", sign_y);
        return apply_tension(detector, range, 0.0, 0.0, step_delay_us);
    }

    double travel_x_mm = fabs(shift_x_px) * scale_mm_per_px;
    double travel_y_mm = fabs(shift_y_px) * scale_mm_per_px;
    double travel_mm   = (travel_x_mm + travel_y_mm) / 2.0;
    double factor      = travel_mm / distance_mm;
    double drum_true   = ACE_DRUM_DIAMETER_MM * factor;
    double spread      = fabs(travel_x_mm - travel_y_mm) / travel_mm * 100.0;

    printf("   tatsaechlich gefahren: x %.1f mm, y %.1f mm (Abweichung %.1f %%)\n",
           travel_x_mm, travel_y_mm, spread);
    printf("   Faktor zum Sollweg: %.3f\n", factor);

    if (spread > 15.0) {
        printf("Ueber 15 %% Unterschied zwischen den Achsen: Kamera verdreht montiert, "
               "Objekt verrutscht, oder eine Winde rutscht.\n");
    }
    if (factor < 0.5 || factor > 2.0) {
        printf("Faktor weit von 1 entfernt: pruef den Wickeldurchmesser und ob "
               "Schritte verloren gehen.\n");
    }

    printf("\nIn geometry.h eintragen:\n");
    printf("  #define ACE_VIEW_WIDTH_MM     %.0f\n",
           scale_mm_per_px * (double)frame_width);
    printf("  #define ACE_DRUM_DIAMETER_MM  %.2f\n", drum_true);
    printf("  #define ACE_IMAGE_TO_FIELD_X  %.1f\n", sign_x);
    printf("  #define ACE_IMAGE_TO_FIELD_Y  %.1f\n\n", sign_y);
    printf("Danach neu bauen und einmal nachkalibrieren: der Faktor muss dann "
           "nahe 1.000 liegen.\n");

    return apply_tension(detector, range, 0.0, 0.0, step_delay_us);
}
