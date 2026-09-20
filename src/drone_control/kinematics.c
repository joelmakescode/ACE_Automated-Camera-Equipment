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
static double g_runtime_trim[ACE_MOTOR_COUNT];
static int    g_untrusted      = -1;
static int    g_hold_height    = 1;
static double g_planned_height = ACE_HOVER_HEIGHT_MM;

void kin_set_hold_height(int enabled) {
    g_hold_height = enabled ? 1 : 0;
}

int kin_hold_height(void) {
    return g_hold_height;
}

double kin_planned_height(void) {
    return g_planned_height;
}

void kin_anchor(int motor, double *x_mm, double *y_mm) {
    if (motor < 0 || motor >= ACE_MOTOR_COUNT) return;
    if (x_mm) *x_mm = anchor_x(motor);
    if (y_mm) *y_mm = anchor_y(motor);
}

void kin_set_untrusted(int motor) {
    g_untrusted = (motor >= 0 && motor < ACE_MOTOR_COUNT) ? motor : -1;
}

int kin_untrusted(void) {
    return g_untrusted;
}

void kin_set_trim(int motor, double mm) {
    if (motor < 0 || motor >= ACE_MOTOR_COUNT) return;
    g_runtime_trim[motor] = mm;
}

double kin_trim(int motor) {
    if (motor < 0 || motor >= ACE_MOTOR_COUNT) return 0.0;
    return g_runtime_trim[motor];
}

double kin_cable_length_at(int motor, double x_mm, double y_mm, double height_mm) {
    if (motor < 0 || motor >= ACE_MOTOR_COUNT) return 0.0;

    double dx = x_mm - anchor_x(motor);
    double dy = y_mm - anchor_y(motor);
    return sqrt(dx * dx + dy * dy + height_mm * height_mm);
}

double kin_cable_length(int motor, double x_mm, double y_mm) {
    return kin_cable_length_at(motor, x_mm, y_mm, ACE_HOVER_HEIGHT_MM);
}

void kin_reset_at(double x_mm, double y_mm, double height_mm,
                  const long motor_steps[ACE_MOTOR_COUNT]) {
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        g_reference_length[i] = kin_cable_length_at(i, x_mm, y_mm, height_mm);
        g_reference_steps[i]  = motor_steps ? motor_steps[i] : 0;
    }
}

void kin_reset(double x_mm, double y_mm, const long motor_steps[ACE_MOTOR_COUNT]) {
    kin_reset_at(x_mm, y_mm, ACE_HOVER_HEIGHT_MM, motor_steps);
}

static double current_length(int motor, const long motor_steps[ACE_MOTOR_COUNT]) {
    long wound = motor_steps[motor] - g_reference_steps[motor];
    return g_reference_length[motor] - (double)wound * ACE_MM_PER_HALFSTEP_AT(motor);
}

void kin_lengths(const long motor_steps[ACE_MOTOR_COUNT],
                 double lengths[ACE_MOTOR_COUNT]) {
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        lengths[i] = current_length(i, motor_steps);
    }
}

/* Zwei Anker mit gleicher Gegenkoordinate spannen eine Achse auf. Aus der
 * Differenz der Laengenquadrate faellt die Hoehe heraus, uebrig bleibt die
 * Achslage. Zwei unabhaengige Paare je Achse werden gemittelt. */
static double axis_from_lengths(int axis, double span,
                                const double lengths[ACE_MOTOR_COUNT]) {
    int pairs[ACE_MOTOR_COUNT][2];
    int count = find_axis_pairs(axis, pairs);
    if (count == 0) return 0.0;

    double sum  = 0.0;
    int    used = 0;

    for (int p = 0; p < count; p++) {
        if (g_untrusted >= 0 &&
            (pairs[p][0] == g_untrusted || pairs[p][1] == g_untrusted)) continue;

        double a = lengths[pairs[p][0]];
        double b = lengths[pairs[p][1]];
        sum += (a * a - b * b) / (2.0 * span);
        used++;
    }

    /* Bliebe kein Paar uebrig, waere die Achse unbestimmt. Dann lieber alle
     * nehmen als eine Null zu melden, die wie eine Messung aussieht. */
    if (used == 0) {
        for (int p = 0; p < count; p++) {
            double a = lengths[pairs[p][0]];
            double b = lengths[pairs[p][1]];
            sum += (a * a - b * b) / (2.0 * span);
        }
        used = count;
    }
    return sum / (double)used;
}

void kin_position_from_lengths(const double lengths[ACE_MOTOR_COUNT],
                               double *x_mm, double *y_mm) {
    if (x_mm) *x_mm = axis_from_lengths(0, ACE_ANCHOR_SPAN_X_MM, lengths);
    if (y_mm) *y_mm = axis_from_lengths(1, ACE_ANCHOR_SPAN_Y_MM, lengths);
}

void kin_position(const long motor_steps[ACE_MOTOR_COUNT],
                  double *x_mm, double *y_mm) {
    double lengths[ACE_MOTOR_COUNT];
    kin_lengths(motor_steps, lengths);
    kin_position_from_lengths(lengths, x_mm, y_mm);
}

double kin_implied_height(int motor, double x_mm, double y_mm, double length_mm) {
    if (motor < 0 || motor >= ACE_MOTOR_COUNT) return 0.0;

    double dx   = x_mm - anchor_x(motor);
    double dy   = y_mm - anchor_y(motor);
    double rest = length_mm * length_mm - dx * dx - dy * dy;

    return (rest > 0.0) ? sqrt(rest) : 0.0;
}

double kin_height(const double lengths[ACE_MOTOR_COUNT],
                  double x_mm, double y_mm) {
    double sum = 0.0;
    int    n   = 0;

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        if (i == g_untrusted) continue;

        double z = kin_implied_height(i, x_mm, y_mm, lengths[i]);
        if (z > 0.0) { sum += z; n++; }        /* 0 heisst: Seil zu kurz     */
    }

    /* Bleibt nichts Brauchbares uebrig, ist die Nennhoehe die ehrlichste
     * Antwort - eine gemittelte Null waere eine erfundene Messung. */
    if (n == 0) return ACE_HOVER_HEIGHT_MM;
    return sum / (double)n;
}

void kin_pose(const long motor_steps[ACE_MOTOR_COUNT],
              double *x_mm, double *y_mm, double *height_mm) {
    double lengths[ACE_MOTOR_COUNT];
    double x, y;

    kin_lengths(motor_steps, lengths);
    kin_position_from_lengths(lengths, &x, &y);

    if (x_mm)      *x_mm      = x;
    if (y_mm)      *y_mm      = y;
    if (height_mm) *height_mm = kin_height(lengths, x, y);
}

double kin_height_spread(const double lengths[ACE_MOTOR_COUNT],
                         double x_mm, double y_mm) {
    double lo = 0.0, hi = 0.0;

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        double z = kin_implied_height(i, x_mm, y_mm, lengths[i]);
        if (i == 0 || z < lo) lo = z;
        if (i == 0 || z > hi) hi = z;
    }
    return hi - lo;
}

/* Laenge, die kin_plan dieser Winde an diesem Punkt vorschreibt, also
 * einschliesslich Durchhang und Trimm. Nur gegen diesen Wert ist eine
 * Abweichung aussagekraeftig: die Vorspannung einer schwachen Winde ist
 * ein gewollter Unterschied, kein Fehler. */
static double commanded_length(int motor, double x_mm, double y_mm,
                               double height_mm) {
    double from_centre = sqrt(x_mm * x_mm + y_mm * y_mm);
    double slack       = ACE_SLACK_PER_100MM * from_centre / 100.0;

    return kin_cable_length_at(motor, x_mm, y_mm, height_mm)
         + slack + ACE_MOTOR_TRIM_MM[motor] + g_runtime_trim[motor];
}

double kin_length_residual(int motor, const long motor_steps[ACE_MOTOR_COUNT]) {
    if (motor < 0 || motor >= ACE_MOTOR_COUNT) return 0.0;

    double lengths[ACE_MOTOR_COUNT];
    double x, y;

    kin_lengths(motor_steps, lengths);
    kin_position_from_lengths(lengths, &x, &y);

    /* Mit der gemessenen Hoehe, nicht mit der Nennhoehe: 10 mm Hoehenfehler
     * sind hier 3.6 mm Laengenfehler und damit ein glatter Fehlalarm. Die
     * Hoehe stammt aus den vertrauenswuerdigen Winden, enthaelt diese also
     * nicht. */
    double h = kin_height(lengths, x, y);

    return lengths[motor] - commanded_length(motor, x, y, h);
}

void kin_reset_motor(int motor, double x_mm, double y_mm,
                     const long motor_steps[ACE_MOTOR_COUNT]) {
    if (motor < 0 || motor >= ACE_MOTOR_COUNT) return;

    double lengths[ACE_MOTOR_COUNT];
    double h = ACE_HOVER_HEIGHT_MM;

    if (motor_steps) {
        kin_lengths(motor_steps, lengths);
        h = kin_height(lengths, x_mm, y_mm);
    }

    g_reference_length[motor] = commanded_length(motor, x_mm, y_mm, h);
    g_reference_steps[motor]  = motor_steps ? motor_steps[motor] : 0;
}

/* Anteil des Plattformgewichts, den Winde <motor> traegt.
 *
 * Fuer einen Massepunkt an vier Seilen lautet das Kraeftegleichgewicht
 * sum(t_i * u_i) = Gewicht, mit u_i dem Einheitsvektor zum Anker i. Setzt
 * man s_i = t_i / l_i, zerfaellt das in sum(s_i * (Anker_i - P)) = 0 in der
 * Ebene und h * sum(s_i) = Gewicht senkrecht. Die erste Bedingung heisst:
 * die mit s_i gewichtete Mitte der vier Anker ist genau die XY-Lage der
 * Plattform. Die Gewichte sind also baryzentrische Koordinaten, und fuer
 * ein Rechteck ist die bilineare Wahl die natuerliche.
 *
 * Das System ist mit vier Seilen und drei Gleichungen einfach
 * ueberbestimmt; welche Verteilung sich real einstellt, haengt an der
 * Seildehnung. Die bilineare Loesung ist eine gueltige und innerhalb des
 * Ankerfelds durchweg positive Verteilung, also brauchbar als Massstab. */
double kin_load_share(int motor, double x_mm, double y_mm) {
    if (motor < 0 || motor >= ACE_MOTOR_COUNT) return 0.0;

    double u = (x_mm + ACE_ANCHOR_SPAN_X_MM / 2.0) / ACE_ANCHOR_SPAN_X_MM;
    double v = (y_mm + ACE_ANCHOR_SPAN_Y_MM / 2.0) / ACE_ANCHOR_SPAN_Y_MM;

    double fx = (ACE_MOTOR_CORNER[motor][0] > 0) ? u : (1.0 - u);
    double fy = (ACE_MOTOR_CORNER[motor][1] > 0) ? v : (1.0 - v);

    return fx * fy;
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

    /* Auf welcher Hoehe haengt die Plattform gerade? Mit der wird geplant,
     * nicht mit der Nennhoehe. Sonst enthaelt jede Fahrt in der Flaeche
     * heimlich einen Hub, und der faellt je Winde verschieden aus. */
    double height = ACE_HOVER_HEIGHT_MM;

    if (g_hold_height) {
        double lengths[ACE_MOTOR_COUNT];
        double x, y;

        kin_lengths(motor_steps, lengths);
        kin_position_from_lengths(lengths, &x, &y);

        double h = kin_height(lengths, x, y);

        /* Nur uebernehmen, wenn sie plausibel ist. Ein voellig verrutschter
         * Zaehlerstand darf die Planung nicht mitreissen. */
        if (h >= ACE_HOLD_HEIGHT_MIN_MM && h <= ACE_HOLD_HEIGHT_MAX_MM) {
            height = h;
        }
    }
    g_planned_height = height;

    double from_centre = sqrt(target_x_mm * target_x_mm + target_y_mm * target_y_mm);
    double slack = ACE_SLACK_PER_100MM * from_centre / 100.0;

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        double now    = current_length(i, motor_steps);
        double wanted = kin_cable_length_at(i, target_x_mm, target_y_mm, height)
                      + slack + ACE_MOTOR_TRIM_MM[i] + g_runtime_trim[i];
        steps[i] = lround((now - wanted) / ACE_MM_PER_HALFSTEP_AT(i));
    }
}
