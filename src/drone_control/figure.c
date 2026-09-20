#include "figure.h"

#include "geometry.h"
#include "kinematics.h"

#include <math.h>
#include <stddef.h>

/* Stuetzstellen je Teilstueck fuer die Trockensimulation. Der groesste
 * Hoehenwiderspruch liegt in der Mitte eines Stuecks, nicht an den Enden. */
#define SIM_SAMPLES_PER_SEGMENT 9

int figure_x_points(double half_x_mm, double half_y_mm,
                    FigurePoint points[ACE_FIGURE_MAX_POINTS]) {
    /* Vier Speichen vom Mittelpunkt in die Diagonalen. Die Reihenfolge ist so
     * gewaehlt, dass zwei aufeinander folgende Speichen gegenueber liegen:
     * Spitze -> Mitte -> gegenueberliegende Spitze ergibt eine durchgehende
     * Diagonale. Zwei solche Diagonalen sind das X. Zwischen den Speichen
     * kehrt die Plattform in die Mitte zurueck, so dass sich nach jedem
     * Viertel am Startpunkt ablesen laesst, ob die Winden noch stimmen. */
    static const int corners[4][2] = {
        { -1,  1 },
        {  1, -1 },
        {  1,  1 },
        { -1, -1 },
    };
    static const char *const names[4] = {
        "links oben", "rechts unten", "rechts oben", "links unten"
    };

    int n = 0;

    points[n].x_mm  = 0.0;
    points[n].y_mm  = 0.0;
    points[n].label = "Mitte";
    n++;

    for (int c = 0; c < 4; c++) {
        points[n].x_mm  = corners[c][0] * half_x_mm;
        points[n].y_mm  = corners[c][1] * half_y_mm;
        points[n].label = names[c];
        n++;

        points[n].x_mm  = 0.0;
        points[n].y_mm  = 0.0;
        points[n].label = "Mitte";
        n++;
    }
    return n;
}

/* Die Berechnung des getragenen Gewichtsanteils steht in kinematics.c,
 * weil kin_clamp sie ebenfalls braucht - siehe kin_load_share. */

void figure_pose(double x_mm, double y_mm, FigurePose *out) {
    if (!out) return;

    out->min_tension        = 0.0;
    out->max_tension        = 0.0;
    out->min_elevation_deg  = 0.0;
    out->inside_anchor_field = true;

    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        double l       = kin_cable_length(i, x_mm, y_mm);
        double flat_sq = l * l - ACE_HOVER_HEIGHT_MM * ACE_HOVER_HEIGHT_MM;
        double flat    = (flat_sq > 0.0) ? sqrt(flat_sq) : 0.0;
        double share   = kin_load_share(i, x_mm, y_mm);

        if (share < 0.0) {
            share = 0.0;
            out->inside_anchor_field = false;
        }

        out->length_mm[i]     = l;
        out->elevation_deg[i] = atan2(ACE_HOVER_HEIGHT_MM, flat) * 180.0 / ACE_PI;
        out->load[i]          = share;
        /* Senkrecht wirkt nur h/l des Seilzugs, also ist der Zug im Seil um
         * l/h groesser als der getragene Gewichtsanteil. */
        out->tension[i]       = share * l / ACE_HOVER_HEIGHT_MM;

        if (i == 0 || out->tension[i] < out->min_tension)
            out->min_tension = out->tension[i];
        if (i == 0 || out->tension[i] > out->max_tension)
            out->max_tension = out->tension[i];
        if (i == 0 || out->elevation_deg[i] < out->min_elevation_deg)
            out->min_elevation_deg = out->elevation_deg[i];
    }
}

/* Sollaenge eines Seils am Wegpunkt, mit denselben Zuschlaegen wie kin_plan.
 * Die Nennhoehe genuegt hier: in dieser Trockenrechnung startet die
 * Plattform auf Nennhoehe, und da jeder Wegpunkt raeumlich widerspruchsfrei
 * ist, bleibt sie dort. Die Hoehe, die kin_plan an der echten Anlage
 * mitfuehrt, waere an jedem Wegpunkt dieselbe. */
static double planned_length(int motor, double x_mm, double y_mm) {
    double from_centre = sqrt(x_mm * x_mm + y_mm * y_mm);
    double slack       = ACE_SLACK_PER_100MM * from_centre / 100.0;

    return kin_cable_length(motor, x_mm, y_mm) + slack + ACE_MOTOR_TRIM_MM[motor];
}

void figure_simulate(double from_x_mm, double from_y_mm,
                     double to_x_mm, double to_y_mm,
                     double segment_mm, FigureSim *out) {
    if (!out) return;

    out->max_offline_mm  = 0.0;
    out->max_height_mm   = 0.0;
    out->end_error_mm    = 0.0;
    out->total_halfsteps = 0;
    out->segments        = 1;

    double dx   = to_x_mm - from_x_mm;
    double dy   = to_y_mm - from_y_mm;
    double dist = sqrt(dx * dx + dy * dy);
    if (dist < 1e-9) return;

    if (segment_mm <= 0.0) segment_mm = dist;
    int segments = (int)ceil(dist / segment_mm);
    if (segments < 1) segments = 1;
    out->segments = segments;

    /* Ausgangszustand: die Laengen, die kin_reset am Startpunkt setzt. */
    double have[ACE_MOTOR_COUNT];
    for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
        have[i] = kin_cable_length(i, from_x_mm, from_y_mm);
    }

    for (int s = 1; s <= segments; s++) {
        double t  = (double)s / (double)segments;
        double wx = from_x_mm + dx * t;
        double wy = from_y_mm + dy * t;

        /* Am Wegpunkt selbst ist die Lage raeumlich widerspruchsfrei: alle
         * vier Laengen stammen aus demselben Punkt (x, y, -h). Dazwischen
         * laufen die Winden linear, und genau dort entsteht der Fehler. */
        double want[ACE_MOTOR_COUNT];
        for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
            double mm_per_step = ACE_MM_PER_HALFSTEP_AT(i);
            long   steps       = lround((have[i] - planned_length(i, wx, wy))
                                        / mm_per_step);

            out->total_halfsteps += (steps >= 0) ? steps : -steps;
            want[i] = have[i] - (double)steps * mm_per_step;
        }

        for (int k = 1; k <= SIM_SAMPLES_PER_SEGMENT; k++) {
            double f = (double)k / (double)SIM_SAMPLES_PER_SEGMENT;
            double now[ACE_MOTOR_COUNT];
            for (int i = 0; i < ACE_MOTOR_COUNT; i++) {
                now[i] = have[i] + (want[i] - have[i]) * f;
            }

            double px, py;
            kin_position_from_lengths(now, &px, &py);

            double off = fabs((px - from_x_mm) * dy - (py - from_y_mm) * dx) / dist;
            if (off > out->max_offline_mm) out->max_offline_mm = off;

            double spread = kin_height_spread(now, px, py);
            if (spread > out->max_height_mm) out->max_height_mm = spread;
        }

        for (int i = 0; i < ACE_MOTOR_COUNT; i++) have[i] = want[i];
    }

    double end_x, end_y;
    kin_position_from_lengths(have, &end_x, &end_y);
    out->end_error_mm = sqrt((end_x - to_x_mm) * (end_x - to_x_mm)
                           + (end_y - to_y_mm) * (end_y - to_y_mm));
}

double figure_margin_without(int skip_motor, double x_mm, double y_mm) {
    double tx[3], ty[3];
    int    n = 0;

    for (int i = 0; i < ACE_MOTOR_COUNT && n < 3; i++) {
        if (i == skip_motor) continue;
        kin_anchor(i, &tx[n], &ty[n]);
        n++;
    }
    if (n < 3) return 0.0;

    /* Umlaufsinn bestimmen, damit "innen" unabhaengig von der Reihenfolge
     * der Anker dasselbe Vorzeichen bekommt. */
    double area2 = (tx[1] - tx[0]) * (ty[2] - ty[0])
                 - (ty[1] - ty[0]) * (tx[2] - tx[0]);
    double turn  = (area2 >= 0.0) ? 1.0 : -1.0;

    double best = 0.0;
    int    have = 0;

    for (int e = 0; e < 3; e++) {
        int    f   = (e + 1) % 3;
        double ex  = tx[f] - tx[e];
        double ey  = ty[f] - ty[e];
        double len = sqrt(ex * ex + ey * ey);
        if (len < 1e-9) continue;

        double d = turn * (ex * (y_mm - ty[e]) - ey * (x_mm - tx[e])) / len;
        if (!have || d < best) { best = d; have = 1; }
    }
    return best;
}

double figure_winch_force_n(int motor) {
    if (motor < 0 || motor >= ACE_MOTOR_COUNT) return 0.0;

    double radius_mm = ACE_MOTOR_DRUM_MM[motor] / 2.0;
    if (radius_mm <= 0.0) return 0.0;

    return ACE_MOTOR_TORQUE_NMM / radius_mm;
}
