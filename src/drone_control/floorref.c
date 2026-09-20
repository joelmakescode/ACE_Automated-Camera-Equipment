#include "floorref.h"

#include "geometry.h"

#include <math.h>
#include <stddef.h>

/* Auf -90..90 zusammenfalten. Eine Linie hat keine Richtung. */
static double fold(double deg) {
    while (deg >   90.0) deg -= 180.0;
    while (deg <= -90.0) deg += 180.0;
    return deg;
}

void floor_default_lines(FloorLine lines[2]) {
    if (!lines) return;

    /* Die Diagonalen des Ankerfelds. Aus den Spannweiten gerechnet, damit
     * sie einer geaenderten Geometrie folgen.
     *
     * lines[0] ist BLAU und laeuft von oben rechts nach unten links, also
     * unter +d. lines[1] ist GRUEN, von oben links nach unten rechts, also
     * unter -d. Wer die Baender andersherum klebt, muss hier tauschen. */
    double d = atan2(ACE_ANCHOR_SPAN_Y_MM, ACE_ANCHOR_SPAN_X_MM) * 180.0 / ACE_PI;

    lines[0].angle_deg = d;
    lines[0].dash_mm   = ACE_LINE_DASH_MM;
    lines[0].colour.h_min = ACE_LINE_A_H_MIN; lines[0].colour.h_max = ACE_LINE_A_H_MAX;
    lines[0].colour.s_min = ACE_LINE_S_MIN;   lines[0].colour.s_max = 255;
    lines[0].colour.v_min = ACE_LINE_V_MIN;   lines[0].colour.v_max = 255;

    lines[1].angle_deg = -d;
    lines[1].dash_mm   = ACE_LINE_DASH_MM;
    lines[1].colour.h_min = ACE_LINE_B_H_MIN; lines[1].colour.h_max = ACE_LINE_B_H_MAX;
    lines[1].colour.s_min = ACE_LINE_S_MIN;   lines[1].colour.s_max = 255;
    lines[1].colour.v_min = ACE_LINE_V_MIN;   lines[1].colour.v_max = 255;
}

void floor_solve(const FloorLine lines[2], const LineResult seen[2],
                 double scale_hint, FloorFix *out) {
    if (!lines || !seen || !out) return;

    out->have_angle         = false;
    out->have_position      = false;
    out->have_scale         = false;
    out->camera_angle_deg   = 0.0;
    out->x_mm               = 0.0;
    out->y_mm               = 0.0;
    out->px_per_mm          = 0.0;
    out->included_deg       = 0.0;
    out->included_error_deg = 0.0;
    out->lines_seen         = 0;

    for (int i = 0; i < 2; i++) if (seen[i].found) out->lines_seen++;
    if (out->lines_seen == 0) return;

    /* ---- Kamerawinkel ---------------------------------------------------
     * Eine Feldrichtung b erscheint im Bild unter b + Kamerawinkel. Ueber
     * beide Linien gemittelt, und zwar ueber den doppelten Winkel: die
     * Messungen sind 180 Grad periodisch, ein gewoehnlicher Mittelwert
     * wuerde bei -89 und +89 Grad in die Mitte statt ueber den Sprung
     * hinweg treffen. */
    double sx = 0.0, sy = 0.0;
    for (int i = 0; i < 2; i++) {
        if (!seen[i].found) continue;
        double a = fold(seen[i].angle_deg - lines[i].angle_deg) * ACE_PI / 180.0;
        sx += cos(2.0 * a);
        sy += sin(2.0 * a);
    }
    if (sx * sx + sy * sy > 1e-12) {
        out->camera_angle_deg = fold(0.5 * atan2(sy, sx) * 180.0 / ACE_PI);
        out->have_angle       = true;
    }

    /* ---- Kippen ---------------------------------------------------------
     * Eine Drehstreckung erhaelt Winkel, eine Perspektive nicht. Weicht der
     * eingeschlossene Winkel im Bild vom bekannten ab, steht die Kamera
     * schief. */
    if (seen[0].found && seen[1].found) {
        out->included_deg = fabs(fold(seen[0].angle_deg - seen[1].angle_deg));
        double want = fabs(fold(lines[0].angle_deg - lines[1].angle_deg));
        out->included_error_deg = out->included_deg - want;
    }

    /* ---- Massstab aus der Strichteilung --------------------------------- */
    double sum = 0.0;
    int    n   = 0;
    for (int i = 0; i < 2; i++) {
        if (seen[i].found && seen[i].dash_px > 1.0 && lines[i].dash_mm > 1.0) {
            sum += seen[i].dash_px / lines[i].dash_mm;
            n++;
        }
    }
    if (n > 0) {
        out->px_per_mm  = sum / n;
        out->have_scale = true;
    }

    double scale = out->have_scale ? out->px_per_mm : scale_hint;
    if (scale <= 1e-6) return;

    /* ---- Lage -----------------------------------------------------------
     * Beide Linien laufen durch den Feldnullpunkt. Der Lotabstand der
     * Plattform von Linie i ist c . n_i, mit n_i der Normalen der Linie im
     * Feld. Im Bild erscheint dieser Abstand mit dem Massstab multipliziert
     * und mit umgekehrtem Vorzeichen, weil das Bild die Lage des Bodens
     * relativ zur Kamera zeigt und nicht umgekehrt:
     *
     *     c . n_i = -offset_i / Massstab
     *
     * Zwei Linien geben zwei solche Gleichungen und damit c. */
    if (!(seen[0].found && seen[1].found)) return;

    double b0 = lines[0].angle_deg * ACE_PI / 180.0;
    double b1 = lines[1].angle_deg * ACE_PI / 180.0;

    double n0x = -sin(b0), n0y = cos(b0);
    double n1x = -sin(b1), n1y = cos(b1);

    double r0 = -seen[0].offset_px / scale;
    double r1 = -seen[1].offset_px / scale;

    double det = n0x * n1y - n0y * n1x;
    if (fabs(det) < 1e-6) return;          /* Linien fast parallel */

    out->x_mm = (r0 * n1y - r1 * n0y) / det;
    out->y_mm = (n0x * r1 - n1x * r0) / det;
    out->have_position = true;
}
