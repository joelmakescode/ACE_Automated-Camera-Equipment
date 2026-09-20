#ifndef ACE_DRONE_CONTROL_FLOORREF_H
#define ACE_DRONE_CONTROL_FLOORREF_H

#include <stdbool.h>

#include "ball_detector.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Zwei Klebebaender auf der Flaeche als absolute Referenz.
 *
 * Sie laufen durch den Mittelpunkt des Ankerfelds, jedes in bekannter
 * Richtung und mit bekannter Strichteilung. Damit liefert ein einziges
 * Bild drei Dinge, die die Bewegungsmessung nicht hergibt:
 *
 *   Winkel    Eine Feldrichtung b erscheint im Bild unter b + Kamerawinkel.
 *             Aus der gemessenen Bildrichtung folgt der Kamerawinkel also
 *             unmittelbar - ohne Fahrt, ohne Nachlauf, ohne Lernphase.
 *
 *   Lage      Der Lotabstand der Linie von der Bildmitte ist der Abstand
 *             der Plattform von dieser Linie. Zwei Linien geben zwei
 *             solche Abstaende und damit die Lage vollstaendig. Das
 *             schlaegt den Schlupf nicht nach, es hebt ihn auf.
 *
 *   Massstab  Die Strichteilung ist in mm bekannt und im Bild messbar.
 *
 * Damit beide Linien unterscheidbar bleiben, haben sie verschiedene
 * Farben. Eine Linie ist 180 Grad periodisch; waeren beide gleich gefaerbt,
 * liesse sich nicht sagen, welche gemessene Richtung zu welcher Linie
 * gehoert, und der Kamerawinkel bliebe bis auf rund 90 Grad offen.
 */

typedef struct {
    double angle_deg;     /* Richtung im Feld, -90 .. +90 */
    double dash_mm;       /* Strichteilung, 0 = keine Marken */
    HsvRange colour;
} FloorLine;

typedef struct {
    bool   have_angle;
    bool   have_position;
    bool   have_scale;

    double camera_angle_deg;   /* Kameradrehung, absolut               */
    double x_mm, y_mm;         /* Lage der Plattform, absolut          */
    double px_per_mm;          /* aus der Strichteilung                */

    double included_deg;       /* eingeschlossener Winkel im Bild      */
    double included_error_deg; /* Abweichung vom bekannten Sollwinkel  */

    /* Kippen der Kamera und was es anrichtet.
     *
     * Steht die Kamera schief, zeigt ihre optische Achse nicht mehr
     * senkrecht nach unten. Die Bildmitte trifft dann nicht den Punkt unter
     * der Kamera, sondern einen um Z*tan(Kippen) versetzten - bei 10 Grad
     * und 300 mm Hoehe sind das 53 mm. Genau um diese Strecke meldet die
     * Lageberechnung falsch, und genau um sie faehrt der Regler die
     * Plattform von der Objektmitte weg.
     *
     * Der Betrag folgt aus dem eingeschlossenen Winkel: eine Drehstreckung
     * erhaelt Winkel, eine Perspektive staucht sie. Die Richtung des
     * Kippens folgt daraus NICHT - zwei Linien geben dafuer zu wenig her.
     * Der Versatz selbst wird darum nicht hieraus gerechnet, sondern beim
     * Start gemessen, wo die Lage der Plattform bekannt ist. Dieser Wert
     * dient dazu, das Veralten jener Messung zu erkennen. */
    double tilt_deg;
    double tilt_offset_mm;

    int    lines_seen;
} FloorFix;

/* Standardbelegung: Diagonalen des Ankerfelds, blau und gruen. */
void floor_default_lines(FloorLine lines[2]);

/* Aus den Bildmessungen beider Linien einen Fix rechnen.
 * scale_hint dient als Massstab, solange keine Strichteilung erkannt
 * wurde; 0 heisst, dann gibt es auch keine Lage. */
void floor_solve(const FloorLine lines[2], const LineResult seen[2],
                 double scale_hint, FloorFix *out);

#ifdef __cplusplus
}
#endif
#endif
