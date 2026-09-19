#ifndef ACE_DRONE_CONTROL_FIGURE_H
#define ACE_DRONE_CONTROL_FIGURE_H

#include <stdbool.h>

#include "pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ACE_FIGURE_MAX_POINTS 16

typedef struct {
    double      x_mm;
    double      y_mm;
    const char *label;
} FigurePoint;

/* Raeumliche Lage an einem Punkt der Flaeche.
 *
 * Die Plattform haengt an vier Seilen, die schraeg nach oben zu den Ankern
 * laufen. Nur der Anteil sin(Hoehenwinkel) = h/l eines Seilzugs traegt
 * senkrecht. Weil die Seile flach liegen, ist der Zug im Seil deutlich
 * groesser als der Gewichtsanteil, den es abnimmt. */
typedef struct {
    double length_mm[ACE_MOTOR_COUNT];      /* 3D-Seillaenge Anker -> Plattform */
    double elevation_deg[ACE_MOTOR_COUNT];  /* Winkel des Seils zur Waagerechten */
    double load[ACE_MOTOR_COUNT];           /* getragener Gewichtsanteil, Summe 1 */
    double tension[ACE_MOTOR_COUNT];        /* Seilzug in Vielfachen des Gewichts */
    double min_tension;
    double max_tension;
    double min_elevation_deg;
    bool   inside_anchor_field;             /* sonst statisch nicht haltbar */
} FigurePose;

/* Ergebnis einer Trockensimulation einer Teilfahrt, ohne Motoren. */
typedef struct {
    double max_offline_mm;    /* groesste Abweichung von der Geraden        */
    double max_height_mm;     /* groesster Hoehenwiderspruch der vier Seile */
    double end_error_mm;      /* Restfehler am Ziel durch Schrittrundung    */
    long   total_halfsteps;   /* Summe der Halbschritte ueber alle Winden   */
    int    segments;
} FigureSim;

/* Legt das X als Speichen vom Mittelpunkt an und liefert die Punktzahl.
 * points[0] ist die Mitte, in der die Plattform von Hand steht. */
int figure_x_points(double half_x_mm, double half_y_mm,
                    FigurePoint points[ACE_FIGURE_MAX_POINTS]);

/* Statik an einem Punkt: Laengen, Winkel, Lastverteilung, Seilzuege. */
void figure_pose(double x_mm, double y_mm, FigurePose *out);

/* Faehrt eine Teilfahrt rechnerisch ab, genau wie path.c sie in Stuecke der
 * Laenge segment_mm zerlegt und kin_plan sie auf ganze Halbschritte rundet.
 * segment_mm <= 0 bedeutet ein einziges Stueck, also die naive Fahrt, bei
 * der die vier Winden stur von A nach B linear durchlaufen. */
void figure_simulate(double from_x_mm, double from_y_mm,
                     double to_x_mm, double to_y_mm,
                     double segment_mm, FigureSim *out);

/* Groesster Seilzug, den eine Winde bei diesem Wickeldurchmesser und
 * ACE_MOTOR_TORQUE_NMM aufbringen kann, in Newton. */
double figure_winch_force_n(int motor);

/* Abstand zur Kante des Dreiecks, das die uebrigen drei Anker aufspannen,
 * in mm; negativ heisst ausserhalb.
 *
 * Faellt eine Winde ganz aus, haengt die Plattform an drei Seilen. Positive
 * Zugkraefte gibt es dann nur noch, solange sie senkrecht ueber diesem
 * Dreieck steht. Bei der Standardgeometrie laeuft die Kante ohne Winde 1
 * genau durch die Mitte des Ankerfelds - die Haelfte der Flaeche haengt
 * also allein an der schwachen Winde. */
double figure_margin_without(int skip_motor, double x_mm, double y_mm);

#ifdef __cplusplus
}
#endif
#endif
