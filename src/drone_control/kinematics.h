#ifndef ACE_DRONE_CONTROL_KINEMATICS_H
#define ACE_DRONE_CONTROL_KINEMATICS_H

#include "geometry.h"
#include "pins.h"

#ifdef __cplusplus
extern "C" {
#endif

void kin_reset(double x_mm, double y_mm, const long motor_steps[ACE_MOTOR_COUNT]);

/* Wie kin_reset, aber auf einer vorgegebenen Hoehe. Wer aus einer Messung
 * neu referenziert, waehrend die Plattform nicht auf Nennhoehe haengt,
 * braucht das - sonst wirft der Reset die mitgefuehrte Hoehe weg. */
void kin_reset_at(double x_mm, double y_mm, double height_mm,
                  const long motor_steps[ACE_MOTOR_COUNT]);

void kin_position(const long motor_steps[ACE_MOTOR_COUNT],
                  double *x_mm, double *y_mm);

void kin_plan(const long motor_steps[ACE_MOTOR_COUNT],
              double target_x_mm, double target_y_mm,
              long steps[ACE_MOTOR_COUNT]);

void kin_clamp(double *x_mm, double *y_mm);

/* Seillaenge zur Nennhoehe ACE_HOVER_HEIGHT_MM. */
double kin_cable_length(int motor, double x_mm, double y_mm);

/* Seillaenge zu einem beliebigen Punkt im Raum. */
double kin_cable_length_at(int motor, double x_mm, double y_mm, double height_mm);

/* ---- Hoehe -------------------------------------------------------------
 *
 * Die Plattform soll sich nicht senkrecht bewegen, aber sie tut es: sie
 * haengt an Seilen, und ihre Hoehe folgt allein aus deren Laengen. Ein
 * Millimeter Seil sind hier rund 2,8 mm Hoehe, die Hebelwirkung ist also
 * betraechtlich.
 *
 * Fuer die Lage in der Flaeche ist das folgenlos: in der Paarformel
 * kuerzt sich die Hoehe exakt heraus, x und y stimmen auf jeder Hoehe.
 * Fuer die Planung ist es das nicht. Wird mit der Nennhoehe geplant,
 * waehrend die Plattform tiefer haengt, sind die kommandierten Seile
 * ungleich zu kurz - jede Fahrt zerrt sie dann schief nach oben.
 *
 * Darum wird die Hoehe aus dem laufenden Zaehlerstand mitgefuehrt und
 * gehalten, statt sie zu unterstellen.
 */

/* Hoehe unter der Ankerebene, die dieser Satz Laengen fordert. Mittel ueber
 * die vertrauenswuerdigen Winden. */
double kin_height(const double lengths[ACE_MOTOR_COUNT],
                  double x_mm, double y_mm);

/* Volle raeumliche Lage aus den Schrittzaehlern. */
void kin_pose(const long motor_steps[ACE_MOTOR_COUNT],
              double *x_mm, double *y_mm, double *height_mm);

/* Planung auf der gemessenen statt der unterstellten Hoehe. Standard ein.
 * Aus heisst: immer mit ACE_HOVER_HEIGHT_MM planen, wie frueher. */
void kin_set_hold_height(int enabled);
int  kin_hold_height(void);

/* Hoehe, mit der zuletzt geplant wurde. */
double kin_planned_height(void);

/* Die koppelnavigierten Seillaengen zu einem Schrittzaehlerstand. */
void kin_lengths(const long motor_steps[ACE_MOTOR_COUNT],
                 double lengths[ACE_MOTOR_COUNT]);

/* Vorwaertsloesung direkt aus Seillaengen statt aus Schrittzaehlern. */
void kin_position_from_lengths(const double lengths[ACE_MOTOR_COUNT],
                               double *x_mm, double *y_mm);

/* Hoehe unter den Ankern, die Seil <motor> bei dieser Laenge und dieser
 * XY-Lage erzwingt. Nur wenn alle vier Seile dieselbe Hoehe fordern, ist
 * der Satz Laengen mit einem einzigen Punkt im Raum vertraeglich. */
double kin_implied_height(int motor, double x_mm, double y_mm, double length_mm);

/* Spanne der vier geforderten Hoehen: 0 heisst raeumlich widerspruchsfrei. */
double kin_height_spread(const double lengths[ACE_MOTOR_COUNT],
                         double x_mm, double y_mm);

/* Ankerlage einer Winde in der Flaeche. */
void kin_anchor(int motor, double *x_mm, double *y_mm);

/* ---- Umgang mit einer Winde, die Schritte verliert ---------------------
 *
 * Vier Seile bei zwei Freiheitsgraden sind einfach redundant: jede Achse
 * wird aus zwei unabhaengigen Ankerpaaren bestimmt. Faellt eine Winde aus
 * der Wertung, bleibt je Achse genau ein Paar uebrig, die Lage ist also
 * weiter vollstaendig bestimmt. Damit laesst sich der Schlupf der
 * ausgeschlossenen Winde sogar messen: ihre koppelnavigierte Laenge wird
 * mit der verglichen, die sich aus der Lage der anderen ergibt.
 */

/* Diese Winde aus der Vorwaertsloesung nehmen; -1 hebt es wieder auf.
 * Mehr als eine geht nicht, dann waere die Lage unterbestimmt. */
void kin_set_untrusted(int motor);
int  kin_untrusted(void);

/* Abweichung der koppelnavigierten Laenge dieser Winde von der Laenge, die
 * kin_plan an der aus den uebrigen Winden bestimmten Lage vorschreibt.
 *
 * ACHTUNG, was das NICHT ist: ein Schlupfmesser. Der Schrittzaehler zaehlt
 * Kommandos, keine Umdrehungen, und kin_plan treibt alle vier Modellaengen
 * auf einen gemeinsamen Punkt. Sie sind damit per Konstruktion
 * widerspruchsfrei - gleichgueltig, was die Mechanik wirklich getan hat.
 * Rutscht eine Winde durch, bleibt dieser Wert bei null.
 *
 * Was er misst, ist die Selbstkonsistenz des Modells: Rundung auf ganze
 * Halbschritte, ein Wegpunktabstand, der zu gross gewaehlt wurde, oder eine
 * von aussen veraenderte Referenz. Mehr kann Koppelnavigation ohne Geber
 * grundsaetzlich nicht leisten; die einzige unabhaengige Messung im Aufbau
 * ist die Kamera. */
double kin_length_residual(int motor, const long motor_steps[ACE_MOTOR_COUNT]);

/* Nur diese eine Winde neu referenzieren, ohne die anderen anzufassen. */
void kin_reset_motor(int motor, double x_mm, double y_mm,
                     const long motor_steps[ACE_MOTOR_COUNT]);

/* Zusatz zur festen ACE_MOTOR_TRIM_MM, zur Laufzeit. Negativ heisst
 * kuerzeres Seil, also mehr Vorspannung auf dieser Winde. */
void   kin_set_trim(int motor, double mm);
double kin_trim(int motor);

#ifdef __cplusplus
}
#endif
#endif
