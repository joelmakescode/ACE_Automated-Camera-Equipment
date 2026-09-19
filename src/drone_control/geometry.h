#ifndef ACE_DRONE_CONTROL_GEOMETRY_H
#define ACE_DRONE_CONTROL_GEOMETRY_H

#include "pins.h"

#define ACE_PI 3.14159265358979323846

#ifndef ACE_ANCHOR_SPAN_X_MM
#define ACE_ANCHOR_SPAN_X_MM 640.0
#endif

#ifndef ACE_ANCHOR_SPAN_Y_MM
#define ACE_ANCHOR_SPAN_Y_MM 440.0
#endif

#ifndef ACE_HOVER_HEIGHT_MM
#define ACE_HOVER_HEIGHT_MM 150.0
#endif

#ifndef ACE_CAMERA_HEIGHT_MM
#define ACE_CAMERA_HEIGHT_MM 400.0
#endif

#define ACE_RIG_HEIGHT_MM (ACE_CAMERA_HEIGHT_MM + ACE_HOVER_HEIGHT_MM)

#ifndef ACE_DRUM_DIAMETER_MM
#define ACE_DRUM_DIAMETER_MM 20.0
#endif

#define ACE_MM_PER_HALFSTEP \
    ((ACE_PI * ACE_DRUM_DIAMETER_MM) / (double)ACE_HALFSTEPS_PER_REV)

#ifndef ACE_MOTOR_DRUM_LIST
#define ACE_MOTOR_DRUM_LIST                             \
    { ACE_DRUM_DIAMETER_MM, ACE_DRUM_DIAMETER_MM,       \
      ACE_DRUM_DIAMETER_MM, ACE_DRUM_DIAMETER_MM }
#endif

#ifndef ACE_MOTOR_TRIM_LIST
#define ACE_MOTOR_TRIM_LIST { 0.0, 0.0, 0.0, 0.0 }
#endif

static const double ACE_MOTOR_DRUM_MM[ACE_MOTOR_COUNT] = ACE_MOTOR_DRUM_LIST;
static const double ACE_MOTOR_TRIM_MM[ACE_MOTOR_COUNT] = ACE_MOTOR_TRIM_LIST;

#define ACE_MM_PER_HALFSTEP_AT(motor) \
    ((ACE_PI * ACE_MOTOR_DRUM_MM[motor]) / (double)ACE_HALFSTEPS_PER_REV)

#ifndef ACE_PATROL_SPAN_X_MM
#define ACE_PATROL_SPAN_X_MM 300.0
#endif

#ifndef ACE_PATROL_SPAN_Y_MM
#define ACE_PATROL_SPAN_Y_MM 300.0
#endif

#ifndef ACE_START_X_MM
#define ACE_START_X_MM 0.0
#endif

#ifndef ACE_START_Y_MM
#define ACE_START_Y_MM 0.0
#endif

#ifndef ACE_REACH_LIMIT_X_MM
#define ACE_REACH_LIMIT_X_MM 220.0
#endif

#ifndef ACE_REACH_LIMIT_Y_MM
#define ACE_REACH_LIMIT_Y_MM 170.0
#endif

#ifndef ACE_SLACK_PER_100MM
#define ACE_SLACK_PER_100MM 0.0
#endif

#ifndef ACE_VIEW_WIDTH_MM
#define ACE_VIEW_WIDTH_MM 325.0
#endif

#ifndef ACE_CENTER_TOLERANCE_PX
#define ACE_CENTER_TOLERANCE_PX 25.0
#endif

#ifndef ACE_HOVER_RELEASE_PX
#define ACE_HOVER_RELEASE_PX 45.0
#endif

#ifndef ACE_MAX_CORRECTION_MM
#define ACE_MAX_CORRECTION_MM 25.0
#endif

#ifndef ACE_REAIM_MS
#define ACE_REAIM_MS 300
#endif

#ifndef ACE_WRONG_WAY_STRIKES
#define ACE_WRONG_WAY_STRIKES 4
#endif

#ifndef ACE_CORRECTION_GAIN
#define ACE_CORRECTION_GAIN 0.6
#endif

#ifndef ACE_LOST_GRACE_MS
#define ACE_LOST_GRACE_MS 3000
#endif

#ifndef ACE_IMAGE_TO_FIELD_X
#define ACE_IMAGE_TO_FIELD_X 1.0
#endif

#ifndef ACE_IMAGE_TO_FIELD_Y
#define ACE_IMAGE_TO_FIELD_Y -1.0
#endif

#ifndef ACE_CALIBRATION_DISTANCE_MM
#define ACE_CALIBRATION_DISTANCE_MM 60.0
#endif

#ifndef ACE_CALIBRATION_SAMPLES
#define ACE_CALIBRATION_SAMPLES 8
#endif

#ifndef ACE_CALIBRATION_MAX_FRAMES
#define ACE_CALIBRATION_MAX_FRAMES 60
#endif

#ifndef ACE_OBJECT_DIAMETER_MM
#define ACE_OBJECT_DIAMETER_MM 0.0
#endif

#ifndef ACE_TENSION_MM
#define ACE_TENSION_MM 1.0
#endif

#ifndef ACE_SEGMENT_MM
#define ACE_SEGMENT_MM 5.0
#endif

#ifndef ACE_TRAVEL_DELAY_US
#define ACE_TRAVEL_DELAY_US 2500u
#endif

/* ---- Figurenfahrt ohne Kamera (ace_figure) ---------------------------- */

/* Kantenlaenge des X, volle Spanne. Die halbe Spanne ist der Abstand
 * Mitte -> Diagonalspitze. 240 mm halten die am weitesten entfernte Winde
 * noch deutlich ueber ACE_MIN_CABLE_TENSION; bei der Suchfahrtspanne von
 * 300 mm faellt sie auf rund 17 % des Plattformgewichts. */
#ifndef ACE_FIGURE_SPAN_X_MM
#define ACE_FIGURE_SPAN_X_MM 240.0
#endif

#ifndef ACE_FIGURE_SPAN_Y_MM
#define ACE_FIGURE_SPAN_Y_MM 240.0
#endif

/* Pause an jeder Spitze, damit die Plattform ausschwingt, bevor der
 * naechste Abschnitt aus den Schrittzaehlern geplant wird. */
#ifndef ACE_FIGURE_SETTLE_MS
#define ACE_FIGURE_SETTLE_MS 400
#endif

/* Sekunden Vorlauf nach dem Start, damit man die Plattform loslassen kann. */
#ifndef ACE_FIGURE_WAIT_S
#define ACE_FIGURE_WAIT_S 3
#endif

/* Unterhalb dieses Seilzugs, gemessen in Vielfachen des Plattformgewichts,
 * gilt ein Seil als zu lose: es traegt dann kaum noch, sein Schrittzaehler
 * beschreibt die Lage nicht mehr. */
#ifndef ACE_MIN_CABLE_TENSION
#define ACE_MIN_CABLE_TENSION 0.05
#endif

/* Groesster Hoehenwiderspruch zwischen den vier Seilen, der waehrend der
 * Fahrt hingenommen wird. Darueber ziehen die Winden gegeneinander. */
#ifndef ACE_MAX_HEIGHT_SPREAD_MM
#define ACE_MAX_HEIGHT_SPREAD_MM 1.0
#endif

/* Haltemoment einer Winde in Nmm. 28BYJ-48 mit ULN2003 an 5 V liefert laut
 * Datenblatt rund 34 Nmm, allerdings nur bei sehr langsamem Lauf. Bei
 * Betriebsdrehzahl bleibt spuerbar weniger uebrig. */
#ifndef ACE_MOTOR_TORQUE_NMM
#define ACE_MOTOR_TORQUE_NMM 34.0
#endif

#ifndef ACE_GRAVITY_MM_S2
#define ACE_GRAVITY_MM_S2 9810.0
#endif

/* ---- Verfolgung mit Kamera (ace_track) -------------------------------- */

/* Als zentriert gilt das Objekt innerhalb dieses Radius um die Bildmitte. */
#ifndef ACE_TRACK_TOLERANCE_PX
#define ACE_TRACK_TOLERANCE_PX 18.0
#endif

/* Daempfung des Regelkreises. Der Kreis bleibt stabil, solange der
 * geschaetzte Kamerawinkel um weniger als acos(gain/2) danebenliegt:
 * bei 1.0 sind das 60 Grad, bei 0.6 schon 72.5 Grad. Da hier ein HDMI-Kabel
 * an der Kamera zieht, ist die Reserve den langsameren Abbau wert. */
#ifndef ACE_TRACK_GAIN
#define ACE_TRACK_GAIN 0.6
#endif

#ifndef ACE_TRACK_MAX_STEP_MM
#define ACE_TRACK_MAX_STEP_MM 30.0
#endif

/* Laenge einer Probefahrt der Lernphase. */
#ifndef ACE_TRACK_PROBE_MM
#define ACE_TRACK_PROBE_MM 30.0
#endif

#ifndef ACE_TRACK_SETTLE_MS
#define ACE_TRACK_SETTLE_MS 500
#endif

/* Treffer, ueber die eine Messung der Objektlage gemittelt wird. */
#ifndef ACE_TRACK_SAMPLES
#define ACE_TRACK_SAMPLES 5
#endif

#ifndef ACE_TRACK_SAMPLE_FRAMES
#define ACE_TRACK_SAMPLE_FRAMES 40
#endif

/* Zuege ohne nennenswerten Fortschritt, nach denen das Bildmodell neu
 * gelernt wird - dann hat sich die Kamera weiter gedreht als der Regler
 * von sich aus wieder einfangen kann. */
#ifndef ACE_TRACK_STALL_LIMIT
#define ACE_TRACK_STALL_LIMIT 2
#endif

/* Weniger als das gilt als kein Fortschritt. */
#ifndef ACE_TRACK_STALL_RATIO
#define ACE_TRACK_STALL_RATIO 0.95
#endif

/* Ausreisserfilter des Bildmodells: Anteil der Vorhersage plus Sockel. */
#ifndef ACE_VIS_GATE_REL
#define ACE_VIS_GATE_REL 0.35
#endif

#ifndef ACE_VIS_GATE_PX
#define ACE_VIS_GATE_PX 12.0
#endif

/* Vergessensfaktor. Nach zehn Messungen zaehlt die aelteste noch zu 20 %. */
#ifndef ACE_VIS_LAMBDA
#define ACE_VIS_LAMBDA 0.85
#endif

/* ---- Winde, die Schritte verliert ------------------------------------- */

/* Diese Winde wird aus der Positionsschaetzung genommen. Die Lage bleibt
 * ueber die uebrigen drei vollstaendig bestimmt, und der Schlupf dieser
 * Winde wird dadurch messbar statt unsichtbar. -1 schaltet es ab. */
#ifndef ACE_WEAK_MOTOR
#define ACE_WEAK_MOTOR 1
#endif

/* Vorspannung auf der schwachen Winde: ihr Seil wird um diesen Betrag
 * kuerzer kommandiert, damit es trotz Schlupf straff bleibt. */
#ifndef ACE_WEAK_PRELOAD_MM
#define ACE_WEAK_PRELOAD_MM 0.8
#endif

/* Ab diesem gemessenen Schlupf wird gewarnt. */
#ifndef ACE_SLIP_WARN_MM
#define ACE_SLIP_WARN_MM 0.6
#endif

/* Ab diesem Schlupf wird die Winde neu referenziert, damit die Planung
 * wieder die richtige Seillaenge kommandiert. */
#ifndef ACE_SLIP_RECOVER_MM
#define ACE_SLIP_RECOVER_MM 1.5
#endif

/* ---- Hoehe halten ------------------------------------------------------
 *
 * Die Plattform haengt; ihre Hoehe folgt aus den Seillaengen und ist kein
 * fester Wert. Bei dieser Geometrie sind 1 mm Seil rund 2,8 mm Hoehe.
 * kin_plan fuehrt die Hoehe darum aus dem Zaehlerstand mit. Faellt sie aus
 * diesen Schranken, wird sie verworfen und mit der Nennhoehe geplant. */
#ifndef ACE_HOLD_HEIGHT_MIN_MM
#define ACE_HOLD_HEIGHT_MIN_MM (0.4 * ACE_HOVER_HEIGHT_MM)
#endif

#ifndef ACE_HOLD_HEIGHT_MAX_MM
#define ACE_HOLD_HEIGHT_MAX_MM (2.5 * ACE_HOVER_HEIGHT_MM)
#endif

/* Abweichung von der Nennhoehe, ab der gewarnt wird. */
#ifndef ACE_HEIGHT_DRIFT_WARN_MM
#define ACE_HEIGHT_DRIFT_WARN_MM 8.0
#endif

/* ---- Laufendes Messen waehrend der Fahrt ------------------------------
 *
 * Statt einer Messung je Zug wird fortlaufend gemessen, sobald die
 * Plattform diese Strecke seit der letzten Stuetzstelle zurueckgelegt hat.
 * Kurz genug, dass sich die Kamera in der Zwischenzeit kaum dreht, lang
 * genug fuer ein brauchbares Verhaeltnis von Signal zu Rauschen. */
#ifndef ACE_TRACK_BASELINE_MM
#define ACE_TRACK_BASELINE_MM 4.0
#endif

/* Streuung zwischen gefahrenem und im Bild gemessenem Weg, ab der auf eine
 * unruhige Mechanik hingewiesen wird. */
#ifndef ACE_DRIFT_WARN_MM
#define ACE_DRIFT_WARN_MM 2.0
#endif

/* Drehgeschwindigkeit des Kamerawinkels, ab der das Kabel als ziehend
 * gemeldet wird. */
#ifndef ACE_ANGLE_RATE_WARN_DPS
#define ACE_ANGLE_RATE_WARN_DPS 3.0
#endif

#endif
