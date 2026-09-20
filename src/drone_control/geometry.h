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

/* Kamera ueber dem Brett, gemessen. */
#ifndef ACE_CAMERA_HEIGHT_MM
#define ACE_CAMERA_HEIGHT_MM 300.0
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

/* Bereich, in dem sich die Plattform bewegen darf: 550 x 350 mm,
 * also 45 mm Rand zum Ankerfeld auf allen vier Seiten. */
#ifndef ACE_REACH_LIMIT_X_MM
#define ACE_REACH_LIMIT_X_MM 275.0
#endif

#ifndef ACE_REACH_LIMIT_Y_MM
#define ACE_REACH_LIMIT_Y_MM 175.0
#endif

#ifndef ACE_SLACK_PER_100MM
#define ACE_SLACK_PER_100MM 0.0
#endif

/* Breite des Sichtfelds auf dem Brett, in mm.
 *
 * Am Aufbau gemessen: mit dem Lineal auf dem Brett rund 210 mm ueber die
 * volle Bildbreite. Gegengeprueft an der Strichteilung - 50 mm ergeben bei
 * diesem Massstab 305 px, gemessen wurden rund 300, das entspraeche 213 mm.
 * Beide Wege stimmen auf anderthalb Prozent ueberein.
 *
 * Das sind 38.6 Grad Blickwinkel und damit deutlich weniger, als das Modul
 * optisch koennte. rpicam-vid nimmt bei 1280x720 offenbar einen Ausschnitt
 * des Sensors statt das volle Feld zu binnen. Wer mehr Flaeche sehen will,
 * muesste den Sensormodus erzwingen; noetig ist es nicht. */
#ifndef ACE_VIEW_WIDTH_MM
#define ACE_VIEW_WIDTH_MM 210.0
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

/* Unterhalb dieses rechnerischen Seilzugs weist ace_figure in der
 * Wegpunkttabelle darauf hin. Nur ein Anhaltspunkt: gemessen wird nichts,
 * und die Verteilung auf vier Seile ist statisch unbestimmt. */
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

/* Index einer Winde, die aus der Positionsschaetzung genommen wird; -1
 * heisst: alle vier zaehlen.
 *
 * Mit gesundem Antrieb ist das Ausschliessen ein Verlust statt eines
 * Gewinns - es kostet die Mittelung ueber beide Ankerpaare je Achse und
 * legt die Pruefung auf Widerspruchsfreiheit der vier Seillaengen lahm.
 * Seit der Motor getauscht ist, steht es darum auf -1.
 *
 * Rutscht wieder eine Winde, hier ihren Index eintragen (0..3). Die Lage
 * bleibt dann ueber die uebrigen drei vollstaendig bestimmt. */
#ifndef ACE_WEAK_MOTOR
#define ACE_WEAK_MOTOR -1
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

/* ---- Bodenlinien als absolute Referenz ---------------------------------
 *
 * Zwei Klebebaender durch den Mittelpunkt des Ankerfelds, entlang seiner
 * Diagonalen. Verschiedene Farben, damit beide unterscheidbar bleiben: eine
 * Linie ist 180 Grad periodisch, bei gleicher Farbe waere die Zuordnung und
 * damit der Kamerawinkel bis auf rund 90 Grad offen.
 *
 * Matt kleben, nicht glaenzend. Ein Glanzpunkt wird im HSV-Raum weiss und
 * faellt aus dem Farbbereich - die Linie reisst dort auf.
 */

/* Strichteilung in mm, Mitte zu Mitte. Daraus kommt der Massstab.
 * 0 schaltet die Massstabsmessung ab, dann zaehlt nur Winkel und Lage. */
#ifndef ACE_LINE_DASH_MM
#define ACE_LINE_DASH_MM 50.0
#endif

/* Linie A: BLAU, von oben rechts nach unten links, also +34.5 Grad.
 * OpenCV skaliert den Farbton auf 0..179, Blau liegt also bei etwa 110. */
#ifndef ACE_LINE_A_H_MIN
#define ACE_LINE_A_H_MIN 95
#endif
#ifndef ACE_LINE_A_H_MAX
#define ACE_LINE_A_H_MAX 130
#endif

/* Linie B: GRUEN, von oben links nach unten rechts, also -34.5 Grad.
 * Gruen liegt bei etwa 60. */
#ifndef ACE_LINE_B_H_MIN
#define ACE_LINE_B_H_MIN 45
#endif
#ifndef ACE_LINE_B_H_MAX
#define ACE_LINE_B_H_MAX 85
#endif

/* Mindestsaettigung, je Linie getrennt: die beiden Baender kommen
 * unterschiedlich kraeftig an. Am Aufbau mit --mask geprueft. */
#ifndef ACE_LINE_A_S_MIN
#define ACE_LINE_A_S_MIN 60      /* blau  */
#endif
#ifndef ACE_LINE_B_S_MIN
#define ACE_LINE_B_S_MIN 40      /* gruen */
#endif

#ifndef ACE_LINE_V_MIN
#define ACE_LINE_V_MIN 50
#endif

/* Abweichung des eingeschlossenen Winkels vom Sollwert, ab der auf ein
 * Kippen der Kamera hingewiesen wird. 10 Grad Kippen machen hier rund
 * 0.8 Grad aus, 20 Grad rund 3.3. */
#ifndef ACE_TILT_WARN_DEG
#define ACE_TILT_WARN_DEG 1.5
#endif

/* Anteil, mit dem ein Lagefix aus den Bodenlinien in die Koppelnavigation
 * einfliesst. 1.0 waere harte Uebernahme und truege das Messrauschen voll
 * hinein; die Koppelnavigation ist kurzfristig die ruhigere Quelle, die
 * Messung langfristig die richtige. */
#ifndef ACE_FIX_BLEND
#define ACE_FIX_BLEND 0.25
#endif

/* Bandbreite in mm. Bei 3 mm und 3.2 px/mm sind das rund 10 Pixel im Bild -
 * schmal genug, dass eine morphologische Oeffnung die Linie aufreissen
 * wuerde. bd_detect_line filtert darum ueber die Flaeche zusammenhaengender
 * Gebiete statt ueber eine Oeffnung. */
#ifndef ACE_LINE_WIDTH_MM
#define ACE_LINE_WIDTH_MM 3.5
#endif

/* Kleinste Flaeche in Pixeln, die als Teil einer Linie durchgeht; alles
 * darunter ist Sprenkel. Bei der gemessenen Sichtbreite sind das 6.1 px/mm,
 * ein ganzer Strich von 25 x 3.5 mm bringt also rund 3250 Pixel mit und ein
 * Millimeter Linienlaenge noch 130. 150 wirft Rauschen weg und laesst auch
 * kurze Bruchstuecke stehen, die zum Verschmelzen gebraucht werden.
 *
 * Haengt am Massstab: aendert sich ACE_VIEW_WIDTH_MM deutlich, muss dieser
 * Wert quadratisch mitziehen. */
#ifndef ACE_LINE_MIN_BLOB_PX
#define ACE_LINE_MIN_BLOB_PX 150
#endif

/* Aenderung des Kippens, ab der der beim Start gemessene Versatz der
 * Bildmitte als veraltet gilt. 2 Grad Kippaenderung sind bei 300 mm Hoehe
 * rund 10 mm Versatz - so viel Lagefehler soll nicht unbemerkt einlaufen. */
#ifndef ACE_TILT_DRIFT_DEG
#define ACE_TILT_DRIFT_DEG 2.0
#endif

/* Waechst der Bildfehler waehrend einer Fahrt um diesen Faktor, zeigt der
 * geplante Weg in die falsche Richtung - das Objekt ist davongelaufen.
 * Dann wird die Fahrt abgebrochen und neu geplant, statt noch Sekunden
 * blind weiterzufahren. */
#ifndef ACE_TRACK_ABORT_GROW
#define ACE_TRACK_ABORT_GROW 1.6
#endif

/* ---- Suchfahrt --------------------------------------------------------
 *
 * Verschwindet das Objekt, faehrt die Plattform das Feld in einer
 * Schlangenlinie ab. Das Raster ist so gewaehlt, dass sich die
 * Kameraausschnitte ueberlappen: bei 210 x 118 mm Sichtfeld sind 180 mm
 * Spaltenabstand und 100 mm Zeilenabstand die groebste Teilung ohne Luecke.
 * Ein voller Durchlauf misst 1260 mm und dauert bei 6.1 mm/s gut drei
 * Minuten - das ist die Mechanik, nicht die Rechnung. */
#ifndef ACE_SEARCH_COLS
#define ACE_SEARCH_COLS 3
#endif
#ifndef ACE_SEARCH_ROWS
#define ACE_SEARCH_ROWS 4
#endif

/* Aeusserste Wegpunkte. Ihr halbes Sichtfeld reicht ueber den Rand des
 * Fahrbereichs hinaus, damit auch dort nichts uebersehen wird. */
#ifndef ACE_SEARCH_X_MM
#define ACE_SEARCH_X_MM 190.0
#endif
#ifndef ACE_SEARCH_Y_MM
#define ACE_SEARCH_Y_MM 170.0
#endif

/* Durchgaenge ohne Objekt, bevor die Suche anlaeuft. Einer dauert schon
 * rund zweieinhalb Sekunden - genug Nachsicht fuer eine kurze Verdeckung. */
#ifndef ACE_SEARCH_AFTER_LOST
#define ACE_SEARCH_AFTER_LOST 2
#endif

/* Treffer in Folge, bevor die Suchfahrt abbricht. Ein einzelnes
 * Rauschpixel soll sie nicht beenden. */
#ifndef ACE_SEARCH_CONFIRM
#define ACE_SEARCH_CONFIRM 3
#endif

/* Fahrweg, unter dem eine Fahrt als wirkungslos gilt. Bleibt weniger uebrig,
 * nachdem kin_clamp das Ziel auf die Fahrgrenze zurueckgeschnitten hat,
 * steht die Plattform am Anschlag. */
#ifndef ACE_PINNED_MM
#define ACE_PINNED_MM 0.5
#endif

/* Zuege am Anschlag, bevor der Regler aufhoert, wirkungslose Fahrbefehle
 * abzusetzen. */
#ifndef ACE_PINNED_LIMIT
#define ACE_PINNED_LIMIT 3
#endif

/* Glaettung der laufenden Versatzmessung. Klein genug, dass einzelne
 * Ausreisser nicht durchschlagen, gross genug, dass ein sich drehendes
 * Kabel innerhalb weniger Zuege ankommt. */
#ifndef ACE_TILT_TRACK_BLEND
#define ACE_TILT_TRACK_BLEND 0.2
#endif

#endif
