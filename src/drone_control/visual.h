#ifndef ACE_DRONE_CONTROL_VISUAL_H
#define ACE_DRONE_CONTROL_VISUAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Verbindung zwischen Fahrweg in der Flaeche und Bildverschiebung.
 *
 * Die Kamera haengt an der Plattform und schaut nach unten. Ein Fahrweg
 * (dx, dy) in mm verschiebt das Objekt im Bild um (du, dv) in Pixeln. Wenn
 * sich die Kamera dreht - und das tut sie hier, weil oben ein HDMI-Kabel
 * zieht -, ist das keine feste Vorzeichenfrage mehr, sondern eine Drehung.
 *
 * Bildkoordinaten sind linkshaendig (v zeigt nach unten), Feldkoordinaten
 * rechtshaendig. Spiegelt man v, wird aus dem Zusammenhang eine reine
 * Drehstreckung, und die laesst sich als eine einzige komplexe Zahl
 * schreiben:
 *
 *     E = a * (Objekt - Kamera),   a = Massstab * exp(i * Kamerawinkel)
 *
 * Damit hat das Modell nur zwei Parameter statt der vier einer allgemeinen
 * 2x2-Matrix. Es ist damit deutlich robuster zu schaetzen, und eine
 * einzige Probefahrt reicht, weil jede Messung zwei Gleichungen liefert.
 * Der noetige Fahrweg ist schlicht E / a.
 */
typedef struct {
    double a_re, a_im;      /* Pixel je mm, als Drehstreckung              */

    double s_num_re;        /* Schaetzer: Zaehler sum(conj(d) * Beobachtung) */
    double s_num_im;
    double s_den;           /* Nenner sum(|d|^2)                           */
    double lambda;          /* Vergessensfaktor, damit Drehungen ankommen  */

    int    samples;
    int    rejected;
    double last_residual_px;
    double last_expected_px;
    int    last_accepted;

    /* Abstand Kamera zur Flaeche, zu dem der Massstab gehoert. Die Plattform
     * haengt, also aendert sich dieser Abstand mit ihrer Hoehe, und der
     * Massstab geht mit 1/Abstand. */
    double ref_distance_mm;

    /* Streuung zwischen gefahrenem und im Bild gemessenem Weg, in mm.
     *
     * Ein gleichbleibender Verlust ist hier nicht sichtbar: der Schaetzer
     * zieht ihn in den Massstab, und der Regelkreis merkt nichts davon.
     * Sichtbar ist allein das Zufaellige - also genau das, was eine
     * rutschende Winde erzeugt. */
    double drift_sq_sum;
    double drift_worst_mm;
    long   drift_count;

    /* Drehgeschwindigkeit des Kamerawinkels in Grad je Sekunde. Zieht das
     * Kabel waehrend der Fahrt, steht sie deutlich ueber null. */
    double angle_rate_dps;
    double last_angle_deg;
    double last_update_s;
} VisualModel;

/* px_per_mm aus ACE_VIEW_WIDTH_MM und der Bildbreite, angle_deg als
 * Startvermutung fuer die Kameradrehung (0 = Bild achsparallel zum Feld). */
void vis_init(VisualModel *m, double px_per_mm, double angle_deg, double lambda);

/* Fahrweg, der das Objekt in die Bildmitte holt. err_u/err_v ist die Lage
 * des Objekts relativ zur Bildmitte, in Bildkoordinaten. */
void vis_required_move(const VisualModel *m, double err_u, double err_v,
                       double *dx_mm, double *dy_mm);

/* Erwartete Abnahme des Bildfehlers fuer einen Fahrweg. */
void vis_predict(const VisualModel *m, double dx_mm, double dy_mm,
                 double *du_drop, double *dv_drop);

/* Eine Beobachtung einarbeiten: gefahrener Weg gegen tatsaechliche Abnahme
 * des Bildfehlers. Weicht sie zu stark von der Vorhersage ab, wird sie
 * verworfen und 0 gemeldet - das ist dann kein Drehen der Kamera, sondern
 * ein bewegtes Objekt oder eine durchrutschende Winde.
 * Rueckgabe: 1 uebernommen, 0 verworfen. */
int vis_update(VisualModel *m, double dx_mm, double dy_mm,
               double du_drop, double dv_drop,
               double gate_rel, double gate_px);

/* Wie vis_update, aber mit Zeitstempel in Sekunden: daraus wird die
 * Drehgeschwindigkeit des Kamerawinkels gebildet und die Streuung zwischen
 * gefahrenem und gemessenem Weg fortgeschrieben. */
int vis_update_at(VisualModel *m, double dx_mm, double dy_mm,
                  double du_drop, double dv_drop,
                  double gate_rel, double gate_px, double now_s);

/* Eine Beobachtung mit absoluten Bildfehlern statt deren Differenz.
 *
 * Dreht sich die Kamera waehrend der Messung um d, so gilt
 *
 *     E_nachher = exp(i*d) * (E_vorher - a * Weg)
 *
 * Der Stoerterm ist also proportional zum Bildfehler selbst und waechst mit
 * der Messdauer - genau wie das Nutzsignal. Kuerzere Strecken verbessern
 * das Verhaeltnis darum nicht; sie liefern aber mehr Messungen und damit
 * eine belastbare Streuung. Ein Herausrechnen von d aus der geschaetzten
 * Drehrate wurde versucht und wieder verworfen, siehe visual.c. */
int vis_observe(VisualModel *m, double dx_mm, double dy_mm,
                double eu_before, double ev_before,
                double eu_after,  double ev_after,
                double gate_rel, double gate_px, double now_s);

/* Abstand Kamera zur Flaeche melden. Aendert er sich, wird der Massstab
 * mit dem Verhaeltnis der Abstaende nachgezogen, statt darauf zu warten,
 * dass der Schaetzer es ueber mehrere Zuege nachholt. */
void   vis_set_camera_distance(VisualModel *m, double distance_mm);
double vis_camera_distance(const VisualModel *m);

/* Streuung Koppelnavigation gegen Bild, in mm. */
double vis_drift_rms_mm(const VisualModel *m);
double vis_drift_worst_mm(const VisualModel *m);
double vis_angle_rate_dps(const VisualModel *m);

/* Kamerawinkel von aussen setzen, gemessen statt geschaetzt.
 *
 * Das ist der Anker, der der reinen Bewegungsmessung fehlt: eine Bodenlinie
 * mit bekannter Feldrichtung gibt die Drehung aus einem einzigen Bild. Der
 * Massstab bleibt dabei unangetastet. */
void vis_set_angle_deg(VisualModel *m, double angle_deg);

/* Massstab von aussen setzen, etwa aus der Strichteilung einer Bodenlinie.
 * Die Drehung bleibt unangetastet. */
void vis_set_scale(VisualModel *m, double px_per_mm);

/* Vor einer Lernphase aufrufen. Die bisherige Messhistorie wird auf das
 * Gewicht der Startvermutung eingedampft, der aktuelle Schaetzwert bleibt
 * als Ausgangspunkt stehen. Ohne das wuerde die alte Historie die frischen
 * Probefahrten ueberstimmen und die Schaetzung bliebe auf halbem Weg
 * zwischen altem und neuem Kamerawinkel haengen. */
void vis_relearn_begin(VisualModel *m);

double vis_scale_px_per_mm(const VisualModel *m);
double vis_angle_deg(const VisualModel *m);
double vis_view_width_mm(const VisualModel *m, int frame_width);

#ifdef __cplusplus
}
#endif
#endif
