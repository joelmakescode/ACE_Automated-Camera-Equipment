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
