#include "visual.h"

#include "geometry.h"

#include <math.h>
#include <stddef.h>

/* Gewicht der Startvermutung, in mm^2. Eine Probefahrt ueber 30 mm bringt
 * 900 mm^2 mit, die Vermutung ist danach also fast bedeutungslos - aber sie
 * verhindert eine Division durch Null vor der ersten Messung. */
#define VIS_PRIOR_MM2 25.0

void vis_init(VisualModel *m, double px_per_mm, double angle_deg, double lambda) {
    if (!m) return;

    double rad = angle_deg * ACE_PI / 180.0;
    if (px_per_mm <= 0.0) px_per_mm = 1.0;

    m->a_re = px_per_mm * cos(rad);
    m->a_im = px_per_mm * sin(rad);

    m->s_den    = VIS_PRIOR_MM2;
    m->s_num_re = m->a_re * VIS_PRIOR_MM2;
    m->s_num_im = m->a_im * VIS_PRIOR_MM2;

    m->lambda           = (lambda > 0.0 && lambda <= 1.0) ? lambda : 1.0;
    m->samples          = 0;
    m->rejected         = 0;
    m->last_residual_px = 0.0;
    m->last_expected_px = 0.0;
    m->last_accepted    = 0;

    m->ref_distance_mm  = 0.0;
    m->drift_sq_sum     = 0.0;
    m->drift_worst_mm   = 0.0;
    m->drift_count      = 0;
    m->angle_rate_dps   = 0.0;
    m->last_angle_deg   = vis_angle_deg(m);
    m->last_update_s    = -1.0;
}

void vis_set_camera_distance(VisualModel *m, double distance_mm) {
    if (!m || distance_mm <= 1.0) return;

    if (m->ref_distance_mm > 1.0) {
        /* Pixel je mm gehen mit Brennweite/Abstand. Wird der Abstand
         * groesser, wird der Massstab kleiner. */
        double f = m->ref_distance_mm / distance_mm;
        if (f > 0.5 && f < 2.0) {          /* Unsinn nicht mitnehmen */
            m->a_re     *= f;
            m->a_im     *= f;
            m->s_num_re *= f;
            m->s_num_im *= f;
        }
    }
    m->ref_distance_mm = distance_mm;
}

double vis_camera_distance(const VisualModel *m) {
    return m ? m->ref_distance_mm : 0.0;
}

double vis_drift_rms_mm(const VisualModel *m) {
    if (!m || m->drift_count <= 0) return 0.0;
    return sqrt(m->drift_sq_sum / (double)m->drift_count);
}

double vis_drift_worst_mm(const VisualModel *m) {
    return m ? m->drift_worst_mm : 0.0;
}

double vis_angle_rate_dps(const VisualModel *m) {
    return m ? m->angle_rate_dps : 0.0;
}

void vis_required_move(const VisualModel *m, double err_u, double err_v,
                       double *dx_mm, double *dy_mm) {
    if (!m) return;

    double den = m->a_re * m->a_re + m->a_im * m->a_im;
    if (den < 1e-12) {
        if (dx_mm) *dx_mm = 0.0;
        if (dy_mm) *dy_mm = 0.0;
        return;
    }

    /* Bild ist linkshaendig, also v spiegeln. Dann ist der Fahrweg E / a,
     * ausgeschrieben E * conj(a) / |a|^2. */
    double e_re = err_u;
    double e_im = -err_v;

    if (dx_mm) *dx_mm = (e_re * m->a_re + e_im * m->a_im) / den;
    if (dy_mm) *dy_mm = (e_im * m->a_re - e_re * m->a_im) / den;
}

void vis_predict(const VisualModel *m, double dx_mm, double dy_mm,
                 double *du_drop, double *dv_drop) {
    if (!m) return;

    /* Abnahme des Bildfehlers = a * Fahrweg. */
    double d_re = m->a_re * dx_mm - m->a_im * dy_mm;
    double d_im = m->a_im * dx_mm + m->a_re * dy_mm;

    if (du_drop) *du_drop =  d_re;
    if (dv_drop) *dv_drop = -d_im;
}

int vis_update(VisualModel *m, double dx_mm, double dy_mm,
               double du_drop, double dv_drop,
               double gate_rel, double gate_px) {
    return vis_update_at(m, dx_mm, dy_mm, du_drop, dv_drop,
                         gate_rel, gate_px, -1.0);
}

int vis_observe(VisualModel *m, double dx_mm, double dy_mm,
                double eu_before, double ev_before,
                double eu_after,  double ev_after,
                double gate_rel, double gate_px, double now_s) {
    if (!m) return 0;

    /* Hier stand einmal eine Rueckdrehung um die geschaetzte Drehrate, nach
     * E_vorher - exp(-i*d)*E_nachher. Sie ist wieder heraus, weil sie den
     * Regelkreis nachweislich zerlegt: die Rate wird aus dem geschaetzten
     * Winkel abgeleitet und korrigiert dann genau die Messungen, aus denen
     * dieser Winkel entsteht. Diese Mitkopplung laeuft ohne unabhaengigen
     * Anker davon - im Versuch lief der Bildfehler von 426 auf 692 px
     * hoch, bei einer Rate mit falschem Vorzeichen.
     *
     * Was gegen das ziehende Kabel bleibt: laufend messen statt einmal je
     * Zug, ein gedaempfter Regler mit Reserve bis 72 Grad Winkelfehler, und
     * der Waechter, der bei ausbleibendem Fortschritt neu lernt. Die
     * Drehrate wird weiter gefuehrt, aber nur als Anzeige. */
    return vis_update_at(m, dx_mm, dy_mm,
                         eu_before - eu_after, ev_before - ev_after,
                         gate_rel, gate_px, now_s);
}

int vis_update_at(VisualModel *m, double dx_mm, double dy_mm,
                  double du_drop, double dv_drop,
                  double gate_rel, double gate_px, double now_s) {
    if (!m) return 0;

    double d2 = dx_mm * dx_mm + dy_mm * dy_mm;
    if (d2 < 1.0) {
        /* Unter einem Millimeter Fahrweg ist das Verhaeltnis von Messrauschen
         * zu Signal zu schlecht, daraus laesst sich nichts lernen. */
        m->last_accepted = 0;
        return 0;
    }

    double pu, pv;
    vis_predict(m, dx_mm, dy_mm, &pu, &pv);

    double ru  = du_drop - pu;
    double rv  = dv_drop - pv;
    double res = sqrt(ru * ru + rv * rv);
    double exp_px = sqrt(pu * pu + pv * pv);

    m->last_residual_px = res;
    m->last_expected_px = exp_px;

    /* Das Residuum in mm ist der Unterschied zwischen dem Weg, den die
     * Koppelnavigation behauptet, und dem, den das Bild belegt. Ueber viele
     * Messungen ist seine Streuung ein Mass fuer das Zufaellige in der
     * Mechanik - ein gleichbleibender Verlust steckt dagegen laengst im
     * geschaetzten Massstab und taucht hier nicht auf. */
    double scale = vis_scale_px_per_mm(m);
    if (scale > 1e-6) {
        double drift_mm = res / scale;
        m->drift_sq_sum += drift_mm * drift_mm;
        m->drift_count++;
        if (drift_mm > m->drift_worst_mm) m->drift_worst_mm = drift_mm;
    }

    if (res > gate_px + gate_rel * exp_px) {
        m->last_accepted = 0;
        m->rejected++;
        return 0;
    }

    /* conj(d) * Beobachtung, Beobachtung wieder mit gespiegeltem v. */
    double o_re = du_drop;
    double o_im = -dv_drop;

    double n_re = dx_mm * o_re + dy_mm * o_im;
    double n_im = dx_mm * o_im - dy_mm * o_re;

    m->s_num_re = m->lambda * m->s_num_re + n_re;
    m->s_num_im = m->lambda * m->s_num_im + n_im;
    m->s_den    = m->lambda * m->s_den    + d2;

    if (m->s_den > 1e-9) {
        m->a_re = m->s_num_re / m->s_den;
        m->a_im = m->s_num_im / m->s_den;
    }

    /* Drehgeschwindigkeit des Kamerawinkels. Zieht das HDMI-Kabel waehrend
     * der Fahrt, steht sie deutlich ueber null - und dann ist die Annahme
     * eines waehrend eines Zuges festen Winkels nicht mehr haltbar. */
    double ang = vis_angle_deg(m);
    if (now_s >= 0.0 && m->last_update_s >= 0.0) {
        double dt = now_s - m->last_update_s;
        if (dt > 0.05) {
            double da = ang - m->last_angle_deg;
            while (da >  180.0) da -= 360.0;
            while (da < -180.0) da += 360.0;

            double rate = da / dt;
            /* Geglaettet, sonst dominiert das Messrauschen. */
            m->angle_rate_dps = 0.7 * m->angle_rate_dps + 0.3 * rate;
        }
    }
    if (now_s >= 0.0) m->last_update_s = now_s;
    m->last_angle_deg = ang;

    m->samples++;
    m->last_accepted = 1;
    return 1;
}

void vis_relearn_begin(VisualModel *m) {
    if (!m) return;

    m->s_den    = VIS_PRIOR_MM2;
    m->s_num_re = m->a_re * VIS_PRIOR_MM2;
    m->s_num_im = m->a_im * VIS_PRIOR_MM2;
}

double vis_scale_px_per_mm(const VisualModel *m) {
    if (!m) return 0.0;
    return sqrt(m->a_re * m->a_re + m->a_im * m->a_im);
}

double vis_angle_deg(const VisualModel *m) {
    if (!m) return 0.0;
    return atan2(m->a_im, m->a_re) * 180.0 / ACE_PI;
}

double vis_view_width_mm(const VisualModel *m, int frame_width) {
    double s = vis_scale_px_per_mm(m);
    if (s < 1e-9 || frame_width <= 0) return 0.0;
    return (double)frame_width / s;
}
