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
