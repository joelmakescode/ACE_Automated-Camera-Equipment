#define _POSIX_C_SOURCE 200809L

#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <unistd.h>

#include "ball_detector.h"
#include "figure.h"
#include "floorref.h"
#include "geometry.h"
#include "kinematics.h"
#include "motion.h"
#include "path.h"
#include "pins.h"
#include "stepper.h"
#include "visual.h"

static volatile sig_atomic_t g_abort = 0;

static void on_signal(int sig) {
    (void)sig;
    g_abort = 1;
}

static void sleep_ms(long ms) {
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/* --------------------------------------------------------- Bedienseite */

/* Bewusst schlicht: ein Bild, ein paar Knoepfe, eine Statuszeile. Kein
 * Framework, keine Abhaengigkeiten - der Pi soll nichts nachladen muessen.
 * Der Platzhalter S wird im Browser durch die gewaehlte Schrittweite
 * ersetzt, damit die Tippfahrten nicht vier Varianten brauchen. */
static const char ACE_PAGE[] =
"<!DOCTYPE html><html lang=\"de\"><head><meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>ACE</title><style>"
"body{font-family:system-ui,sans-serif;background:#111;color:#ddd;margin:0;padding:10px}"
"img{width:100%;max-width:860px;display:block;background:#000;border:1px solid #333}"
"button{font-size:15px;padding:9px 13px;margin:2px;background:#2a2a2a;color:#eee;"
"border:1px solid #555;border-radius:4px;cursor:pointer}"
"button:active{background:#444}"
"fieldset{border:1px solid #333;margin:8px 0;padding:8px}"
"legend{color:#999;font-size:13px}"
"pre{font-size:13px;line-height:1.4;margin:0;white-space:pre-wrap}"
"select{font-size:15px;padding:6px;background:#2a2a2a;color:#eee;border:1px solid #555}"
"</style></head><body>"
"<img src=\"/stream.mjpg\" alt=\"Kamera\">"
"<fieldset><legend>Kamera bewegen</legend>"
"<div><button onclick=\"c('jog=0,S')\">hoch</button></div>"
"<div><button onclick=\"c('jog=-S,0')\">links</button>"
"<button onclick=\"c('jog=S,0')\">rechts</button></div>"
"<div><button onclick=\"c('jog=0,-S')\">runter</button></div>"
"<div style=\"margin-top:6px\">Schrittweite "
"<select id=\"s\"><option>1</option><option>5</option>"
"<option selected>10</option><option>25</option></select> mm"
"<button onclick=\"c('center')\">zur Mitte</button>"
"<button onclick=\"c('stop')\">Halt</button></div>"
"</fieldset>"
"<fieldset><legend>Verfolgung</legend>"
"<button onclick=\"c('track=1')\">starten</button>"
"<button onclick=\"c('track=0')\">anhalten</button>"
"<button onclick=\"c('learn')\">neu lernen</button>"
"</fieldset>"
"<fieldset><legend>Status</legend><pre id=\"st\">wird geladen ...</pre></fieldset>"
"<script>"
"function c(x){var s=document.getElementById('s').value;"
"fetch('/cmd?'+x.split('S').join(s));}"
"function u(){fetch('/status').then(function(r){return r.text();})"
".then(function(t){document.getElementById('st').textContent=t;})"
".catch(function(){});}"
"setInterval(u,1000);u();"
"</script></body></html>";

/* ------------------------------------------------------------- Zustand */

typedef struct {
    BallDetector *cam;
    HsvRange      range;
    int           width, height;

    VisualModel   vis;
    unsigned int  delay_us;
    double        gain;
    double        tolerance_px;
    double        max_step_mm;
    double        probe_mm;
    long          settle_ms;
    int           quiet;
    int           stream_on;

    int    weak;                /* Winde, die Schritte verliert, -1 = keine */
    double slip_total_mm;       /* seit dem letzten Referenzieren           */
    int    slip_events;
    double slip_worst_mm;

    double height_min_mm;
    double height_max_mm;
    int    height_warned;

    /* Bodenlinien als absolute Referenz. */
    int       use_lines;
    FloorLine lines[2];
    long      fix_angle;        /* Bilder mit Winkelfix */
    long      fix_position;     /* Bilder mit Lagefix   */
    long      fix_scale;
    double    last_tilt_deg;
    int       tilt_warned;
    double    last_fix_err_mm;  /* Lagefix gegen Koppelnavigation */
    double    fix_err_worst_mm;

    /* Versatz der Bildmitte durch das Kippen der Kamera.
     *
     * Steht die Kamera schief, trifft die Bildmitte nicht den Punkt unter
     * der Kamera. floor_solve meldet dann nicht die Lage der Plattform,
     * sondern die des getroffenen Punktes. Beim Start ist die Lage der
     * Plattform aber bekannt - sie steht von Hand auf dem Kreuzungspunkt -,
     * und die Differenz ist genau dieser Versatz. Einmal gemessen, danach
     * abgezogen. Er enthaelt zugleich den Stellfehler von Hand, was richtig
     * ist: der Nullpunkt ist da, wo du die Plattform hingestellt hast. */
    int       bias_set;
    double    bias_x_mm, bias_y_mm;
    double    bias_tilt_deg;    /* Kippen, als der Versatz gemessen wurde */
    int       bias_stale;       /* Kippen hat sich seither geaendert      */
    int       fix_pos_on;   /* Linien als Lagegrundlage statt Koppelnav. */

    long   cycles;
    long   observations;        /* Messungen waehrend der Fahrten */
    int    relearns;
    int    lost;
    int    drift_warned;
    int    cable_warned;

    /* Betriebsart. Mit Bedienseite wartet das Programm, statt sofort zu
     * verfolgen - sonst liefe es schon, bevor man den Knopf sieht. */
    int    tracking;            /* 0 = wartet, 1 = verfolgt */
    int    learned;             /* Bildmodell schon eingemessen? */
    double prev_err_px;         /* fuer den Waechter, ueber Zuege hinweg */
    int    stall;
    int    lost_run;

    /* Suchfahrt. */
    int    search_on;
    int    search_index;
    int    search_laps;
    int    searches_hit;
    int    pinned;
} Tracker;

/* Lage des Objekts relativ zur Bildmitte, gemittelt ueber mehrere Treffer.
 * Nebenbei wird dabei die Kamerapipe leergelesen - wird waehrend einer
 * Fahrt nicht gelesen, staut sich rpicam-vid auf und das naechste Bild ist
 * Sekunden alt. */
static int measure(Tracker *t, double *err_u, double *err_v, double *radius) {
    double sum_u = 0.0, sum_v = 0.0, sum_r = 0.0;
    int    hits  = 0;

    for (int f = 0; f < ACE_TRACK_SAMPLE_FRAMES && !g_abort; f++) {
        DetectionResult r;
        if (bd_detect(t->cam, &t->range, &r) != 0) return -1;

        if (t->stream_on) bd_stream_push(t->cam, &r);

        if (r.found) {
            sum_u += r.x - t->width  / 2.0;
            sum_v += r.y - t->height / 2.0;
            sum_r += r.radius;
            if (++hits >= ACE_TRACK_SAMPLES) break;
        }
    }

    if (hits < ACE_TRACK_SAMPLES) return 1;      /* Objekt nicht sicher da */

    if (err_u)  *err_u  = sum_u / hits;
    if (err_v)  *err_v  = sum_v / hits;
    if (radius) *radius = sum_r / hits;
    return 0;
}

static void read_state(double *x_mm, double *y_mm, long steps[ACE_MOTOR_COUNT]) {
    motion_positions(steps);
    kin_position(steps, x_mm, y_mm);
}

/* Die Hoehe ist keine Konstante: die Plattform haengt, und ihre Hoehe folgt
 * aus den Seillaengen. Fuer x und y ist das folgenlos, weil sich z in der
 * Paarformel herauskuerzt - fuer die Planung nicht, darum wird sie
 * mitgefuehrt und hier ueberwacht. */
static double read_height(Tracker *t) {
    long   steps[ACE_MOTOR_COUNT];
    double x, y, h;

    motion_positions(steps);
    kin_pose(steps, &x, &y, &h);

    if (h < t->height_min_mm) t->height_min_mm = h;
    if (h > t->height_max_mm) t->height_max_mm = h;

    if (!t->height_warned &&
        fabs(h - ACE_HOVER_HEIGHT_MM) > ACE_HEIGHT_DRIFT_WARN_MM) {
        t->height_warned = 1;
        fprintf(stderr,
                "\nDie Plattform haengt auf %.1f mm statt %.0f mm unter den "
                "Ankern.\nGeplant wird auf der gemessenen Hoehe, die Fahrt "
                "bleibt also waagerecht.\nDie Ursache bleibt: Seildehnung, "
                "Schlupf oder falscher Wickeldurchmesser.\n",
                h, ACE_HOVER_HEIGHT_MM);
    }
    return h;
}

static double now_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

/* Eine einzelne Beobachtung waehrend der Fahrt einarbeiten.
 *
 * Gemessen wird nicht einmal je Zug, sondern laufend ueber kurze
 * Basislinien. Das ist gegen das ziehende HDMI-Kabel gerichtet: dreht sich
 * die Kamera um einen Winkel, verschiebt das die Objektlage um die
 * Bildmitte, und dieser Stoerterm waechst mit dem aktuellen Bildfehler.
 * Ueber einen ganzen Zug hinweg faengt man ihn voll ein, ueber ein paar
 * Millimeter nur einen Bruchteil davon. */
static void feed_observation(Tracker *t, double from_x, double from_y,
                             double eu0, double ev0,
                             double to_x, double to_y,
                             double eu1, double ev1) {
    vis_observe(&t->vis, to_x - from_x, to_y - from_y,
                eu0, ev0, eu1, ev1,
                ACE_VIS_GATE_REL, ACE_VIS_GATE_PX, now_seconds());
}

/* Bodenlinien auswerten und daraus setzen, was sie hergeben.
 *
 * Das ist der unabhaengige Anker: Winkel und Massstab werden hier gemessen
 * statt geschaetzt, und die Lage wird nicht koppelnavigiert, sondern
 * abgelesen. Damit faellt auch der Schlupf weg - nicht abgefedert, sondern
 * ueberschrieben. Arbeitet auf dem Bild, das bd_detect zuletzt geholt hat. */
static void read_floor(Tracker *t, int calibrating) {
    if (!t->use_lines) return;

    LineResult seen[2];
    for (int i = 0; i < 2; i++) {
        if (bd_detect_line(t->cam, &t->lines[i].colour, &seen[i]) != 0) {
            seen[i].found = false;
        }
    }

    FloorFix fix;
    floor_solve(t->lines, seen, vis_scale_px_per_mm(&t->vis), &fix);
    if (fix.lines_seen == 0) return;

    if (fix.have_scale) {
        vis_set_scale(&t->vis, fix.px_per_mm);
        t->fix_scale++;
    }
    if (fix.have_angle) {
        vis_set_angle_deg(&t->vis, fix.camera_angle_deg);
        t->fix_angle++;
    }

    if (fix.lines_seen == 2) {
        t->last_tilt_deg = fix.included_error_deg;
        if (!t->tilt_warned && fabs(fix.included_error_deg) > ACE_TILT_WARN_DEG) {
            t->tilt_warned = 1;
            fprintf(stderr,
                    "\nDer eingeschlossene Winkel der Bodenlinien weicht um "
                    "%+.2f Grad ab.\nEine Drehstreckung erhaelt Winkel, eine "
                    "Perspektive nicht: die Kamera steht\nschief. Das Bild "
                    "bleibt brauchbar, die Lage verschiebt sich aber.\n",
                    fix.included_error_deg);
        }
    }

    /* Beim Start steht die Plattform von Hand auf dem Kreuzungspunkt, ihre
     * Lage ist also null. Was floor_solve trotzdem meldet, ist der Versatz
     * der Bildmitte durch das Kippen - hier gemessen, spaeter abgezogen.
     * Winkel und Massstab sind oben schon uebernommen worden, die gelten
     * auch waehrend der Kalibrierung. */
    if (calibrating) {
        if (fix.have_position) {
            t->bias_x_mm     = fix.x_mm;
            t->bias_y_mm     = fix.y_mm;
            t->bias_tilt_deg = fix.tilt_deg;
            t->bias_set      = 1;

            printf("Kippen der Kamera   %.1f Grad, Bildmitte zeigt %.0f mm "
                   "daneben (%+.0f, %+.0f)\n", fix.tilt_deg,
                   sqrt(fix.x_mm * fix.x_mm + fix.y_mm * fix.y_mm),
                   fix.x_mm, fix.y_mm);
            printf("                    gemessen und ab jetzt von jedem "
                   "Lagefix abgezogen\n");
        } else {
            printf("Versatz durch Kippen nicht messbar: dafuer muessen beide "
                   "Bodenlinien im\nBild sein, sichtbar ist nur %d. Der "
                   "Lagefix bleibt aus; Winkel und\nMassstab kommen "
                   "trotzdem.\n", fix.lines_seen);
        }
        return;
    }

    if (!fix.have_position) return;

    long   steps[ACE_MOTOR_COUNT];
    double x, y, h;
    motion_positions(steps);
    kin_pose(steps, &x, &y, &h);

    /* Versatz der Bildmitte laufend messen.
     *
     * Solange eine Winde rutschte, war das unmoeglich: die Lage aus der
     * Koppelnavigation war selbst fragwuerdig, und beide Groessen liessen
     * sich nicht trennen. Mit gesundem Antrieb ist sie vertrauenswuerdig,
     * und damit ist der Versatz schlicht die Differenz:
     *
     *     Versatz = Lage aus den Linien - Lage aus der Koppelnavigation
     *
     * Das ist genauer als jeder Umweg ueber den eingeschlossenen Winkel.
     * Der ist unterhalb von zehn Grad Kippen so unempfindlich, dass ein
     * Fuenftelgrad Messfehler schon zweistellige Millimeter im Versatz
     * bedeutet - als Aenderungsanzeige taugt er, als Messgeraet nicht.
     *
     * Leicht geglaettet, damit einzelne Ausreisser nicht durchschlagen. */
    double dx = fix.x_mm - x;
    double dy = fix.y_mm - y;

    if (!t->bias_set) {
        t->bias_x_mm = dx;
        t->bias_y_mm = dy;
        t->bias_set  = 1;
    } else {
        double a = ACE_TILT_TRACK_BLEND;
        t->bias_x_mm += a * (dx - t->bias_x_mm);
        t->bias_y_mm += a * (dy - t->bias_y_mm);
    }
    t->last_fix_err_mm = sqrt(dx * dx + dy * dy);
    if (t->last_fix_err_mm > t->fix_err_worst_mm) {
        t->fix_err_worst_mm = t->last_fix_err_mm;
    }
    t->fix_position++;

    /* Die Lage aus den Linien zu uebernehmen, waere jetzt ein Zirkel: der
     * Versatz wird ja gerade aus der Differenz zu ihr bestimmt. Eines von
     * beiden muss die Grundlage sein, und mit gesundem Antrieb ist das die
     * Koppelnavigation - sie plant jedes Teilstueck neu aus den echten
     * Zaehlerstaenden und driftet nicht. Wer die Linien trotzdem als
     * Grundlage will, schaltet es hier ein; dann friert der Versatz auf
     * seinem ersten Wert ein. */
    if (t->fix_pos_on) {
        double bx = fix.x_mm - t->bias_x_mm;
        double by = fix.y_mm - t->bias_y_mm;
        double blend = ACE_FIX_BLEND;
        kin_reset_at(x + blend * (bx - x), y + blend * (by - y), h, steps);
    }
}

/* Massstab an den aktuellen Kameraabstand koppeln. Die Plattform haengt,
 * ihre Hoehe ist keine Konstante, und Pixel je mm gehen mit 1/Abstand. */
static void sync_scale_to_height(Tracker *t) {
    double h = read_height(t);
    double camera_above_floor = ACE_RIG_HEIGHT_MM - h;

    if (camera_above_floor > 10.0) {
        vis_set_camera_distance(&t->vis, camera_above_floor);
    }
}

typedef enum { DRIVE_PLAIN, DRIVE_TRACK, DRIVE_SEARCH } DriveMode;

/* Fahrt abwarten und dabei weiter Bilder holen: das haelt die Kamerapipe
 * leer und liefert nebenbei die laufenden Messungen.
 *
 * start_err_px > 0 schaltet den vorzeitigen Ausstieg ein. Eine Fahrt ueber
 * den vollen Weg dauert knapp vier Sekunden; sie blind zu Ende zu fahren
 * heisst, vier Sekunden lang nicht auf das Bild zu reagieren. Fuer ein
 * stehendes Objekt ist das gleichgueltig, fuer ein wanderndes nicht.
 * Darum wird unterwegs abgebrochen, sobald das Bild es verlangt:
 *
 *   - Fehler unter die Toleranz gefallen: das Ziel ist erreicht, oft weil
 *     das Objekt uns entgegengekommen ist. Weiterfahren hiesse ueberfahren.
 *   - Fehler deutlich gewachsen: das Objekt ist in die andere Richtung
 *     gelaufen, der geplante Weg zeigt in die falsche Richtung.
 *
 * Rueckgabe: 0 durchgefahren, 1 Abbruch durch den Nutzer, 2 vorzeitig
 * beendet (neu planen bzw. Objekt gefunden), -1 Kamerafehler. */
static int drive_and_drain(Tracker *t, DriveMode mode, double start_err_px) {
    int    have_anchor = 0;
    double ax = 0.0, ay = 0.0, aeu = 0.0, aev = 0.0;
    int    early = 0;
    int    seen_run = 0;

    while (path_busy() && !g_abort) {
        DetectionResult seen;
        if (bd_detect(t->cam, &t->range, &seen) != 0) return -1;
        if (t->stream_on) bd_stream_push(t->cam, &seen);

        /* Waehrend der Suchfahrt zaehlt jedes Bild, nicht nur die
         * Wegpunkte - sonst rauschte die Plattform an einem Objekt vorbei,
         * das sie zwischendurch laengst im Bild hatte. Mehrere Treffer in
         * Folge, damit ein einzelnes Rauschpixel die Suche nicht abbricht. */
        if (mode == DRIVE_SEARCH) {
            seen_run = seen.found ? seen_run + 1 : 0;
            if (seen_run >= ACE_SEARCH_CONFIRM) {
                path_abort();
                early = 2;
                break;
            }
        }

        if (!seen.found) continue;

        long   steps[ACE_MOTOR_COUNT];
        double x, y;
        motion_positions(steps);
        kin_position(steps, &x, &y);

        double eu = seen.x - t->width  / 2.0;
        double ev = seen.y - t->height / 2.0;

        if (mode == DRIVE_TRACK && start_err_px > 0.0) {
            double err = sqrt(eu * eu + ev * ev);
            if (err <= t->tolerance_px ||
                err >  start_err_px * ACE_TRACK_ABORT_GROW) {
                path_abort();
                early = 2;
                break;
            }
        }

        if (!have_anchor) {
            ax = x; ay = y; aeu = eu; aev = ev;
            have_anchor = 1;
            continue;
        }

        double base = sqrt((x - ax) * (x - ax) + (y - ay) * (y - ay));
        if (base >= ACE_TRACK_BASELINE_MM) {
            feed_observation(t, ax, ay, aeu, aev, x, y, eu, ev);
            t->observations++;
            ax = x; ay = y; aeu = eu; aev = ev;
        }
    }
    if (g_abort) { path_abort(); return 1; }

    if (t->settle_ms > 0) {
        struct timespec t0;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        for (;;) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            long ms = (now.tv_sec - t0.tv_sec) * 1000L
                    + (now.tv_nsec - t0.tv_nsec) / 1000000L;
            if (ms >= t->settle_ms || g_abort) break;

            DetectionResult seen;
            if (bd_detect(t->cam, &t->range, &seen) != 0) return -1;
            if (t->stream_on) bd_stream_push(t->cam, &seen);
        }
    }
    return early;
}

/* ---------------------------------------------------- schwache Winde */

/* Ueberwacht die Selbstkonsistenz des Modells fuer die schwache Winde.
 *
 * Kein Schlupfmesser - siehe kin_length_residual. Ein echtes Durchrutschen
 * bleibt hier unsichtbar, weil der Schrittzaehler Kommandos zaehlt und die
 * vier Modellaengen per Konstruktion zusammenpassen. Der Wert schlaegt bei
 * Rundungsdrift oder verstellter Referenz an, nicht bei rutschender
 * Mechanik. Gegen die hilft im laufenden Betrieb allein der Bildregelkreis,
 * der ohnehin auf das Bild und nicht auf die Koppelnavigation regelt. */
static void check_slip(Tracker *t) {
    if (t->weak < 0) return;

    long steps[ACE_MOTOR_COUNT];
    motion_positions(steps);

    double slip = kin_length_residual(t->weak, steps);
    double mag  = fabs(slip);

    if (mag > t->slip_worst_mm) t->slip_worst_mm = mag;
    t->slip_total_mm = slip;

    if (mag < ACE_SLIP_WARN_MM) return;

    double x, y;
    kin_position(steps, &x, &y);

    if (mag >= ACE_SLIP_RECOVER_MM) {
        kin_reset_motor(t->weak, x, y, steps);
        t->slip_events++;
        fprintf(stderr,
                "\nWinde %d (%s) ist um %+.2f mm durchgerutscht, neu referenziert "
                "(%d. Mal).\n", t->weak, ACE_MOTOR_NAMES[t->weak], slip,
                t->slip_events);
    } else if (!t->quiet) {
        fprintf(stderr, "\nWinde %d (%s): %+.2f mm Schlupf.\n",
                t->weak, ACE_MOTOR_NAMES[t->weak], slip);
    }
}

/* ------------------------------------------------------------ Lernphase */

/* Vier feste Probefahrten, deren Bildantwort das Modell bestimmt. Der
 * Ausreisserfilter bleibt dabei offen: er soll bewegte Objekte abwehren,
 * wuerde hier aber genau die Messungen abweisen, die ein grob falsches
 * Modell geraderuecken. Das Objekt muss in dieser Phase stillliegen. */
static int learn(Tracker *t) {
    static const double dir[4][2] = { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } };

    double probe_dx[4], probe_dy[4], drop_u[4], drop_v[4];
    int    good = 0;

    vis_relearn_begin(&t->vis);

    for (int k = 0; k < 4 && !g_abort; k++) {
        double eu0, ev0, eu1, ev1;
        double x0, y0, x1, y1;
        long   steps[ACE_MOTOR_COUNT];

        int m = measure(t, &eu0, &ev0, NULL);
        if (m != 0) {
            fprintf(stderr, "Lernphase: Objekt nicht sichtbar. HSV-Bereich oder "
                            "Beleuchtung pruefen.\n");
            return -1;
        }
        read_state(&x0, &y0, steps);

        double tx = x0 + dir[k][0] * t->probe_mm;
        double ty = y0 + dir[k][1] * t->probe_mm;
        kin_clamp(&tx, &ty);

        if (path_start(tx, ty, t->delay_us) != 0) return -1;
        if (drive_and_drain(t, DRIVE_PLAIN, 0.0) != 0) return -1;

        m = measure(t, &eu1, &ev1, NULL);
        if (m != 0) {
            fprintf(stderr, "Lernphase: Objekt nach der Probefahrt verloren. "
                            "--probe-mm verkleinern.\n");
            return -1;
        }
        read_state(&x1, &y1, steps);

        /* Der tatsaechlich gefahrene Weg aus den vertrauenswuerdigen Winden,
         * nicht der kommandierte: so faelscht ein Schlupf das Bildmodell
         * nicht. */
        probe_dx[good] = x1 - x0;
        probe_dy[good] = y1 - y0;
        drop_u[good]   = eu0 - eu1;
        drop_v[good]   = ev0 - ev1;

        sync_scale_to_height(t);
        vis_update_at(&t->vis, probe_dx[good], probe_dy[good],
                      drop_u[good], drop_v[good], 1e9, 1e9, now_seconds());
        good++;

        check_slip(t);
    }

    if (g_abort) return 1;
    if (good < 2) {
        fprintf(stderr, "Lernphase: zu wenige brauchbare Probefahrten.\n");
        return -1;
    }

    /* Die Probefahrten gegen das fertige Modell zurueckrechnen. Passen sie
     * nicht zueinander, hat sich das Objekt bewegt oder eine Winde ist
     * gerutscht - dann taugt das Modell nichts. */
    double worst = 0.0;
    for (int k = 0; k < good; k++) {
        double pu, pv;
        vis_predict(&t->vis, probe_dx[k], probe_dy[k], &pu, &pv);
        double r = sqrt((drop_u[k] - pu) * (drop_u[k] - pu)
                      + (drop_v[k] - pv) * (drop_v[k] - pv));
        if (r > worst) worst = r;
    }

    printf("Bildmodell   %.3f px/mm, Kamera %+.1f Grad verdreht"
           "   (Sichtbreite %.0f mm)\n",
           vis_scale_px_per_mm(&t->vis), vis_angle_deg(&t->vis),
           vis_view_width_mm(&t->vis, t->width));
    printf("             groesste Abweichung der Probefahrten %.1f px\n", worst);

    if (worst > 0.25 * vis_scale_px_per_mm(&t->vis) * t->probe_mm) {
        printf("             Das ist viel. Lag das Objekt still? Eine Winde, die\n"
               "             waehrend der Probefahrt rutscht, sieht genauso aus.\n");
    }
    return 0;
}

/* ------------------------------------------------------------- Regelung */

/* Die Suchfahrt steht weiter unten, wird aber schon im Regelzug gebraucht. */
static int search_tick(Tracker *t);

/* Ein Regelzug: messen, entscheiden, fahren, nachmessen.
 *
 * Frueher war das eine Endlosschleife. Jetzt ist es ein einzelner Zug, weil
 * die Bedienseite zwischen den Zuegen zu Wort kommen muss - sonst liesse
 * sich die Verfolgung nicht anhalten, ohne das Programm zu beenden. Was
 * ueber Zuege hinweg gilt, steht darum im Tracker statt auf dem Stapel. */
static int track_cycle(Tracker *t) {
    double eu, ev, radius;

    int m = measure(t, &eu, &ev, &radius);
    if (m < 0) return -1;
    if (m > 0) {
        t->lost++;
        t->lost_run++;

        if (!t->search_on) {
            if (t->lost_run == 1) {
                printf("\nObjekt nicht im Bild, Plattform bleibt stehen.\n");
            }
            return 0;
        }

        /* Erst ein paar Durchgaenge abwarten. Eine kurze Verdeckung - eine
         * Hand, ein Schatten - soll die Plattform nicht gleich quer ueber
         * das Feld schicken. Ein Durchgang ohne Fund dauert schon rund
         * zweieinhalb Sekunden, weil measure so lange nach Treffern sucht. */
        if (t->lost_run < ACE_SEARCH_AFTER_LOST) return 0;

        if (t->lost_run == ACE_SEARCH_AFTER_LOST) {
            printf("\nObjekt nicht im Bild, Suchfahrt beginnt.\n");
        }
        return search_tick(t);
    }
    if (t->lost_run > 0) {
        printf("Objekt wieder da.\n");
        t->lost_run = 0;
    }

    double err = sqrt(eu * eu + ev * ev);
    t->cycles++;

    /* Erst die Hoehenkopplung, dann die Bodenlinien: die Linien setzen den
     * Massstab aus der Strichteilung und sollen das letzte Wort haben, weil
     * sie ihn messen statt ihn zu rechnen. */
    sync_scale_to_height(t);
    read_floor(t, 0);

    /* Die Streuung misst das Zufaellige in der Mechanik. Ein
     * gleichbleibender Verlust steckt im Massstab und stoert nicht. */
    if (!t->drift_warned && t->vis.drift_count > 20 &&
        vis_drift_rms_mm(&t->vis) > ACE_DRIFT_WARN_MM) {
        t->drift_warned = 1;
        fprintf(stderr,
                "\nKoppelnavigation und Bild weichen um %.1f mm (RMS) "
                "voneinander ab.\nDas ist der zufaellige Anteil - eine "
                "rutschende Winde sieht genau so aus.\nDer Regelkreis "
                "faengt es ab, die angezeigte Lage wird aber ungenau.\n",
                vis_drift_rms_mm(&t->vis));
    }
    if (!t->cable_warned &&
        fabs(vis_angle_rate_dps(&t->vis)) > ACE_ANGLE_RATE_WARN_DPS) {
        t->cable_warned = 1;
        fprintf(stderr,
                "\nDer Kamerawinkel wandert mit %.1f Grad/s. Das Kabel "
                "zieht waehrend der Fahrt.\nGemessen wird ohnehin laufend "
                "ueber kurze Strecken, die Schaetzung kommt also mit -\n"
                "aber eine Zugentlastung am HDMI-Kabel waere das Richtige.\n",
                vis_angle_rate_dps(&t->vis));
    }

    if (err <= t->tolerance_px) {
        double h = read_height(t);
        if (!t->quiet) {
            printf("\rzentriert  %5.1f px  r%4.0f  Kam %+5.1f  h%5.1f"
                   "  Schlupf %+5.2f    ",
                   err, radius, vis_angle_deg(&t->vis), h, t->slip_total_mm);
            fflush(stdout);
        }
        t->prev_err_px = err;
        check_slip(t);
        return 0;
    }

    /* Neu lernen nur, wenn der Winkel wirklich geschaetzt ist.
     *
     * Liefern die Bodenlinien ihn, ist er gemessen und kann nicht der Grund
     * fuer ausbleibenden Fortschritt sein - dann laeuft das Objekt schlicht
     * schneller davon, als die Plattform folgen kann. Eine Lernphase waere
     * dort sogar schaedlich: sie faehrt vier feste Probefahrten mit weit
     * offenem Ausreisserfilter und wuerde die Bewegung des Objekts als
     * Bildmodell lernen. */
    int angle_measured = (t->use_lines && t->fix_angle > 0);

    if (!angle_measured && t->prev_err_px > 0.0 &&
        err > t->prev_err_px * ACE_TRACK_STALL_RATIO) {
        if (++t->stall >= ACE_TRACK_STALL_LIMIT) {
            printf("\nBildfehler klingt nicht ab (%.0f -> %.0f px). "
                   "Kamera hat sich gedreht,\nBildmodell wird neu gelernt.\n",
                   t->prev_err_px, err);
            t->stall = 0;
            t->relearns++;
            if (learn(t) < 0) return -1;
            t->prev_err_px = -1.0;
            return 0;
        }
    } else {
        t->stall = 0;
    }

    double dx, dy;
    vis_required_move(&t->vis, eu, ev, &dx, &dy);
    dx *= t->gain;
    dy *= t->gain;

    double step = sqrt(dx * dx + dy * dy);
    if (step > t->max_step_mm) {
        dx *= t->max_step_mm / step;
        dy *= t->max_step_mm / step;
    }

    double x0, y0;
    long   steps[ACE_MOTOR_COUNT];
    read_state(&x0, &y0, steps);

    double tx = x0 + dx;
    double ty = y0 + dy;
    kin_clamp(&tx, &ty);

    /* Am Anschlag?
     *
     * Liegt das Objekt ausserhalb des Fahrbereichs, schneidet kin_clamp das
     * Ziel auf die Grenze zurueck, der kommandierte Weg wird null und der
     * Bildfehler bleibt stehen. Ohne diese Pruefung feuert der Regler
     * endlos Fahrbefehle ab, die nichts bewirken - das Zucken, das dabei
     * herauskommt, ist nur noch Rundungsrest. Naeher als an die Grenze
     * kommt die Plattform nicht heran; also dort stehenbleiben, weiter
     * zusehen und von selbst weitermachen, sobald es wieder geht. */
    double want_mm = sqrt(dx * dx + dy * dy);
    double real_mm = sqrt((tx - x0) * (tx - x0) + (ty - y0) * (ty - y0));

    if (want_mm > 1.0 && real_mm < ACE_PINNED_MM) {
        t->pinned++;
        if (t->pinned == ACE_PINNED_LIMIT) {
            printf("\nAnschlag bei %+.0f, %+.0f mm. Das Objekt liegt rund "
                   "%.0f mm ausserhalb\ndes Fahrbereichs von +/-%.0f x "
                   "+/-%.0f mm - naeher kommt die Plattform nicht.\n"
                   "Sie bleibt stehen und macht weiter, sobald es wieder "
                   "geht.\n",
                   x0, y0, err / (vis_scale_px_per_mm(&t->vis) > 0.01
                                  ? vis_scale_px_per_mm(&t->vis) : 1.0),
                   ACE_REACH_LIMIT_X_MM, ACE_REACH_LIMIT_Y_MM);
        }
        if (t->pinned >= ACE_PINNED_LIMIT) {
            if (!t->quiet) {
                printf("\rAnschlag %+6.1f,%+6.1f  Fehler %5.0f px  "
                       "Kam %+5.1f Grad      ", x0, y0, err,
                       vis_angle_deg(&t->vis));
                fflush(stdout);
            }
            t->prev_err_px = err;
            check_slip(t);
            return 0;
        }
    } else {
        if (t->pinned >= ACE_PINNED_LIMIT) {
            printf("\nWieder im Fahrbereich.\n");
        }
        t->pinned = 0;
    }

    /* Kurz gehalten, damit ein 80 Zeichen breites Terminal sie nicht
     * umbricht - sonst springt das Wagenrueckl auf die umgebrochene Zeile
     * und hinterlaesst Bruchstuecke. */
    if (!t->quiet) {
        printf("\rZug %-5ld %5.0f px -> %+6.1f,%+6.1f mm  @%+6.1f,%+6.1f"
               "  Kam %+5.1f    ",
               t->cycles, err, dx, dy, tx, ty, vis_angle_deg(&t->vis));
        fflush(stdout);
    }

    if (path_start(tx, ty, t->delay_us) != 0) return -1;
    int r = drive_and_drain(t, DRIVE_TRACK, err);
    if (r < 0) return -1;
    if (r == 1) return 0;       /* Ctrl-C, die aeussere Schleife bricht ab */
    /* r == 2 heisst nur: unterwegs abgebrochen, weil das Bild es verlangte.
     * Der naechste Zug plant aus der neuen Lage - die Teilfahrt ist eine
     * gueltige Beobachtung wie jede andere. */

    double x1, y1;
    read_state(&x1, &y1, steps);

    double eu1, ev1;
    if (measure(t, &eu1, &ev1, NULL) == 0) {
        vis_update(&t->vis, x1 - x0, y1 - y0, eu - eu1, ev - ev1,
                   ACE_VIS_GATE_REL, ACE_VIS_GATE_PX);
    }

    check_slip(t);
    t->prev_err_px = err;
    return 0;
}

/* ------------------------------------------------------------ Suchfahrt */

/* Wegpunkt der Suchfahrt: Schlangenlinie ueber den Fahrbereich.
 *
 * Das Sichtfeld ist mit 210 x 118 mm klein gegen die 440 x 340 mm, die die
 * Plattform erreicht - ein Blick von einer Stelle aus reicht also bei
 * weitem nicht. Drei Spalten mal vier Zeilen ist das kleinste Raster, bei
 * dem sich die Ausschnitte noch ueberlappen: 180 mm Spaltenabstand gegen
 * 210 mm Sichtbreite, 100 mm Zeilenabstand gegen 118 mm Sichthoehe. Die
 * Aussenpunkte liegen so, dass ihr halbes Sichtfeld ueber den Rand des
 * Fahrbereichs hinausreicht.
 *
 * Jede zweite Spalte wird rueckwaerts abgefahren, damit keine Leerfahrt
 * quer ueber das ganze Feld noetig wird. */
static void search_point(int idx, double *x_mm, double *y_mm) {
    const int cols = ACE_SEARCH_COLS;
    const int rows = ACE_SEARCH_ROWS;
    const int n    = cols * rows;

    idx = ((idx % n) + n) % n;

    int col = idx / rows;
    int row = idx % rows;
    if (col & 1) row = rows - 1 - row;      /* Schlangenlinie */

    double sx = (cols > 1) ? (2.0 * ACE_SEARCH_X_MM / (cols - 1)) : 0.0;
    double sy = (rows > 1) ? (2.0 * ACE_SEARCH_Y_MM / (rows - 1)) : 0.0;

    *x_mm = (cols > 1) ? (-ACE_SEARCH_X_MM + col * sx) : 0.0;
    *y_mm = (rows > 1) ? (-ACE_SEARCH_Y_MM + row * sy) : 0.0;
}

/* Einen Abschnitt der Suchfahrt abfahren. Bricht ab, sobald das Objekt
 * auftaucht - der naechste Regelzug findet es dann von selbst. */
static int search_tick(Tracker *t) {
    double sx, sy;
    search_point(t->search_index, &sx, &sy);
    kin_clamp(&sx, &sy);

    if (!t->quiet) {
        printf("\rSuchfahrt %2d/%-2d  ->  %+6.1f,%+6.1f mm            ",
               (t->search_index % (ACE_SEARCH_COLS * ACE_SEARCH_ROWS)) + 1,
               ACE_SEARCH_COLS * ACE_SEARCH_ROWS, sx, sy);
        fflush(stdout);
    }

    if (path_start(sx, sy, t->delay_us) != 0) return -1;

    int r = drive_and_drain(t, DRIVE_SEARCH, 0.0);
    if (r < 0) return -1;
    if (r == 1) return 0;                   /* Ctrl-C */

    if (r == 2) {
        printf("\nObjekt gefunden, Suchfahrt beendet.\n");
        t->searches_hit++;
        return 0;                           /* Wegpunkt beibehalten */
    }

    t->search_index++;
    if (t->search_index % (ACE_SEARCH_COLS * ACE_SEARCH_ROWS) == 0) {
        t->search_laps++;
        printf("\nSuchfahrt: Feld einmal abgesucht, nichts gefunden.\n");
    }
    return 0;
}

/* ----------------------------------------------------------- Bedienseite */

/* Wartezustand: Bilder weiterholen, damit der Stream lebt und die
 * Kamerapipe nicht volllaeuft, und die Bodenlinien weiter auswerten. */
static int idle_tick(Tracker *t) {
    for (int f = 0; f < 5 && !g_abort; f++) {
        DetectionResult seen;
        if (bd_detect(t->cam, &t->range, &seen) != 0) return -1;
        if (t->stream_on) bd_stream_push(t->cam, &seen);
    }
    sync_scale_to_height(t);
    read_floor(t, 0);
    return 0;
}

static void update_status(Tracker *t) {
    if (!t->stream_on) return;

    long   steps[ACE_MOTOR_COUNT];
    double x, y, h;
    motion_positions(steps);
    kin_pose(steps, &x, &y, &h);

    const char *mode =
        !t->tracking ? (t->learned ? "wartet" : "wartet, noch nicht eingemessen")
      : (t->lost_run >= ACE_SEARCH_AFTER_LOST && t->search_on) ? "Suchfahrt"
      : t->pinned >= ACE_PINNED_LIMIT ? "am Anschlag"

      : t->lost_run > 0 ? "Objekt verloren"
      : "verfolgt";

    char buf[900];
    snprintf(buf, sizeof(buf),
        "Betrieb      %s\n"
        "Lage         %+.1f, %+.1f mm      Hoehe %.1f mm\n"
        "Kamera       %+.1f Grad verdreht, %.2f px/mm\n"
        "Kippen       %+.2f Grad im eingeschlossenen Winkel%s\n"
        "Bodenlinien  Winkel %ld x   Massstab %ld x   Lage %ld x\n"
        "Zuege        %ld   ohne Objekt %d   neu gelernt %d\n"
        "Suchfahrt    %s   Punkt %d/%d   Durchlaeufe %d   Funde %d\n"
        "Kippversatz  %+.0f, %+.0f mm   Bildmitte gegen das Lot unter der Kamera\n"
        "Streuung     %.2f mm RMS   Kamera dreht %+.2f Grad/s\n",
        mode,
        x, y, h,
        vis_angle_deg(&t->vis), vis_scale_px_per_mm(&t->vis), t->last_tilt_deg,
        t->fix_pos_on ? "  (Linien als Lagegrundlage)" : "",
        t->fix_angle, t->fix_scale, t->fix_position,
        t->cycles, t->lost, t->relearns,
        t->search_on ? "ein" : "aus",
        (t->search_index % (ACE_SEARCH_COLS * ACE_SEARCH_ROWS)) + 1,
        ACE_SEARCH_COLS * ACE_SEARCH_ROWS, t->search_laps, t->searches_hit,
        t->bias_x_mm, t->bias_y_mm,
        vis_drift_rms_mm(&t->vis), vis_angle_rate_dps(&t->vis));

    bd_stream_set_status(buf);
}

/* Eine Fahrt von Hand, relativ zur jetzigen Lage. */
static int jog(Tracker *t, double dx, double dy) {
    long   steps[ACE_MOTOR_COUNT];
    double x, y;
    read_state(&x, &y, steps);

    double tx = x + dx;
    double ty = y + dy;
    kin_clamp(&tx, &ty);

    printf("\nHandfahrt %+.0f,%+.0f mm  ->  %+.1f,%+.1f\n", dx, dy, tx, ty);
    if (path_start(tx, ty, t->delay_us) != 0) return -1;

    int r = drive_and_drain(t, DRIVE_PLAIN, 0.0);
    return (r < 0) ? -1 : 0;
}

/* Befehle der Bedienseite abarbeiten. Sie kommen aus einem anderen Thread,
 * werden hier aber im Takt der Regelschleife ausgefuehrt - nebenlaeufig an
 * den Motoren zu drehen waere der sichere Weg ins Chaos. */
static int handle_commands(Tracker *t) {
    char cmd[128];

    while (bd_stream_take_command(cmd, (int)sizeof(cmd))) {
        double dx, dy;
        int    on;

        if (sscanf(cmd, "jog=%lf,%lf", &dx, &dy) == 2) {
            t->tracking = 0;            /* Handbetrieb schlaegt Verfolgung */
            path_abort();
            if (jog(t, dx, dy) < 0) return -1;

        } else if (sscanf(cmd, "track=%d", &on) == 1) {
            if (on) {
                if (!t->learned) {
                    printf("\nErst einmessen. Objekt bitte still liegen "
                           "lassen.\n");
                    if (learn(t) < 0) return -1;
                    t->learned = 1;
                }
                t->tracking    = 1;
                t->prev_err_px = -1.0;
                t->stall       = 0;
                printf("\nVerfolgung laeuft.\n");
            } else {
                t->tracking = 0;
                path_abort();
                printf("\nVerfolgung angehalten.\n");
            }

        } else if (strcmp(cmd, "center") == 0) {
            t->tracking = 0;
            path_abort();
            long   steps[ACE_MOTOR_COUNT];
            double x, y;
            read_state(&x, &y, steps);
            if (jog(t, -x, -y) < 0) return -1;

        } else if (strcmp(cmd, "stop") == 0) {
            t->tracking = 0;
            path_abort();
            printf("\nHalt.\n");

        } else if (strcmp(cmd, "learn") == 0) {
            t->tracking = 0;
            path_abort();
            printf("\nEinmessen. Objekt bitte still liegen lassen.\n");
            if (learn(t) < 0) return -1;
            t->learned = 1;
        }
        if (g_abort) break;
    }
    return 0;
}

/* Aeussere Schleife: Befehle, dann entweder ein Regelzug oder warten. */
static int run_loop(Tracker *t, long max_cycles) {
    while (!g_abort && (max_cycles <= 0 || t->cycles < max_cycles)) {
        if (handle_commands(t) < 0) return -1;
        if (g_abort) break;

        if (t->tracking) {
            if (track_cycle(t) < 0) return -1;
        } else {
            if (idle_tick(t) < 0) return -1;
        }
        update_status(t);
    }
    return 0;
}

/* --------------------------------------------------------------- Bericht */

static void print_setup(const Tracker *t) {
    printf("ACE Objektverfolgung mit Kamera\n\n");
    printf("Bild           %d x %d px, Mitte bei %d,%d\n",
           t->width, t->height, t->width / 2, t->height / 2);
    printf("Ziel           Objekt innerhalb %.0f px um die Bildmitte\n",
           t->tolerance_px);
    printf("Regler         Gain %.2f, hoechstens %.0f mm je Zug\n",
           t->gain, t->max_step_mm);
    printf("               stabil bis %.0f Grad Fehler im Kamerawinkel\n",
           acos(t->gain / 2.0 > 1.0 ? 1.0 : t->gain / 2.0) * 180.0 / ACE_PI);

    if (t->weak >= 0) {
        printf("Schwache Winde %d (%s): aus der Lageschaetzung genommen,\n",
               t->weak, ACE_MOTOR_NAMES[t->weak]);
        printf("               die Lage kommt aus den anderen drei.\n");
        printf("               Vorspannung %.1f mm, damit ihr Seil straff bleibt\n"
               "               und seltener rutscht.\n",
               (double)ACE_WEAK_PRELOAD_MM);
        printf("               Ein Durchrutschen selbst ist per Koppelnavigation\n"
               "               nicht messbar: der Schrittzaehler zaehlt Kommandos.\n"
               "               Aufgefangen wird es vom Bildregelkreis, der auf das\n"
               "               Bild regelt und nicht auf die gerechnete Lage.\n");

        double margin = figure_margin_without(t->weak, 0.0, 0.0);
        printf("Arbeitsraum    Faellt sie ganz aus, tragen drei Seile nur noch\n"
               "               ueber ihrem Dreieck. In der Mitte betraegt der\n"
               "               Abstand zu dessen Kante %+.0f mm.\n", margin);
        if (margin <= 1.0) {
            printf("               Das heisst: eine Haelfte der Flaeche haengt an\n"
                   "               dieser Winde. Faellt sie aus, sackt die Plattform\n"
                   "               dort weg. Die Vorspannung ist genau dagegen da.\n");
        }
    } else {
        printf("Schwache Winde keine, alle vier gehen in die Lage ein.\n");
    }
    printf("\n");
}

static void print_result(const Tracker *t) {
    printf("\n\nErgebnis\n");
    printf("  Zuege                        %ld\n", t->cycles);
    printf("  Bildmodell neu gelernt       %d x\n", t->relearns);
    printf("  Kamera zuletzt               %+.1f Grad, %.3f px/mm"
           "  (Sichtbreite %.0f mm)\n",
           vis_angle_deg(&t->vis), vis_scale_px_per_mm(&t->vis),
           vis_view_width_mm(&t->vis, t->width));
    printf("  Messungen uebernommen        %d, verworfen %d"
           "  (%ld waehrend der Fahrten)\n",
           t->vis.samples, t->vis.rejected, t->observations);
    printf("  Kamerawinkel wandert mit     %+.2f Grad/s%s\n",
           vis_angle_rate_dps(&t->vis),
           fabs(vis_angle_rate_dps(&t->vis)) > ACE_ANGLE_RATE_WARN_DPS
               ? "   (Kabel zieht)" : "");

    if (t->vis.drift_count > 0) {
        printf("  Koppelnavigation ./. Bild    %.2f mm RMS, schlechtestens %.2f mm\n",
               vis_drift_rms_mm(&t->vis), vis_drift_worst_mm(&t->vis));
        printf("                               aus %ld Messungen. Das ist der\n"
               "                               zufaellige Anteil; ein gleich-\n"
               "                               bleibender Verlust steckt im\n"
               "                               Massstab und stoert nicht.\n",
               t->vis.drift_count);
    }
    printf("  Bilder ohne Objekt           %d\n", t->lost);

    if (t->use_lines) {
        printf("\n  Bodenlinien\n");
        printf("    Winkel gemessen            %ld x\n", t->fix_angle);
        printf("    Massstab gemessen          %ld x\n", t->fix_scale);
        printf("    Lage gemessen              %ld x", t->fix_position);
        if (t->fix_position > 0) {
            printf("   (zuletzt %.1f mm neben der\n"
                   "                               Koppelnavigation, "
                   "schlechtestens %.1f mm)",
                   t->last_fix_err_mm, t->fix_err_worst_mm);
        }
        printf("\n");
        printf("    Kippen der Kamera          %+.2f Grad im eingeschlossenen "
               "Winkel%s\n", t->last_tilt_deg,
               fabs(t->last_tilt_deg) > ACE_TILT_WARN_DEG ? "  (schief)" : "");

        if (t->bias_set) {
            printf("    Versatz der Bildmitte      %+.0f, %+.0f mm"
                   "   (zuletzt gemessen, Betrag %.0f mm)\n",
                   t->bias_x_mm, t->bias_y_mm,
                   sqrt(t->bias_x_mm * t->bias_x_mm
                      + t->bias_y_mm * t->bias_y_mm));
            printf("    Um so weit zeigt die Bildmitte neben dem Lot unter "
                   "der Kamera.\n"
                   "    Die Plattform steht also um diesen Betrag neben dem "
                   "Objekt, wenn\n"
                   "    es in der Bildmitte steht - das ist das Kippen, kein "
                   "Regelfehler.\n");
            printf("    Grundlage der Lage         %s\n",
                   t->fix_pos_on ? "Bodenlinien (--fix-position)"
                                 : "Koppelnavigation");
        } else {
            printf("    Versatz der Bildmitte      nicht gemessen - dafuer "
                   "muessen beide\n"
                   "                               Linien gleichzeitig im "
                   "Bild sein.\n");
        }
        if (t->fix_angle == 0) {
            printf("    Keine Linie erkannt. Farbbereiche in geometry.h "
                   "pruefen\n"
                   "    (ACE_LINE_A_* blau, ACE_LINE_B_* gruen) oder mit\n"
                   "    --no-lines ohne Referenz fahren.\n");
        }
    }

    if (t->height_max_mm > t->height_min_mm) {
        printf("  Hoehe unter den Ankern       %.1f .. %.1f mm"
               "  (Nennmass %.0f)\n",
               t->height_min_mm, t->height_max_mm, ACE_HOVER_HEIGHT_MM);
        printf("                               geplant wurde zuletzt auf %.1f mm\n",
               kin_planned_height());
    }

    if (t->weak >= 0) {
        printf("  Winde %d neu referenziert     %d x\n", t->weak, t->slip_events);
        printf("  groesster Schlupf            %.2f mm"
               "  (%ld Halbschritte)\n",
               t->slip_worst_mm,
               lround(t->slip_worst_mm / ACE_MM_PER_HALFSTEP_AT(t->weak)));
        if (t->slip_events > 0) {
            printf("\n  Der Schlupf wurde jedes Mal herausgerechnet, die Lage der\n"
                   "  Plattform blieb dadurch richtig. Haeufen sich die Ereignisse,\n"
                   "  hilft nur die Mechanik: Getriebe, Seilfuehrung, Wickelreibung.\n");
        }
    }
    printf("\n  Die Lage stammt aus den Schrittzaehlern der gesunden Winden.\n"
           "  Ob das Objekt wirklich mittig steht, sagt allein das Bild - und\n"
           "  das steht oben als Bildfehler.\n");
}

/* ------------------------------------------------------------------ main */

static void print_usage(const char *prog) {
    HsvRange d = bd_default_hsv_range();

    fprintf(stderr,
        "Verwendung: %s [--device X] [--width N] [--height N]\n"
        "               [--h-min N] [--h-max N] [--s-min N] [--s-max N]\n"
        "               [--v-min N] [--v-max N] [--stream[=port]]\n"
        "               [--weak-motor N] [--preload MM] [--no-learn]\n"
        "               [--gain G] [--tolerance PX] [--max-step MM]\n"
        "               [--probe-mm MM] [--delay-us N] [--cycles N]\n"
        "               [--tension MM] [--wait N] [--dry-run] [--quiet]\n"
        "\n"
        "Haelt ein farbiges Objekt in der Bildmitte. Die Plattform steht zu\n"
        "Beginn von Hand in der Mitte des Ankerfelds; dort liegt der Nullpunkt.\n"
        "\n"
        "  --weak-motor N Winde, die Schritte verliert, Standard %d. Sie wird\n"
        "                 aus der Lageschaetzung genommen, wodurch ihr Schlupf\n"
        "                 messbar wird. -1 schaltet das ab\n"
        "  --preload MM   Vorspannung auf dieser Winde, Standard %.1f mm\n"
        "  --no-learn     ohne Lernphase starten, mit den Werten aus geometry.h\n"
        "  --probe-mm MM  Laenge einer Probefahrt der Lernphase, Standard %.0f\n"
        "  --gain G       Daempfung, Standard %.2f. Kleiner = mehr Reserve\n"
        "                 gegen eine verdrehte Kamera, aber langsamer\n"
        "  --tolerance PX Objekt gilt innerhalb davon als zentriert, Std. %.0f\n"
        "  --max-step MM  laengster Fahrweg je Zug, Standard %.0f\n"
        "  --cycles N     nach N Zuegen beenden, Standard 0 = endlos\n"
        "  --tension MM   vor dem Start alle vier Seile anspannen\n"
        "  --stream       Livebild mit Markierung auf Port 8080\n"
        "  --jpeg Q       JPEG-Guete des Streams 1..100, Standard 70\n"
        "  --stream-scale F  Bild vorher verkleinern, 0.1..1.0, Standard 0.5.\n"
        "                 Halbe Kante = ein Viertel Datenmenge. Betrifft nur\n"
        "                 den Stream, nicht die Erkennung\n"
        "  --focus MODE   manual, continuous, auto oder default.\n"
        "                 Standard manual: der Abstand steht fest, ein\n"
        "                 pumpender Autofokus brachte nur Unschaerfe und\n"
        "                 einen wandernden Massstab\n"
        "  --lens DPT     Linsenposition in Dioptrien, also 1/Abstand[m].\n"
        "                 Standard %.2f, passend zu %.0f mm aus geometry.h\n"
        "\n"
        "Farbe: Standard Rot, h %d-%d s %d-%d v %d-%d. Ist h-min groesser als\n"
        "       h-max, wird ueber den Nullpunkt der Hue-Skala hinweg gesucht.\n",
        prog, ACE_WEAK_MOTOR, (double)ACE_WEAK_PRELOAD_MM,
        (double)ACE_TRACK_PROBE_MM, (double)ACE_TRACK_GAIN,
        (double)ACE_TRACK_TOLERANCE_PX, (double)ACE_TRACK_MAX_STEP_MM,
        1000.0 / ACE_CAMERA_HEIGHT_MM, (double)ACE_CAMERA_HEIGHT_MM,
        d.h_min, d.h_max, d.s_min, d.s_max, d.v_min, d.v_max);
}

int main(int argc, char **argv) {
    const char *device      = "/dev/video0";
    int         width       = 1280;
    int         height      = 720;
    int         stream_on   = 0;
    int         stream_port = 8080;
    int         jpeg_quality  = 70;
    double      stream_scale  = 0.5;
    const char *focus_mode    = "manual";
    /* Dioptrien = 1 / Abstand in Metern. Der Abstand steht fest in
     * geometry.h, also laesst sich die Linsenposition ausrechnen. */
    double      lens_position = 1000.0 / ACE_CAMERA_HEIGHT_MM;
    int         do_learn    = 1;
    int         dry_run     = 0;
    int         wait_s      = ACE_FIGURE_WAIT_S;
    double      tension_mm  = 0.0;
    double      preload_mm  = ACE_WEAK_PRELOAD_MM;
    long        max_cycles  = 0;

    Tracker t;
    t.range        = bd_default_hsv_range();
    t.delay_us     = ACE_TRAVEL_DELAY_US;
    t.gain         = ACE_TRACK_GAIN;
    t.tolerance_px = ACE_TRACK_TOLERANCE_PX;
    t.max_step_mm  = ACE_TRACK_MAX_STEP_MM;
    t.probe_mm     = ACE_TRACK_PROBE_MM;
    t.settle_ms    = ACE_TRACK_SETTLE_MS;
    t.quiet        = 0;
    t.stream_on    = 0;
    t.use_lines    = 1;
    t.fix_angle    = 0;
    t.fix_position = 0;
    t.fix_scale    = 0;
    t.last_tilt_deg = 0.0;
    t.tilt_warned  = 0;
    t.last_fix_err_mm = 0.0;
    t.fix_err_worst_mm = 0.0;
    t.bias_set     = 0;
    t.bias_x_mm    = 0.0;
    t.bias_y_mm    = 0.0;
    t.bias_tilt_deg = 0.0;
    t.bias_stale   = 0;
    t.fix_pos_on   = 0;
    t.tracking     = 0;
    t.learned      = 0;
    t.prev_err_px  = -1.0;
    t.stall        = 0;
    t.lost_run     = 0;
    t.search_on    = 1;
    t.search_index = 0;
    t.search_laps  = 0;
    t.searches_hit = 0;
    t.pinned       = 0;
    floor_default_lines(t.lines);
    t.weak         = ACE_WEAK_MOTOR;
    t.slip_total_mm = 0.0;
    t.slip_events  = 0;
    t.slip_worst_mm = 0.0;
    t.height_min_mm = 1e9;
    t.height_max_mm = -1e9;
    t.height_warned = 0;
    t.cycles       = 0;
    t.observations = 0;
    t.relearns     = 0;
    t.lost         = 0;
    t.drift_warned = 0;
    t.cable_warned = 0;
    t.cam          = NULL;

    static struct option lo[] = {
        {"device",      required_argument, 0, 'd'},
        {"width",       required_argument, 0, 'w'},
        {"height",      required_argument, 0, 'H'},
        {"stream",      optional_argument, 0, 't'},
        {"weak-motor",  required_argument, 0, 'W'},
        {"preload",     required_argument, 0, 'P'},
        {"no-learn",    no_argument,       0, 'L'},
        {"no-lines",    no_argument,       0, 'N'},
        {"no-search",   no_argument,       0, 'X'},
        {"fix-position",no_argument,       0, 'Q'},
        {"gain",        required_argument, 0, 'g'},
        {"tolerance",   required_argument, 0, 'e'},
        {"max-step",    required_argument, 0, 'M'},
        {"probe-mm",    required_argument, 0, 'b'},
        {"delay-us",    required_argument, 0, 'u'},
        {"cycles",      required_argument, 0, 'c'},
        {"tension",     required_argument, 0, 'T'},
        {"settle-ms",   required_argument, 0, 'S'},
        {"wait",        required_argument, 0, 'a'},
        {"dry-run",     no_argument,       0, 'n'},
        {"quiet",       no_argument,       0, 'q'},
        {"focus",       required_argument, 0, 'f'},
        {"lens",        required_argument, 0, 'l'},
        {"jpeg",        required_argument, 0, 'j'},
        {"stream-scale",required_argument, 0, 'z'},
        {"h-min",       required_argument, 0, 1},
        {"h-max",       required_argument, 0, 2},
        {"s-min",       required_argument, 0, 3},
        {"s-max",       required_argument, 0, 4},
        {"v-min",       required_argument, 0, 5},
        {"v-max",       required_argument, 0, 6},
        {"help",        no_argument,       0, 7},
        {0, 0, 0, 0}
    };

    int opt, oi = 0;
    while ((opt = getopt_long(argc, argv,
                              "d:w:H:t::W:P:LNXQg:e:M:b:u:c:T:S:a:nqf:l:j:z:",
                              lo, &oi)) != -1) {
        switch (opt) {
            case 'f': focus_mode     = optarg; break;
            case 'l': lens_position  = atof(optarg); break;
            case 'j': jpeg_quality   = atoi(optarg); break;
            case 'z': stream_scale   = atof(optarg); break;
            case 'd': device         = optarg; break;
            case 'w': width          = atoi(optarg); break;
            case 'H': height         = atoi(optarg); break;
            case 'W': t.weak         = atoi(optarg); break;
            case 'P': preload_mm     = atof(optarg); break;
            case 'L': do_learn       = 0; break;
            case 'N': t.use_lines    = 0; break;
            case 'X': t.search_on    = 0; break;
            case 'Q': t.fix_pos_on   = 1; break;
            case 'g': t.gain         = atof(optarg); break;
            case 'e': t.tolerance_px = atof(optarg); break;
            case 'M': t.max_step_mm  = atof(optarg); break;
            case 'b': t.probe_mm     = atof(optarg); break;
            case 'u': t.delay_us     = (unsigned int)strtoul(optarg, NULL, 10); break;
            case 'c': max_cycles     = atol(optarg); break;
            case 'T': tension_mm     = atof(optarg); break;
            case 'S': t.settle_ms    = atol(optarg); break;
            case 'a': wait_s         = atoi(optarg); break;
            case 'n': dry_run        = 1; break;
            case 'q': t.quiet        = 1; break;
            case 't':
                stream_on = 1;
                if (optarg) stream_port = atoi(optarg);
                break;
            case 1: t.range.h_min = atoi(optarg); break;
            case 2: t.range.h_max = atoi(optarg); break;
            case 3: t.range.s_min = atoi(optarg); break;
            case 4: t.range.s_max = atoi(optarg); break;
            case 5: t.range.v_min = atoi(optarg); break;
            case 6: t.range.v_max = atoi(optarg); break;
            case 7: print_usage(argv[0]); return 0;
            default: print_usage(argv[0]); return 1;
        }
    }

    if (width <= 0 || height <= 0) {
        fprintf(stderr, "--width und --height muessen positiv sein.\n");
        return 1;
    }
    if (t.gain <= 0.0 || t.gain > 1.5) {
        fprintf(stderr, "--gain sollte zwischen 0 und 1.5 liegen.\n");
        return 1;
    }
    if (t.weak >= ACE_MOTOR_COUNT) {
        fprintf(stderr, "--weak-motor muss zwischen -1 und %d liegen.\n",
                ACE_MOTOR_COUNT - 1);
        return 1;
    }
    if (t.weak < 0) t.weak = -1;
    if (t.delay_us < ACE_MIN_STEP_DELAY_US) {
        fprintf(stderr, "Schrittzeit %u us zu kurz, auf %u us begrenzt.\n",
                t.delay_us, ACE_MIN_STEP_DELAY_US);
        t.delay_us = ACE_MIN_STEP_DELAY_US;
    }
    if (wait_s < 0)       wait_s    = 0;
    if (t.settle_ms < 0)  t.settle_ms = 0;

    t.width  = width;
    t.height = height;

    /* Schon hier belegen, nicht erst nach dem Countdown: bricht der Nutzer
     * vorher ab, liest der Schlussbericht sonst uninitialisierten Speicher. */
    vis_init(&t.vis, (double)width / ACE_VIEW_WIDTH_MM, 0.0, ACE_VIS_LAMBDA);

    /* ACE_VIEW_WIDTH_MM gilt fuer den Nennabstand. Ab hier wird der
     * Massstab an den wirklich gemessenen Abstand gekoppelt. */
    vis_set_camera_distance(&t.vis, ACE_CAMERA_HEIGHT_MM);

    print_setup(&t);

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    if (dry_run) stepper_set_dry_run(1);

    /* Vor bd_create_camera: der Fokus geht in die Kommandozeile von
     * rpicam-vid ein. Fester Abstand, also fester Fokus - ein pumpender
     * Autofokus bringt Unschaerfe und einen wandernden Massstab, und beides
     * verfaelscht das Bildmodell. */
    bd_set_focus(focus_mode, lens_position);
    bd_set_stream_quality(jpeg_quality);
    bd_set_stream_scale(stream_scale);

    printf("Kamera         Fokus %s", focus_mode);
    if (focus_mode && focus_mode[0] == 'm') {
        printf(", Linse %.2f dpt (= %.0f mm Abstand)",
               lens_position, lens_position > 0.0 ? 1000.0 / lens_position : 0.0);
    }
    printf("\n\n");

    t.cam = bd_create_camera(device, width, height);
    if (!t.cam) {
        fprintf(stderr, "Konnte Kamera nicht initialisieren.\n");
        return 1;
    }

    if (motion_init() != 0) {
        bd_release(t.cam);
        return 1;
    }
    if (motion_thread_start() != 0) {
        motion_shutdown();
        bd_release(t.cam);
        return 1;
    }

    if (stream_on) {
        if (bd_stream_start(stream_port) != 0) {
            fprintf(stderr, "Stream-Server auf Port %d fehlgeschlagen.\n",
                    stream_port);
        } else {
            t.stream_on = 1;
            bd_stream_set_page(ACE_PAGE);
            fprintf(stderr, "Livebild unter http://<pi-ip>:%d/  "
                            "(Guete %d, Groesse %.0f %%)\n",
                    stream_port, jpeg_quality, stream_scale * 100.0);
        }
    }

    printf("Die Plattform muss jetzt von Hand in der Mitte stehen.\n");
    for (int s = wait_s; s > 0 && !g_abort; s--) {
        printf("\rStart in %d s ... ", s);
        fflush(stdout);
        sleep_ms(1000);
    }
    printf("\r                      \r");

    int rc = 0;

    if (!g_abort && tension_mm != 0.0) {
        long steps[ACE_MOTOR_COUNT];
        for (int i = 0; i < ACE_MOTOR_COUNT; i++)
            steps[i] = lround(tension_mm / ACE_MM_PER_HALFSTEP_AT(i));
        printf("Spannen: alle vier Seile %+.1f mm.\n", tension_mm);
        if (motion_start(steps, t.delay_us) == 0) {
            while (motion_busy() && !g_abort) sleep_ms(20);
        }
    }

    if (!g_abort) {
        long steps[ACE_MOTOR_COUNT];
        motion_positions(steps);
        kin_reset(0.0, 0.0, steps);

        /* Erst nach dem Nullpunkt: sonst wuerde die Vorspannung schon in die
         * Referenzlaenge einfliessen und sich damit selbst aufheben. */
        kin_set_untrusted(t.weak);
        if (t.weak >= 0) kin_set_trim(t.weak, -preload_mm);

        printf("Nullpunkt gesetzt.\n\n");

        double px_per_mm = (double)width / ACE_VIEW_WIDTH_MM;
        vis_init(&t.vis, px_per_mm, 0.0, ACE_VIS_LAMBDA);
        vis_set_camera_distance(&t.vis, ACE_CAMERA_HEIGHT_MM);

        /* Jetzt, wo die Plattform nachweislich auf dem Kreuzungspunkt
         * steht, den Versatz der Bildmitte messen. Danach ist die Lage der
         * Plattform nie wieder unabhaengig bekannt, also gibt es keine
         * zweite Gelegenheit dafuer.
         *
         * Ein paar Bilder vorher holen: bd_detect_line wertet das zuletzt
         * geholte aus, und das Objekt muss dafuer nicht sichtbar sein. */
        for (int f = 0; f < 5 && !g_abort; f++) {
            DetectionResult ignored;
            if (bd_detect(t.cam, &t.range, &ignored) != 0) break;
            if (t.stream_on) bd_stream_push(t.cam, &ignored);
        }
        read_floor(&t, 1);
        printf("\n");

        if (do_learn) {
            printf("Lernphase: vier Probefahrten ueber %.0f mm. Objekt bitte\n"
                   "still liegen lassen.\n", t.probe_mm);
            int lr = learn(&t);
            if (lr < 0) rc = 1;
            printf("\n");
        } else {
            printf("Ohne Lernphase, Startwerte aus geometry.h: %.3f px/mm, "
                   "0 Grad.\n\n", px_per_mm);
        }

        t.learned = (rc == 0 && do_learn);

        /* Mit Bedienseite wird gewartet, statt sofort loszufahren - sonst
         * liefe die Verfolgung schon, bevor man den Knopf ueberhaupt
         * sieht. Ohne Seite bleibt es beim alten Verhalten. */
        t.tracking = !t.stream_on;

        if (t.stream_on) {
            printf("Bedienseite unter http://<pi-ip>:%d/ - die Verfolgung\n"
                   "startet dort ueber den Knopf.\n\n", stream_port);
        }

        if (rc == 0 && !g_abort) {
            if (run_loop(&t, max_cycles) < 0) rc = 1;
        }
    }

    if (g_abort) {
        path_abort();
        printf("\nAbbruch.\n");
    }

    print_result(&t);

    if (stream_on) bd_stream_stop();

    path_abort();
    motion_release();
    motion_shutdown();
    bd_release(t.cam);
    return rc;
}
