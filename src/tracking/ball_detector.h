#ifndef ACE_TRACKING_BALL_DETECTOR_H
#define ACE_TRACKING_BALL_DETECTOR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int h_min, s_min, v_min;
    int h_max, s_max, v_max;
} HsvRange;

    HsvRange bd_default_hsv_range(void);

    typedef struct {
        bool found;
        bool clipped;
        double x;
        double y;
        double radius;
    } DetectionResult;

    typedef struct {
        int  h_med, s_med, v_med;
        int  h_lo, h_hi;
        int  s_lo, s_hi;
        int  v_lo, v_hi;
        int  b_mean, g_mean, r_mean;
        bool hue_wraps;
    } PatchStats;

    typedef struct {
        PatchStats centre;
        PatchStats blob;
        bool   blob_found;
        double blob_x, blob_y, blob_radius, best_area;
        long   mask_pixels;
        long   frame_pixels;
    } ProbeResult;

    /* Eine der beiden Bodenlinien, wie sie im Bild erscheint.
     *
     * Winkel und Lotabstand sind im rechtshaendigen Bildrahmen angegeben:
     * u nach rechts, v nach oben, also Bild-v gespiegelt. So ist der
     * Zusammenhang zum Feld eine reine Drehstreckung, siehe visual.h. */
    typedef struct {
        bool   found;
        double angle_deg;   /* Richtung der Linie, -90 .. +90            */
        double offset_px;   /* vorzeichenbehafteter Lotabstand zur Mitte */
        double dash_px;     /* gemessene Strichperiode, 0 = unbestimmt   */
        long   pixels;      /* Maskengroesse, als Guetemass              */
    } LineResult;

    typedef struct BallDetector BallDetector;
    BallDetector *bd_create_camera(const char *device_path, int width, int height);
    BallDetector *bd_create_from_image(const char *image_path);

    int bd_detect(BallDetector *detector, const HsvRange *range, DetectionResult *result);

    int bd_probe(BallDetector *detector, const HsvRange *range, int window_px,
                 ProbeResult *result);

    /* Eine Bodenlinie im zuletzt geholten Bild suchen. Arbeitet auf dem
     * Bild, das bd_detect zuletzt geholt hat - erst bd_detect rufen, dann
     * hiermit je Linienfarbe nachfassen, sonst kostet es zusaetzliche
     * Bilder und die Messungen gehoeren zu verschiedenen Zeitpunkten. */
    int bd_detect_line(BallDetector *detector, const HsvRange *range,
                       LineResult *result);

    void bd_set_view_mask(int enabled);

    /* Fokus der Kamera. Vor bd_create_camera aufrufen, danach wirkungslos.
     *
     * Der Aufbau hat einen festen Abstand zur Flaeche, also gibt es fuer den
     * Autofokus nichts zu gewinnen: er pumpt, und jedes Nachfokussieren
     * bringt Unschaerfe und einen leicht anderen Massstab. Beides stoert die
     * Erkennung und verfaelscht das Bildmodell der Verfolgung. Standard ist
     * darum fester Fokus auf ACE_CAMERA_HEIGHT_MM.
     *
     * mode: "manual", "auto", "continuous" oder "default".
     * lens_position in Dioptrien, also 1/Abstand in Metern. 400 mm sind 2.5.
     * Nur bei "manual" wirksam. */
    void bd_set_focus(const char *mode, double lens_position);

    /* JPEG-Guete des Streams, 1..100. Der OpenCV-Standard 95 erzeugt bei
     * 1280x720 rund 200 KB je Bild, also mehrere MB/s. */
    void bd_set_stream_quality(int quality);

    /* Bild vor dem Kodieren verkleinern, 0.1 bis 1.0. Halbe Kantenlaenge
     * bedeutet ein Viertel der Datenmenge. Betrifft nur den Stream, nicht
     * die Erkennung. */
    void bd_set_stream_scale(double factor);

    int bd_save_annotated(BallDetector *detector, const DetectionResult *result, const char *out_path);

    int bd_stream_start(int port);

    /* ---- Bedienseite -----------------------------------------------------
     *
     * Der Streamserver kann zusaetzlich eine Seite ausliefern und Befehle
     * von ihr entgegennehmen. Die Bibliothek kennt deren Bedeutung nicht -
     * sie reicht Zeichenketten durch. Was ein Befehl heisst, entscheidet
     * allein das Programm, das bd_stream_take_command aufruft. */

    /* Seite, die unter "/" ausgeliefert wird. NULL nimmt die eingebaute. */
    void bd_stream_set_page(const char *html);

    /* Text, den die Seite unter "/status" abholen kann. */
    void bd_stream_set_status(const char *text);

    /* Naechsten Befehl aus der Warteschlange holen; 1 wenn einer da war.
     * Der Inhalt ist die Abfrage hinter "/cmd?", unveraendert. */
    int bd_stream_take_command(char *out, int out_size);

    int bd_stream_push(BallDetector *detector, const DetectionResult *result);

    void bd_stream_stop(void);

    void bd_release(BallDetector *detector);

#ifdef __cplusplus
}
#endif
#endif