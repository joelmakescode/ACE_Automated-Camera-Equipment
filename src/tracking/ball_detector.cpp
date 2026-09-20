#include "ball_detector.h"

#include <opencv2/opencv.hpp>
#if CV_VERSION_MAJOR >= 5
/* OpenCV 5 verschob contourArea/minEnclosingCircle aus imgproc.hpp hierher;
 * auf OpenCV 4 (z.B. via apt auf Raspberry Pi OS) existiert dieser Header nicht. */
#include <opencv2/geometry/2d.hpp>
#endif
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <atomic>
#include <chrono>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

struct BallDetector {
    bool use_camera;
    FILE *pipe = nullptr;
    int cam_width = 0;
    int cam_height = 0;
    std::vector<unsigned char> yuv_buf;
    cv::Mat static_image;
    cv::Mat frame_buf;
    cv::Mat hsv_buf;
    cv::Mat mask_buf;
};

extern "C" HsvRange bd_default_hsv_range(void) {
    HsvRange range;
    range.h_min = 170; range.s_min = 120; range.v_min = 70;
    range.h_max = 10;  range.s_max = 255; range.v_max = 255;
    return range;
}

/* Fokus: fest auf den Arbeitsabstand, nicht automatisch. Siehe Kommentar an
 * bd_set_focus in der Kopfdatei. */
static std::string g_focus_mode     = "manual";
static double      g_lens_position  = 2.5;   /* Dioptrien, 1/0.4 m */

extern "C" void bd_set_focus(const char *mode, double lens_position) {
    if (mode && *mode) g_focus_mode = mode;
    g_lens_position = lens_position;
}

extern "C" BallDetector *bd_create_camera(const char *device_path, int width, int height) {
    (void)device_path;

    BallDetector *bd = new BallDetector();
    bd->use_camera = true;
    bd->cam_width  = width  > 0 ? width  : 1280;
    bd->cam_height = height > 0 ? height : 720;
    bd->yuv_buf.resize(static_cast<size_t>(bd->cam_width) * bd->cam_height * 3 / 2);

    std::string cmd = "rpicam-vid --nopreview -t 0 --codec yuv420"
                       " --width " + std::to_string(bd->cam_width) +
                       " --height " + std::to_string(bd->cam_height) +
                       " --framerate 15";

    if (g_focus_mode != "default") {
        cmd += " --autofocus-mode " + g_focus_mode;
        if (g_focus_mode == "manual") {
            char pos[32];
            std::snprintf(pos, sizeof(pos), "%.3f", g_lens_position);
            cmd += std::string(" --lens-position ") + pos;
        }
    }
    cmd += " -o - 2>/dev/null";

    bd->pipe = popen(cmd.c_str(), "r");
    if (!bd->pipe) {
        std::fprintf(stderr, "Failed to start rpicam-vid\n");
        delete bd;
        return nullptr;
    }

    return bd;
}

extern "C" BallDetector *bd_create_from_image(const char *image_path) {
    BallDetector *bd = new BallDetector();
    bd->use_camera = false;
    bd->static_image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (bd->static_image.empty()) {
        std::fprintf(stderr, "Failed to open image %s\n", image_path);
        delete bd;
        return nullptr;
    }
    return bd;
}

static constexpr double MIN_CONTOUR_AREA = 50.0;

static bool g_view_mask = false;

extern "C" void bd_set_view_mask(int enabled) {
    g_view_mask = enabled != 0;
}

static const cv::Mat *grab_frame(BallDetector *bd) {
    if (!bd->use_camera) return &bd->static_image;

    size_t n = std::fread(bd->yuv_buf.data(), 1, bd->yuv_buf.size(), bd->pipe);
    if (n != bd->yuv_buf.size()) {
        std::fprintf(stderr, "Failed to read frame from rpicam-vid\n");
        return nullptr;
    }

    cv::Mat yuv(bd->cam_height * 3 / 2, bd->cam_width, CV_8UC1, bd->yuv_buf.data());
    cv::cvtColor(yuv, bd->frame_buf, cv::COLOR_YUV2BGR_I420);
    return &bd->frame_buf;
}

static void build_mask(BallDetector *bd, const cv::Mat &frame, const HsvRange *range) {
    cv::cvtColor(frame, bd->hsv_buf, cv::COLOR_BGR2HSV);
    if (range->h_min <= range->h_max) {
        cv::inRange(bd->hsv_buf, cv::Scalar(range->h_min, range->s_min, range->v_min), cv::Scalar(range->h_max, range->s_max, range->v_max), bd->mask_buf);
    } else {
        cv::Mat mask_low, mask_high;
        cv::inRange(bd->hsv_buf,
                    cv::Scalar(range->h_min, range->s_min, range->v_min),
                    cv::Scalar(179, range->s_max, range->v_max),
                    mask_low);
        cv::inRange(bd->hsv_buf,
                    cv::Scalar(0, range->s_min, range->v_min),
                    cv::Scalar(range->h_max, range->s_max, range->v_max),
                    mask_high);
        cv::bitwise_or(mask_low, mask_high, bd->mask_buf);
    }

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(bd->mask_buf, bd->mask_buf, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(bd->mask_buf, bd->mask_buf, cv::MORPH_CLOSE, kernel);
}

extern "C" int bd_detect(BallDetector *bd, const HsvRange *range, DetectionResult *out_result) {
    if (!bd || !range || !out_result) return -1;

    out_result->found = false;
    out_result->clipped = false;
    out_result->x = 0.0;
    out_result->y = 0.0;
    out_result->radius = 0.0;

    const cv::Mat *frame_ptr = grab_frame(bd);
    if (!frame_ptr) return -1;

    build_mask(bd, *frame_ptr, range);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bd->mask_buf, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return 0;
    }

    size_t best_idx = 0;
    double best_area = 0.0;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > best_area) {
            best_area = area;
            best_idx = i;
        }
    }

    if (best_area < MIN_CONTOUR_AREA) {
        return 0;
    }

    cv::Point2f center;
    float radius = 0.0f;
    cv::minEnclosingCircle(contours[best_idx], center, radius);

    cv::Rect box = cv::boundingRect(contours[best_idx]);
    const int margin = 2;

    out_result->found   = true;
    out_result->clipped = box.x <= margin || box.y <= margin ||
                          box.x + box.width  >= bd->mask_buf.cols - margin ||
                          box.y + box.height >= bd->mask_buf.rows - margin;
    out_result->x       = center.x;
    out_result->y       = center.y;
    out_result->radius  = radius;
    return 0;
}

static int percentile(const std::vector<unsigned char> &sorted, double q) {
    if (sorted.empty()) return 0;
    size_t i = static_cast<size_t>(q * (sorted.size() - 1));
    return sorted[i];
}

static int hue_median_circular(std::vector<unsigned char> hues) {
    if (hues.empty()) return 0;

    size_t near_zero = 0, near_top = 0;
    for (unsigned char h : hues) {
        if (h <= 30)       near_zero++;
        else if (h >= 150) near_top++;
    }

    if (near_zero > 0 && near_top > 0 && near_zero + near_top > hues.size() / 2) {
        long sum = 0;
        for (unsigned char h : hues) sum += (h >= 90) ? (int)h - 180 : (int)h;
        int m = static_cast<int>(sum / (long)hues.size());
        return (m < 0) ? m + 180 : m;
    }

    std::sort(hues.begin(), hues.end());
    return hues[hues.size() / 2];
}

static void patch_stats(const cv::Mat &hsv, const cv::Mat &bgr,
                        cv::Rect roi, PatchStats *out) {
    roi &= cv::Rect(0, 0, hsv.cols, hsv.rows);
    if (roi.width <= 0 || roi.height <= 0) { *out = PatchStats{}; return; }

    cv::Mat patch = hsv(roi);
    std::vector<unsigned char> hs, ss, vs;
    hs.reserve(roi.area()); ss.reserve(roi.area()); vs.reserve(roi.area());
    for (int y = 0; y < patch.rows; ++y) {
        const cv::Vec3b *row = patch.ptr<cv::Vec3b>(y);
        for (int x = 0; x < patch.cols; ++x) {
            hs.push_back(row[x][0]); ss.push_back(row[x][1]); vs.push_back(row[x][2]);
        }
    }

    out->h_med = hue_median_circular(hs);

    std::sort(hs.begin(), hs.end());
    std::sort(ss.begin(), ss.end());
    std::sort(vs.begin(), vs.end());

    out->h_lo = percentile(hs, 0.05);
    out->h_hi = percentile(hs, 0.95);
    out->s_lo = percentile(ss, 0.05);
    out->s_hi = percentile(ss, 0.95);
    out->v_lo = percentile(vs, 0.05);
    out->v_hi = percentile(vs, 0.95);
    out->s_med = percentile(ss, 0.5);
    out->v_med = percentile(vs, 0.5);
    out->hue_wraps = (out->h_lo <= 30 && out->h_hi >= 150);

    cv::Scalar m = cv::mean(bgr(roi));
    out->b_mean = static_cast<int>(m[0]);
    out->g_mean = static_cast<int>(m[1]);
    out->r_mean = static_cast<int>(m[2]);
}

extern "C" int bd_probe(BallDetector *bd, const HsvRange *range, int window_px,
                        ProbeResult *out) {
    if (!bd || !range || !out) return -1;

    const cv::Mat *frame_ptr = grab_frame(bd);
    if (!frame_ptr) return -1;

    build_mask(bd, *frame_ptr, range);

    *out = ProbeResult{};
    out->mask_pixels  = cv::countNonZero(bd->mask_buf);
    out->frame_pixels = static_cast<long>(bd->mask_buf.total());

    int w = window_px > 0 ? window_px : 80;
    patch_stats(bd->hsv_buf, *frame_ptr,
                cv::Rect((frame_ptr->cols - w) / 2, (frame_ptr->rows - w) / 2, w, w),
                &out->centre);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bd->mask_buf, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    size_t best_idx = 0;
    for (size_t i = 0; i < contours.size(); ++i) {
        double a = cv::contourArea(contours[i]);
        if (a > out->best_area) { out->best_area = a; best_idx = i; }
    }

    if (out->best_area >= MIN_CONTOUR_AREA) {
        cv::Point2f c;
        float r = 0.0f;
        cv::minEnclosingCircle(contours[best_idx], c, r);
        out->blob_found  = true;
        out->blob_x      = c.x;
        out->blob_y      = c.y;
        out->blob_radius = r;

        int side = static_cast<int>(r);
        if (side < 8) side = 8;
        patch_stats(bd->hsv_buf, *frame_ptr,
                    cv::Rect(static_cast<int>(c.x) - side / 2,
                             static_cast<int>(c.y) - side / 2, side, side),
                    &out->blob);
    }
    return 0;
}

/* ---- Bodenlinien --------------------------------------------------------
 *
 * Zwei Klebebaender auf der Flaeche, in bekannter Richtung und mit
 * bekannter Strichteilung. Sie liefern, was die Bewegungsmessung nicht
 * kann: den Kamerawinkel absolut aus einem einzigen Bild, ueber den
 * Lotabstand eine Lagemessung, und ueber die Strichteilung den Massstab.
 */

static const cv::Mat *current_frame(BallDetector *bd) {
    if (!bd->use_camera) {
        return bd->static_image.empty() ? nullptr : &bd->static_image;
    }
    return bd->frame_buf.empty() ? nullptr : &bd->frame_buf;
}

/* Strichteilung: alle Maskenpunkte auf die Linienrichtung projizieren, die
 * Mitten der belegten Abschnitte suchen und deren Abstand als Median
 * nehmen. Der Median, weil einzelne Striche von der Plattform oder vom
 * Objekt verdeckt sein koennen. */
static double measure_dash(const std::vector<cv::Point> &pts,
                           double cx, double cy, double ux, double uy) {
    if (pts.size() < 50) return 0.0;

    std::vector<double> ts;
    ts.reserve(pts.size());
    double tmin = 1e18, tmax = -1e18;

    for (const cv::Point &p : pts) {
        double px = p.x - cx;
        double py = -(p.y - cy);
        double t  = px * ux + py * uy;
        ts.push_back(t);
        if (t < tmin) tmin = t;
        if (t > tmax) tmax = t;
    }

    int n = static_cast<int>(std::ceil(tmax - tmin)) + 1;
    if (n < 60 || n > 20000) return 0.0;

    std::vector<int> occ(static_cast<size_t>(n), 0);
    for (double t : ts) {
        int i = static_cast<int>(t - tmin);
        if (i >= 0 && i < n) occ[static_cast<size_t>(i)]++;
    }

    std::vector<double> centres;
    for (int i = 0; i < n; ) {
        if (!occ[static_cast<size_t>(i)]) { i++; continue; }
        int start = i;
        while (i < n && occ[static_cast<size_t>(i)]) i++;
        centres.push_back((start + i - 1) / 2.0);
    }
    if (centres.size() < 4) return 0.0;      /* zu wenige Striche im Bild */

    std::vector<double> gaps;
    for (size_t k = 1; k < centres.size(); k++) {
        gaps.push_back(centres[k] - centres[k - 1]);
    }
    std::sort(gaps.begin(), gaps.end());
    return gaps[gaps.size() / 2];
}

extern "C" int bd_detect_line(BallDetector *bd, const HsvRange *range,
                              LineResult *out) {
    if (!bd || !range || !out) return -1;

    out->found     = false;
    out->angle_deg = 0.0;
    out->offset_px = 0.0;
    out->dash_px   = 0.0;
    out->pixels    = 0;

    const cv::Mat *frame = current_frame(bd);
    if (!frame) return -1;

    /* Eigene Puffer, damit die Objektmaske aus bd_detect stehen bleibt. */
    cv::Mat hsv, mask;
    cv::cvtColor(*frame, hsv, cv::COLOR_BGR2HSV);

    if (range->h_min <= range->h_max) {
        cv::inRange(hsv, cv::Scalar(range->h_min, range->s_min, range->v_min),
                         cv::Scalar(range->h_max, range->s_max, range->v_max), mask);
    } else {
        cv::Mat lo, hi;
        cv::inRange(hsv, cv::Scalar(range->h_min, range->s_min, range->v_min),
                         cv::Scalar(179, range->s_max, range->v_max), lo);
        cv::inRange(hsv, cv::Scalar(0, range->s_min, range->v_min),
                         cv::Scalar(range->h_max, range->s_max, range->v_max), hi);
        cv::bitwise_or(lo, hi, mask);
    }

    /* Nur oeffnen, nicht schliessen: die Luecken der Strichlinie werden
     * fuer den Massstab gebraucht und duerfen nicht zugebuegelt werden. */
    cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k);

    std::vector<cv::Point> pts;
    cv::findNonZero(mask, pts);
    out->pixels = static_cast<long>(pts.size());
    if (pts.size() < 200) return 0;

    /* Huber statt kleinster Quadrate: ein blauer Fleck im Hintergrund soll
     * die Gerade nicht verziehen. */
    cv::Vec4f fit;
    cv::fitLine(pts, fit, cv::DIST_HUBER, 0, 0.01, 0.01);

    double cx = frame->cols / 2.0;
    double cy = frame->rows / 2.0;

    /* In den rechtshaendigen Rahmen: v spiegeln. */
    double ux = fit[0];
    double uy = -fit[1];
    double len = std::sqrt(ux * ux + uy * uy);
    if (len < 1e-9) return 0;
    ux /= len;
    uy /= len;

    double px = fit[2] - cx;
    double py = -(fit[3] - cy);

    out->offset_px = px * (-uy) + py * ux;          /* Lot auf die Normale */
    out->angle_deg = std::atan2(uy, ux) * 180.0 / CV_PI;

    /* Eine Linie hat keine Richtung, also auf -90..90 zusammenfalten. */
    while (out->angle_deg >   90.0) out->angle_deg -= 180.0;
    while (out->angle_deg <= -90.0) out->angle_deg += 180.0;

    out->dash_px = measure_dash(pts, cx, cy, ux, uy);
    out->found   = true;
    return 0;
}

static cv::Mat draw_overlay(const cv::Mat &frame, const DetectionResult *result) {
    cv::Mat annotated = frame.clone();
    if (result->found) {
        cv::Point center(static_cast<int>(result->x), static_cast<int>(result->y));
        int radius = static_cast<int>(result->radius);
        cv::circle(annotated, center, radius, cv::Scalar(0, 255, 0), 2);
        cv::circle(annotated, center, 3, cv::Scalar(0, 0, 255), -1);
    }
    return annotated;
}

static cv::Mat view_for_output(BallDetector *bd, const DetectionResult *result) {
    const cv::Mat &frame = bd->use_camera ? bd->frame_buf : bd->static_image;
    if (frame.empty()) return cv::Mat();

    if (g_view_mask && !bd->mask_buf.empty()) {
        cv::Mat shown;
        cv::cvtColor(bd->mask_buf, shown, cv::COLOR_GRAY2BGR);
        return draw_overlay(shown, result);
    }
    return draw_overlay(frame, result);
}

extern "C" int bd_save_annotated(BallDetector *bd, const DetectionResult *result, const char *out_path) {
    if (!bd || !result || !out_path) return -1;

    cv::Mat annotated = view_for_output(bd, result);
    if (annotated.empty()) return -1;
    return cv::imwrite(out_path, annotated) ? 0 : -1;
}

namespace {
    std::atomic<bool> g_stream_running{false};
    int g_listen_fd = -1;
    std::thread g_accept_thread;

    std::mutex              g_frame_mutex;
    std::condition_variable g_frame_cv;
    std::vector<uchar>      g_latest_jpeg;
    unsigned long           g_frame_seq = 0;   /* zaehlt jedes neue Bild */

    /* Der Thread meldet ueber das Flag, dass er fertig ist - nur dann darf
     * er eingesammelt werden, ohne zu blockieren. */
    struct ClientSlot {
        std::thread                        thread;
        std::shared_ptr<std::atomic<bool>> done;
    };

    std::mutex              g_clients_mutex;
    std::vector<ClientSlot> g_client_threads;

    std::atomic<int>    g_jpeg_quality{70};
    std::atomic<double> g_stream_scale{1.0};

    /* Ohne MSG_NOSIGNAL beendet ein geschlossener Browser-Tab per SIGPIPE
     * den ganzen Prozess - mitten in der Fahrt. */
    bool send_all(int fd, const void *data, size_t len) {
        const char *p = static_cast<const char *>(data);
        while (len > 0) {
            ssize_t n = send(fd, p, len, MSG_NOSIGNAL);
            if (n <= 0) return false;
            p   += n;
            len -= static_cast<size_t>(n);
        }
        return true;
    }

    bool send_text(int fd, const char *s) {
        return send_all(fd, s, std::strlen(s));
    }

    /* Erste Zeile der Anfrage lesen und den Pfad herausziehen. Die Anfrage
     * wurde vorher gar nicht gelesen - dadurch bekam auch /favicon.ico einen
     * endlosen Bildstrom, und der Ladebalken des Tabs kam nie zur Ruhe. */
    bool read_path(int client_fd, std::string &path) {
        char    buf[1024];
        ssize_t n = recv(client_fd, buf, sizeof(buf) - 1, 0);
        if (n <= 0) return false;
        buf[n] = '\0';

        const char *sp1 = std::strchr(buf, ' ');
        if (!sp1) return false;
        const char *sp2 = std::strchr(sp1 + 1, ' ');
        if (!sp2) return false;

        path.assign(sp1 + 1, static_cast<size_t>(sp2 - sp1 - 1));
        size_t q = path.find('?');
        if (q != std::string::npos) path.erase(q);
        return true;
    }

    void serve_index(int client_fd) {
        static const char *body =
            "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
            "<title>ACE</title><style>"
            "html,body{margin:0;height:100%;background:#111;}"
            "body{display:flex;align-items:center;justify-content:center;}"
            "img{max-width:100%;max-height:100vh;image-rendering:auto;}"
            "</style></head><body><img src=\"/stream.mjpg\" alt=\"Kamera\">"
            "</body></html>";

        char head[256];
        std::snprintf(head, sizeof(head),
            "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
            "Content-Length: %zu\r\nConnection: close\r\n\r\n",
            std::strlen(body));

        if (send_text(client_fd, head)) send_text(client_fd, body);
    }

    void serve_404(int client_fd) {
        send_text(client_fd,
            "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n"
            "Connection: close\r\n\r\n");
    }

    void serve_stream(int client_fd) {
        if (!send_text(client_fd,
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
                "Cache-Control: no-cache, private\r\n"
                "Pragma: no-cache\r\n"
                "Connection: close\r\n\r\n")) {
            return;
        }

        unsigned long seen = 0;

        while (g_stream_running) {
            std::vector<uchar> jpeg_copy;
            {
                /* Auf ein wirklich neues Bild warten, statt dasselbe im
                 * Takt erneut zu schicken. */
                std::unique_lock<std::mutex> lock(g_frame_mutex);
                g_frame_cv.wait_for(lock, std::chrono::milliseconds(250),
                                    [&] { return !g_stream_running ||
                                                 g_frame_seq != seen; });
                if (!g_stream_running) break;
                if (g_frame_seq == seen || g_latest_jpeg.empty()) continue;

                seen      = g_frame_seq;
                jpeg_copy = g_latest_jpeg;
            }

            char part[128];
            int  hlen = std::snprintf(part, sizeof(part),
                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                jpeg_copy.size());

            if (!send_all(client_fd, part, static_cast<size_t>(hlen))) break;
            if (!send_all(client_fd, jpeg_copy.data(), jpeg_copy.size())) break;
            if (!send_all(client_fd, "\r\n", 2)) break;
        }
    }

    void stream_client_loop(int client_fd) {
        /* Ein Client, der verbindet und nichts sendet, darf keinen Thread
         * auf Dauer binden. */
        struct timeval tv = { 5, 0 };
        setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        std::string path;
        if (read_path(client_fd, path)) {
            if (path == "/stream.mjpg" || path == "/stream" || path == "/video") {
                serve_stream(client_fd);
            } else if (path == "/" || path == "/index.html") {
                serve_index(client_fd);
            } else {
                serve_404(client_fd);      /* auch /favicon.ico */
            }
        }
        close(client_fd);
    }

    /* Threads beendeter Clients einsammeln, sonst waechst die Liste mit
     * jedem Neuladen der Seite. */
    void reap_clients() {
        std::lock_guard<std::mutex> lock(g_clients_mutex);
        for (auto it = g_client_threads.begin(); it != g_client_threads.end(); ) {
            if (it->done->load()) {
                if (it->thread.joinable()) it->thread.join();
                it = g_client_threads.erase(it);
            } else {
                ++it;
            }
        }
    }

    void stream_accept_loop() {
        while (g_stream_running) {
            sockaddr_in client_addr{};
            socklen_t addr_len = sizeof(client_addr);
            int client_fd = accept(g_listen_fd,
                                   reinterpret_cast<sockaddr *>(&client_addr),
                                   &addr_len);
            if (client_fd < 0) {
                if (!g_stream_running) break;
                /* Bei dauerhaftem Fehler nicht heisslaufen. */
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            reap_clients();

            auto done = std::make_shared<std::atomic<bool>>(false);
            std::thread th([client_fd, done] {
                stream_client_loop(client_fd);
                done->store(true);
            });

            std::lock_guard<std::mutex> lock(g_clients_mutex);
            g_client_threads.push_back(ClientSlot{ std::move(th), done });
        }
    }
}

extern "C" int bd_stream_start(int port) {
    g_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_listen_fd < 0) return -1;

    int opt = 1;
    setsockopt(g_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port));

    if (bind(g_listen_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "bd_stream_start: bind auf Port %d fehlgeschlagen\n", port);
        close(g_listen_fd);
        g_listen_fd = -1;
        return -1;
    }
    if (listen(g_listen_fd, 4) < 0) {
        close(g_listen_fd);
        g_listen_fd = -1;
        return -1;
    }

    g_stream_running = true;
    g_accept_thread = std::thread(stream_accept_loop);
    return 0;
}

extern "C" void bd_set_stream_quality(int quality) {
    if (quality < 1)   quality = 1;
    if (quality > 100) quality = 100;
    g_jpeg_quality = quality;
}

extern "C" void bd_set_stream_scale(double factor) {
    if (factor < 0.1) factor = 0.1;
    if (factor > 1.0) factor = 1.0;
    g_stream_scale = factor;
}

extern "C" int bd_stream_push(BallDetector *bd, const DetectionResult *result) {
    if (!bd || !result || !g_stream_running) return -1;

    cv::Mat annotated = view_for_output(bd, result);
    if (annotated.empty()) return -1;

    double scale = g_stream_scale.load();
    if (scale < 0.999) {
        cv::Mat small;
        cv::resize(annotated, small, cv::Size(), scale, scale, cv::INTER_AREA);
        annotated = small;
    }

    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, g_jpeg_quality.load() };

    std::vector<uchar> jpeg_buf;
    if (!cv::imencode(".jpg", annotated, jpeg_buf, params)) return -1;

    {
        std::lock_guard<std::mutex> lock(g_frame_mutex);
        g_latest_jpeg = std::move(jpeg_buf);
        g_frame_seq++;
    }
    g_frame_cv.notify_all();
    return 0;
}

extern "C" void bd_stream_stop(void) {
    if (!g_stream_running) return;
    g_stream_running = false;

    /* Wartende Client-Threads aufwecken, sonst haengen sie bis zum
     * Zeitablauf in der Bedingungsvariablen. */
    g_frame_cv.notify_all();

    if (g_listen_fd >= 0) {
        shutdown(g_listen_fd, SHUT_RDWR);
        close(g_listen_fd);
        g_listen_fd = -1;
    }
    if (g_accept_thread.joinable()) g_accept_thread.join();

    std::lock_guard<std::mutex> lock(g_clients_mutex);
    for (auto &c : g_client_threads) {
        if (c.thread.joinable()) c.thread.join();
    }
    g_client_threads.clear();
}

extern "C" void bd_release(BallDetector *detector) {
    if (!detector) return;
    if (detector->use_camera && detector->pipe) {
        pclose(detector->pipe);
    }
    delete detector;
}
