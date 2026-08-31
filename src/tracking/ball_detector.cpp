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
#include <thread>
#include <mutex>
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
    range.h_min = 5; range.s_min = 100; range.v_min = 100;
    range.h_max = 25; range.s_max = 255; range.v_max = 255;
    return range;
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
                       " --framerate 15 -o - 2>/dev/null";

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

extern "C" int bd_detect(BallDetector *bd, const HsvRange *range, DetectionResult *out_result) {
    if (!bd || !range || !out_result) return -1;

    out_result->found = false;
    out_result->x = 0.0;
    out_result->y = 0.0;
    out_result->radius = 0.0;

    const cv::Mat *frame_ptr;
    if (bd->use_camera) {
        size_t n = std::fread(bd->yuv_buf.data(), 1, bd->yuv_buf.size(), bd->pipe);
        if (n != bd->yuv_buf.size()) {
            std::fprintf(stderr, "Failed to read frame from rpicam-vid\n");
            return -1;
        }

        cv::Mat yuv(bd->cam_height * 3 / 2, bd->cam_width, CV_8UC1, bd->yuv_buf.data());
        cv::cvtColor(yuv, bd->frame_buf, cv::COLOR_YUV2BGR_I420);
        frame_ptr = &bd->frame_buf;
    }
    else {
        frame_ptr = &bd->static_image;
    }

    cv::cvtColor(*frame_ptr, bd->hsv_buf, cv::COLOR_BGR2HSV);
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

    out_result->found  = true;
    out_result->x      = center.x;
    out_result->y      = center.y;
    out_result->radius = radius;
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

extern "C" int bd_save_annotated(BallDetector *bd, const DetectionResult *result, const char *out_path) {
    if (!bd || !result || !out_path) return -1;

    const cv::Mat &frame = bd->use_camera ? bd->frame_buf : bd->static_image;
    if (frame.empty()) return -1;

    cv::Mat annotated = draw_overlay(frame, result);
    return cv::imwrite(out_path, annotated) ? 0 : -1;
}

/* --- MJPEG-Live-Stream (multipart/x-mixed-replace) --- */

namespace {
    std::atomic<bool> g_stream_running{false};
    int g_listen_fd = -1;
    std::thread g_accept_thread;
    std::mutex g_frame_mutex;
    std::vector<uchar> g_latest_jpeg;
    std::mutex g_clients_mutex;
    std::vector<std::thread> g_client_threads;

    void stream_client_loop(int client_fd) {
        const char *header =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\n\r\n";
        if (send(client_fd, header, std::strlen(header), 0) < 0) {
            close(client_fd);
            return;
        }

        while (g_stream_running) {
            std::vector<uchar> jpeg_copy;
            {
                std::lock_guard<std::mutex> lock(g_frame_mutex);
                jpeg_copy = g_latest_jpeg;
            }
            if (jpeg_copy.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            char part_header[128];
            int hlen = std::snprintf(part_header, sizeof(part_header),
                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                jpeg_copy.size());
            if (send(client_fd, part_header, hlen, 0) < 0) break;
            if (send(client_fd, jpeg_copy.data(), jpeg_copy.size(), 0) < 0) break;
            if (send(client_fd, "\r\n", 2, 0) < 0) break;

            std::this_thread::sleep_for(std::chrono::milliseconds(66)); /* ~15 fps */
        }
        close(client_fd);
    }

    void stream_accept_loop() {
        while (g_stream_running) {
            sockaddr_in client_addr{};
            socklen_t addr_len = sizeof(client_addr);
            int client_fd = accept(g_listen_fd, reinterpret_cast<sockaddr *>(&client_addr), &addr_len);
            if (client_fd < 0) {
                if (!g_stream_running) break;
                continue;
            }
            std::lock_guard<std::mutex> lock(g_clients_mutex);
            g_client_threads.emplace_back(stream_client_loop, client_fd);
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

extern "C" int bd_stream_push(BallDetector *bd, const DetectionResult *result) {
    if (!bd || !result || !g_stream_running) return -1;

    const cv::Mat &frame = bd->use_camera ? bd->frame_buf : bd->static_image;
    if (frame.empty()) return -1;

    cv::Mat annotated = draw_overlay(frame, result);

    std::vector<uchar> jpeg_buf;
    cv::imencode(".jpg", annotated, jpeg_buf);

    std::lock_guard<std::mutex> lock(g_frame_mutex);
    g_latest_jpeg = std::move(jpeg_buf);
    return 0;
}

extern "C" void bd_stream_stop(void) {
    if (!g_stream_running) return;
    g_stream_running = false;

    if (g_listen_fd >= 0) {
        shutdown(g_listen_fd, SHUT_RDWR);
        close(g_listen_fd);
        g_listen_fd = -1;
    }
    if (g_accept_thread.joinable()) g_accept_thread.join();

    std::lock_guard<std::mutex> lock(g_clients_mutex);
    for (auto &t : g_client_threads) {
        if (t.joinable()) t.join();
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
