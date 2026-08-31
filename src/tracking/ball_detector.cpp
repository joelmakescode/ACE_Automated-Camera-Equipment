#include "ball_detector.h"

#include <opencv2/opencv.hpp>
#if CV_VERSION_MAJOR >= 5
/* OpenCV 5 verschob contourArea/minEnclosingCircle aus imgproc.hpp hierher;
 * auf OpenCV 4 (z.B. via apt auf Raspberry Pi OS) existiert dieser Header nicht. */
#include <opencv2/geometry/2d.hpp>
#endif
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

struct BallDetector {
    bool use_camera;
    FILE *pipe = nullptr;      /* rpicam-vid Subprozess (Kameramodus) */
    int cam_width = 0;
    int cam_height = 0;
    std::vector<unsigned char> yuv_buf;
    cv::Mat static_image;      /* Bildmodus */
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

/* Die Arducam IMX519 liefert ueber /dev/video0 (unicam) nur rohe, gepackte
 * Bayer-Daten - dafuer muesste die Media-Controller-Pipeline manuell
 * konfiguriert werden, was cv::VideoCapture(CAP_V4L2) nicht tut (fuehrt zu
 * "select() timeout"). Stattdessen startet rpicam-vid die Kamera ueber den
 * libcamera-Stack und liefert fertige YUV420-Frames ueber eine Pipe.
 * device_path wird aktuell nicht genutzt (rpicam-vid waehlt den erkannten
 * Sensor automatisch); Parameter bleibt fuer eine spaetere --camera-Auswahl
 * bei mehreren angeschlossenen Kameras erhalten. */
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
        /* YUV420-Planar (I420): Hoehe*1.5 Zeilen als 1-Kanal-Mat interpretieren,
         * dann in BGR konvertieren. */
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

extern "C" void bd_release(BallDetector *detector) {
    if (!detector) return;
    if (detector->use_camera && detector->pipe) {
        pclose(detector->pipe);
    }
    delete detector;
}
