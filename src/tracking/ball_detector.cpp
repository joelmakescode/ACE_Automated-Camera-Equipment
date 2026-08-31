#include "ball_detector.h"

#include <opencv2/opencv.hpp>
#if CV_VERSION_MAJOR >= 5
/* OpenCV 5 verschob contourArea/minEnclosingCircle aus imgproc.hpp hierher;
 * auf OpenCV 4 (z.B. via apt auf Raspberry Pi OS) existiert dieser Header nicht. */
#include <opencv2/geometry/2d.hpp>
#endif
#include <cstdio>
#include <vector>

struct BallDetector {
    bool use_camera;
    cv::VideoCapture capture;
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
    BallDetector *bd = new BallDetector();
    bd->use_camera = true;
    bd->capture.open(device_path, cv::CAP_V4L2);
    if (!bd -> capture.isOpened()) {
        std::fprintf(stderr, "Failed to open camera\n");
        delete bd;
        return nullptr;
    }

    if (width > 0) bd->capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
    if (height > 0) bd->capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);

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
        if (!bd->capture.read(bd->frame_buf) || bd->frame_buf.empty()) {
            std::fprintf(stderr, "Failed to read frame\n");
            return -1;
        }
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
    if (detector->use_camera && detector->capture.isOpened()) {
        detector->capture.release();
    }
    delete detector;
}
