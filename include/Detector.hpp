#pragma once
#include <array>
#include <fmt/format.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

#include "Binarizer.hpp"
#include "config.hpp"
#include "utils.hpp"

struct Rune {
    std::vector<cv::Point> contour;
    cv::Rect2f roi;
};

// key_points:
// 0 & 1 = A's outer edge
// 2 & 3 = B's inner edge
// clockwise
struct TargetLeaf {
    cv::RotatedRect rect_A;
    cv::RotatedRect rect_B;
    std::vector<cv::Point2f> key_points;
    double orientation;     // orientation of ray out from rune center, -pi ~ pi
    cv::Rect2f roi;
};

struct DetectResult {
    std::vector<cv::Point2f> key_points;
    cv::Point2f center;
};

class Detector {
public:
    int run();

private:
    cv::Mat src;
    cv::Mat bin_low, bin_high, bin_logo;

    double OUTER_A_RADIUS = 655;
    double INNER_B_RADIUS = 747;
    double AIM_CENTER_RADIUS = 700;
    double LOGO_SIZE = 52 * std::sqrt(2);

    cv::Mat& read_frame();
    std::optional<DetectResult> detect(const cv::Mat& src);
    std::optional<cv::Rect> getTargetLeafROI(const cv::Mat& src, const cv::Mat& bin);
    std::optional<TargetLeaf> getTargetLeaf(const cv::Mat& src, const cv::Mat& bin, const cv::Rect& roi);
    std::optional<std::tuple<cv::Rect, double>> getLogoROIAndSize(const TargetLeaf& target_leaf, cv::MatSize src_size);
    std::optional<cv::Rect> getLogo(const cv::Mat& src, const cv::Mat& bin, const cv::Rect& logo_roi, double expected_size);
};
