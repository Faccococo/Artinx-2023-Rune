#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <optional>
#include <fmt/format.h>

#include "DetectResult.hpp"
#include "config.hpp"
#include "Binarizer.hpp"
#include "utils.hpp"

struct Rune{
    std::vector<cv::Point> contour;
    cv::Rect2f roi;
};

struct TargetLeaf {
    std::vector<cv::Point> contour_teeth_A;
    std::vector<cv::Point> contour_teeth_B;
    cv::Rect2f roi;
};

struct LogoROI{
    cv::Point2f center;
    double radius;
    double expected_size;
};

class Detector {
public:
    int run();

private:

    cv::Mat src;
    cv::Mat bin_low, bin_high, bin_logo;
    cv::Point2f last_center = cv::Point2f(0, 0);

    cv::Mat& read_frame();
    std::optional<DetectResult> detect(const cv::Mat& src);
    std::optional<cv::Rect> getTargetLeafROI(const cv::Mat& src, const cv::Mat& bin);
    std::optional<TargetLeaf> getTargetLeaf(const cv::Mat& src, const cv::Mat& bin, const cv::Rect& roi);
    std::optional<LogoROI> getLogoROI(const cv::Mat& src, const cv::Mat& bin);

};

