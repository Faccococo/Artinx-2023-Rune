#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <fmt/format.h>

#include "DetectResult.hpp"
#include "config.hpp"
#include "Binarizer.hpp"
#include "utils.hpp"

struct Rune{
    std::vector<cv::Point> contour;
    cv::Rect2f roi;
};

class Detector{
public:
    int run();

private:
    cv::Mat& read_frame();

    std::optional<DetectResult> detect(const cv::Mat& src, const cv::Mat& bin);

};

