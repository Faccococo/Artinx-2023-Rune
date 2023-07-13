#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

struct DetectResult{
    std::vector<cv::Point2f> key_points;
    cv::Point2f center;
};