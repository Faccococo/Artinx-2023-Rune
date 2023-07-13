#pragma once
#include <opencv2/opencv.hpp>
#include "config.hpp"
#include "utils.hpp"

class Binarizer{
public:
    cv::Mat binarize(const cv::Mat& src);
    cv::Mat b_r_binarize(const cv::Mat& src);
};