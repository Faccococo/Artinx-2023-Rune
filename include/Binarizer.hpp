#pragma once
#include "config.hpp"
#include "utils.hpp"
#include <opencv2/opencv.hpp>

class Binarizer {
public:
    cv::Mat binarize(const cv::Mat& src, int threshold);
    cv::Mat b_r_binarize(const cv::Mat& src, int threshold);
    cv::Mat hsv_binarizer(const cv::Mat& src, const cv::Scalar& upper_bound, const cv::Scalar& lower_bound);
};