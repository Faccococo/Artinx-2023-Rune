#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "DetectResult.hpp"
#include "config.hpp"
#include "Binarizer.hpp"
#include "utils.hpp"

class Detector{
public:
    int run();

private:
    cv::Mat& read_frame();

    DetectResult detect(const cv::Mat& src, const cv::Mat& bin);

};

