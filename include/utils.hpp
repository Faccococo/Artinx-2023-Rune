#pragma once
#include <string>
#include <opencv2/opencv.hpp>

typedef void (*TrackbarCallBack)(int, void*);

static int show_image(const cv::Mat &src, std::string windows_name, std::string trackbar_name, TrackbarCallBack onTrackBarSlide, int max_value=255)
{
    cv::imshow(windows_name, src);
    cv::createTrackbar(trackbar_name, windows_name, NULL, max_value, onTrackBarSlide);
    return 0;
}

static int show_image(const cv::Mat &src, std::string window_name)
{
    cv::imshow(window_name, src);
    return 0;
}
