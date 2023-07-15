#pragma once
#include <string>
#include <opencv2/opencv.hpp>

typedef void (*TrackbarCallBack)(int, void *);

static int window_size_x = 640;
static int window_size_y = 480;

static int add_trackbar(std::string window_name, std::string trackbar_name, TrackbarCallBack onTrackBarSlide, int max_value = 100)
{
    cv::createTrackbar(trackbar_name, window_name, NULL, max_value, onTrackBarSlide);
    return 0;
}

static int show_image(const cv::Mat &src, std::string window_name, std::string trackbar_name, TrackbarCallBack onTrackBarSlide, int max_value = 255)
{
    try
    {
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name, window_size_x, window_size_y);
        cv::createTrackbar(trackbar_name, window_name, NULL, max_value, onTrackBarSlide);
    }
    catch (cv::Exception)
    {
        // std::cout << "no image to show" << std::endl;
    }
    return 0;
}

static int show_image(const cv::Mat &src, std::string window_name)
{
    try
    {
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name, window_size_x, window_size_y);
        cv::imshow(window_name, src);
    }
    catch (cv::Exception)
    {
        // std::cout << "no image to show" << std::endl;
    }
    return 0;
}