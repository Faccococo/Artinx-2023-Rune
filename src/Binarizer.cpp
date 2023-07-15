#include "Binarizer.hpp"


cv::Mat Binarizer::binarize(const cv::Mat& src, int threshold)
{   
    cv::Mat bin;
    cv::cvtColor(src, bin, cv::COLOR_BGR2GRAY);
    cv::threshold(bin, bin, threshold, 255, cv::THRESH_BINARY);
    if (config::debug)
    {
        show_image(bin, "bin");
    }
    return bin;
}

cv::Mat Binarizer::b_r_binarize(const cv::Mat& src, int threshold)
{
    cv::Mat bin, sub, b, g, r;
    std::vector<cv::Mat> img_channel;
    cv::split(src, img_channel);
    b = img_channel[0], g = img_channel[1], r = img_channel[2];
    sub = config::detect_color == Color::Blue? b - r : r - b;
    std::cout << config::b_r_threshold << std::endl;
    cv::threshold(sub, bin, threshold, 255, cv::THRESH_BINARY);
    if (config::debug)
    {
        show_image(bin, "bin");
    }
    return bin;
}
