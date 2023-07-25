#pragma once
#include <opencv2/opencv.hpp>

enum class Color
{
    Blue,
    Red
};

namespace config
{
    static bool debug = true;
    static Color detect_color = Color::Blue;
    // const static std::string file_path = "/Users/huangzitong/workspace/Robomaster/Artinx-2023-Rune/data/images/red.png";
    // const static std::string file_path = "/Users/huangzitong/workspace/Robomaster/Artinx-2023-Rune/data/images/blue.png";
    const static std::string file_path = "/Users/huangzitong/workspace/Robomaster/Artinx-2023-Rune/data/videos/buff_blue.mp4";
    // const static std::string file_path = "/Users/huangzitong/workspace/Robomaster/Artinx-2023-Rune/data/videos/blue_2.mp4";
    // const static std::string file_path = "/Users/huangzitong/workspace/Robomaster/Artinx-2023-Rune/data/videos/red_sim_1.mp4";

    static int binary_threshold = 100;

    static int roi_binary_thresh = 50; // TODO

    static int b_r_threshold = 30;
    static int dilate_kernel_size = 11;
    static double min_convex_hull_thresh = 0.6;
    static double max_convex_hull_thresh = 0.85;
    static int min_contour_area = 100;

    static int hsv_offset = 0;

    static int lower_blue = 70;
    static int upper_blue = 180;

    static int lower_red = 100;
    static int upper_red = 180;

    static cv::Scalar lowerBlue(lower_blue, 0, 30);  // 蓝色范围的下限
    static cv::Scalar upperBlue(upper_blue, 255, 255); // 蓝色范围的上限

    static cv::Scalar lowerRed(lower_red, 50, 50);
    static cv::Scalar upperRed(upper_red, 255, 255);

    static void set_binary_thresh(int pos, void *data)
    {
        config::binary_threshold = pos * 10;
    }

    static void set_roi_binary_thresh(int pos, void *data)
    {
        config::roi_binary_thresh = pos * 10;
    }
    static void set_b_r_threshold(int pos, void *data)
    {
        config::b_r_threshold = pos * 10;
        std::cout << config::b_r_threshold << std::endl;
    }
    static void set_dilate_kernel_size(int pos, void *data)
    {
        config::dilate_kernel_size = pos;
    }
    static void set_max_convex_hull_thresh(int pos, void *data)
    {
        config::max_convex_hull_thresh = pos / 100.0;
    }
    static void set_min_convex_hull_thresh(int pos, void *data)
    {
        config::min_convex_hull_thresh = pos / 100.0;
    }
    static void set_min_contour_area(int pos, void *data)
    {
        config::min_contour_area = pos * 100;
    }

}
