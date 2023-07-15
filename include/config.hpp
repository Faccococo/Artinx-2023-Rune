#pragma once
#include <opencv2/opencv.hpp>

enum class Color
{
    Blue,
    Red
};

namespace config{
    static bool debug = true;
    static Color detect_color = Color::Red;
    const static std::string file_path = "/Users/huangzitong/workspace/Robomaster/Artinx-2023-Rune/data/images/red.png";
    // const static std::string file_path = "/Users/huangzitong/workspace/Robomaster/Artinx-2023-Rune/data/videos/blue_2.mp4";

    static int binary_threshold = 100;
    static int b_r_threshold = 15;
    static int dilate_kernel_size = 6;
    static double convex_hull_thresh = 0.5; // real ratio is convec_hull_thresh / 100
    static int min_contour_area = 100;


    static void set_binary_thresh(int pos, void* data)
    {
        config::binary_threshold = pos;
    }
    static void set_b_r_threshold(int pos, void* data)
    {
        config::b_r_threshold = pos;
    }
    static void set_convex_hull_thresh(int pos, void* data)
    {
        config::convex_hull_thresh = pos / 100.0;
    }
    static void set_min_contour_area(int pos, void* data)
    {
        config::min_contour_area = pos;
    }

}


