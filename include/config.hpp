#pragma once
#include "utils.hpp"
#include <opencv2/opencv.hpp>

enum class Color { Blue, Red };

namespace config {
    static bool debug = true;
    static Color detect_color = Color::Red;
    // const static std::string file_path = "/Users/huangzitong/workspace/Robomaster/Artinx-2023-Rune/data/images/red.png";
    // const static std::string file_path = "/Users/huangzitong/workspace/Robomaster/Artinx-2023-Rune/data/images/blue.png";
    // const static std::string file_path = "/home/static/workspace/Artinx-2023-Rune/data/videos/buff_blue.mp4";
    const static std::string file_path = "/home/static/workspace/Artinx-2023-Rune/data/images/2023_7_26 phone photo/r6.jpg";
    // const static std::string file_path = "/Users/huangzitong/workspace/Robomaster/Artinx-2023-Rune/data/videos/blue_2.mp4";
    // const static std::string file_path = "/Users/huangzitong/workspace/Robomaster/Artinx-2023-Rune/data/videos/red_sim_1.mp4";

    static int bin_low_threshold = 100;
    static int bin_high_threshold = 55;
    static int bin_logo_threshold = 100;
    static int b_r_threshold = 30;
    static int rough_dilate_kernel_size = 8;
    static double min_rough_target_hull_thresh = 0.6;
    static double max_rough_target_hull_thresh = 0.85;  // real ratio is convec_hull_thresh / 100
    static int min_rough_target_area = 100;

    static int dilate_kernel_size = 11;
    static double min_teeth_A_hull_thresh = 0.6;
    static double max_teeth_A_hull_thresh = 0.85;
    static double min_teeth_B_hull_thresh = 0.6;
    static double max_teeth_B_hull_thresh = 0.85;
    static int min_teeth_area = 100;
    static double logo_roi_size = 0.6;     // refer to the 700mm radius of Rune
    static double logo_size_tolerance = 0.3;

    static double erase_ratio = 0.1;

    static int hsv_offset = 0;

    static int lower_blue = 70;
    static int upper_blue = 180;

    static int lower_red = 100;
    static int upper_red = 180;

    static cv::Scalar lowerBlue(lower_blue, 0, 30);     // 蓝色范围的下限
    static cv::Scalar upperBlue(upper_blue, 255, 255);  // 蓝色范围的上限

    static cv::Scalar lowerRed(lower_red, 0, 30);
    static cv::Scalar upperRed(upper_red, 255, 255);

    // // 修改某个滑块变量的回调函数(但是搞不动)
    //     template <typename T>
    //     static TrackbarCallBack slide_setter(T& var, int multiplier = 1) {
    //         return [&var, multiplier](int pos, void* data) -> void { var = pos * multiplier; };
    //     }

    static void set_bin_low_thresh(int pos, void* data) {
        config::bin_low_threshold = pos * 10;
    }
    static void set_bin_high_thresh(int pos, void* data) {
        config::bin_high_threshold = pos;
    }
    static void set_bin_logo_thresh(int pos, void* data) {
        config::bin_logo_threshold = pos;
    }
    static void set_lower_h(int pos, void* data) {
        if(config::detect_color == Color::Blue) {
            config::lowerBlue[0] = pos;
        } else {
            config::lowerRed[0] = pos;
        }
    }
    static void set_upper_h(int pos, void* data) {
        if(config::detect_color == Color::Blue) {
            config::upperBlue[0] = pos;
        } else {
            config::upperRed[0] = pos;
        }
    }

    // static void set_roi_binary_thresh(int pos, void *data)
    // {
    //     config::roi_binary_thresh = pos * 10;
    // }
    static void set_b_r_threshold(int pos, void* data) {
        config::b_r_threshold = pos * 10;
        std::cout << config::b_r_threshold << std::endl;
    }
    static void set_rough_dilate_kernel_size(int pos, void* data) {
        config::rough_dilate_kernel_size = pos;
    }
    static void set_min_rough_target_hull_thresh(int pos, void* data) {
        config::min_rough_target_hull_thresh = pos / 100.0;
    }
    static void set_max_rough_target_hull_thresh(int pos, void* data) {
        config::max_rough_target_hull_thresh = pos / 100.0;
    }
    static void set_min_rough_target_area(int pos, void* data) {
        config::min_rough_target_area = pos * 100;
    }
    static void set_min_teeth_area(int pos, void* data) {
        config::min_teeth_area = pos * 100;
    }
    static void set_logo_roi_size(int pos, void* data) {
        config::logo_roi_size = pos * 0.01;
    }

}  // namespace config
