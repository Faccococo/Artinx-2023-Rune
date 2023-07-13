#include "Detector.hpp"

static Binarizer binarizer;

int Detector::run()
{
    DetectResult result;
    while (true)
    {
        // std::cout << "threshold valur is " << config::binary_threshold << std::endl;
        cv::Mat src = read_frame();
        if (config::debug)
        {
            show_image(src, "src");
        }
        cv::Mat bin = binarizer.b_r_binarize(src);
        result = detect(src, bin);
        if (cv::waitKey(1) == 27)
        {
            break;
        }
    }
    return 0;
}

DetectResult Detector::detect(const cv::Mat& src, const cv::Mat &bin)
{

    // 二值化
    auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(config::dilate_kernel_size, config::dilate_kernel_size));
    cv::dilate(bin, bin, kernel);
    if (config::debug)
    {
        // show_image(bin, "dilate");
    }

    // 提取轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.size() <= 0)
    {
        std::cout << "no contour found!!!" << std::endl;
    }
    std::cout << "src have " << contours.size() << " contours." << std::endl;

    //绘制轮廓
    // if (config::debug)
    // {
    //     cv::Mat contour_img = cv::Mat::zeros(src.size(), src.type());
    //     cv::drawContours(contour_img, contours, -1, cv::Scalar(255, 255, 255), 3);
    //     show_image(contour_img, "contours");
    // }

    //遍历轮廓
    double minArea = std::numeric_limits<double>::max();
    int minContourIndex = -1;

    std::vector<std::vector<cv::Point>> candidate_contours;

    for (int i = 0; i < contours.size(); i++) {
        // Calculate contour area
        double contourArea = cv::contourArea(contours[i]);

        // Calculate convex hull
        std::vector<cv::Point> hull;
        cv::convexHull(contours[i], hull);

        // Calculate convex hull area
        double hullArea = cv::contourArea(hull);

        // Calculate ratio
        double ratio = contourArea / hullArea;

        if (ratio < config::convex_hull_thresh)
        {
            continue;
        }

        candidate_contours.push_back(contours[i]);

        // If ratio is below threshold and area is smaller than current minimum
        if (contourArea < minArea) {
            minArea = contourArea;
            minContourIndex = i;
        }
    }

    if (config::debug)
    {
        cv::Mat candidate_contour_img = cv::Mat::zeros(src.size(), src.type());
        if(candidate_contours.size() > 0)
        {
            cv::drawContours(candidate_contour_img, candidate_contour_img, -1, cv::Scalar(255, 255, 255), 3);
            cv::drawContours(candidate_contour_img, candidate_contours, minContourIndex, cv::Scalar(0, 0, 255), 3);
        }
        std::cout << "candidate contour number: " << candidate_contours.size() << std::endl;
        show_image(candidate_contour_img, "R", "convex_thresh", config::set_convex_hull_thresh, 100);
    }



    DetectResult result;
    return result;
}

cv::Mat &Detector::read_frame()
{
    static cv::Mat frame;
    static bool img_read = false; // 是否已经读取过图像
    if (config::file_path.find(".mp4") != std::string::npos)
    {
        // 输入为视频
        static cv::VideoCapture cap(config::file_path);
        cap >> frame;
    }
    else if ((config::file_path.find(".jpg") != std::string::npos ||
     config::file_path.find(".png") != std::string::npos) 
     && !img_read)
    {
        // 输入为图片
        frame = cv::imread(config::file_path);
        img_read = true;
    }
    return frame;
}


