#include "Detector.hpp"

static Binarizer binarizer;

std::optional<cv::Mat> getROI(const cv::Mat &src, const cv::Mat &bin)
{
    // 二值化
    auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(config::dilate_kernel_size, config::dilate_kernel_size));
    cv::dilate(bin, bin, kernel);
    if (config::debug)
    {
        show_image(bin, "dilate");
    }

    // 提取轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bin, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    if (contours.size() <= 0)
    {
        std::cout << "no contour found!!!" << std::endl;
    }
    // std::cout << "src have " << contours.size() << " contours." << std::endl;

    // 遍历轮廓
    double max_area = 0;
    int max_contour_index = -1;

    std::vector<std::vector<cv::Point>> candidate_contours;
    std::vector<cv::Point> target_contour;
    std::vector<std::vector<cv::Point>> hulls;

    for (int i = 0; i < contours.size(); i++)
    {

        double contour_area = cv::contourArea(contours[i]);

        // 面积过小
        // if (contour_area < config::min_contour_area)
        // {
        //     continue;
        // }

        // 有子轮廓
        if (hierarchy[i][2] != -1)
        {
            continue;
        }

        // 凸包检测
        std::vector<cv::Point> hull;
        cv::convexHull(contours[i], hull);

        // 计算轮廓面积与轮廓凸包面积比
        double hull_area = cv::contourArea(hull);
        double ratio = contour_area / hull_area;

        // std::cout << config::min_convex_hull_thresh << std::endl;
        // 筛掉凸包轮廓
        if (ratio > config::max_convex_hull_thresh || ratio < config::min_convex_hull_thresh)
        {
            continue;
        }

        hulls.push_back(hull);
        candidate_contours.push_back(contours[i]);

        if (contour_area > max_area)
        {
            max_area = contour_area;
            max_contour_index = i;
            target_contour = contours[i];
        }
    }
    // std::cout << candidate_contours.size() << std::endl;
    if (config::debug)
    {
        cv::Mat candidate_contour_img = cv::Mat::zeros(src.size(), src.type());
        if (candidate_contours.size() > 0 && hulls.size() > 0)
        {
            for (auto contour : candidate_contours)
            {
                cv::Moments mom = cv::moments(contour);
                cv::Point center(mom.m10 / mom.m00, mom.m01 / mom.m00);
                auto area = cv::contourArea(contour);
                std::vector<cv::Point> hull_debug;
                cv::convexHull(contour, hull_debug);
                auto hull_area = cv::contourArea(hull_debug);
                cv::putText(candidate_contour_img, fmt::format("{:.2f}", area / hull_area), center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            }
            cv::drawContours(candidate_contour_img, contours, -1, cv::Scalar(255, 255, 255), 3);
            cv::drawContours(candidate_contour_img, hulls, -1, cv::Scalar(255, 0, 255), 3);
            cv::drawContours(candidate_contour_img, candidate_contours, -1, cv::Scalar(0, 255, 255), 3);
            cv::drawContours(candidate_contour_img, contours, max_contour_index, cv::Scalar(0, 0, 255), 3);
        }
        // std::cout << "candidate contour number: " << candidate_contours.size() << std::endl;
        show_image(candidate_contour_img, "target");
    }
    if (max_contour_index != -1)
    {
        Rune targetRune;
        targetRune.contour = target_contour;
        cv::Rect2f roi = cv::boundingRect(targetRune.contour);
        cv::Mat roi_img = src(roi);
        return roi_img;
    }
}

std::optional<DetectResult> Detector::detect(const cv::Mat &src, const cv::Mat &bin)
{
    auto roi_img_optional = getROI(src, bin);
    if (!roi_img_optional.has_value())
        return std::nullopt;
    cv::Mat roi_img = roi_img_optional.value();
    if (config::debug)
    {
        show_image(roi_img, "target roi");
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
        bool success = cap.read(frame);
        if (!success)
        {
            cap.set(cv::CAP_PROP_POS_FRAMES, 0); // 将视频帧设置为起始位置，以重复播放
            cap.read(frame);
        }
    }
    else if ((config::file_path.find(".jpg") != std::string::npos ||
              config::file_path.find(".png") != std::string::npos) &&
             !img_read)
    {
        // 输入为图片
        frame = cv::imread(config::file_path);
        img_read = true;
    }
    return frame;
}

int Detector::run()
{
    DetectResult result;
    cv::namedWindow("src", cv::WINDOW_NORMAL);
    add_trackbar("src", "BinThresh", config::set_binary_thresh, 25);
    add_trackbar("src", "BRThresh", config::set_b_r_threshold, 25);
    add_trackbar("src", "kernelSize", config::set_dilate_kernel_size, 21);
    add_trackbar("src", "maxAreaRatio", config::set_max_convex_hull_thresh, 100);
    add_trackbar("src", "minAreaRatio", config::set_min_convex_hull_thresh, 100);
    add_trackbar("src", "minArea", config::set_min_contour_area);
    while (true)
    {

        cv::Mat src = read_frame();
        if (config::debug)
        {
            cv::imshow("src", src);
        }
        auto startTime = std::chrono::high_resolution_clock::now();
        cv::Mat bin = binarizer.binarize(src, config::binary_threshold);
        // cv::Mat bin = binarizer.b_r_binarize(src, config::b_r_threshold);
        auto maybe_result = detect(src, bin);
        auto endTime = std::chrono::high_resolution_clock::now();
        std::cout << "Execution time: " 
                << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() 
                << " milliseconds" << std::endl;

        if (maybe_result.has_value())
            result = maybe_result.value();
        if (cv::waitKey(1) == 27)
        {
            break;
        }
    }
    return 0;
}
