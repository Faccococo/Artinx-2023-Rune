#include "Detector.hpp"

static Binarizer binarizer;

using Contour = std::vector<cv::Point>;

std::optional<cv::Rect> Detector::getTargetLeafROI(const cv::Mat& src, const cv::Mat& bin) {
    // 二值化
    auto open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN, open_kernel);

    if(config::rough_dilate_kernel_size % 2 == 0) {  // 避免kernel_size为偶数或0
        config::rough_dilate_kernel_size += 1;
    }
    auto close_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                  cv::Size(config::rough_dilate_kernel_size, config::rough_dilate_kernel_size));
    cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, close_kernel);
    // cv::dilate(bin, bin, kernel);
    if(config::debug) {
        show_image(bin, "dilate");
    }

    // 提取轮廓
    std::vector<Contour> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bin, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    if(contours.size() <= 0) {
        std::cout << "no contour found!!!" << std::endl;
    }
    // std::cout << "src have " << contours.size() << " contours." << std::endl;

    // 遍历轮廓
    double max_area = 0;
    int max_contour_index = -1;

    std::vector<Contour> candidate_contours;
    Contour target_contour;
    std::vector<Contour> hulls;

    for(int i = 0; i < contours.size(); i++) {

        double contour_area = cv::contourArea(contours[i]);
        // 面积过小
        if(contour_area < config::min_rough_target_area) {
            continue;
        }
        // 有父/子轮廓
        if(hierarchy[i][2] != -1 || hierarchy[i][3] != -1) {
            continue;
        }

        // 凸包检测
        Contour hull;
        cv::convexHull(contours[i], hull);

        // 计算轮廓面积与轮廓凸包面积比
        double hull_area = cv::contourArea(hull);
        double ratio = contour_area / hull_area;

        // std::cout << config::min_rough_target_hull_thresh << std::endl;
        // 筛掉凸包轮廓
        if(ratio > config::max_rough_target_hull_thresh || ratio < config::min_rough_target_hull_thresh) {
            continue;
        }

        hulls.push_back(hull);
        candidate_contours.push_back(contours[i]);

        // 寻找符合条件的最大轮廓
        if(contour_area > max_area) {
            max_area = contour_area;
            max_contour_index = i;
            target_contour = contours[i];
        }
    }
    // std::cout << candidate_contours.size() << std::endl;
    if(config::debug) {
        cv::Mat candidate_contour_img = cv::Mat::zeros(src.size(), src.type());
        if(candidate_contours.size() > 0 && hulls.size() > 0) {
            for(auto contour : candidate_contours) {
                cv::Moments mom = cv::moments(contour);
                cv::Point center(mom.m10 / mom.m00, mom.m01 / mom.m00);
                auto area = cv::contourArea(contour);
                Contour hull_debug;
                cv::convexHull(contour, hull_debug);
                auto hull_area = cv::contourArea(hull_debug);
                cv::putText(candidate_contour_img, fmt::format("{:.2f}", area / hull_area), center, cv::FONT_HERSHEY_SIMPLEX, 1,
                            cv::Scalar(255, 255, 255), 2);
            }
            cv::drawContours(candidate_contour_img, contours, -1, cv::Scalar(255, 255, 255), 3);
            cv::drawContours(candidate_contour_img, hulls, -1, cv::Scalar(255, 0, 255), 3);
            cv::drawContours(candidate_contour_img, candidate_contours, -1, cv::Scalar(0, 255, 255), 3);
            cv::drawContours(candidate_contour_img, contours, max_contour_index, cv::Scalar(0, 0, 255), 3);
        }
        // std::cout << "candidate contour number: " << candidate_contours.size() << std::endl;
        show_image(candidate_contour_img, "target");
    }
    if(max_contour_index != -1) {
        return cv::boundingRect(target_contour);
    }
    return std::nullopt;
}

std::optional<TargetLeaf> Detector::getTargetLeaf(const cv::Mat& src, const cv::Mat& bin, const cv::Rect& roi) {
    cv::Mat roi_bin = bin(roi);
    // // 闭运算
    // auto close_kernel =
    //     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(config::dilate_kernel_size, config::dilate_kernel_size));
    // cv::morphologyEx(roi_bin, roi_bin, cv::MORPH_CLOSE, close_kernel);
    // if(config::debug) {
    //     show_image(roi_bin, "getTargetLeaf: close");
    // }

    // 提取轮廓
    std::vector<Contour> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(roi_bin, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if(contours.size() <= 0) {
        std::cout << "no contour found!!!" << std::endl;
    }

    // 遍历轮廓
    double area_A_record = 0;
    double area_B_record = 0;
    int max_contour_A_index = -1;
    int max_contour_B_index = -1;

    std::vector<Contour> candidate_contours;
    // std::vector<Contour> hulls_A;
    TargetLeaf target_leaf_local;
    target_leaf_local.roi = roi;

    for(int i = 0; i < contours.size(); i++) {

        double contour_area = cv::contourArea(contours[i]);
        // 面积过小
        if(contour_area < config::min_teeth_area) {
            continue;
        }
        // // 凸包检测
        // Contour hull;
        // cv::convexHull(contours[i], hull);

        // // 计算轮廓面积与轮廓凸包面积比
        // double hull_area = cv::contourArea(hull);
        // double ratio = contour_area / hull_area;

        // // std::cout << config::min_rough_target_hull_thresh << std::endl;
        // // 分类凸包轮廓
        // if(ratio > config::max_rough_target_hull_thresh || ratio < config::min_rough_target_hull_thresh) {
        //     continue;
        // }

        // hulls.push_back(hull);
        candidate_contours.push_back(contours[i]);

        // 寻找符合条件的最大轮廓
        if(contour_area > area_A_record) {
            area_B_record = area_A_record;
            area_A_record = contour_area;
            max_contour_B_index = max_contour_A_index;
            max_contour_A_index = i;
        } else if(contour_area > area_B_record) {
            area_B_record = contour_area;
            max_contour_B_index = i;
        }
    }
    if(area_A_record < config::min_teeth_area || area_B_record < config::min_teeth_area) {
        return std::nullopt;
    }
    cv::RotatedRect rect_A = cv::minAreaRect(contours[max_contour_A_index]);
    cv::RotatedRect rect_B = cv::minAreaRect(contours[max_contour_B_index]);
    cv::Point2f vertices_A[4];
    cv::Point2f vertices_B[4];
    rect_A.points(vertices_A);
    rect_B.points(vertices_B);
    target_leaf_local.key_points.resize(4);  // not initialized
    // orientation of vec{AB}
    target_leaf_local.orientation = std::atan2(rect_B.center.y - rect_A.center.y, rect_B.center.x - rect_A.center.x);
    // std::cout << rect_A.angle << " " << rect_B.angle << " " << target_leaf_local.orientation << std::endl;
    double rel_orient_A = target_leaf_local.orientation - rect_A.angle / 180 * CV_PI;
    double rel_orient_B = target_leaf_local.orientation - rect_B.angle / 180 * CV_PI;
    if(rel_orient_A < -CV_PI)
        rel_orient_A += 2 * CV_PI;
    if(rel_orient_B < -CV_PI)
        rel_orient_B += 2 * CV_PI;
    // [0] of Rotated Rect is at br now!
    std::array<int, 2> iA, iB;
    if(rel_orient_A < -3 * CV_PI / 4 || rel_orient_A > 3 * CV_PI / 4) {
        iA = { 1, 0 };
    } else if(rel_orient_A < -CV_PI / 4) {
        iA = { 2, 1 };
    } else if(rel_orient_A < CV_PI / 4) {
        iA = { 3, 2 };
    } else {
        iA = { 0, 3 };
    }
    if(rel_orient_B < -3 * CV_PI / 4 || rel_orient_B > 3 * CV_PI / 4) {
        iB = { 3, 2 };
    } else if(rel_orient_B < -CV_PI / 4) {
        iB = { 0, 3 };
    } else if(rel_orient_B < CV_PI / 4) {
        iB = { 1, 0 };
    } else {
        iB = { 2, 1 };
    }
    target_leaf_local.key_points = { vertices_A[iA[0]], vertices_A[iA[1]], vertices_B[iB[0]], vertices_B[iB[1]] };

    if(config::debug) {
        cv::Mat debug_img = cv::Mat::zeros(roi_bin.size(), src.type());
        cv::drawContours(debug_img, std::vector<Contour>{ contours[max_contour_A_index] }, -1, cv::Scalar(0, 255, 255), 3);
        cv::drawContours(debug_img, std::vector<Contour>{ contours[max_contour_B_index] }, -1, cv::Scalar(0, 255, 255), 3);
        for(int i = 0; i < 3; i++) {
            cv::line(debug_img, vertices_A[i], vertices_A[(i + 1) % 4], cv::Scalar(255, 255, 0), 2);
            cv::line(debug_img, vertices_B[i], vertices_B[(i + 1) % 4], cv::Scalar(255, 255, 0), 2);
            cv::putText(debug_img, std::to_string(i), vertices_A[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
            cv::putText(debug_img, std::to_string(i), vertices_B[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
            cv::line(debug_img, static_cast<cv::Point2i>(target_leaf_local.key_points[i]),
                     static_cast<cv::Point2i>(target_leaf_local.key_points[(i + 1) % 4]), cv::Scalar(255, 255, 0), 2);
        }
        show_image(debug_img, "target leaf points");
    }

    TargetLeaf target_leaf;

    target_leaf.rect_A = rect_A;
    target_leaf.rect_B = rect_B;
    target_leaf.rect_A.center += cv::Point2f(roi.x, roi.y);
    target_leaf.rect_B.center += cv::Point2f(roi.x, roi.y);

    target_leaf.key_points = target_leaf_local.key_points;
    for(auto& point : target_leaf.key_points) {
        point.x += roi.x;
        point.y += roi.y;
    }

    target_leaf.orientation = target_leaf_local.orientation;

    return target_leaf;
}

std::optional<std::tuple<cv::Rect, double>> Detector::getLogoROIAndSize(const TargetLeaf& target_leaf, cv::MatSize src_size) {
    double lerp_coeff = -OUTER_A_RADIUS / (INNER_B_RADIUS - OUTER_A_RADIUS);
    cv::Rect roi;
    cv::Point2f mid_A = (target_leaf.key_points[0] + target_leaf.key_points[1]) / 2;
    cv::Point2f mid_B = (target_leaf.key_points[2] + target_leaf.key_points[3]) / 2;
    cv::Point2f center = (1 - lerp_coeff) * mid_A + lerp_coeff * mid_B;
    double scale = cv::norm(mid_A - mid_B) / (INNER_B_RADIUS - OUTER_A_RADIUS);
    double roi_size = scale * AIM_CENTER_RADIUS * config::logo_roi_size;
    roi.width = roi.height = roi_size;
    roi.x = center.x - roi_size / 2;
    roi.y = center.y - roi_size / 2;
    if(center.x - roi_size / 2 < 0 || center.y - roi_size / 2 < 0 || center.x + roi_size / 2 > src_size[1] ||
       center.y + roi_size / 2 > src_size[0]) {
        std::cout << "logo roi out of range!!!" << std::endl;
        return std::nullopt;
    }
    return { { roi, scale * LOGO_SIZE } };
}

std::optional<cv::Rect> Detector::getLogo(const cv::Mat& src, const cv::Mat& bin, const cv::Rect& logo_roi,
                                          double expected_size) {
    cv::Mat roi_bin = bin(logo_roi);
    std::vector<Contour> contours;
    cv::findContours(roi_bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if(contours.size() <= 0) {
        std::cout << "no logo found!!!" << std::endl;
    }

    double min_error_record = std::numeric_limits<double>::max();
    double contourIdx = -1;
    cv::Rect min_error_rect;
    bool found = false;
    for(int i = 0; i < contours.size(); i++) {
        // circle roi
        bool out = false;
        for(int j = 0; j < contours[i].size(); j++) {
            if(cv::norm(contours[i][j] - cv::Point(logo_roi.width / 2, logo_roi.height / 2)) > logo_roi.width / 2) {
                out = true;
                break;
            }
        }
        if(out) {
            continue;
        }
        cv::Rect rect = cv::boundingRect(contours[i]);
        double error = std::abs(rect.width - expected_size) + std::abs(rect.height - expected_size);
        if(error < min_error_record) {
            min_error_record = error;
            min_error_rect = rect;
            contourIdx = i;
            found = true;
        }
    }
    std::cout << min_error_rect.width << " " << expected_size << std::endl;
    if(found && min_error_record < expected_size * config::logo_size_tolerance) {
        if(config::debug) {
            cv::Mat logo_result = cv::Mat::zeros(roi_bin.size(), src.type());
            cv::rectangle(logo_result, min_error_rect, cv::Scalar(0, 255, 0), 2);
            cv::drawContours(logo_result, contours, contourIdx, cv::Scalar(0, 0, 255), 2);
            show_image(logo_result, "logo result");
        }
        min_error_rect.x += logo_roi.x;
        min_error_rect.y += logo_roi.y;
        return min_error_rect;
    }
    return std::nullopt;
}

std::optional<DetectResult> Detector::detect(const cv::Mat& src) {
    auto leaf_roi = getTargetLeafROI(src, bin_low);
    if(!leaf_roi.has_value())
        return std::nullopt;
    cv::Mat leaf_roi_img = src(leaf_roi.value());
    cv::Mat leaf_roi_bin = bin_high(leaf_roi.value());

    if(config::debug) {
        show_image(leaf_roi_img, "target leaf roi");
        show_image(leaf_roi_bin, "roi bin");
    }
    auto target_leaf = getTargetLeaf(src, bin_high, leaf_roi.value());
    if(!target_leaf.has_value())
        return std::nullopt;

    auto temp = getLogoROIAndSize(target_leaf.value(), src.size);
    if(!temp.has_value())
        return std::nullopt;
    auto [logo_roi, expected_size] = temp.value();
    if(config::debug) {
        cv::Mat logo_roi_locate = src.clone();
        cv::rectangle(logo_roi_locate, logo_roi, cv::Scalar(0, 255, 0), 2);
        for(int i = 0; i < 4; i++) {
            cv::putText(logo_roi_locate, std::to_string(i), target_leaf.value().key_points[i], cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(255, 255, 255), 2);
            cv::line(logo_roi_locate, static_cast<cv::Point2i>(target_leaf.value().key_points[i]),
                     static_cast<cv::Point2i>(target_leaf.value().key_points[(i + 1) % 4]), cv::Scalar(255, 255, 0), 2);
        }
        show_image(logo_roi_locate, "logo roi location");
    }
    cv::Mat logo_roi_img = src(logo_roi);
    cv::Mat logo_roi_bin = bin_logo(logo_roi);
    if(config::debug) {
        show_image(logo_roi_img, "logo roi");
        show_image(logo_roi_bin, "logo roi bin");
    }
    auto logo = getLogo(src, bin_logo, logo_roi, expected_size);
    if(!logo.has_value())
        return std::nullopt;

    DetectResult result;
    result.key_points = target_leaf.value().key_points;
    result.center = logo.value().tl() + cv::Point(logo.value().width / 2, logo.value().height / 2);
    return result;
}

cv::Mat& Detector::read_frame() {
    static cv::Mat frame;
    static bool img_read = false;  // 是否已经读取过图像
    if(config::file_path.find(".mp4") != std::string::npos || config::file_path.find(".MP4") != std::string::npos) {
        // 输入为视频
        static cv::VideoCapture cap(config::file_path);
        bool success = cap.read(frame);
        if(!success) {
            cap.set(cv::CAP_PROP_POS_FRAMES, 0);  // 将视频帧设置为起始位置，以重复播放
            cap.read(frame);
        }
    } else if((config::file_path.find(".jpg") != std::string::npos || config::file_path.find(".png") != std::string::npos) &&
              !img_read) {
        // 输入为图片
        frame = cv::imread(config::file_path);
        img_read = true;
    }
    return frame;
}

int Detector::run() {
    DetectResult result;
    cv::namedWindow("src", cv::WINDOW_NORMAL);
    add_trackbar("src", "BinLowThresh", config::set_bin_low_thresh, 25);
    add_trackbar("src", "BinHighThresh", config::set_bin_high_thresh, 255);
    add_trackbar("src", "BinLogoThresh", config::set_bin_logo_thresh, 255);
    add_trackbar("src", "BRThresh", config::set_b_r_threshold, 25);
    add_trackbar("src", "lower_hue", config::set_lower_h, 255);
    add_trackbar("src", "upper_hue", config::set_upper_h, 255);
    add_trackbar("src", "kernelSize", config::set_rough_dilate_kernel_size, 100);
    // add_trackbar("src", "maxAreaRatio", config::set_max_convex_hull_thresh, 100);
    // add_trackbar("src", "minAreaRatio", config::set_min_convex_hull_thresh, 100);
    // add_trackbar("src", "minArea", config::set_min_contour_area);
    // add_trackbar("src", "hsv_upper", config::set_hsv_upper, 120);
    // add_trackbar("src", "hsv_lower", config::set_hsv_lower, 120);
    // add_trackbar("src", "roi_bin_thresh", config::set_roi_binary_thresh, 25);

    while(true) {

        cv::Mat src = read_frame();
        if(config::debug) {
            cv::imshow("src", src);
        }
        auto startTime = std::chrono::high_resolution_clock::now();

        bin_low = config::detect_color == Color::Blue ? binarizer.hsv_binarizer(src, config::upperBlue, config::lowerBlue) :
                                                        binarizer.hsv_binarizer(src, config::upperRed, config::lowerRed);
        bin_high = binarizer.binarize(src, config::bin_high_threshold);
        bin_logo = binarizer.binarize(src, config::bin_logo_threshold);
        // cv::Mat bin = binarizer.b_r_binarize(src, config::b_r_threshold);

        auto maybe_result = detect(src);
        auto endTime = std::chrono::high_resolution_clock::now();
        // std::cout << "Execution time: "
        //         << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count()
        //         << " milliseconds" << std::endl;

        if(maybe_result.has_value())
            result = maybe_result.value();
        if(cv::waitKey(1) == 27) {
            break;
        }
    }
    return 0;
}
