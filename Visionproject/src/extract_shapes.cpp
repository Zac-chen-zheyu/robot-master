// ---------------------------------------------------------------------------
//  extract_shapes.cpp —— 从掩膜里提取候选轮廓
//  与 视觉识别相关函数.txt 里的实现完全一致（原样搬过来，便于一起编译）
//  如果你工程里已经有同名函数，把本文件从 CMake 目标里去掉即可。
// ---------------------------------------------------------------------------
#include <opencv2/opencv.hpp>
#include <vector>

std::vector<std::vector<cv::Point>> extractAllShapes(const cv::Mat &mask, int shape_type)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> good_shapes;

    for (const auto &c : contours)
    {
        double area = cv::contourArea(c);
        if (area < 50 || area > 500000)
            continue;

        cv::Rect r = cv::boundingRect(c);
        double ratio = (double)r.width / r.height;
        if (ratio < 0.2 || ratio > 5.0)
            continue;

        // 把所有符合面积和长宽比的轮廓都返回，形状判定交给 main
        good_shapes.push_back(c);
    }
    return good_shapes;
}
