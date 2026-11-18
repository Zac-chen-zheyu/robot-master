#pragma once
// ---------------------------------------------------------------------------
//  synth_scene.h —— 合成场景（没有摄像头时用来跑通全链路）
//
//  用代码画一个和比赛标签一模一样的东西：黑底板 + 白三角 + 一个圆 + 一个方，
//  圆/方颜色会周期性在"同色 / 异色"之间切换，标签本身左右往复移动。
//  于是：识别、判据、侧别确认、闭环转向、结果存图，全都能在虚拟机里验证。
//
//  用法： ./mortat_aim --sim --source synth
// ---------------------------------------------------------------------------
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace mortat
{

    inline cv::Mat makeSynthFrame(double t_s)
    {
        const int W = 640;
        const int H = 480;

        cv::Mat img(H, W, CV_8UC3, cv::Scalar(232, 235, 238)); // 浅灰背景

        // 标签水平往复移动（周期 8s）
        const double period = 8.0;
        const double phase = std::fmod(t_s, period) / period;               // 0..1
        const double tri = phase < 0.5 ? phase * 2.0 : (1.0 - phase) * 2.0; // 0..1..0
        const double cx = 140.0 + (W - 280.0) * tri;

        // 每 4 秒换一次版面：同色 -> 异色 -> 同色 ...
        const bool same_color = (std::fmod(t_s, 8.0) < 4.0);
        // 每 8 秒换一次三角朝向
        const bool apex_right = (std::fmod(t_s, 16.0) >= 8.0);

        const int pw = 130; // 底板边长
        const int px = (int)(cx - pw / 2.0);
        const int py = H / 2 - pw / 2;
        const cv::Rect panel(px, py, pw, pw);

        cv::rectangle(img, panel, cv::Scalar(0, 0, 0), -1); // 黑底板

        // 白三角：固定在底板左半边（和真实标签一致），只翻转朝向
        //   apex_right = true  -> 竖边在左、尖端朝右
        //   apex_right = false -> 竖边在中、尖端朝左
        const int ty0 = py + 14;
        const int ty1 = py + pw - 14;
        const cv::Point apex = apex_right ? cv::Point(px + 70, py + pw / 2)
                                          : cv::Point(px + 14, py + pw / 2);
        const int base_x = apex_right ? (px + 14) : (px + 70);
        const cv::Point tri_pts[3] = {apex, cv::Point(base_x, ty0), cv::Point(base_x, ty1)};
        cv::fillConvexPoly(img, tri_pts, 3, cv::Scalar(255, 255, 255));

        // 彩色图形：圆在上、方在下，都在底板右半边
        const cv::Scalar kBlue(255, 180, 0); // BGR，对应 HSV 里 H≈97 的青蓝
        const cv::Scalar kRed(0, 0, 255);

        const cv::Scalar circle_color = kBlue;
        const cv::Scalar square_color = same_color ? kBlue : kRed;

        cv::circle(img, cv::Point(px + 96, py + 36), 16, circle_color, -1);
        cv::rectangle(img, cv::Rect(px + 77, py + 62, 36, 36), square_color, -1);

        return img;
    }

} // namespace mortat
