// ============================================================================
//  vision_aim.cpp —— 你的识别程序 + 炮塔自主转向
//
//  这就是 桌面 main.txt 的原文，只加了 8 处（都标了 "// >>> AIM"），
//  你的识别流程、调参按键、20 帧投票、30 秒超时，一个都没动。
//
//  编译（在工程根目录）：
//    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
//    cmake --build build -j$(nproc)          # 目标名 vision_aim
//  运行：
//    ./build/vision_aim                      接真板子
//    MORTAT_SIM=1 ./build/vision_aim         仿真，不碰串口
// ============================================================================
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <map>
#include <chrono>
#include <string>
#include <cmath>
#include <cstdlib>
#include <ctime>

extern std::vector<std::vector<cv::Point>> extractAllShapes(const cv::Mat &mask, int shape_type);

// >>> AIM ①：引入转向模块，全局一个实例
#include "turret_aim.h"
static mortat::TurretAim g_aim;

int main()
{
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened())
        return -1;

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    cv::namedWindow("Original Frame", cv::WINDOW_NORMAL);
    cv::namedWindow("Corrected Board", cv::WINDOW_NORMAL);
    cv::namedWindow("Red Mask", cv::WINDOW_NORMAL);
    cv::namedWindow("Blue Mask", cv::WINDOW_NORMAL);
    cv::namedWindow("White Mask", cv::WINDOW_NORMAL);

    // >>> AIM ②：初始化串口（MORTAT_SIM=1 时走仿真，不碰真板子）
    {
        const char *simEnv = std::getenv("MORTAT_SIM");
        const bool sim = (simEnv != nullptr && simEnv[0] == '1');
        g_aim.init("config/aim.ini", sim);
        std::cout << g_aim.info() << std::endl;
        std::cout << "炮塔按键: F=发射  C=回中  X=急停  R=复位（不影响你的调参键）" << std::endl;
    }

    int v_max_black = 100;
    int h_min_b = 85, s_min_b = 100, v_min_b = 80;
    int h_max_b = 115, s_max_b = 255, v_max_b = 255;
    int h_min_r1 = 0, s_min_r1 = 80, v_min_r1 = 70;
    int h_max_r1 = 15, s_max_r1 = 255, v_max_r1 = 255;
    int h_min_r2 = 150, s_min_r2 = 80, v_min_r2 = 70;
    int h_max_r2 = 180, s_max_r2 = 255, v_max_r2 = 255;
    int s_max_w = 50, v_min_w = 180;

    cv::Mat frame, hsv;

    std::map<std::string, int> vote_counts;
    int valid_frame_count = 0;
    bool is_finished = false;
    std::string final_result = "unknown";

    bool is_started = false;
    bool do_preview = false;
    std::chrono::steady_clock::time_point start_time;
    bool is_timeout = false;
    int timeout_seconds = 30;

    // >>> AIM ③：结算只通知一次；每帧的识别结论先存这里
    bool aim_finished_notified = false;
    bool frame_ok = false;
    bool frame_left = false;
    float board_cx = -1.0f;

    std::cout << "================ 键盘控制说明 ================" << std::endl;
    std::cout << "P 键 : 预识别 | Enter 键 : 开始正式识别 | ESC : 退出" << std::endl;
    std::cout << "==============================================" << std::endl;

    while (true)
    {
        cap >> frame;
        if (frame.empty())
            break;

        // >>> AIM ④：每帧重置识别结论
        frame_ok = false;
        frame_left = false;
        board_cx = -1.0f;

        if (is_started && !is_finished)
        {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

            if (elapsed_seconds >= timeout_seconds)
            {
                is_timeout = true;
                is_finished = true;
                std::cout << "\n\n[!] 检测超时 (" << timeout_seconds << "秒)！强制结算..." << std::endl;
                if (vote_counts["left"] > vote_counts["right"])
                    final_result = "left";
                else if (vote_counts["right"] > vote_counts["left"])
                    final_result = "right";
                else
                    final_result = "right";
                std::cout << "超时结算结果 => 最终判定: high score: " << final_result << std::endl;
            }
        }

        cv::Mat filtered_frame;
        cv::GaussianBlur(frame, filtered_frame, cv::Size(3, 3), 0);

        cv::Mat lab_img;
        cv::cvtColor(filtered_frame, lab_img, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> lab_planes(3);
        cv::split(lab_img, lab_planes);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(lab_planes[0], lab_planes[0]);

        cv::Mat enhanced_frame;
        cv::merge(lab_planes, lab_img);
        cv::cvtColor(lab_img, enhanced_frame, cv::COLOR_Lab2BGR);
        cv::cvtColor(enhanced_frame, hsv, cv::COLOR_BGR2HSV);

        cv::Mat black_mask;
        cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, v_max_black), black_mask);
        cv::Mat kernel_black = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
        cv::morphologyEx(black_mask, black_mask, cv::MORPH_CLOSE, kernel_black);

        std::vector<std::vector<cv::Point>> black_contours;
        cv::findContours(black_mask, black_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double max_area = 0;
        std::vector<cv::Point> largest_board_contour;
        for (const auto &c : black_contours)
        {
            double area = cv::contourArea(c);
            if (area > 3000 && area < 2000000)
            {
                cv::Rect rect = cv::boundingRect(c);
                double aspect_ratio = (double)rect.width / rect.height;
                if (aspect_ratio > 0.6 && aspect_ratio < 1.5)
                {
                    if (area > max_area)
                    {
                        max_area = area;
                        largest_board_contour = c;
                    }
                }
            }
        }

        if (max_area > 0)
        {
            std::vector<cv::Point2f> src_pts(4);
            std::vector<cv::Point> approx_board;

            double peri = cv::arcLength(largest_board_contour, true);
            cv::approxPolyDP(largest_board_contour, approx_board, 0.02 * peri, true);

            // 防交叉排序
            if (approx_board.size() == 4)
            {
                std::vector<cv::Point2f> pts(4);
                for (int i = 0; i < 4; i++)
                    pts[i] = approx_board[i];

                int tl_idx = 0, br_idx = 0;
                float min_sum = 999999, max_sum = -999999;
                for (int i = 0; i < 4; i++)
                {
                    float sum = pts[i].x + pts[i].y;
                    if (sum < min_sum)
                    {
                        min_sum = sum;
                        tl_idx = i;
                    }
                    if (sum > max_sum)
                    {
                        max_sum = sum;
                        br_idx = i;
                    }
                }
                src_pts[0] = pts[tl_idx];
                src_pts[2] = pts[br_idx];

                int tr_idx = 0, bl_idx = 0;
                float min_diff = 999999, max_diff = -999999;
                for (int i = 0; i < 4; i++)
                {
                    if (i == tl_idx || i == br_idx)
                        continue;
                    float diff = pts[i].y - pts[i].x;
                    if (diff < min_diff)
                    {
                        min_diff = diff;
                        tr_idx = i;
                    }
                    if (diff > max_diff)
                    {
                        max_diff = diff;
                        bl_idx = i;
                    }
                }
                src_pts[1] = pts[tr_idx];
                src_pts[3] = pts[bl_idx];
            }
            else
            {
                cv::Rect br = cv::boundingRect(largest_board_contour);
                src_pts[0] = cv::Point2f(br.x, br.y);
                src_pts[1] = cv::Point2f(br.x + br.width, br.y);
                src_pts[2] = cv::Point2f(br.x + br.width, br.y + br.height);
                src_pts[3] = cv::Point2f(br.x, br.y + br.height);
            }

            // >>> AIM ⑤：标签中心像素 x（给"直接瞄准标签"模式用，side_abs 模式用不到）
            board_cx = (src_pts[0].x + src_pts[1].x + src_pts[2].x + src_pts[3].x) / 4.0f;

            int board_size = 600;
            std::vector<cv::Point2f> dst_pts = {
                cv::Point2f(0, 0), cv::Point2f(board_size, 0),
                cv::Point2f(board_size, board_size), cv::Point2f(0, board_size)};

            cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
            cv::Mat corrected_board;
            cv::warpPerspective(frame, corrected_board, M, cv::Size(board_size, board_size));

            for (int i = 0; i < 4; i++)
                cv::line(frame, src_pts[i], src_pts[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);

            cv::Mat zoomed_img;
            cv::resize(corrected_board, zoomed_img, cv::Size(), 2.0, 2.0, cv::INTER_LINEAR);
            cv::Mat zoom_hsv;
            cv::cvtColor(zoomed_img, zoom_hsv, cv::COLOR_BGR2HSV);

            cv::Mat mask_b, mask_r1, mask_r2, mask_r, mask_w;
            cv::inRange(zoom_hsv, cv::Scalar(h_min_b, s_min_b, v_min_b), cv::Scalar(h_max_b, s_max_b, v_max_b), mask_b);
            cv::inRange(zoom_hsv, cv::Scalar(h_min_r1, s_min_r1, v_min_r1), cv::Scalar(h_max_r1, s_max_r1, v_max_r1), mask_r1);
            cv::inRange(zoom_hsv, cv::Scalar(h_min_r2, s_min_r2, v_min_r2), cv::Scalar(h_max_r2, s_max_r2, v_max_r2), mask_r2);
            cv::bitwise_or(mask_r1, mask_r2, mask_r);
            cv::inRange(zoom_hsv, cv::Scalar(0, 0, v_min_w), cv::Scalar(180, s_max_w, 255), mask_w);

            cv::Mat k_small = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::Mat k_med = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
            cv::morphologyEx(mask_b, mask_b, cv::MORPH_OPEN, k_small);
            cv::morphologyEx(mask_b, mask_b, cv::MORPH_CLOSE, k_med);
            cv::morphologyEx(mask_r, mask_r, cv::MORPH_OPEN, k_small);
            cv::morphologyEx(mask_r, mask_r, cv::MORPH_CLOSE, k_med);
            cv::morphologyEx(mask_w, mask_w, cv::MORPH_OPEN, k_small);
            cv::morphologyEx(mask_w, mask_w, cv::MORPH_CLOSE, k_med);

            int flat_side = 0; // 1=平边在左(钝角指右), 2=平边在右(钝角指左)
            std::vector<int> detected_colors;
            std::string found_shapes_str = "";

            // ====== 提取红色 ======
            auto red_contours = extractAllShapes(mask_r, 0);
            for (const auto &c : red_contours)
            {
                double area = cv::contourArea(c);
                cv::RotatedRect rect = cv::minAreaRect(c);
                double fill_ratio = area / (rect.size.width * rect.size.height);
                if (fill_ratio > 0.85)
                {
                    detected_colors.push_back(1);
                    found_shapes_str += "红方 ";
                    cv::rectangle(zoomed_img, cv::boundingRect(c), cv::Scalar(0, 0, 255), 2);
                }
                else
                {
                    detected_colors.push_back(1);
                    found_shapes_str += "红圆 ";
                    cv::rectangle(zoomed_img, cv::boundingRect(c), cv::Scalar(0, 0, 255), 2);
                }
            }

            // ====== 提取蓝色 ======
            auto blue_contours = extractAllShapes(mask_b, 0);
            for (const auto &c : blue_contours)
            {
                double area = cv::contourArea(c);
                cv::RotatedRect rect = cv::minAreaRect(c);
                double fill_ratio = area / (rect.size.width * rect.size.height);
                if (fill_ratio > 0.85)
                {
                    detected_colors.push_back(2);
                    found_shapes_str += "蓝方 ";
                    cv::rectangle(zoomed_img, cv::boundingRect(c), cv::Scalar(255, 0, 0), 2);
                }
                else
                {
                    detected_colors.push_back(2);
                    found_shapes_str += "蓝圆 ";
                    cv::rectangle(zoomed_img, cv::boundingRect(c), cv::Scalar(255, 0, 0), 2);
                }
            }

            // ====== 提取白色三角形（找平边） ======
            auto white_triangles = extractAllShapes(mask_w, 3);
            if (!white_triangles.empty())
            {
                std::vector<cv::Point> largest_tri;
                double max_tri_area = 0;
                for (const auto &shape : white_triangles)
                {
                    double area = cv::contourArea(shape);
                    if (area > max_tri_area)
                    {
                        max_tri_area = area;
                        largest_tri = shape;
                    }
                }

                cv::Rect r = cv::boundingRect(largest_tri);
                cv::rectangle(zoomed_img, r, cv::Scalar(0, 255, 255), 2);
                found_shapes_str += "白三角 ";

                std::vector<cv::Point> approx;
                cv::approxPolyDP(largest_tri, approx, 0.04 * cv::arcLength(largest_tri, true), true);

                if (approx.size() >= 3)
                {
                    double d01 = cv::norm(approx[0] - approx[1]);
                    double d12 = cv::norm(approx[1] - approx[2]);
                    double d20 = cv::norm(approx[2] - approx[0]);

                    double max_d = std::max(d01, std::max(d12, d20));
                    cv::Point2f p1, p2;

                    if (max_d == d01)
                    {
                        p1 = approx[0];
                        p2 = approx[1];
                    }
                    else if (max_d == d12)
                    {
                        p1 = approx[1];
                        p2 = approx[2];
                    }
                    else
                    {
                        p1 = approx[2];
                        p2 = approx[0];
                    }

                    double flat_x = (p1.x + p2.x) / 2.0;
                    double center_x = r.x + r.width / 2.0;

                    if (flat_x < center_x)
                    {
                        flat_side = 1; // 平边在左
                    }
                    else
                    {
                        flat_side = 2; // 平边在右
                    }

                    // 标出平边（粗红线）
                    cv::line(zoomed_img, p1, p2, cv::Scalar(0, 0, 255), 4);
                }
            }

            // ================= 最终绝杀判定 =================
            std::string current_result = "";
            bool logic_passed = false;

            if (flat_side != 0 && detected_colors.size() >= 2)
            {
                // 判断同色还是异色
                bool is_same_color = false;
                if (detected_colors.size() == 2)
                {
                    if (detected_colors[0] == detected_colors[1])
                        is_same_color = true;
                }
                else if (detected_colors.size() > 2)
                {
                    // 如果找出多个，只要里面有红有蓝就是异色，全红或全蓝就是同色
                    int reds = 0, blues = 0;
                    for (int c : detected_colors)
                    {
                        if (c == 1)
                            reds++;
                        else
                            blues++;
                    }
                    if (reds > 0 && blues > 0)
                        is_same_color = false;
                    else
                        is_same_color = true;
                }

                if (is_same_color)
                {
                    // 同色局：直接看钝角指向
                    if (flat_side == 1)
                        current_result = "right"; // 平边在左 -> 钝角指右
                    else if (flat_side == 2)
                        current_result = "left"; // 平边在右 -> 钝角指左
                    logic_passed = true;
                }
                else
                {
                    // 异色局：看钝角相反侧（即平边所在侧）
                    if (flat_side == 1)
                        current_result = "left"; // 平边在左 -> 反向为左
                    else if (flat_side == 2)
                        current_result = "right"; // 平边在右 -> 反向为右
                    logic_passed = true;
                }
            }

            // >>> AIM ⑥：把这一帧的结论交给炮塔
            frame_ok = logic_passed;
            frame_left = (current_result == "left");

            if (do_preview)
            {
                if (logic_passed)
                {
                    std::cout << "\n--- [预识别成功] ---" << std::endl;
                    std::cout << "当前找到: " << found_shapes_str << std::endl;
                    std::cout << "-> 平边方向: " << (flat_side == 1 ? "左(钝角指右)" : "右(钝角指左)") << std::endl;
                    std::cout << "-> 判定结果: high score: " << current_result << std::endl;
                    std::cout << "--------------------" << std::endl;
                }
                else
                {
                    std::cout << "\n[预识别失败] 当前找到了: " << (found_shapes_str.empty() ? "无" : found_shapes_str) << std::endl;
                    std::cout << "  >> 失败原因: 缺少图形" << std::endl;
                }
                do_preview = false;
            }

            if (is_started && logic_passed && !is_finished)
            {
                vote_counts[current_result]++;
                valid_frame_count++;
                std::cout << "收集进度: " << valid_frame_count << "/20 | 当前 left:"
                          << vote_counts["left"] << " right:" << vote_counts["right"] << "\r" << std::flush;

                if (valid_frame_count >= 20)
                {
                    is_finished = true;
                    int max_votes = 0;
                    for (const auto &pair : vote_counts)
                    {
                        if (pair.second > max_votes)
                        {
                            max_votes = pair.second;
                            final_result = pair.first;
                        }
                    }
                    std::cout << "\n=====================================" << std::endl;
                    std::cout << "【20次投票结束】 left票数: " << vote_counts["left"] << " | right票数: " << vote_counts["right"] << std::endl;
                    std::cout << "=> 最终判定: high score: " << final_result << std::endl;
                    std::cout << "=====================================" << std::endl;
                }
            }

            cv::imshow("Corrected Board", zoomed_img);
            cv::imshow("Red Mask", mask_r);
            cv::imshow("Blue Mask", mask_b);
            cv::imshow("White Mask", mask_w);
        }

        // >>> AIM ⑦：每帧喂给炮塔（结算后用最终结果；否则用当帧结论）
        //   注意：未按 Enter 开始前不驱动炮塔，免得调参时炮塔乱扫
        if (is_started)
        {
            g_aim.observe(is_finished ? true : frame_ok,
                          is_finished ? (final_result == "left") : frame_left,
                          board_cx, frame.cols);
        }

        if (!is_started)
        {
            cv::putText(frame, "Mode: Adjusting", cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            cv::putText(frame, "Press [P] to Preview", cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, "Press [ENTER] to Start", cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }
        else if (is_finished)
        {
            cv::rectangle(frame, cv::Point(10, 10), cv::Point(450, 80), cv::Scalar(0, 0, 0), -1);
            cv::putText(frame, "high score: " + final_result, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 3);
            if (is_timeout)
                cv::putText(frame, "Timeout!", cv::Point(20, 110), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

            // >>> AIM ⑧：结算时通知炮塔转向最终侧，并存一张结果图留证
            if (!aim_finished_notified)
            {
                aim_finished_notified = true;
                g_aim.finish(final_result == "left");

                std::time_t tt = std::time(nullptr);
                std::tm tmv{};
#if defined(_WIN32)
                localtime_s(&tmv, &tt);
#else
                localtime_r(&tt, &tmv);
#endif
                char name[160];
                std::snprintf(name, sizeof(name), "results/result_%04d%02d%02d_%02d%02d%02d_%s.png",
                              tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                              tmv.tm_hour, tmv.tm_min, tmv.tm_sec, final_result.c_str());
                std::system("mkdir -p results");
                cv::imwrite(name, frame);
                std::cout << "== 已保存结果图: " << name << std::endl;
            }
        }
        else
        {
            cv::putText(frame, "Collecting: " + std::to_string(valid_frame_count) + "/20", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        }

        // >>> AIM ⑨：把炮塔状态画在画面左下角，现场一眼能看到它在不在转
        cv::putText(frame, g_aim.statusText(), cv::Point(20, frame.rows - 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

        cv::imshow("Original Frame", frame);

        char key = static_cast<char>(cv::waitKey(30));

        // >>> AIM ⑩：F=发射 C=回中 X=急停 R=复位，其它键原样留给你的调参逻辑
        g_aim.handleKey(key);

        if (key == 27)
            break;

        if (key == 13 && !is_started)
        {
            is_started = true;
            start_time = std::chrono::steady_clock::now();
            std::cout << "\n\n>>> 开始正式识别！30秒倒计时启动！ <<<\n"
                      << std::endl;
        }
        if (key == 'p' || key == 'P')
        {
            if (!is_started)
                do_preview = true;
        }

        if (key == 'w' || key == 'W')
        {
            v_min_w += 5;
            std::cout << ">> 白(V_Min) = " << v_min_w << std::endl;
        }
        if (key == 's' || key == 'S')
        {
            v_min_w -= 5;
            std::cout << ">> 白(V_Min) = " << v_min_w << std::endl;
        }
        if (key == 'a' || key == 'A')
        {
            s_max_w -= 5;
            std::cout << ">> 白(S_Max)收紧 = " << s_max_w << std::endl;
        }
        if (key == 'd' || key == 'D')
        {
            s_max_w += 5;
            std::cout << ">> 白(S_Max)放宽 = " << s_max_w << std::endl;
        }
        if (key == 'b' || key == 'B')
        {
            s_min_b += 5;
            std::cout << ">> 蓝(S_Min)收紧 = " << s_min_b << std::endl;
        }
        if (key == 'n' || key == 'N')
        {
            s_min_b -= 5;
            std::cout << ">> 蓝(S_Min)放宽 = " << s_min_b << std::endl;
        }
        if (key == 't' || key == 'T')
        {
            s_min_r1 += 5;
            s_min_r2 += 5;
            std::cout << ">> 红(S_Min)收紧 = " << s_min_r1 << std::endl;
        }
        if (key == 'y' || key == 'Y')
        {
            s_min_r1 -= 5;
            s_min_r2 -= 5;
            std::cout << ">> 红(S_Min)放宽 = " << s_min_r1 << std::endl;
        }
        if (key == 'k' || key == 'K')
        {
            v_max_black -= 5;
            std::cout << ">> 黑(V_Max) = " << v_max_black << std::endl;
        }
        if (key == 'l' || key == 'L')
        {
            v_max_black += 5;
            std::cout << ">> 黑(V_Max) = " << v_max_black << std::endl;
        }
    }

    // >>> AIM ⑪：退出前发急停并关串口
    g_aim.close();

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
