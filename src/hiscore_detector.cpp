// ---------------------------------------------------------------------------
//  hiscore_detector.cpp —— 高分区标签识别实现
//
//  流程：黑底板 -> 白三角尖端朝向 -> 圆/方颜色 -> 判据 -> 高分区侧
//  坐标约定：对外一律返回原图坐标；内部先在 work_width 宽度的小图上算，又快又稳。
// ---------------------------------------------------------------------------
#include "hiscore_detector.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace mortat
{
    namespace
    {

        struct Blob
        {
            std::vector<cv::Point> pts;
            cv::Rect box;
            double area = 0.0;
            double fill = 0.0; // 面积 / 外接矩形面积：圆 ~0.78，方 ~1.0
            double circ = 0.0; // 圆度 4πA/P²：圆 ~1.0，方 ~0.79
            cv::Point2f c{0.f, 0.f};
        };

        std::vector<Blob> blobsOf(const cv::Mat &mask_in, int min_area)
        {
            std::vector<Blob> out;
            if (mask_in.empty())
                return out;

            cv::Mat mask = mask_in.clone(); // 兼容旧版 findContours(InputOutputArray)
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (size_t i = 0; i < contours.size(); ++i)
            {
                const std::vector<cv::Point> &c = contours[i];
                const double a = cv::contourArea(c);
                if (a < (double)min_area)
                    continue;

                Blob b;
                b.pts = c;
                b.area = a;
                b.box = cv::boundingRect(c);
                if (b.box.width < 4 || b.box.height < 4)
                    continue;

                const double box_area = (double)b.box.width * (double)b.box.height;
                b.fill = a / box_area;

                const double per = cv::arcLength(c, true);
                b.circ = (per > 1e-6) ? (4.0 * 3.14159265358979 * a / (per * per)) : 0.0;

                const cv::Moments m = cv::moments(c);
                if (m.m00 <= 1e-9)
                    continue;
                b.c = cv::Point2f((float)(m.m10 / m.m00), (float)(m.m01 / m.m00));

                out.push_back(b);
            }

            std::sort(out.begin(), out.end(),
                      [](const Blob &a, const Blob &b)
                      { return a.area > b.area; });
            return out;
        }

        inline double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }

        cv::Scalar colorBgr(BlobColor c)
        {
            return (c == BlobColor::Blue) ? cv::Scalar(255, 180, 0) : cv::Scalar(0, 0, 255);
        }

    } // namespace

    DetectResult HisCoreDetector::detect(const cv::Mat &bgr)
    {
        DetectResult res;
        if (bgr.empty() || bgr.channels() != 3)
            return res;

        // ---------------- 1. 缩图 ----------------
        cv::Mat img;
        double scale = 1.0;
        if (p_.work_width > 0 && bgr.cols > p_.work_width)
        {
            scale = (double)bgr.cols / (double)p_.work_width;
            cv::resize(bgr, img, cv::Size(), 1.0 / scale, 1.0 / scale, cv::INTER_AREA);
        }
        else
        {
            img = bgr;
        }

        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

        // ---------------- 2. 颜色分割 ----------------
        cv::Mat mBlack, mWhite, mRed, mBlue, r1, r2;
        cv::inRange(hsv, cv::Scalar(0, 0, 0),
                    cv::Scalar(179, p_.black_s_max, p_.black_v_max), mBlack);
        cv::inRange(hsv, cv::Scalar(0, 0, p_.white_v_min),
                    cv::Scalar(179, p_.white_s_max, 255), mWhite);

        cv::inRange(hsv, cv::Scalar(0, p_.color_s_min, p_.color_v_min),
                    cv::Scalar(p_.red_h_low, 255, 255), r1);
        cv::inRange(hsv, cv::Scalar(p_.red_h_high, p_.color_s_min, p_.color_v_min),
                    cv::Scalar(179, 255, 255), r2);
        cv::bitwise_or(r1, r2, mRed);

        cv::inRange(hsv, cv::Scalar(p_.blue_h_low, p_.color_s_min, p_.color_v_min),
                    cv::Scalar(p_.blue_h_high, 255, 255), mBlue);

        const cv::Mat k3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        const cv::Mat k5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mBlack, mBlack, cv::MORPH_CLOSE, k5);
        cv::morphologyEx(mBlack, mBlack, cv::MORPH_OPEN, k5);
        cv::morphologyEx(mWhite, mWhite, cv::MORPH_CLOSE, k3);
        cv::morphologyEx(mRed, mRed, cv::MORPH_OPEN, k3);
        cv::morphologyEx(mBlue, mBlue, cv::MORPH_OPEN, k3);

        // ---------------- 2.5 掩膜调试图（按 d 键切到这一档时才有） ----------------
        //   黑底板 -> 灰白    白三角 -> 绿    红图形 -> 红    蓝图形 -> 蓝
        //   调颜色阈值时：哪块该亮的没亮，就是那一类颜色的阈值没框住
        if (p_.debug_masks)
        {
            cv::Mat view;
            cv::cvtColor(mBlack, view, cv::COLOR_GRAY2BGR);
            view.setTo(cv::Scalar(0, 255, 0), mWhite);
            view.setTo(cv::Scalar(0, 0, 255), mRed);
            view.setTo(cv::Scalar(255, 180, 0), mBlue);
            res.mask_view = view;
        }

        // ---------------- 3. 找黑底板（最大且内部含三角+彩色图形） ----------------
        std::vector<Blob> panels = blobsOf(mBlack, p_.min_panel_area);
        int best = -1;
        double best_score = -1.0;

        for (size_t i = 0; i < panels.size(); ++i)
        {
            const Blob &b = panels[i];
            if (b.box.width < 30 || b.box.height < 30)
                continue;

            const double asp = (double)b.box.width / (double)std::max(1, b.box.height);
            if (asp < 0.55 || asp > 1.85)
                continue;
            if (b.fill < p_.min_panel_fill)
                continue;

            const double area = (double)b.box.width * (double)b.box.height;
            const double wr = cv::countNonZero(mWhite(b.box)) / area;
            const double cr = (cv::countNonZero(mRed(b.box)) +
                               cv::countNonZero(mBlue(b.box))) /
                              area;
            if (wr < 0.04 || wr > 0.80)
                continue; // 必须有白三角
            if (cr < 0.03 || cr > 0.70)
                continue; // 必须有彩色图形

            const double s = b.fill + 0.5 * std::min(0.5, cr) + 0.3 * std::min(0.4, wr);
            if (s > best_score)
            {
                best_score = s;
                best = (int)i;
            }
        }

        TagInfo tag;

        // ---------------- 4. 底板内部：三角朝向 + 圆/方 ----------------
        if (best >= 0)
        {
            const Blob &pb = panels[(size_t)best];
            tag.valid = true;

            // 4.1 白三角尖端：比较三角外接框最左/最右窄带的白像素厚度（尖端薄、底边厚）
            const cv::Mat w = mWhite(pb.box);
            Side apex = Side::Unknown;
            cv::Point2f tri_c(-1.f, -1.f);
            cv::Rect tri_box;

            std::vector<Blob> tris = blobsOf(w, 40);
            if (!tris.empty())
            {
                const Blob &t = tris.front();
                tri_c = t.c;
                tri_box = t.box;
                const int band = std::max(2, t.box.width / 6);
                if (t.box.width > 2 * band)
                {
                    const double lth = cv::countNonZero(
                                           w(cv::Rect(t.box.x, t.box.y, band, t.box.height))) /
                                       (double)band;
                    const double rth = cv::countNonZero(
                                           w(cv::Rect(t.box.x + t.box.width - band, t.box.y, band, t.box.height))) /
                                       (double)band;
                    // 左侧薄 -> 尖端朝左
                    apex = (lth < rth) ? Side::Left : Side::Right;
                }
            }
            tag.apex = apex;

            // 4.2 彩色图形：圆 = 填充率低且圆度高；方 = 填充率高
            struct ColoredBlob
            {
                Blob b;
                BlobColor color;
                bool circle;
            };
            std::vector<ColoredBlob> cbs;

            const cv::Mat red_roi = mRed(pb.box);
            const cv::Mat blue_roi = mBlue(pb.box);

            const cv::Mat *masks[2] = {&red_roi, &blue_roi};
            const BlobColor cols[2] = {BlobColor::Red, BlobColor::Blue};
            for (int m = 0; m < 2; ++m)
            {
                std::vector<Blob> bs = blobsOf(*masks[m], p_.min_color_area);
                for (size_t k = 0; k < bs.size() && k < 4; ++k)
                {
                    ColoredBlob cb;
                    cb.b = bs[k];
                    cb.color = cols[m];
                    cb.circle = (bs[k].fill < p_.circle_fill_max) &&
                                (bs[k].circ >= p_.circle_circ_min);
                    cbs.push_back(cb);
                }
            }

            const ColoredBlob *circ = nullptr;
            const ColoredBlob *sq = nullptr;
            for (size_t i = 0; i < cbs.size(); ++i)
            {
                const ColoredBlob &cb = cbs[i];
                if (cb.circle)
                {
                    if (!circ || cb.b.area > circ->b.area)
                        circ = &cb;
                }
                else
                {
                    if (!sq || cb.b.area > sq->b.area)
                        sq = &cb;
                }
            }

            if (circ && sq)
            {
                tag.circle = circ->color;
                tag.square = sq->color;
                tag.same_color = (circ->color == sq->color);
                tag.high = decideHighSide(apex, tag.same_color);
            }

            // 4.3 编号 id = 尖端(左0/右1)*4 + 圆蓝*2 + 方蓝
            if (apex != Side::Unknown && circ && sq)
            {
                const int a = (apex == Side::Left) ? 0 : 1;
                const int cb = (circ->color == BlobColor::Blue) ? 1 : 0;
                const int sb = (sq->color == BlobColor::Blue) ? 1 : 0;
                tag.id = a * 4 + cb * 2 + sb;
            }

            // 4.4 置信度：底板填充 + 三要素齐全 + 圆在上/方在下
            double sc = 0.35 * clamp01(pb.fill);
            if (apex != Side::Unknown)
                sc += 0.30;
            if (circ)
                sc += 0.15;
            if (sq)
                sc += 0.15;
            if (circ && sq && sq->b.c.y > circ->b.c.y)
                sc += 0.05;
            tag.score = clamp01(sc);

            // 4.5 换算回原图坐标
            tag.center = cv::Point2f((float)((pb.box.x + pb.box.width * 0.5) * scale),
                                     (float)((pb.box.y + pb.box.height * 0.5) * scale));
            tag.width_px = (float)(pb.box.width * scale);
            tag.box = cv::Rect(cvRound(pb.box.x * scale), cvRound(pb.box.y * scale),
                               cvRound(pb.box.width * scale), cvRound(pb.box.height * scale));
            tag.text = std::string("high score: ") + sideName(tag.high);

            res.found = true;
            res.tag = tag;

            if (p_.debug)
            {
                cv::Mat dbg = bgr.clone();
                const int thick = std::max(2, dbg.cols / 320);
                cv::rectangle(dbg, tag.box, cv::Scalar(0, 220, 0), thick);

                // 三角中心 + 朝向箭头
                if (tri_c.x >= 0.f)
                {
                    cv::Point tc((int)(tri_c.x * scale), (int)(tri_c.y * scale));
                    cv::circle(dbg, tc, thick * 3, cv::Scalar(0, 255, 255), thick);
                    if (apex != Side::Unknown)
                    {
                        const int dirx = (apex == Side::Left) ? -1 : 1;
                        cv::arrowedLine(dbg, tc, cv::Point(tc.x + dirx * 60, tc.y),
                                        cv::Scalar(0, 255, 255), thick * 2);
                    }
                }
                // 圆/方颜色点
                if (circ)
                {
                    cv::circle(dbg,
                               cv::Point((int)((pb.box.x + circ->b.c.x) * scale),
                                         (int)((pb.box.y + circ->b.c.y) * scale)),
                               thick * 3, colorBgr(circ->color), -1);
                }
                if (sq)
                {
                    cv::rectangle(dbg,
                                  cv::Rect((int)((pb.box.x + sq->b.box.x) * scale),
                                           (int)((pb.box.y + sq->b.box.y) * scale),
                                           (int)(sq->b.box.width * scale),
                                           (int)(sq->b.box.height * scale)),
                                  colorBgr(sq->color), thick);
                }
                // 结论文字（比赛要求：屏幕上显示 high score: left/right）
                cv::putText(dbg, tag.text, cv::Point(12, 44),
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), thick, cv::LINE_AA);
                char buf[128];
                std::snprintf(buf, sizeof(buf), "apex=%s same=%d circ=%s sq=%s id=%d",
                              sideName(apex), tag.same_color ? 1 : 0,
                              tag.circle == BlobColor::Blue ? "blue" : "red",
                              tag.square == BlobColor::Blue ? "blue" : "red", tag.id);
                cv::putText(dbg, buf, cv::Point(12, 80),
                            cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(200, 255, 200), thick, cv::LINE_AA);
                res.debug = dbg;
            }
            return res;
        }

        // ---------------- 5. 退化模式：底板被切掉/太暗，只认两个彩色图形 ----------------
        if (p_.allow_shape_only)
        {
            struct CB
            {
                Blob b;
                BlobColor color;
                bool circle;
            };
            std::vector<CB> all;
            const cv::Mat *masks[2] = {&mRed, &mBlue};
            const BlobColor cols[2] = {BlobColor::Red, BlobColor::Blue};
            for (int m = 0; m < 2; ++m)
            {
                std::vector<Blob> bs = blobsOf(*masks[m], p_.min_color_area);
                for (size_t k = 0; k < bs.size() && k < 6; ++k)
                {
                    CB cb;
                    cb.b = bs[k];
                    cb.color = cols[m];
                    cb.circle = (bs[k].fill < p_.circle_fill_max) &&
                                (bs[k].circ >= p_.circle_circ_min);
                    all.push_back(cb);
                }
            }
            const CB *circ = nullptr;
            const CB *sq = nullptr;
            for (size_t i = 0; i < all.size(); ++i)
            {
                if (all[i].circle)
                {
                    if (!circ || all[i].b.area > circ->b.area)
                        circ = &all[i];
                }
                else
                {
                    if (!sq || all[i].b.area > sq->b.area)
                        sq = &all[i];
                }
            }
            if (circ && sq)
            {
                TagInfo t;
                t.valid = true;
                t.shape_only = true;
                t.circle = circ->color;
                t.square = sq->color;
                t.same_color = (circ->color == sq->color);
                t.center = cv::Point2f((float)((circ->b.c.x + sq->b.c.x) * 0.5 * scale),
                                       (float)((circ->b.c.y + sq->b.c.y) * 0.5 * scale));
                t.width_px = (float)(std::fabs(circ->b.c.y - sq->b.c.y) * 2.0 * scale);
                t.score = 0.25;
                t.text = "high score: unknown (no triangle)";
                t.box = cv::Rect(cvRound(std::min(circ->b.c.x, sq->b.c.x) * scale),
                                 cvRound(std::min(circ->b.c.y, sq->b.c.y) * scale),
                                 cvRound(std::fabs(circ->b.c.x - sq->b.c.x) * scale),
                                 cvRound(std::fabs(circ->b.c.y - sq->b.c.y) * scale));
                res.found = true;
                res.tag = t;
            }
        }
        return res;
    }

} // namespace mortat
