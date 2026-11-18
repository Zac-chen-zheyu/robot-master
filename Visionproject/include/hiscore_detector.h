#pragma once
// ---------------------------------------------------------------------------
//  hiscore_detector.h —— 高分区标签识别（OpenCV 4）
//
//  如果你已经有自己的识别代码，可以完全不使用本类：
//  只要把识别结果填进 aim_controller 的 Observation 即可，见 README "接入已有识别代码"。
//  本类是一个可直接运行的参考实现，判据见 tag_rule.h。
// ---------------------------------------------------------------------------
#include <opencv2/core.hpp>

#include <string>

#include "detect_params.h"
#include "tag_rule.h"

namespace mortat
{

    // 彩色图形颜色
    enum class BlobColor
    {
        Unknown = -1,
        Red = 0,
        Blue = 1
    };

    struct TagInfo
    {
        bool valid = false;
        Side apex = Side::Unknown; // 白三角尖端（钝角）指向
        Side high = Side::Unknown; // 高分区侧（判据结果）
        BlobColor circle = BlobColor::Unknown;
        BlobColor square = BlobColor::Unknown;
        bool same_color = false;
        int id = -1;                    // 0..7，见 README 的标签编号表
        cv::Rect box;                   // 原图坐标
        cv::Point2f center{-1.f, -1.f}; // 原图坐标，瞄准点
        float width_px = 0.f;           // 原图坐标下底板宽度，用于粗测距离
        float score = 0.f;              // 0..1
        bool shape_only = false;        // 只看到两个彩色图形（底板被切掉/太暗）
        std::string text;               // "high score: left"
    };

    struct DetectResult
    {
        bool found = false;
        TagInfo tag;
        cv::Mat debug;     // BGR 调试图（原图尺度），params.debug=false 时为空
        cv::Mat mask_view; // 颜色掩膜图：黑底板=灰白、白三角=绿、红=红、蓝=蓝
                           // 调这三类颜色阈值时看它，params.debug_masks=false 时为空
    };

    class HisCoreDetector
    {
    public:
        explicit HisCoreDetector(const DetectParams &p = DetectParams()) : p_(p) {}

        DetectResult detect(const cv::Mat &bgr);

        const DetectParams &params() const { return p_; }
        DetectParams &params() { return p_; }

    private:
        DetectParams p_;
    };

} // namespace mortat
