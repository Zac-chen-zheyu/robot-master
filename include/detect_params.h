#pragma once
// ---------------------------------------------------------------------------
//  detect_params.h —— 识别参数（纯数值，不依赖 OpenCV，便于 config 复用）
// ---------------------------------------------------------------------------
namespace mortat
{

    struct DetectParams
    {
        int work_width = 640;         // 先缩到这个宽度再算，稳又快
        int min_panel_area = 2500;    // 底板最小面积（work_width 尺度下）
        double min_panel_fill = 0.45; // 底板填充率（底板被三角/图形挖空，不会到 1）
        int black_s_max = 90;         // 底板：S 上限
        int black_v_max = 75;         // 底板：V 上限
        int white_s_max = 45;         // 白三角：S 上限
        int white_v_min = 195;        // 白三角：V 下限
        int color_s_min = 110;        // 彩色图形：S 下限
        int color_v_min = 90;         // 彩色图形：V 下限
        int min_color_area = 120;     // 单个彩色图形最小面积
        int red_h_low = 12;           // 红色 H 区间 [0, red_h_low] ∪ [red_h_high, 179]
        int red_h_high = 170;
        int blue_h_low = 82; // 蓝色/青色 H 区间
        int blue_h_high = 115;
        double circle_fill_max = 0.90; // 填充率 < 它 -> 圆形
        double circle_circ_min = 0.70; // 圆度 > 它 -> 圆形
        bool allow_shape_only = true;  // 底板缺失时，退化到"只看两个彩色图形"
        bool debug = true;             // 是否生成调试图 res.debug
        bool debug_masks = false;      // 是否生成掩膜图 res.mask_view（调颜色阈值时看）
    };

} // namespace mortat
