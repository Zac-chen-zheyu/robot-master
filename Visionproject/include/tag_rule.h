#pragma once
// ---------------------------------------------------------------------------
//  tag_rule.h —— 高分区判据（纯逻辑，不依赖 OpenCV）
//
//  规则手册 4.1：
//    标签 = 黑色正方形底板 + 一个白色三角形 + 一个红色或蓝色圆形
//           + 一个红色或蓝色正方形
//    · 圆形与正方形【颜色相同】 -> 三角形"钝角指向"的那一侧 是 高分区
//    · 圆形与正方形【颜色不同】 -> 三角形指向的【相反】一侧 是 高分区
//
//  也就是说：高分区可能在三角形所指的反面，不能只看三角形。
// ---------------------------------------------------------------------------
namespace mortat
{

    enum class Side
    {
        Unknown = -1,
        Left = 0,
        Right = 1
    };

    inline Side opposite(Side s)
    {
        if (s == Side::Left)
            return Side::Right;
        if (s == Side::Right)
            return Side::Left;
        return Side::Unknown;
    }

    inline const char *sideName(Side s)
    {
        switch (s)
        {
        case Side::Left:
            return "left";
        case Side::Right:
            return "right";
        default:
            return "unknown";
        }
    }

    // 核心判据
    inline Side decideHighSide(Side apex_side, bool same_color)
    {
        if (apex_side == Side::Unknown)
            return Side::Unknown;
        return same_color ? apex_side : opposite(apex_side);
    }

} // namespace mortat
