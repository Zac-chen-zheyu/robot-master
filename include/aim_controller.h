#pragma once
// ---------------------------------------------------------------------------
//  aim_controller.h —— 从"识别结果"到"炮塔转向指令"的决策器
//
//  输入：某一帧的识别结果（有没有标签、高分区在哪一侧、标签中心像素 / 帧宽）
//        + 下位机状态帧（当前角度、步长…）+ 时间戳
//  输出：这一拍该下发的动作（左/右转几步，或者不动）
//
//  它不碰串口、不碰 OpenCV，纯逻辑，方便在 PC 上跑单元测试。
// ---------------------------------------------------------------------------
#include <string>

#include "mortat_protocol.h"

namespace mortat
{

    // 角度归一化到 (-180, 180]，用于"走最近的路"
    inline double wrap180Deg(double a)
    {
        while (a > 180.0)
            a -= 360.0;
        while (a <= -180.0)
            a += 360.0;
        return a;
    }

    // 瞄准模式
    enum AimMode
    {
        AIM_TAG = 0,      // 直接瞄准标签（用像素偏差算方位角），连续跟踪
        AIM_SIDE_ABS = 1, // 高分区在左/右 -> 转到固定的绝对角度 yaw_left/yaw_right
        AIM_SIDE_REL = 2, // 高分区在左/右 -> 在当前角度基础上 ± side_offset
    };

    struct AimParams
    {
        int axis = 1; // 0 = CH0 180° 舵机，1 = CH1 360° 步进
        AimMode mode = AIM_SIDE_ABS;

        double yaw_left_deg = 55.0;    // AIM_SIDE_ABS：左高分区对应的炮塔角度
        double yaw_right_deg = 125.0;  // AIM_SIDE_ABS：右高分区对应的炮塔角度
        double side_offset_deg = 35.0; // AIM_SIDE_REL：偏离当前角度的量
        double aim_offset_deg = 0.0;   // AIM_TAG：瞄准标签后的微调

        double hfov_deg = 60.0;         // 相机水平视场角（决定 像素->角度）
        double cam_mount_yaw_deg = 0.0; // 相机光轴在炮塔坐标系里的绝对角；
                                        // 相机与炮管反向(差180°)时填 180
        bool mirror_x = false;          // 图像被左右镜像时置 true
        bool invert_dir = false;        // 左/右转指令与实际方向相反时置 true

        double tolerance_deg = 2.0; // 大于它才继续转
        double deadband_deg = 1.0;  // 小于它认为已对准
        int confirm_frames = 3;     // 连续 N 帧同侧才确认（防抖）
        bool latch_side = true;     // 确认后锁存侧别
        double unlatch_s = 2.0;     // 连续异侧超过这么久才改判

        int min_steps = 1;
        int max_steps_per_cmd = 6; // 一条指令最多几步，配合状态回读做闭环
        double step_deg = 2.0;     // 与固件 step 一致，用来换算指令步数

        bool seek_enabled = true; // 视野里没有标签时左右扫视找
        double seek_range_deg = 120.0;
        int seek_step = 2;
        double seek_interval_s = 0.9;

        double servo_min_deg = 5.0; // 仅 axis=0 时生效（180° 舵机机械范围）
        double servo_max_deg = 175.0;
    };

    // 一帧识别结果（你已有的识别代码只要填这个结构）
    struct Observation
    {
        bool found = false;
        int side = -1;       // 0 = 左高分区, 1 = 右高分区, -1 = 不知道
        float tag_cx = -1.f; // 标签中心像素 x
        int frame_w = 0;     // 图像宽度
    };

    enum class AimState
    {
        Search,
        Confirm,
        Align,
        Settle,
        Done,
        Lost
    };
    const char *aimStateName(AimState s);

    struct AimDecision
    {
        enum Action
        {
            None = 0,
            Turn = 1,
            Center = 2
        };
        Action action = None;
        int steps = 0;        // Turn：这一步走几步
        int dir = 0;          // +1 = 右转(CMD_TURN_RIGHT)，-1 = 左转
        double err_deg = 0.0; // 目标角度 - 当前角度（已归一化到 ±180）
        double target_yaw_deg = 0.0;
        double cur_yaw_deg = 0.0;
        int side = -1; // 当前锁存的高分区侧
        bool side_confirmed = false;
        float tag_azimuth_deg = 0.f;
        std::string note;
    };

    class AimController
    {
    public:
        explicit AimController(const AimParams &p);

        AimDecision update(const Observation &obs, const Status &st, double now_s);

        AimState state() const { return state_; }
        int confirmedSide() const { return latched_side_; }
        void reset();

        // 外部已经有了权威结论（比如你的 20 帧投票结果）时，直接指定侧别，
        // 跳过"连续 N 帧确认"，之后照常闭环对准。
        void forceSide(int side);

    private:
        AimParams p_;

        AimState state_ = AimState::Search;

        int cand_side_ = -1;
        int cand_count_ = 0;
        int latched_side_ = -1;
        double latched_t_ = -1e9;
        double disagree_t_ = -1e9;

        double rel_base_deg_ = 0.0;
        bool rel_base_valid_ = false;

        int stable_ = 0;
        double last_seek_t_ = -1e9;
        double seek_accum_ = 0.0;
        int seek_dir_ = 1;
    };

} // namespace mortat
