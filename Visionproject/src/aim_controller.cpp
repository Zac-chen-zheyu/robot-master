// ---------------------------------------------------------------------------
//  aim_controller.cpp —— 瞄准决策实现
//
//  角度约定：
//    · 炮塔角度 pos 来自状态帧，单位 0.1°，900 = 90.0°
//    · 误差 err = 目标角 - 当前角，归一化到 ±180°（所以"在另一侧"自然走最近的路）
//    · err > 0 -> 右转（CMD_TURN_RIGHT），err < 0 -> 左转（CMD_TURN_LEFT）
//      若你的机械装反了，把 AimParams::invert_dir 置 true
// ---------------------------------------------------------------------------
#include "aim_controller.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mortat
{
    namespace
    {

        constexpr double kPi = 3.14159265358979323846;
        constexpr double kDeg2Rad = kPi / 180.0;

    } // namespace

    const char *aimStateName(AimState s)
    {
        switch (s)
        {
        case AimState::Search:
            return "SEARCH";
        case AimState::Confirm:
            return "CONFIRM";
        case AimState::Align:
            return "ALIGN";
        case AimState::Settle:
            return "SETTLE";
        case AimState::Done:
            return "DONE";
        case AimState::Lost:
            return "LOST";
        }
        return "?";
    }

    AimController::AimController(const AimParams &p) : p_(p) {}

    void AimController::reset()
    {
        state_ = AimState::Search;
        cand_side_ = -1;
        cand_count_ = 0;
        latched_side_ = -1;
        latched_t_ = -1e9;
        disagree_t_ = -1e9;
        rel_base_deg_ = 0.0;
        rel_base_valid_ = false;
        stable_ = 0;
        last_seek_t_ = -1e9;
        seek_accum_ = 0.0;
        seek_dir_ = 1;
    }

    void AimController::forceSide(int side)
    {
        if (side != 0 && side != 1)
            return;
        latched_side_ = side;
        latched_t_ = -1e9; // 之后仍允许被新的确认结果改判
        cand_side_ = side;
        cand_count_ = std::max(1, p_.confirm_frames);
        rel_base_valid_ = false;
        stable_ = 0;
        disagree_t_ = -1e9;
    }

    AimDecision AimController::update(const Observation &obs, const Status &st, double now)
    {
        AimDecision d;
        d.cur_yaw_deg = st.posDeg();

        // ---------------------------------------------------------------- 没看到标签
        if (!obs.found)
        {
            stable_ = 0;
            state_ = AimState::Lost;
            d.side = latched_side_;
            d.side_confirmed = (latched_side_ >= 0);

            if (p_.seek_enabled)
            {
                if (now - last_seek_t_ >= p_.seek_interval_s)
                {
                    last_seek_t_ = now;
                    d.action = AimDecision::Turn;
                    d.dir = seek_dir_;
                    d.steps = std::max(1, p_.seek_step);
                    seek_accum_ += d.dir * d.steps * p_.step_deg;
                    if (seek_accum_ > p_.seek_range_deg)
                        seek_dir_ = -1;
                    if (seek_accum_ < -p_.seek_range_deg)
                        seek_dir_ = 1;

                    char buf[128];
                    std::snprintf(buf, sizeof(buf), "没看到标签，向%s扫视 %d 步",
                                  d.dir > 0 ? "右" : "左", d.steps);
                    d.note = buf;
                }
                else
                {
                    d.note = "没看到标签，等下一次扫视";
                }
            }
            else
            {
                d.note = "没看到标签";
            }
            return d;
        }

        // 一旦看到标签，扫视计数清零
        last_seek_t_ = now;
        seek_accum_ = 0.0;

        // ---------------------------------------------------------------- 侧别确认
        if (obs.side == 0 || obs.side == 1)
        {
            if (obs.side == cand_side_)
            {
                ++cand_count_;
            }
            else
            {
                cand_side_ = obs.side;
                cand_count_ = 1;
            }

            // 首次确认立刻生效；改判要等 unlatch_s，防止单帧误识别把炮塔甩出去
            const bool can_switch = (latched_side_ < 0) || (now - latched_t_ >= p_.unlatch_s);
            if (cand_count_ >= std::max(1, p_.confirm_frames) && can_switch)
            {
                if (latched_side_ != cand_side_)
                {
                    latched_side_ = cand_side_;
                    latched_t_ = now;
                    rel_base_deg_ = st.posDeg();
                    rel_base_valid_ = true;
                    disagree_t_ = -1e9;
                }
            }
        }

        d.side = latched_side_;
        d.side_confirmed = (latched_side_ >= 0);

        if (latched_side_ < 0 && p_.mode != AIM_TAG)
        {
            state_ = AimState::Confirm;
            char buf[128];
            std::snprintf(buf, sizeof(buf), "标签已检出，等侧别确认（%d/%d）",
                          cand_count_, std::max(1, p_.confirm_frames));
            d.note = buf;
            return d;
        }

        // ---------------------------------------------------------------- 目标角度
        double target = st.posDeg();

        if (p_.mode == AIM_TAG)
        {
            if (obs.frame_w > 0 && obs.tag_cx >= 0.f)
            {
                const double fx = 0.5 * (double)obs.frame_w /
                                  std::tan(0.5 * p_.hfov_deg * kDeg2Rad);
                const double dx = (double)obs.tag_cx - 0.5 * (double)obs.frame_w;
                double az = std::atan2(dx, std::max(1.0, fx)) / kDeg2Rad;
                if (p_.mirror_x)
                    az = -az;
                d.tag_azimuth_deg = (float)az;
                target = wrap180Deg(p_.cam_mount_yaw_deg + az + p_.aim_offset_deg);
            }
        }
        else if (p_.mode == AIM_SIDE_ABS)
        {
            target = (latched_side_ == 0) ? p_.yaw_left_deg : p_.yaw_right_deg;
        }
        else
        { // AIM_SIDE_REL
            const double off = (latched_side_ == 0) ? -p_.side_offset_deg : p_.side_offset_deg;
            const double base = rel_base_valid_ ? rel_base_deg_ : st.posDeg();
            target = wrap180Deg(base + off);
        }
        d.target_yaw_deg = target;

        // ---------------------------------------------------------------- 误差与限位
        double err = wrap180Deg(target - st.posDeg());
        if (p_.invert_dir)
            err = -err;

        const double err_before_limit = err;
        if (p_.axis == 0)
        { // 180° 舵机有硬限位，转过去也不能硬顶
            const double want = st.posDeg() + err;
            if (want > p_.servo_max_deg)
                err = p_.servo_max_deg - st.posDeg();
            if (want < p_.servo_min_deg)
                err = p_.servo_min_deg - st.posDeg();
        }
        const bool hit_limit = std::fabs(err - err_before_limit) > 0.5;
        d.err_deg = err;

        // ---------------------------------------------------------------- 输出决策
        if (std::fabs(err) <= p_.deadband_deg)
        {
            if (hit_limit)
            {
                state_ = AimState::Align;
                d.note = "已到 180° 行程限位：目标在另一侧，请改用 CH1(360°) 或放宽 servo 限位";
                return d;
            }
            ++stable_;
            state_ = (stable_ >= 2) ? AimState::Done : AimState::Settle;
            d.note = (state_ == AimState::Done) ? "已对准" : "微调到位";
            return d;
        }

        stable_ = 0;

        // 误差已经落在容差里：不再动，连续几帧都稳就直接判定"已对准"
        if (std::fabs(err) <= p_.tolerance_deg)
        {
            ++stable_;
            state_ = (stable_ >= 3) ? AimState::Done : AimState::Settle;
            d.note = (state_ == AimState::Done) ? "已对准（在容差内）"
                                                : "误差已进入容差，等待驱动停稳";
            return d;
        }

        const int steps = std::min(
            std::max((int)std::lround(std::fabs(err) / std::max(0.05, p_.step_deg)),
                     std::max(1, p_.min_steps)),
            std::max(1, p_.max_steps_per_cmd));

        state_ = AimState::Align;
        d.action = AimDecision::Turn;
        d.steps = steps;
        d.dir = (err > 0) ? +1 : -1;

        char buf[160];
        std::snprintf(buf, sizeof(buf), "对准中：%s %d 步（误差 %+.1f°，目标 %+.1f°）",
                      d.dir > 0 ? "右转" : "左转", d.steps, d.err_deg, d.target_yaw_deg);
        d.note = buf;
        return d;
    }

} // namespace mortat
