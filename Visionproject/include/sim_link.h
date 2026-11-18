#pragma once
// ---------------------------------------------------------------------------
//  sim_link.h —— 无硬件仿真链路
//
//  作用：没有板子 / 没有舵机的时候，把"识别 -> 决策 -> 转向"整条链路跑通。
//  行为：收到转向指令就按 step 角"假装"转动（可设置到位率，模拟舵机走不准），
//        收到状态查询就回报当前角度；收到发射指令就模拟几拍发射过程。
// ---------------------------------------------------------------------------
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include "mortat_link.h"

namespace mortat
{

    class SimLink : public IMortatLink
    {
    public:
        // apply_ratio：一条转向指令实际到位多少（0.9 = 每次少走 10%，用来考验闭环）
        explicit SimLink(double step_deg = 2.0, double apply_ratio = 0.9)
            : step_deg_(step_deg), apply_ratio_(apply_ratio)
        {
            st_.step_tenths = (int)std::lround(step_deg * 10.0);
            st_.pos_tenths = kPosCenterTenths;
            st_.valid = true;
        }

        bool connected() const override { return true; }

        bool send(const std::vector<uint8_t> &f) override
        {
            if (f.size() < 5 || f[0] != kSync0 || f[1] != kSync1)
                return false;
            ++frames_;
            const uint8_t cmd = f[3];
            const std::vector<uint8_t> p(f.begin() + 4, f.end() - 1);

            switch (cmd)
            {
            case CMD_TURN_LEFT:
                if (!p.empty())
                    turn(-(int)p[0]);
                break;
            case CMD_TURN_RIGHT:
                if (!p.empty())
                    turn(+(int)p[0]);
                break;
            case CMD_CENTER:
                st_.pos_tenths = kPosCenterTenths;
                break;
            case CMD_SET_POS:
                if (p.size() >= 2)
                    st_.pos_tenths = p[0] | (p[1] << 8);
                break;
            case CMD_SET_STEP:
                if (p.size() >= 2)
                {
                    st_.step_tenths = p[0] | (p[1] << 8);
                    step_deg_ = st_.step_tenths / 10.0;
                }
                break;
            case CMD_FIRE:
                ++fires_;
                st_.state = 1;
                st_.progress = 0;
                fire_left_ = 3;
                break;
            case CMD_STOP:
                st_.state = 0;
                fire_left_ = 0;
                break;
            case CMD_SELECT_TARGET:
            case CMD_SET_FIRE_MS:
            case CMD_SET_STEP_MS:
            case CMD_SET_BALANCE:
            case CMD_SET_SPIN_TRIM:
            case CMD_SET_STEP_TRIM:
            case CMD_GET_STATUS:
            case CMD_GET_VERSION:
            default:
                break;
            }
            return true;
        }

        void pump() override
        {
            if (fire_left_ > 0)
            {
                --fire_left_;
                st_.progress = (uint8_t)((3 - fire_left_) * 33);
                if (fire_left_ == 0)
                {
                    st_.state = 0;
                    st_.progress = 100;
                }
            }
            if (!texts_.empty())
                last_text_ = texts_.front();
        }

        const Status &status() const override { return st_; }

        std::string takeText() override
        {
            if (texts_.empty())
                return std::string();
            std::string s = texts_.front();
            texts_.pop_front();
            return s;
        }

        // 测试用
        int framesSent() const { return frames_; }
        int fireCount() const { return fires_; }
        void setPos(double deg) { st_.pos_tenths = (int)std::lround(deg * 10.0); }
        void pushText(const std::string &s) { texts_.push_back(s); }

    private:
        void turn(int steps)
        {
            if (steps == 0)
                return;
            // 仿真的舵机按时间估算角度，注定走不准：乘一个到位率
            const double delta = steps * step_deg_ * apply_ratio_;
            st_.pos_tenths += (int)std::lround(delta * 10.0);
            st_.pos_tenths = std::max(0, std::min(1800, st_.pos_tenths));
        }

        double step_deg_ = 2.0;
        double apply_ratio_ = 0.9;
        Status st_;
        int frames_ = 0;
        int fires_ = 0;
        int fire_left_ = 0;
        std::deque<std::string> texts_;
        std::string last_text_;
    };

} // namespace mortat
