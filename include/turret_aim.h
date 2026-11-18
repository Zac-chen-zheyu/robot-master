#pragma once
// ---------------------------------------------------------------------------
//  turret_aim.h —— 把"视觉判出的左右"接到"炮塔自主转向"的最小模块
//
//  设计目标：不改你 main.cpp 的识别流程、不动你任何调参按键，
//            只加 8 行调用，炮塔就能跟着识别结果自己转。
//
//  用法（详见 README「把你的程序接上转向」）：
//      static mortat::TurretAim g_aim;                       // ① 顶部
//      g_aim.init("config/aim.ini", sim);                    // ② 初始化
//      g_aim.observe(found, sideLeft, board_cx, frame.cols); // ③ 每帧
//      g_aim.finish(final_result == "left");                 // ④ 投票结束
//      g_aim.handleKey(key);                                 // ⑤ 按键
//      g_aim.close();                                        // ⑥ 退出
//
//  它自己会：查状态帧 → 算目标角 → 每 turn_interval_s 发一条"左/右转 N 步" →
//            回读角度继续修正 → 到位后按配置决定是否发射。
// ---------------------------------------------------------------------------
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "aim_config.h"
#include "aim_controller.h"
#include "mortat_link.h"
#include "serial_link.h"
#include "sim_link.h"
#include "tag_rule.h"

namespace mortat {

class TurretAim {
public:
    using Clock = std::chrono::steady_clock;

    ~TurretAim() { close(); }

    // cfgPath 读不到就用内置默认值（会打印一行提示）；sim=true 时不碰真串口
    bool init(const std::string& cfgPath = "config/aim.ini", bool sim = false, bool verbose = true) {
        verbose_ = verbose;
        std::string err;
        if (!cfg_.load(cfgPath, &err)) {
            info_ = "!! " + err + "（用默认参数继续）";
        } else {
            info_ = "== 瞄准配置已加载：" + cfgPath;
        }

        if (sim) {
            simLink_.reset(new SimLink(cfg_.aim.step_deg, 0.9));
            link_ = simLink_.get();
            info_ += "\n== 仿真模式：转向不会发到真板子";
        } else {
            serLink_.reset(new SerialLink(cfg_.device, cfg_.baud));
            if (!serLink_->open(&err)) {
                info_ += "\n!! 串口打不开：" + err + "\n!! 可以用 MORTAT_SIM=1 先跑仿真";
                link_ = nullptr;
                return false;
            }
            link_ = serLink_.get();
            info_ += "\n== 串口已打开 " + cfg_.device;
        }

        ctrl_.reset(new AimController(cfg_.aim));
        t0_ = Clock::now();

        // 初始化下位机：控制轴、步长、每圈毫秒
        sendFrames(buildFrame1(CMD_SELECT_TARGET, (uint8_t)cfg_.aim.axis));
        sendFrames(buildU16(CMD_SET_STEP, (uint16_t)std::lround(cfg_.aim.step_deg * 10.0)));
        sendFrames(buildU16(CMD_SET_FIRE_MS, (uint16_t)cfg_.ms_per_turn));
        sendFrames(buildU16(CMD_SET_STEP_MS, (uint16_t)cfg_.ms_per_turn));
        sendFrames(buildFrame(CMD_GET_VERSION));
        return true;
    }

    bool ready() const { return link_ != nullptr && ctrl_ != nullptr; }
    const std::string& info() const { return info_; }

    // 每帧调用一次。
    //   found    这一帧是否给出了有效结论（logic_passed 或已结算）
    //   sideLeft 高分区是否在左边
    //   boardCx  标签中心像素 x（没有就传 -1）
    //   frameW   图像宽度
    void observe(bool found, bool sideLeft, float boardCx, int frameW) {
        if (!ready()) return;
        const double now = nowS();

        if (now - lastPoll_ >= cfg_.status_poll_s) {
            lastPoll_ = now;
            sendFrames(buildFrame(CMD_GET_STATUS));
        }
        link_->pump();

        Observation obs;
        obs.found   = found;
        obs.side    = found ? (sideLeft ? 0 : 1) : -1;
        obs.tag_cx  = boardCx;
        obs.frame_w = frameW;

        const AimDecision dec = ctrl_->update(obs, link_->status(), now);

        if (dec.action == AimDecision::Turn && now - lastTurn_ >= cfg_.turn_interval_s) {
            const std::vector<uint8_t> f =
                    (dec.dir > 0) ? buildFrame1(CMD_TURN_RIGHT, (uint8_t)std::min(255, dec.steps))
                                  : buildFrame1(CMD_TURN_LEFT,  (uint8_t)std::min(255, dec.steps));
            if (sendFrames(f)) {
                lastTurn_ = now;
                ++turns_;
                if (verbose_)
                    std::printf("[炮塔] %s %d 步（误差 %+.1f°，目标 %.1f°）\n",
                                dec.dir > 0 ? "右转" : "左转", dec.steps,
                                dec.err_deg, dec.target_yaw_deg);
            }
        }

        if (ctrl_->state() == AimState::Done) {
            if (doneSince_ < 0) doneSince_ = now;
            if (cfg_.fire_enabled && ctrl_->confirmedSide() >= 0 &&
                now - lastFire_ >= cfg_.fire_cooldown_s &&
                now - doneSince_ >= cfg_.fire_delay_s) {
                fire();
            }
        } else {
            doneSince_ = -1.0;
        }

        makeStatus(dec);
    }

    // 投票结算后调用：以最终结论强制指定侧别并立即开始对准
    void finish(bool sideLeft) {
        if (!ready()) return;
        ctrl_->forceSide(sideLeft ? 0 : 1);
        finalized_ = true;
        if (verbose_)
            std::printf("[炮塔] 最终判定：高分区在%s，开始自主转向\n", sideLeft ? "左" : "右");
    }

    // 把 waitKey 拿到的键原样丢进来；返回 true 表示这个键被炮塔模块吃掉了
    bool handleKey(char k) {
        if (!ready()) return false;
        switch (k) {
            case 'f': case 'F':
                fire();
                return true;
            case 'c': case 'C':
                sendFrames(buildFrame(CMD_CENTER));
                if (verbose_) std::printf("[炮塔] 回中\n");
                return true;
            case 'x': case 'X':
                sendFrames(buildFrame(CMD_STOP));
                if (verbose_) std::printf("[炮塔] 急停\n");
                return true;
            case 'r': case 'R':
                ctrl_->reset();
                finalized_ = false;
                doneSince_ = -1.0;
                if (verbose_) std::printf("[炮塔] 控制器复位\n");
                return true;
            default:
                return false;
        }
    }

    void close() {
        if (!link_) return;
        link_->send(buildFrame(CMD_STOP));
        link_->close();
        link_ = nullptr;
    }

    // 现场 / 自测可调项
    void setTurnInterval(double s) { cfg_.turn_interval_s = s; }   // 0 = 不限速
    void setFireEnabled(bool on)   { cfg_.fire_enabled = on; }
    void setVerbose(bool on)       { verbose_ = on; }

    // 状态文字，直接 putText 到画面上
    const std::string& statusText() const { return status_; }
    double posDeg()   const { return link_ ? link_->status().posDeg() : 0.0; }
    int    framesSent() const { return frames_; }
    int    turnsSent()  const { return turns_; }
    int    firesSent()  const { return fires_; }

private:
    double nowS() const {
        return std::chrono::duration<double>(Clock::now() - t0_).count();
    }

    bool sendFrames(const std::vector<uint8_t>& f) {
        if (!link_) return false;
        if (!link_->send(f)) return false;
        ++frames_;
        return true;
    }

    void fire() {
        if (!ready()) return;
        sendFrames(buildU16(CMD_SET_FIRE_MS, (uint16_t)cfg_.ms_per_turn));
        if (sendFrames(buildFrame2(CMD_FIRE, (uint8_t)cfg_.fire_turns, (uint8_t)cfg_.fire_speed))) {
            lastFire_ = nowS();
            ++fires_;
            if (verbose_)
                std::printf("[炮塔] 发射 %d 圈 @%d%%\n", cfg_.fire_turns, cfg_.fire_speed);
        }
    }

    void makeStatus(const AimDecision& dec) {
        const int s = ctrl_->confirmedSide();
        const Side side = (s == 0) ? Side::Left : (s == 1 ? Side::Right : Side::Unknown);
        char b[192];
        std::snprintf(b, sizeof(b), "AIM %-7s side=%-7s pos=%.1f tgt=%+.1f err=%+.1f%s",
                      aimStateName(ctrl_->state()), sideName(side),
                      link_->status().posDeg(), dec.target_yaw_deg, dec.err_deg,
                      finalized_ ? "  [已结算]" : "");
        status_ = b;
    }

    AppConfig  cfg_;
    std::unique_ptr<AimController> ctrl_;
    std::unique_ptr<SerialLink>    serLink_;
    std::unique_ptr<SimLink>       simLink_;
    IMortatLink* link_ = nullptr;   // 指向上面两个之一，不持有

    Clock::time_point t0_;
    double lastPoll_  = -1e9;
    double lastTurn_  = -1e9;
    double lastFire_  = -1e9;
    double doneSince_ = -1.0;
    bool   finalized_ = false;
    bool   verbose_   = true;

    int    frames_ = 0;
    int    turns_  = 0;
    int    fires_  = 0;

    std::string info_;
    std::string status_ = "AIM --";
};

}  // namespace mortat
