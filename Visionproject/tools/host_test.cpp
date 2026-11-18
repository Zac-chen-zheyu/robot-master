// ---------------------------------------------------------------------------
//  host_test.cpp —— 不依赖 OpenCV 的单元测试
//
//  覆盖：
//    1) 协议组帧与 CRC（对照 ServoConsole.cs 的算法，手工核对的期望值）
//    2) 状态帧解析
//    3) 流式解析器（文本/坏帧/半包）
//    4) 高分区判据（8 种标签组合，与附件样图一一对应）
//    5) 瞄准闭环收敛（含"目标在另一侧"、180° 限位、相对偏转不漂移）
//
//  编译（Linux 或 Windows+MinGW 都可以）：
//      g++ -std=c++17 -O2 -Iinclude tools/host_test.cpp
//          src/mortat_protocol.cpp src/aim_controller.cpp -o host_test
// ---------------------------------------------------------------------------
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "aim_controller.h"
#include "mortat_protocol.h"
#include "sim_link.h"
#include "tag_rule.h"

using namespace mortat;

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond)                                                       \
    do                                                                    \
    {                                                                     \
        if (cond)                                                         \
        {                                                                 \
            ++g_pass;                                                     \
        }                                                                 \
        else                                                              \
        {                                                                 \
            ++g_fail;                                                     \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        }                                                                 \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                         \
    do                                                                \
    {                                                                 \
        const double va_ = (double)(a), vb_ = (double)(b);            \
        if (std::fabs(va_ - vb_) <= (double)(eps))                    \
        {                                                             \
            ++g_pass;                                                 \
        }                                                             \
        else                                                          \
        {                                                             \
            ++g_fail;                                                 \
            std::printf("  FAIL %s:%d  |%.3f - %.3f| > %.3f\n",       \
                        __FILE__, __LINE__, va_, vb_, (double)(eps)); \
        }                                                             \
    } while (0)

// =============================================================== 1. 协议组帧
static void testFrames()
{
    std::printf("[1] 协议组帧与 CRC8\n");
    // 期望值由独立实现（按 ServoConsole.cs 的 Crc8 / Build 逐步重算）得到
    CHECK(hex(buildFrame(CMD_STOP)) == "AA 55 01 08 2D");
    CHECK(hex(buildFrame(CMD_GET_STATUS)) == "AA 55 01 0B 24");
    CHECK(hex(buildFrame(CMD_CENTER)) == "AA 55 01 09 2A");
    CHECK(hex(buildFrame1(CMD_TURN_RIGHT, 1)) == "AA 55 02 02 01 FB");
    CHECK(hex(buildFrame1(CMD_TURN_LEFT, 3)) == "AA 55 02 01 03 CA");
    CHECK(hex(buildFrame2(CMD_FIRE, 1, 80)) == "AA 55 03 07 01 50 8E");
    CHECK(hex(buildU16(CMD_SET_STEP, 20)) == "AA 55 03 0C 14 00 C3");
    CHECK(hex(buildFrame1(CMD_SELECT_TARGET, 1)) == "AA 55 02 20 01 7F");
    CHECK(hex(buildU16(CMD_SET_STEP_MS, 1400)) == "AA 55 03 21 78 05 03");

    // CRC 自身：多项式 0x07、初值 0
    const uint8_t d[3] = {0x01, 0x08, 0x00};
    CHECK(crc8(d, 0, 2) == 0x2D);
}

// =============================================================== 2. 状态帧
static void testStatus()
{
    std::printf("[2] 状态帧解析\n");
    std::vector<uint8_t> p(15, 0);
    p[0] = 0x84;
    p[1] = 0x03; // pos   = 900  (90.0°)
    p[2] = 0xFF;
    p[3] = 0x7F; // memL  = 0x7FFF (无)
    p[4] = 0xFF;
    p[5] = 0x7F; // memR  = 0x7FFF
    p[6] = 0x14;
    p[7] = 0x00; // step  = 20   (2.0°)
    p[8] = 1;    // state = 顺时针
    p[9] = 33;   // progress
    p[10] = 0xFB;
    p[11] = 0xFF; // trim  = -5
    p[12] = 0x78;
    p[13] = 0x05; // ms    = 1400
    p[14] = 100;  // balance

    Status s;
    CHECK(s.parse(p));
    CHECK(s.pos_tenths == 900);
    CHECK_NEAR(s.posDeg(), 90.0, 1e-9);
    CHECK(s.mem_left == kMemNone);
    CHECK(s.step_tenths == 20);
    CHECK(s.state == 1);
    CHECK(s.progress == 33);
    CHECK(s.trim_us == -5);
    CHECK(s.ms_per_turn == 1400);
    CHECK(s.balance == 100);
    CHECK(s.firing());

    std::vector<uint8_t> bad(10, 0); // 长度不对必须拒绝
    CHECK(!s.parse(bad));
}

// =============================================================== 3. 流式解析
static void testParser()
{
    std::printf("[3] 流式解析器（文本 / 坏帧 / 半包）\n");
    FrameParser fp;
    int nf = 0;
    std::vector<std::string> texts;
    fp.onFrame = [&](uint8_t, const std::vector<uint8_t> &)
    { ++nf; };
    fp.onText = [&](const std::string &t)
    { texts.push_back(t); };

    const std::string boot = "MORTAT v2.4\r\n";
    std::vector<uint8_t> buf(boot.begin(), boot.end());

    const std::vector<uint8_t> f1 = buildFrame(CMD_STOP);
    const std::vector<uint8_t> f2 = buildFrame1(CMD_TURN_LEFT, 3);
    buf.insert(buf.end(), f1.begin(), f1.end());

    // 中间塞垃圾，模拟上电毛刺
    buf.push_back(0x00);
    buf.push_back(0xAA);
    buf.push_back(0x13);

    buf.insert(buf.end(), f2.begin(), f2.end());

    // 先喂一半，再喂另一半：验证半包缓存
    const size_t half = buf.size() / 2;
    fp.feed(buf.data(), half);
    fp.feed(buf.data() + half, buf.size() - half);

    CHECK(nf == 2);
    CHECK(texts.size() == 1);
    CHECK(!texts.empty() && texts[0] == "MORTAT v2.4");

    // CRC 错一帧，必须被丢弃并重新同步
    std::vector<uint8_t> bad = buildFrame(CMD_STOP);
    bad[bad.size() - 1] ^= 0xFF;
    fp.feed(bad.data(), bad.size());
    CHECK(nf == 2);
}

// =============================================================== 4. 判据
static void testRule()
{
    std::printf("[4] 高分区判据（8 种标签）\n");
    CHECK(decideHighSide(Side::Left, true) == Side::Left);
    CHECK(decideHighSide(Side::Left, false) == Side::Right);
    CHECK(decideHighSide(Side::Right, true) == Side::Right);
    CHECK(decideHighSide(Side::Right, false) == Side::Left);
    CHECK(decideHighSide(Side::Unknown, true) == Side::Unknown);

    // 与 附件_识别高分区 的 8 张样图一一对应
    struct Case
    {
        const char *name;
        Side apex;
        bool same;
        Side expect;
    };
    const Case cases[] = {
        {"左01 尖角左 同色(蓝/蓝)", Side::Left, true, Side::Left},
        {"左02 尖角左 同色(红/红)", Side::Left, true, Side::Left},
        {"左03 尖角右 异色(红/蓝)", Side::Right, false, Side::Left},
        {"左04 尖角右 异色(蓝/红)", Side::Right, false, Side::Left},
        {"右01 尖角右 同色(蓝/蓝)", Side::Right, true, Side::Right},
        {"右02 尖角右 同色(红/红)", Side::Right, true, Side::Right},
        {"右03 尖角左 异色(红/蓝)", Side::Left, false, Side::Right},
        {"右04 尖角左 异色(蓝/红)", Side::Left, false, Side::Right},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        const Side got = decideHighSide(cases[i].apex, cases[i].same);
        if (got != cases[i].expect)
        {
            std::printf("  FAIL %s\n", cases[i].name);
            ++g_fail;
        }
        else
        {
            ++g_pass;
        }
    }
}

// =============================================================== 5. 闭环
struct AimRun
{
    double final_pos = 0.0;
    int iters = 0;
    AimState state = AimState::Search;
};

static AimRun runAim(const AimParams &p, double start_deg, int side,
                     double apply_ratio = 1.0, int max_iters = 600)
{
    SimLink link(p.step_deg, apply_ratio);
    link.setPos(start_deg);
    AimController c(p);

    Observation o;
    o.found = true;
    o.side = side;
    o.tag_cx = 320.f;
    o.frame_w = 640;

    double t = 0.0;
    int i = 0;
    for (; i < max_iters; ++i)
    {
        const AimDecision d = c.update(o, link.status(), t);
        if (d.action == AimDecision::Turn)
        {
            const std::vector<uint8_t> f =
                (d.dir > 0) ? buildFrame1(CMD_TURN_RIGHT, (uint8_t)d.steps)
                            : buildFrame1(CMD_TURN_LEFT, (uint8_t)d.steps);
            link.send(f);
        }
        link.pump();
        t += 0.05;
        if (c.state() == AimState::Done)
            break;
    }

    AimRun r;
    r.final_pos = link.status().posDeg();
    r.iters = i;
    r.state = c.state();
    return r;
}

static void testAim()
{
    std::printf("[5] 瞄准闭环\n");

    AimParams p; // 默认：CH1、side_abs、左 55°、右 125°
    p.step_deg = 2.0;
    p.tolerance_deg = 2.0;
    p.deadband_deg = 1.0;
    p.max_steps_per_cmd = 6;
    p.confirm_frames = 3;

    // 5.1 左高分区：90° -> 55°
    {
        const AimRun r = runAim(p, 90.0, 0);
        std::printf("    左：90° -> %.2f°（%d 拍，状态 %s）\n", r.final_pos, r.iters,
                    aimStateName(r.state));
        CHECK(r.state == AimState::Done);
        CHECK_NEAR(r.final_pos, 55.0, p.tolerance_deg);
    }

    // 5.2 右高分区：90° -> 125°
    {
        const AimRun r = runAim(p, 90.0, 1);
        std::printf("    右：90° -> %.2f°（%d 拍，状态 %s）\n", r.final_pos, r.iters,
                    aimStateName(r.state));
        CHECK(r.state == AimState::Done);
        CHECK_NEAR(r.final_pos, 125.0, p.tolerance_deg);
    }

    // 5.3 "高分区在另一侧"：从 10° 转到 160°，跨 150°
    {
        const AimRun r = runAim(p, 10.0, 1);
        std::printf("    另一侧：10° -> %.2f°（%d 拍）\n", r.final_pos, r.iters);
        CHECK(r.state == AimState::Done);
        CHECK_NEAR(r.final_pos, 125.0, p.tolerance_deg);
    }

    // 5.4 角度归一化：190° -> -170°，-190° -> 170°（"走最近的路"的基础）
    {
        CHECK_NEAR(wrap180Deg(190.0), -170.0, 1e-9);
        CHECK_NEAR(wrap180Deg(-190.0), 170.0, 1e-9);
        CHECK_NEAR(wrap180Deg(540.0), 180.0, 1e-9);
        CHECK_NEAR(wrap180Deg(30.0), 30.0, 1e-9);
        std::printf("    归一化：190°->%.1f°  -190°->%.1f°\n",
                    wrap180Deg(190.0), wrap180Deg(-190.0));
    }

    // 5.5 AIM_TAG 模式：标签偏画面右边 -> 方位角为正值 -> 右转
    {
        AimParams q = p;
        q.mode = AIM_TAG;
        q.hfov_deg = 60.0;
        q.cam_mount_yaw_deg = 90.0;

        AimController c(q);
        Status st;
        st.pos_tenths = 900;

        Observation o;
        o.found = true;
        o.side = -1;
        o.tag_cx = 480.f;
        o.frame_w = 640;

        const AimDecision d = c.update(o, st, 0.0);
        std::printf("    TAG：右偏 160px -> 方位 %+.2f°，目标 %.2f°，%s\n",
                    d.tag_azimuth_deg, d.target_yaw_deg, d.note.c_str());
        // fx = 320 / tan(30°) = 554.3，dx = 160 -> atan(160/554.3) = 16.1°
        CHECK(d.tag_azimuth_deg > 15.f && d.tag_azimuth_deg < 17.f);
        CHECK_NEAR(d.target_yaw_deg, 90.0 + d.tag_azimuth_deg, 0.01);
        CHECK(d.action == AimDecision::Turn);
        CHECK(d.dir > 0);
    }

    // 5.5 步进舵机走不准（每次只到位 85%）：闭环必须还能收敛
    {
        const AimRun r = runAim(p, 90.0, 0, 0.85);
        std::printf("    走不准(85%%)：90° -> %.2f°（%d 拍）\n", r.final_pos, r.iters);
        CHECK(r.state == AimState::Done);
        CHECK_NEAR(r.final_pos, 55.0, p.tolerance_deg);
        CHECK(r.iters < 40);
    }

    // 5.6 相对偏转模式：确认后不应每帧都继续偏，最后要停住
    {
        AimParams q = p;
        q.mode = AIM_SIDE_REL;
        q.side_offset_deg = 35.0;

        SimLink link(p.step_deg, 1.0);
        link.setPos(90.0);
        AimController c(q);
        Observation o;
        o.found = true;
        o.side = 0;
        o.tag_cx = 320.f;
        o.frame_w = 640;

        double t = 0.0;
        double pos_after_done = 0.0;
        for (int i = 0; i < 100; ++i)
        {
            const AimDecision d = c.update(o, link.status(), t);
            if (d.action == AimDecision::Turn)
            {
                link.send((d.dir > 0) ? buildFrame1(CMD_TURN_RIGHT, (uint8_t)d.steps)
                                      : buildFrame1(CMD_TURN_LEFT, (uint8_t)d.steps));
            }
            link.pump();
            t += 0.05;
            if (c.state() == AimState::Done)
            {
                pos_after_done = link.status().posDeg();
                break;
            }
        }
        // 再喂 20 帧，位置不许漂
        for (int i = 0; i < 20; ++i)
        {
            const AimDecision d = c.update(o, link.status(), t);
            if (d.action == AimDecision::Turn)
            {
                link.send((d.dir > 0) ? buildFrame1(CMD_TURN_RIGHT, (uint8_t)d.steps)
                                      : buildFrame1(CMD_TURN_LEFT, (uint8_t)d.steps));
            }
            link.pump();
            t += 0.05;
        }
        std::printf("    相对偏转：停在 %.2f°，20 帧后 %.2f°\n",
                    pos_after_done, link.status().posDeg());
        CHECK_NEAR(pos_after_done, 55.0, p.tolerance_deg);
        CHECK_NEAR(link.status().posDeg(), pos_after_done, 0.5);
    }

    // 5.10 180° 舵机顶到限位：必须停手并给出提示
    {
        AimParams q = p;
        q.axis = 0;
        q.servo_min_deg = 5.0;
        q.servo_max_deg = 175.0;
        q.yaw_right_deg = 178.0;

        SimLink link(p.step_deg, 1.0);
        link.setPos(175.0);
        AimController c(q);
        Observation o;
        o.found = true;
        o.side = 1;
        o.tag_cx = 320.f;
        o.frame_w = 640;

        AimDecision d;
        for (int i = 0; i < 3; ++i)
            d = c.update(o, link.status(), i * 0.05); // 先攒够确认帧

        std::printf("    限位：%s\n", d.note.c_str());
        CHECK(d.action == AimDecision::None);
        CHECK(d.note.find("限位") != std::string::npos);
    }

    // 5.8 侧别还没确认：不许动
    {
        AimParams q = p;
        AimController c(q);
        SimLink link(p.step_deg, 1.0);
        link.setPos(90.0);
        Observation o;
        o.found = true;
        o.side = -1;
        o.tag_cx = 320.f;
        o.frame_w = 640;

        const AimDecision d = c.update(o, link.status(), 0.0);
        CHECK(d.action == AimDecision::None);
        CHECK(!d.side_confirmed);
    }

    // 5.9 没看到标签：按间隔扫视
    {
        AimParams q = p;
        q.seek_enabled = true;
        q.seek_interval_s = 0.5;
        q.seek_step = 2;
        AimController c(q);
        Status st;

        Observation o;
        o.found = false;

        const AimDecision d1 = c.update(o, st, 0.0);
        CHECK(d1.action == AimDecision::Turn);
        CHECK(d1.dir > 0);

        const AimDecision d2 = c.update(o, st, 0.2);
        CHECK(d2.action == AimDecision::None); // 间隔没到，不动

        const AimDecision d3 = c.update(o, st, 0.6);
        CHECK(d3.action == AimDecision::Turn);
    }
}

// =============================================================== 6. 仿真链路
static void testSimLink()
{
    std::printf("[6] 仿真链路\n");
    SimLink link(2.0, 1.0);
    link.setPos(90.0);
    link.send(buildFrame1(CMD_TURN_RIGHT, 3));
    link.pump();
    CHECK_NEAR(link.status().posDeg(), 96.0, 0.01);

    link.send(buildFrame1(CMD_TURN_LEFT, 3));
    CHECK_NEAR(link.status().posDeg(), 90.0, 0.01);

    link.send(buildFrame(CMD_CENTER));
    CHECK_NEAR(link.status().posDeg(), 90.0, 0.01);

    link.send(buildFrame2(CMD_FIRE, 1, 80));
    CHECK(link.fireCount() == 1);
    CHECK(link.status().state == 1);
    for (int i = 0; i < 5; ++i)
        link.pump();
    CHECK(link.status().state == 0);
}

int main()
{
    std::printf("========== MORTAT 视觉炮塔 host_test ==========\n");
    testFrames();
    testStatus();
    testParser();
    testRule();
    testAim();
    testSimLink();
    std::printf("----------------------------------------------\n");
    std::printf("通过 %d 项，失败 %d 项\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
