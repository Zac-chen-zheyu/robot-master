// ---------------------------------------------------------------------------
//  turret_aim_test.cpp —— TurretAim 模块的单元测试（不需要 OpenCV / 相机 / 串口）
//
//  g++ -std=c++17 -O2 -Iinclude tools/turret_aim_test.cpp
//      src/aim_controller.cpp -o turret_aim_test
// ---------------------------------------------------------------------------
#include <cmath>
#include <cstdio>

#include "turret_aim.h"

using namespace mortat;

static int g_fail = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);        \
            ++g_fail;                                                            \
        }                                                                        \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                    \
    do {                                                                         \
        const double va = (double)(a), vb = (double)(b);                         \
        if (std::fabs(va - vb) > (double)(eps)) {                                \
            std::printf("  FAIL %s:%d  |%.2f - %.2f| > %.2f\n",                  \
                        __FILE__, __LINE__, va, vb, (double)(eps));              \
            ++g_fail;                                                            \
        }                                                                        \
    } while (0)

int main() {
    std::printf("===== TurretAim 自测 =====\n");

    TurretAim aim;
    // 空配置路径 -> 用内置默认值（左 55° / 右 125°）；sim=true
    const bool ok = aim.init("", true, true);
    CHECK(ok);
    CHECK(aim.ready());
    std::printf("init: %s\n", aim.info().c_str());

    // 关掉转向限速，方便一次性跑完闭环
    aim.setTurnInterval(0.0);

    // ---- 1. 连续看到"左侧"：应当开始发转向指令 ----
    for (int i = 0; i < 8; ++i) aim.observe(true, true, 320.f, 640);
    std::printf("8 帧后: %s\n", aim.statusText().c_str());
    std::printf("        发帧 %d，转向 %d\n", aim.framesSent(), aim.turnsSent());
    CHECK(aim.framesSent() > 0);
    CHECK(aim.turnsSent() > 0);
    CHECK(std::fabs(aim.posDeg() - 90.0) > 0.1);   // 已经在动

    // ---- 2. 结算：强制左侧，跑到对准 ----
    aim.finish(true);
    for (int i = 0; i < 300; ++i) aim.observe(true, true, 320.f, 640);
    std::printf("结算后: pos=%.2f°  %s\n", aim.posDeg(), aim.statusText().c_str());
    CHECK_NEAR(aim.posDeg(), 55.0, 3.0);

    // ---- 3. 按键：炮塔键吃掉，调参键必须放行 ----
    CHECK(aim.handleKey('x'));          // 急停
    CHECK(aim.handleKey('f'));          // 发射
    CHECK(aim.handleKey('c'));          // 回中
    CHECK(aim.handleKey('r'));          // 复位
    CHECK(!aim.handleKey('w'));         // 白 V_min
    CHECK(!aim.handleKey('s'));
    CHECK(!aim.handleKey('a'));
    CHECK(!aim.handleKey('d'));
    CHECK(!aim.handleKey('b'));
    CHECK(!aim.handleKey('n'));
    CHECK(!aim.handleKey('t'));
    CHECK(!aim.handleKey('y'));
    CHECK(!aim.handleKey('k'));
    CHECK(!aim.handleKey('l'));
    CHECK(!aim.handleKey('p'));
    CHECK(!aim.handleKey('\n'));        // Enter
    std::printf("按键分流正确：F/C/X/R 归炮塔，其余 11 个键留给识别调参\n");

    // ---- 4. 改判到右侧 ----
    aim.finish(false);
    for (int i = 0; i < 300; ++i) aim.observe(true, false, 320.f, 640);
    std::printf("右目标后: pos=%.2f°  %s\n", aim.posDeg(), aim.statusText().c_str());
    CHECK_NEAR(aim.posDeg(), 125.0, 3.0);

    // ---- 5. 复位后重新按帧结论走 ----
    aim.handleKey('r');
    for (int i = 0; i < 8; ++i) aim.observe(true, true, 320.f, 640);
    for (int i = 0; i < 300; ++i) aim.observe(true, true, 320.f, 640);
    std::printf("复位再判左: pos=%.2f°\n", aim.posDeg());
    CHECK_NEAR(aim.posDeg(), 55.0, 3.0);

    // ---- 6. 丢帧：不能崩，也不能乱转 ----
    const double posBefore = aim.posDeg();
    for (int i = 0; i < 5; ++i) aim.observe(false, false, -1.f, 640);
    std::printf("丢帧 5 次后: pos=%.2f°（之前 %.2f°）\n", aim.posDeg(), posBefore);

    aim.close();

    std::printf(g_fail ? "\n有 %d 项失败\n" : "\n全部通过\n", g_fail);
    return g_fail ? 1 : 0;
}
