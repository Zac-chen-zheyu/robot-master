#pragma once
// ---------------------------------------------------------------------------
//  aim_config.h —— 全部可调参数 + INI 读取
// ---------------------------------------------------------------------------
#include <string>

#include "aim_controller.h"
#include "detect_params.h"

namespace mortat
{

    struct AppConfig
    {
        // ---- [serial] ----
        std::string device = "/dev/ttyUSB0";
        int baud = 115200;

        // ---- [camera] ----
        std::string source = "0"; // "0" / "/dev/video0" / 视频文件路径
        int width = 640;
        int height = 480;
        int fps = 60;

        // ---- [detect] ----
        DetectParams detect;

        // ---- [aim] ----
        AimParams aim;

        // ---- [fire] ----
        bool fire_enabled = false; // 默认不开火，只转向
        int fire_turns = 1;
        int fire_speed = 80;
        int ms_per_turn = 1200; // 每圈毫秒（v2.5 固件/上位机默认 1200）
        double fire_cooldown_s = 5.0;
        double fire_delay_s = 0.4; // 对准后再等多久才打

        // ---- [run] ----
        bool sim = false;
        bool gui = true;
        bool freeze_result = true; // 识别成功后冻结单帧并保存（比赛要提交/给裁判看）
        bool print_config = true;
        double status_poll_s = 0.05;
        double turn_interval_s = 0.18; // 两条转向指令之间的最小间隔
        std::string result_dir = "results";
        bool save_result = true;

        bool load(const std::string &path, std::string *err);
        std::string dump() const;
    };

} // namespace mortat
