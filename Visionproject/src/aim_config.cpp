// ---------------------------------------------------------------------------
//  aim_config.cpp —— INI 读取与打印
// ---------------------------------------------------------------------------
#include "aim_config.h"

#include <sstream>

#include "ini_file.h"

namespace mortat
{
    namespace
    {

        AimMode parseMode(const std::string &s, AimMode def)
        {
            if (s == "tag" || s == "0")
                return AIM_TAG;
            if (s == "side_abs" || s == "abs" || s == "1")
                return AIM_SIDE_ABS;
            if (s == "side_rel" || s == "rel" || s == "2")
                return AIM_SIDE_REL;
            return def;
        }

        const char *modeName(AimMode m)
        {
            switch (m)
            {
            case AIM_TAG:
                return "tag（直接瞄标签）";
            case AIM_SIDE_ABS:
                return "side_abs（按侧别转到固定角度）";
            case AIM_SIDE_REL:
                return "side_rel（按侧别相对偏转）";
            }
            return "?";
        }

    } // namespace

    bool AppConfig::load(const std::string &path, std::string *err)
    {
        IniFile ini;
        if (!ini.load(path, err))
            return false;

        // ------------------------------------------------ [serial]
        device = ini.getStr("serial", "device", device);
        baud = ini.getInt("serial", "baud", baud);

        // ------------------------------------------------ [camera]
        source = ini.getStr("camera", "source", source);
        width = ini.getInt("camera", "width", width);
        height = ini.getInt("camera", "height", height);
        fps = ini.getInt("camera", "fps", fps);

        // ------------------------------------------------ [detect]
        detect.work_width = ini.getInt("detect", "work_width", detect.work_width);
        detect.min_panel_area = ini.getInt("detect", "min_panel_area", detect.min_panel_area);
        detect.min_panel_fill = ini.getDbl("detect", "min_panel_fill", detect.min_panel_fill);
        detect.black_s_max = ini.getInt("detect", "black_s_max", detect.black_s_max);
        detect.black_v_max = ini.getInt("detect", "black_v_max", detect.black_v_max);
        detect.white_s_max = ini.getInt("detect", "white_s_max", detect.white_s_max);
        detect.white_v_min = ini.getInt("detect", "white_v_min", detect.white_v_min);
        detect.color_s_min = ini.getInt("detect", "color_s_min", detect.color_s_min);
        detect.color_v_min = ini.getInt("detect", "color_v_min", detect.color_v_min);
        detect.min_color_area = ini.getInt("detect", "min_color_area", detect.min_color_area);
        detect.red_h_low = ini.getInt("detect", "red_h_low", detect.red_h_low);
        detect.red_h_high = ini.getInt("detect", "red_h_high", detect.red_h_high);
        detect.blue_h_low = ini.getInt("detect", "blue_h_low", detect.blue_h_low);
        detect.blue_h_high = ini.getInt("detect", "blue_h_high", detect.blue_h_high);
        detect.circle_fill_max = ini.getDbl("detect", "circle_fill_max", detect.circle_fill_max);
        detect.circle_circ_min = ini.getDbl("detect", "circle_circ_min", detect.circle_circ_min);
        detect.allow_shape_only = ini.getBool("detect", "allow_shape_only", detect.allow_shape_only);
        detect.debug = ini.getBool("detect", "debug", detect.debug);

        // ------------------------------------------------ [aim]
        aim.axis = ini.getInt("aim", "axis", aim.axis);
        aim.mode = parseMode(ini.getStr("aim", "mode", "side_abs"), aim.mode);
        aim.yaw_left_deg = ini.getDbl("aim", "yaw_left_deg", aim.yaw_left_deg);
        aim.yaw_right_deg = ini.getDbl("aim", "yaw_right_deg", aim.yaw_right_deg);
        aim.side_offset_deg = ini.getDbl("aim", "side_offset_deg", aim.side_offset_deg);
        aim.aim_offset_deg = ini.getDbl("aim", "aim_offset_deg", aim.aim_offset_deg);
        aim.hfov_deg = ini.getDbl("aim", "hfov_deg", aim.hfov_deg);
        aim.cam_mount_yaw_deg = ini.getDbl("aim", "cam_mount_yaw_deg", aim.cam_mount_yaw_deg);
        aim.mirror_x = ini.getBool("aim", "mirror_x", aim.mirror_x);
        aim.invert_dir = ini.getBool("aim", "invert_dir", aim.invert_dir);
        aim.tolerance_deg = ini.getDbl("aim", "tolerance_deg", aim.tolerance_deg);
        aim.deadband_deg = ini.getDbl("aim", "deadband_deg", aim.deadband_deg);
        aim.confirm_frames = ini.getInt("aim", "confirm_frames", aim.confirm_frames);
        aim.latch_side = ini.getBool("aim", "latch_side", aim.latch_side);
        aim.unlatch_s = ini.getDbl("aim", "unlatch_s", aim.unlatch_s);
        aim.min_steps = ini.getInt("aim", "min_steps", aim.min_steps);
        aim.max_steps_per_cmd = ini.getInt("aim", "max_steps_per_cmd", aim.max_steps_per_cmd);
        aim.step_deg = ini.getDbl("aim", "step_deg", aim.step_deg);
        aim.seek_enabled = ini.getBool("aim", "seek_enabled", aim.seek_enabled);
        aim.seek_range_deg = ini.getDbl("aim", "seek_range_deg", aim.seek_range_deg);
        aim.seek_step = ini.getInt("aim", "seek_step", aim.seek_step);
        aim.seek_interval_s = ini.getDbl("aim", "seek_interval_s", aim.seek_interval_s);
        aim.servo_min_deg = ini.getDbl("aim", "servo_min_deg", aim.servo_min_deg);
        aim.servo_max_deg = ini.getDbl("aim", "servo_max_deg", aim.servo_max_deg);

        // ------------------------------------------------ [fire]
        fire_enabled = ini.getBool("fire", "enabled", fire_enabled);
        fire_turns = ini.getInt("fire", "turns", fire_turns);
        fire_speed = ini.getInt("fire", "speed", fire_speed);
        ms_per_turn = ini.getInt("fire", "ms_per_turn", ms_per_turn);
        fire_cooldown_s = ini.getDbl("fire", "cooldown_s", fire_cooldown_s);
        fire_delay_s = ini.getDbl("fire", "delay_s", fire_delay_s);

        // ------------------------------------------------ [run]
        gui = ini.getBool("run", "gui", gui);
        freeze_result = ini.getBool("run", "freeze_result", freeze_result);
        status_poll_s = ini.getDbl("run", "status_poll_s", status_poll_s);
        turn_interval_s = ini.getDbl("run", "turn_interval_s", turn_interval_s);
        result_dir = ini.getStr("run", "result_dir", result_dir);
        save_result = ini.getBool("run", "save_result", save_result);

        // 基本合法性检查
        if (baud < 9600)
            baud = 115200;
        if (width <= 0 || height <= 0)
        {
            width = 640;
            height = 480;
        }
        if (aim.step_deg < 0.1)
            aim.step_deg = 0.1;
        if (aim.tolerance_deg < aim.deadband_deg)
            aim.tolerance_deg = aim.deadband_deg;
        if (aim.confirm_frames < 1)
            aim.confirm_frames = 1;
        if (aim.max_steps_per_cmd < 1)
            aim.max_steps_per_cmd = 1;
        if (aim.axis != 0)
            aim.axis = 1;
        return true;
    }

    std::string AppConfig::dump() const
    {
        std::ostringstream o;
        o << "串口      : " << device << " @" << baud << " 8N1\n";
        o << "相机      : " << source << "  " << width << "x" << height << " @ " << fps << "fps\n";
        o << "控制轴    : " << (aim.axis == 0 ? "CH0 180°舵机" : "CH1 360°步进") << "\n";
        o << "瞄准模式  : " << modeName(aim.mode) << "\n";
        o << "左右角度  : 左 " << aim.yaw_left_deg << "°  右 " << aim.yaw_right_deg << "°"
          << "   相对偏转 " << aim.side_offset_deg << "°\n";
        o << "像素->角度: HFOV " << aim.hfov_deg << "°  相机安装角 " << aim.cam_mount_yaw_deg
          << "°  mirror_x=" << (aim.mirror_x ? 1 : 0) << " invert_dir=" << (aim.invert_dir ? 1 : 0) << "\n";
        o << "容差/死区 : " << aim.tolerance_deg << "° / " << aim.deadband_deg << "°"
          << "  确认帧 " << aim.confirm_frames << "  单次最多 " << aim.max_steps_per_cmd << " 步\n";
        o << "扫视      : " << (aim.seek_enabled ? "开" : "关")
          << "  范围 ±" << aim.seek_range_deg << "°  每 " << aim.seek_interval_s << "s 走 "
          << aim.seek_step << " 步\n";
        o << "开火      : " << (fire_enabled ? "开" : "关")
          << "  圈数 " << fire_turns << "  速度 " << fire_speed << "%\n";
        o << "运行      : gui=" << (gui ? 1 : 0) << " sim=" << (sim ? 1 : 0)
          << " 冻结单帧=" << (freeze_result ? 1 : 0) << " 结果目录 " << result_dir << "\n";
        return o.str();
    }

} // namespace mortat
