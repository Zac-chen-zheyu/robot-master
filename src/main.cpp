// ---------------------------------------------------------------------------
//  main.cpp —— 视觉识别 + 炮塔自动转向 主程序
//
//  数据流：
//    摄像头帧 -> HisCoreDetector（黑底板 / 白三角 / 圆方颜色 -> 高分区侧）
//             -> AimController（侧别确认 -> 目标角度 -> 差多少度 -> 走几步）
//             -> IMortatLink（0xAA55 帧，CRC8）
//             -> 下位机 PCA9685 驱动 CH0 180° 舵机 / CH1 360° 步进
//
//  编译：cmake -S . -B build && cmake --build build -j
//  运行：./build/mortat_aim --config config/aim.ini
//        没有板子：./build/mortat_aim --config config/aim.ini --sim
// ---------------------------------------------------------------------------
#include <algorithm>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cmath>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "aim_config.h"
#include "aim_controller.h"
#include "hiscore_detector.h"
#include "mortat_link.h"
#include "serial_link.h"
#include "sim_link.h"
#include "synth_scene.h"

namespace
{

    volatile std::sig_atomic_t g_quit = 0;
    void onSignal(int) { g_quit = 1; }

    std::string stampMs()
    {
        using namespace std::chrono;
        const auto t = system_clock::now();
        const auto ms = duration_cast<milliseconds>(t.time_since_epoch()) % 1000;
        const std::time_t tt = system_clock::to_time_t(t);
        std::tm tmv{};
#if defined(_WIN32)
        localtime_s(&tmv, &tt);
#else
        localtime_r(&tt, &tmv);
#endif
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d",
                      tmv.tm_hour, tmv.tm_min, tmv.tm_sec, (int)ms.count());
        return std::string(buf);
    }

    void logLine(const std::string &s)
    {
        std::printf("[%s] %s\n", stampMs().c_str(), s.c_str());
        std::fflush(stdout);
    }

    void sleepMs(int ms)
    {
        if (ms > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

    std::string fileStamp()
    {
        const std::time_t tt = std::time(nullptr);
        std::tm tmv{};
#if defined(_WIN32)
        localtime_s(&tmv, &tt);
#else
        localtime_r(&tt, &tmv);
#endif
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
                      tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                      tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
        return std::string(buf);
    }

    struct Args
    {
        std::string config = "config/aim.ini";
        std::string source;
        std::string result_dir;
        bool sim = false;
        bool gui = true;
        bool freeze = true;
        bool fire = false;
        bool help = false;
    };

    void printUsage()
    {
        std::printf(
            "用法: mortat_aim [选项]\n"
            "  --config <path>      配置文件，默认 config/aim.ini\n"
            "  --source <0|dev|file|synth>\n"
            "                       相机编号(0) / /dev/video0 / 视频文件 / synth=合成场景\n"
            "  --sim                无硬件仿真：不下发真实串口，舵机走位在内存里模拟\n"
            "  --no-gui             不弹窗口（在无显示器的 Ubuntu 上跑）\n"
            "  --fire               允许自动发射（默认只转向不发射）\n"
            "  --no-freeze          识别成功后不冻结保存单帧\n"
            "  --result-dir <dir>   结果图片目录，默认 results\n"
            "  -h, --help           显示本帮助\n");
    }

    bool parseArgs(int argc, char **argv, Args *a)
    {
        for (int i = 1; i < argc; ++i)
        {
            const std::string s(argv[i]);
            if (s == "-h" || s == "--help")
            {
                a->help = true;
                return true;
            }
            else if (s == "--config" && i + 1 < argc)
            {
                a->config = argv[++i];
            }
            else if (s == "--source" && i + 1 < argc)
            {
                a->source = argv[++i];
            }
            else if (s == "--result-dir" && i + 1 < argc)
            {
                a->result_dir = argv[++i];
            }
            else if (s == "--sim")
            {
                a->sim = true;
            }
            else if (s == "--no-gui")
            {
                a->gui = false;
            }
            else if (s == "--fire")
            {
                a->fire = true;
            }
            else if (s == "--no-freeze")
            {
                a->freeze = false;
            }
            else
            {
                std::printf("未知参数: %s\n", s.c_str());
                return false;
            }
        }
        return true;
    }

    // 比赛要求的"最终单帧识别结果"图：大字 high score: left/right
    cv::Mat annotateResult(const cv::Mat &frame, const mortat::TagInfo &tag)
    {
        cv::Mat img = frame.clone();
        const double k = std::max(0.6, img.cols / 640.0);
        const int thick = std::max(1, (int)(2 * k));

        cv::rectangle(img, cv::Point(0, 0), cv::Point(img.cols - 1, img.rows - 1),
                      cv::Scalar(0, 200, 0), thick * 4);

        const std::string text = std::string("high score: ") + mortat::sideName(tag.high);
        const cv::Point org((int)(16 * k), (int)(img.rows - 26 * k));
        cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, 1.4 * k,
                    cv::Scalar(0, 0, 0), thick * 5, cv::LINE_AA);
        cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, 1.4 * k,
                    cv::Scalar(0, 255, 255), thick * 2, cv::LINE_AA);

        char small[192];
        std::snprintf(small, sizeof(small), "%s  apex=%s  same_color=%d  id=%d  score=%.2f",
                      fileStamp().c_str(), mortat::sideName(tag.apex),
                      tag.same_color ? 1 : 0, tag.id, tag.score);
        cv::putText(img, small, cv::Point((int)(16 * k), (int)(26 * k)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6 * k, cv::Scalar(230, 230, 230), thick, cv::LINE_AA);
        return img;
    }

    void drawHud(cv::Mat *img, const mortat::AimDecision &dec, mortat::AimState state,
                 const mortat::DetectResult &det, const mortat::Status &st,
                 bool aim_on, bool sim)
    {
        const double k = std::max(0.6, img->cols / 640.0);
        const int thick = std::max(1, (int)(1.6 * k));

        // 画面中心参考线
        cv::line(*img, cv::Point(img->cols / 2, 0), cv::Point(img->cols / 2, img->rows),
                 cv::Scalar(90, 90, 90), thick);
        cv::line(*img, cv::Point(0, img->rows / 2), cv::Point(img->cols, img->rows / 2),
                 cv::Scalar(90, 90, 90), thick);

        // 目标中心 -> 画面中心 的连线
        if (det.found && det.tag.center.x > 0.f)
        {
            cv::circle(*img, cv::Point((int)det.tag.center.x, (int)det.tag.center.y),
                       (int)(5 * k), cv::Scalar(0, 255, 255), -1);
            cv::line(*img, cv::Point(img->cols / 2, img->rows / 2),
                     cv::Point((int)det.tag.center.x, (int)det.tag.center.y),
                     cv::Scalar(0, 255, 255), thick, cv::LINE_AA);
        }

        char line1[256];
        std::snprintf(line1, sizeof(line1),
                      "state=%-7s side=%-7s pos=%5.1f  target=%6.1f  err=%+6.1f",
                      mortat::aimStateName(state),
                      det.found ? mortat::sideName(det.tag.high) : "none",
                      st.posDeg(), dec.target_yaw_deg, dec.err_deg);
        cv::putText(*img, line1, cv::Point((int)(10 * k), (int)(24 * k)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55 * k, cv::Scalar(0, 255, 0), thick, cv::LINE_AA);

        char line2[256];
        std::snprintf(line2, sizeof(line2),
                      "%s  step=%.1fdeg  aim=%s  %s",
                      sim ? "SIM" : "REAL", st.stepDeg(),
                      aim_on ? "ON" : "OFF", dec.note.c_str());
        cv::putText(*img, line2, cv::Point((int)(10 * k), (int)(48 * k)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5 * k, cv::Scalar(200, 255, 200), thick, cv::LINE_AA);

        // 结论横幅
        if (det.found && det.tag.high != mortat::Side::Unknown)
        {
            const std::string s = std::string("high score: ") + mortat::sideName(det.tag.high);
            int base = 0;
            cv::Size sz = cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, 1.0 * k, thick * 2, &base);
            cv::rectangle(*img, cv::Point(0, img->rows - sz.height - (int)(18 * k)),
                          cv::Point(sz.width + (int)(20 * k), img->rows), cv::Scalar(0, 0, 0), -1);
            cv::putText(*img, s, cv::Point((int)(10 * k), img->rows - (int)(8 * k)),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0 * k, cv::Scalar(0, 255, 255), thick * 2, cv::LINE_AA);
        }
    }

    // ------------------------------------------------------------ 调参面板
    struct TuneItem
    {
        const char *key;   // 对应 aim.ini 里的键名
        const char *label; // 中文说明
        int *value;        // 直接指向 detector 的参数
        int step;          // 一次加减多少
        int lo;
        int hi;
    };

    void drawTunePanel(cv::Mat *img, const std::vector<TuneItem> &items, int sel)
    {
        const double k = std::max(0.6, img->cols / 640.0);
        const int thick = std::max(1, (int)(1.2 * k));
        const int pad = (int)(8 * k);
        const int lh = (int)(17 * k);

        int w = (int)(330 * k);
        if (w > img->cols / 2)
            w = img->cols / 2;
        const int h = pad * 2 + (int)(20 * k) + lh * (int)items.size() + (int)(16 * k);
        const int x0 = img->cols - w - (int)(4 * k);
        const int y0 = (int)(56 * k);

        cv::rectangle(*img, cv::Rect(x0, y0, w, h), cv::Scalar(25, 25, 25), -1);
        cv::putText(*img, "TUNE  w/s:+/-  a/d:select  q/e:step  x:dump  t:exit",
                    cv::Point(x0 + pad, y0 + (int)(14 * k)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.40 * k, cv::Scalar(0, 255, 255), thick, cv::LINE_AA);

        for (size_t i = 0; i < items.size(); ++i)
        {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "%s %-18s %4d  (%d)",
                          ((int)i == sel) ? ">" : " ", items[i].label,
                          *items[i].value, items[i].step);
            cv::putText(*img, buf,
                        cv::Point(x0 + pad, y0 + (int)(34 * k) + (int)i * lh),
                        cv::FONT_HERSHEY_SIMPLEX, 0.46 * k,
                        ((int)i == sel) ? cv::Scalar(0, 255, 0) : cv::Scalar(205, 205, 205),
                        thick, cv::LINE_AA);
        }
    }

} // namespace

int main(int argc, char **argv)
{
    using namespace mortat;

    Args args;
    if (!parseArgs(argc, argv, &args) || args.help)
    {
        printUsage();
        return args.help ? 0 : 1;
    }

    // ------------------------------------------------------------ 配置
    AppConfig cfg;
    std::string err;
    if (!cfg.load(args.config, &err))
    {
        logLine("!! " + err);
        logLine("!! 用内置默认参数继续运行（建议先 cd 到工程目录，或 --config 指定绝对路径）");
    }
    if (!args.source.empty())
        cfg.source = args.source;
    if (!args.result_dir.empty())
        cfg.result_dir = args.result_dir;
    if (args.sim)
        cfg.sim = true;
    if (!args.gui)
        cfg.gui = false;
    if (!args.freeze)
        cfg.freeze_result = false;
    if (args.fire)
        cfg.fire_enabled = true;

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    if (cfg.print_config)
    {
        std::printf("=========== MORTAT 视觉自动瞄准 ===========\n%s", cfg.dump().c_str());
        std::printf("==========================================\n");
    }

    // ------------------------------------------------------------ 链路
    IMortatLink *link = nullptr;
    SimLink *sim = nullptr;
    SerialLink *ser = nullptr;

    if (cfg.sim)
    {
        sim = new SimLink(cfg.aim.step_deg, 0.9);
        link = sim;
        logLine("== 仿真模式：动作不会真的发到串口 ==");
    }
    else
    {
        ser = new SerialLink(cfg.device, cfg.baud);
        if (!ser->open(&err))
        {
            logLine("!! 串口打开失败：" + err);
            logLine("!! 可以用 --sim 先验证识别与决策，再回来接板子");
            return 2;
        }
        link = ser;
        logLine("== 串口已打开 " + cfg.device + " @" + std::to_string(cfg.baud) + " ==");
    }

    // ------------------------------------------------------------ 视频源
    cv::VideoCapture cap;
    const std::string src = cfg.source;
    const bool synth = (src == "synth"); // 合成场景：没有相机也能跑
    const bool by_index = !synth && src.size() <= 2 &&
                          std::isdigit((unsigned char)(src.empty() ? 'x' : src[0]));
    if (!synth)
    {
        if (by_index)
        {
            const int idx = std::atoi(src.c_str());
#if defined(__linux__)
            cap.open(idx, cv::CAP_V4L2);
            if (!cap.isOpened())
                cap.open(idx);
#else
            cap.open(idx);
#endif
        }
        else
        {
            cap.open(src);
        }
        if (!cap.isOpened())
        {
            logLine("!! 打不开视频源：" + src);
            logLine("!! 虚拟机/无相机时用 --source synth 跑合成场景；也可用 --source <视频文件>");
            logLine("!! 检查 ls /dev/video* 、摄像头是否被别的程序占用");
            link->close();
            return 3;
        }
        cap.set(cv::CAP_PROP_FRAME_WIDTH, cfg.width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.height);
        cap.set(cv::CAP_PROP_FPS, cfg.fps);
        logLine("== 视频源已就绪：" + src + "  " +
                std::to_string((int)cap.get(cv::CAP_PROP_FRAME_WIDTH)) + "x" +
                std::to_string((int)cap.get(cv::CAP_PROP_FRAME_HEIGHT)) + " ==");
    }
    else
    {
        logLine("== 合成场景：黑底板+白三角+圆方，左右往复 + 同色/异色切换 ==");
    }

    // ------------------------------------------------------------ 初始化下位机
    link->send(buildFrame1(CMD_SELECT_TARGET, (uint8_t)cfg.aim.axis));
    link->send(buildU16(CMD_SET_STEP, (uint16_t)std::lround(cfg.aim.step_deg * 10.0)));
    link->send(buildU16(CMD_SET_FIRE_MS, (uint16_t)cfg.ms_per_turn));
    link->send(buildU16(CMD_SET_STEP_MS, (uint16_t)cfg.ms_per_turn));
    link->send(buildFrame(CMD_GET_VERSION));
    link->pump();

    HisCoreDetector detector(cfg.detect);
    AimController ctrl(cfg.aim);

    // 调参模式（t 键）能改的项：直接指向 detector 的参数，改完立刻生效
    DetectParams &dp = detector.params();
    std::vector<TuneItem> tune = {
        {"black_v_max", "黑底板亮度上限", &dp.black_v_max, 5, 0, 200},
        {"black_s_max", "黑底板饱和度上限", &dp.black_s_max, 5, 0, 255},
        {"white_v_min", "白三角亮度下限", &dp.white_v_min, 5, 100, 255},
        {"white_s_max", "白三角饱和度上限", &dp.white_s_max, 5, 0, 150},
        {"color_s_min", "彩色饱和度下限", &dp.color_s_min, 5, 0, 255},
        {"color_v_min", "彩色亮度下限", &dp.color_v_min, 5, 0, 255},
        {"red_h_low", "红色色相下限", &dp.red_h_low, 1, 0, 90},
        {"red_h_high", "红色色相上限", &dp.red_h_high, 1, 90, 179},
        {"blue_h_low", "蓝色色相下限", &dp.blue_h_low, 1, 0, 179},
        {"blue_h_high", "蓝色色相上限", &dp.blue_h_high, 1, 0, 179},
        {"min_panel_area", "底板最小面积", &dp.min_panel_area, 100, 200, 20000},
        {"min_color_area", "彩色图形最小面积", &dp.min_color_area, 20, 30, 3000},
    };

    const auto t_start = std::chrono::steady_clock::now();
    auto nowS = [&t_start]()
    {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    };

    bool aim_on = true;
    int debug_level = cfg.detect.debug ? 1 : 0; // d 键循环：0=关 1=识别框 2=掩膜
    bool tuning = false;                        // t 键：进入调参模式
    int tune_sel = 0;
    double last_turn_t = -1e9;
    double last_poll_t = -1e9;
    double last_fire_t = -1e9;
    double done_since = -1.0;
    int turns_sent = 0;
    int frames_fired = 0;
    bool result_saved = false;
    cv::Mat result_frame;
    std::string result_path;
    AimState last_state = AimState::Search;
    int last_side = -2;

    if (cfg.gui)
    {
        cv::namedWindow("MORTAT aim", cv::WINDOW_NORMAL);
        cv::resizeWindow("MORTAT aim", cfg.width, cfg.height);
    }

    cv::Mat frame;
    while (!g_quit)
    {
        const double now = nowS();

        if (synth)
        {
            frame = makeSynthFrame(now);
        }
        else if (!cap.read(frame) || frame.empty())
        {
            if (by_index)
            {
                logLine("!! 读帧失败（相机掉线？）");
                sleepMs(50);
            }
            else
            {
                logLine("== 视频文件播放完毕 ==");
                break;
            }
            continue;
        }

        // ---- 识别（d 键可以关掉调试图，省 CPU）
        detector.params().debug = (debug_level >= 1);
        detector.params().debug_masks = (debug_level == 2);
        DetectResult det = detector.detect(frame);

        // ---- 收串口
        link->pump();
        std::string ack;
        while (!(ack = link->takeAck()).empty())
            logLine("< " + ack);
        std::string txt;
        while (!(txt = link->takeText()).empty())
            logLine("< " + txt);

        const Status &st = link->status();

        // ---- 定期查状态（闭环就靠它）
        if (now - last_poll_t >= cfg.status_poll_s)
        {
            last_poll_t = now;
            link->send(buildFrame(CMD_GET_STATUS));
        }

        // ---- 决策
        Observation obs;
        obs.found = det.found;
        obs.side = (det.found && det.tag.high != Side::Unknown) ? (int)det.tag.high : -1;
        obs.tag_cx = det.found ? det.tag.center.x : -1.f;
        obs.frame_w = frame.cols;

        AimDecision dec = ctrl.update(obs, st, now);

        // ---- 执行
        if (aim_on && !tuning && dec.action == AimDecision::Turn &&
            now - last_turn_t >= cfg.turn_interval_s)
        {
            const std::vector<uint8_t> f = (dec.dir > 0)
                                               ? buildFrame1(CMD_TURN_RIGHT, (uint8_t)std::min(255, dec.steps))
                                               : buildFrame1(CMD_TURN_LEFT, (uint8_t)std::min(255, dec.steps));
            if (link->send(f))
            {
                last_turn_t = now;
                ++turns_sent;
                logLine(">> " + std::string(dec.dir > 0 ? "右转 " : "左转 ") +
                        std::to_string(dec.steps) + " 步  " + dec.note);
            }
        }

        // ---- 对准 -> 冻结结果 / 发射
        const bool aligned = (ctrl.state() == AimState::Done);
        if (aligned)
        {
            if (done_since < 0)
                done_since = now;
        }
        else
        {
            done_since = -1.0;
        }

        if (det.found && det.tag.high != Side::Unknown && !result_saved && cfg.freeze_result)
        {
            result_frame = annotateResult(frame, det.tag);
            result_saved = true;

            if (cfg.save_result)
            {
                std::string dir = cfg.result_dir;
                // 简单建目录（不想依赖 filesystem 的老编译器）
                std::string mk = "mkdir -p '" + dir + "'";
                if (std::system(mk.c_str()) != 0)
                { /* 目录已存在或失败，imwrite 会再报 */
                }
                result_path = dir + "/result_" + fileStamp() + "_" +
                              sideName(det.tag.high) + ".png";
                cv::imwrite(result_path, result_frame);
                logLine("== 已保存单帧识别结果：" + result_path + " ==");
            }
            if (cfg.gui)
            {
                cv::namedWindow("RESULT", cv::WINDOW_NORMAL);
                cv::imshow("RESULT", result_frame);
            }
        }

        if (aligned && cfg.fire_enabled && ctrl.confirmedSide() >= 0 &&
            now - last_fire_t >= cfg.fire_cooldown_s &&
            done_since > 0 && now - done_since >= cfg.fire_delay_s)
        {
            link->send(buildU16(CMD_SET_FIRE_MS, (uint16_t)cfg.ms_per_turn));
            const std::vector<uint8_t> f =
                buildFrame2(CMD_FIRE, (uint8_t)cfg.fire_turns, (uint8_t)cfg.fire_speed);
            if (link->send(f))
            {
                ++frames_fired;
                last_fire_t = now;
                logLine(">> 发射：圈数 " + std::to_string(cfg.fire_turns) +
                        " 速度 " + std::to_string(cfg.fire_speed) + "%");
            }
        }

        // ---- 状态变化日志
        if (ctrl.state() != last_state)
        {
            logLine(std::string("== 状态 ") + aimStateName(last_state) + " -> " +
                    aimStateName(ctrl.state()));
            last_state = ctrl.state();
        }
        if (ctrl.confirmedSide() != last_side)
        {
            if (ctrl.confirmedSide() >= 0)
            {
                logLine(std::string("== 高分区侧确认：") +
                        sideName(ctrl.confirmedSide() == 0 ? Side::Left : Side::Right));
            }
            last_side = ctrl.confirmedSide();
        }

        // ---- 显示
        if (cfg.gui)
        {
            cv::Mat show;
            if (debug_level == 2 && !det.mask_view.empty())
                show = det.mask_view; // 掩膜档：看三类颜色框住没有
            else if (debug_level >= 1 && !det.debug.empty())
                show = det.debug; // 识别框档
            else
                show = frame.clone();

            drawHud(&show, dec, ctrl.state(), det, st, aim_on, cfg.sim);
            if (tuning)
                drawTunePanel(&show, tune, tune_sel);
            cv::imshow("MORTAT aim", show);

            const int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27)
                break;

            // ---- 调参模式（模态：开启后 a/s/d/w 只用于调参，不再触发动作）
            if (key == 't' || key == 'T')
            {
                tuning = !tuning;
                if (tuning)
                {
                    debug_level = 2; // 自动切到掩膜视图，边调边看
                    logLine("== 进入调参模式：w/s 加减  a/d 换参数  q/e 调步幅  x 导出  t 退出");
                    logLine("   已切到颜色掩膜视图，调参期间暂停自动转向（炮塔不会动）");
                }
                else
                {
                    logLine("== 退出调参模式，当前调过的值：");
                    for (size_t i = 0; i < tune.size(); ++i)
                    {
                        char b[96];
                        std::snprintf(b, sizeof(b), "   %s = %d", tune[i].key, *tune[i].value);
                        logLine(b);
                    }
                }
            }

            if (tuning)
            {
                TuneItem &it = tune[(size_t)tune_sel];
                if (key == 'w' || key == '+')
                {
                    *it.value = std::min(it.hi, *it.value + it.step);
                }
                else if (key == 's' || key == '-')
                {
                    *it.value = std::max(it.lo, *it.value - it.step);
                }
                else if (key == 'a')
                {
                    tune_sel = (tune_sel + (int)tune.size() - 1) % (int)tune.size();
                }
                else if (key == 'd')
                {
                    tune_sel = (tune_sel + 1) % (int)tune.size();
                }
                else if (key == 'q')
                {
                    it.step = std::max(1, it.step / 2);
                }
                else if (key == 'e')
                {
                    it.step = std::min(100, it.step * 2);
                }
                else if (key == 'x')
                {
                    std::ofstream out("config/aim.tuned.ini");
                    if (out)
                    {
                        out << "; 调参模式导出（只含 [detect] 段）\n[detect]\n";
                        for (size_t i = 0; i < tune.size(); ++i)
                            out << tune[i].key << " = " << *tune[i].value << "\n";
                        out.close();
                        logLine("== 已导出 config/aim.tuned.ini，把里面的值抄进 config/aim.ini 的 [detect] 段");
                    }
                    else
                    {
                        logLine("!! 导出失败：请在工程根目录运行（需要 config/ 目录）");
                    }
                }
            }
            else
            {
                if (key == 'a')
                {
                    aim_on = !aim_on;
                    logLine(std::string("== 自动转向 ") + (aim_on ? "开" : "关"));
                }
                if (key == 's')
                {
                    link->send(buildFrame(CMD_STOP));
                    logLine(">> 急停");
                }
                if (key == 'c')
                {
                    link->send(buildFrame(CMD_CENTER));
                    logLine(">> 回中");
                }
                if (key == 'f')
                {
                    link->send(buildFrame2(CMD_FIRE, (uint8_t)cfg.fire_turns, (uint8_t)cfg.fire_speed));
                    logLine(">> 手动发射");
                }
                if (key == 'r')
                {
                    ctrl.reset();
                    result_saved = false;
                    logLine("== 控制器已复位");
                }

                // ---- 以下三个是调试键
                if (key == 'd')
                {
                    debug_level = (debug_level + 1) % 3;
                    const char *names[3] = {"关（省 CPU）", "识别框", "颜色掩膜（黑=灰白 白=绿 红=红 蓝=蓝）"};
                    logLine(std::string("== 调试显示：") + names[debug_level]);
                }
                if (key == 'i')
                {
                    if (!det.found)
                    {
                        logLine("== 识别详情：这一帧没有标签");
                    }
                    else
                    {
                        const TagInfo &t = det.tag;
                        char b[320];
                        std::snprintf(b, sizeof(b),
                                      "== 识别详情：apex=%s high=%s 圆=%s 方=%s 同色=%d id=%d "
                                      "score=%.2f 中心=(%.0f,%.0f) 底板宽=%.0f%s",
                                      sideName(t.apex), sideName(t.high),
                                      t.circle == BlobColor::Blue ? "蓝" : "红",
                                      t.square == BlobColor::Blue ? "蓝" : "红",
                                      t.same_color ? 1 : 0, t.id, t.score,
                                      t.center.x, t.center.y, t.width_px,
                                      t.shape_only ? "  [退化:只有彩色图形]" : "");
                        logLine(b);
                    }
                    const Status &sc = link->status();
                    char b2[256];
                    std::snprintf(b2, sizeof(b2),
                                  "== 状态帧：pos=%.1f° step=%.1f° 发射=%s(%d) %d%% "
                                  "每圈=%dms 平衡=%d 中位偏移=%dus 有效=%d",
                                  sc.posDeg(), sc.stepDeg(), stateText(sc.state), sc.state,
                                  sc.progress, sc.ms_per_turn, sc.balance, sc.trim_us,
                                  sc.valid ? 1 : 0);
                    logLine(b2);
                }
                if (key == 'p')
                {
                    std::printf("%s", cfg.dump().c_str());
                    std::printf("当前状态: state=%s side=%d pos=%.1f° target=%.1f° err=%+.1f° "
                                "自动转向=%s 仿真=%d 调试档=%d\n",
                                aimStateName(ctrl.state()), ctrl.confirmedSide(), st.posDeg(),
                                dec.target_yaw_deg, dec.err_deg, aim_on ? "ON" : "OFF",
                                cfg.sim ? 1 : 0, debug_level);
                    std::printf("累计: 转向指令 %d 条, 发射 %d 次\n", turns_sent, frames_fired);
                    std::fflush(stdout);
                }
            }
        }
        else
        {
            sleepMs(2);
        }
    }

    // ------------------------------------------------------------ 收尾
    link->send(buildFrame(CMD_STOP));
    link->close();
    if (cfg.gui)
        cv::destroyAllWindows();

    logLine("== 退出 ==");
    std::printf("汇总：转向指令 %d 条，发射 %d 次", turns_sent, frames_fired);
    if (!result_path.empty())
        std::printf("，识别结果图 %s", result_path.c_str());
    std::printf("\n");

    delete ser;
    delete sim;
    return 0;
}
