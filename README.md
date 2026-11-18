# MORTAT 视觉自动瞄准（C++17 / OpenCV 4 / Ubuntu 64）

把「识别高分区标签」和「炮塔自动转向」接在一起：相机看到标签 → 按规则判出高分区在左还是右 →
算出炮塔该转到哪个角度 → 通过和 `ServoConsole.cs` 完全相同的串口协议驱动下位机转向，
对准后再（可选）发射。

> 判据（规则手册 4.1）已经写进代码里：
> **圆与方颜色相同 → 三角形所指那侧是高分区；颜色不同 → 所指的相反那侧是高分区。**
> 所以"高分区和标签朝向不在同一侧"是正常情况，代码里由 `decideHighSide()` 统一处理。

---

## 1. 目录结构

```
炮塔视觉自动瞄准/
├── CMakeLists.txt
├── README.md                      ← 本文件
├── config/
│   └── aim.ini                    ← 所有可调参数（改完不用重新编译）
├── include/
│   ├── mortat_protocol.h          ← 0xAA55 协议、CRC8、状态帧、流式解析（与 ServoConsole.cs 一致）
│   ├── tag_rule.h                 ← 高分区判据（纯逻辑，可单独测）
│   ├── detect_params.h            ← 识别阈值
│   ├── hiscore_detector.h         ← 标签识别（参考实现，可换成你自己的）
│   ├── aim_controller.h           ← 侧别确认 → 目标角度 → 该转几步（纯逻辑，可单独测）
│   ├── turret_aim.h               ← ★ 塞进你 main.cpp 就能让炮塔自主转向的头文件
│   ├── aim_config.h               ← 参数结构 + INI 读取
│   ├── ini_file.h                 ← 极简 INI 解析
│   ├── mortat_link.h              ← 链路抽象
│   ├── serial_link.h              ← 真实串口（termios，115200 8N1）
│   ├── sim_link.h                 ← 无硬件仿真（没有板子也能跑通整条链路）
│   └── synth_scene.h              ← 合成场景（没有摄像头也能看到识别+转向）
├── src/
│   ├── main.cpp                   ← 主循环：采集 → 识别 → 决策 → 转向 → 显示
│   ├── hiscore_detector.cpp
│   ├── aim_controller.cpp
│   ├── aim_config.cpp
│   └── serial_link.cpp
├── tools/
│   ├── host_test.cpp              ← 单元测试（不需要相机/串口/OpenCV）
│   └── detect_images.cpp          ← 拿样图批量验识别
└── scripts/
    └── start_match.sh             ← 比赛现场启动脚本（自动切到工程根目录）
```

---

## 2. 编译（Ubuntu 64）

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev

cd 炮塔视觉自动瞄准
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

产物：`build/mortat_aim`、`build/detect_images`、`build/host_test`。
`cmake` 结束时应该打印一行 `OpenCV 版本: 4.x.x`；没打印说明 `libopencv-dev` 没装好。

---

## 3. 先跑自测（不用相机，不用板子）

```bash
./build/host_test
```

期望最后一行：`通过 72 项，失败 0 项`。它验证了：

* 组帧字节与 `ServoConsole.cs` 的 `Proto.Build` 逐字节一致（`AA 55 01 08 2D` 等 9 条对照）
* 状态帧 15 字节的解析、ACK 文本、半包/坏帧重同步
* 8 种标签组合的判据（与 `附件_识别高分区` 的 8 张样图一一对应）
* 瞄准闭环：正常收敛、跨 150° 的"另一侧"、机身只到位 85% 时仍收敛、相对偏转不漂移、180° 限位停手、侧别未确认不动、扫视节拍

拿 8 张样图验识别（可选，需要 OpenCV）：

```bash
./build/detect_images /path/to/附件_识别高分区 --out ./results
```

期望 8 张全部 `找到=是`，且 `左01/左02/左03/左04` 判成 `left`、`右0x` 判成 `right`。

---

## 4. 运行

先仿真，确认识别和决策都正常（不会真的驱动舵机）：

```bash
./build/mortat_aim --config config/aim.ini --sim
```

**虚拟机里没有摄像头**时，用合成场景——程序会自己画一个和比赛标签一样的图案（黑底板+白三角+圆方），
左右往复移动、并在"同色/异色"之间切换，可以直接看到识别结论和转向动作：

```bash
./build/mortat_aim --config config/aim.ini --sim --source synth
```

接板子：

```bash
# 只需做一次：把当前用户加进 dialout 组，然后注销重登
sudo usermod -aG dialout $USER

ls /dev/ttyUSB*                       # 确认口名，必要时改 config/aim.ini 的 device
./build/mortat_aim --config config/aim.ini
```

无显示器的机器：加 `--no-gui`。

窗口里的按键：

| 键 | 作用 |
|---|---|
| `q` / `Esc` | 退出（退出前会发 STOP） |
| `a` | 自动转向 开/关 |
| `s` | 急停（STOP） |
| `c` | 回中（CENTER） |
| `f` | 手动发射一次 |
| `r` | 控制器复位（重新确认侧别） |
| `d` | 调试：显示三档循环（关 / 识别框 / 颜色掩膜） |
| `t` | 调参模式开关（`w/s` 加减、`a/d` 换参数、`q/e` 调步幅、`x` 导出） |
| `i` | 调试：打印这一帧的识别详情 + 状态帧 |
| `p` | 调试：打印当前生效参数、状态机、累计动作次数 |

按键只对画面窗口生效，按之前先点一下窗口（否则会被终端吃掉）。

**调参模式（`t`）** 是模态的：进入后 `a/s/d/w` 只用于改颜色阈值（画面自动切到掩膜：
黑底板=灰白、白三角=绿、红=红、蓝=蓝），再按 `t` 退出后恢复原义。三个颜色的调法
（什么现象调哪个参数、往哪调、副作用）见 `比赛现场操作手册.txt` 的附录 E。

命令行参数：

```
--config <path>        配置文件
--source <0|dev|file|synth>  相机编号 / /dev/video0 / 视频文件 / synth 合成场景
--sim                  仿真，不发真实串口
--no-gui               不弹窗
--fire                 允许自动发射（默认只转向）
--no-freeze            识别成功后不冻结保存单帧
--result-dir <dir>     结果图目录
```

---

## 5. 七步标定（第一次用照着做）

1. **串口**：`ls /dev/ttyUSB*` 看名字，填进 `[serial] device`；权限见上。把 `ServoConsole.exe` 先关掉，
   两边不能同时占串口。
2. **控制轴**：`[aim] axis = 1`（CH1 360° 步进）推荐。要转另一侧经常要超过 90°，180° 舵机（CH0）行程不够；
   选 0 时程序会按 `servo_min_deg/servo_max_deg` 硬性限位并提示。
3. **转向符号**：按一下 `左转`，如果炮塔实际往右走了，把 `invert_dir = 1`。
4. **左右目标角度**：先用 `ServoConsole` 手动把炮塔转到"打左高分区"的姿态，读状态里的 `位置`，
   填进 `yaw_left_deg`；同理填 `yaw_right_deg`。（`mode = side_rel` 则只需填 `side_offset_deg`。）
5. **相机安装**：
   * 相机与炮管**反着装**（看着后方）→ `cam_mount_yaw_deg = 180`
   * 图像左右是镜像的 → `mirror_x = 1`
   * 只有 `mode = tag` 才需要把 `hfov_deg` 填准（手机查型号视场角，或用两点标定）
6. **精度**：`tolerance_deg = 2.0`、`deadband_deg = 1.0` 起步；想更快就加大 `max_steps_per_cmd`，
   想更稳就减到 3~4。步进舵机是"按时间估算角度"，本来就走不准，所以必须靠 `CMD_GET_STATUS`
   回读角度做闭环——程序每 50ms 查一次，自动补偿。
7. **找不到标签**：`seek_enabled = 1`，炮塔会以 ±`seek_range_deg`、每 `seek_interval_s` 走 `seek_step` 步来回扫，
   把标签扫进视野后就切成对准。

---

## 6. "高分区和标签不在同一侧" 的三种情况

| 情况 | 表现 | 代码处理 |
|---|---|---|
| 判据上反侧 | 圆/方颜色不同，高分区在三角尖端的反面 | `tag_rule.h: decideHighSide()`，异色取 `opposite()` |
| 炮塔当前在另一侧 | 目标角与当前角差 >90° | `wrap180Deg()` 走最近的路；CH0 到限位会停手并提示改用 CH1 |
| 标签不在视野里 | 一开始可能背对标签 | `seek_*` 扫视，看到后立即进入 CONFIRM/ALIGN |

侧别是**锁存**的：连续 `confirm_frames` 帧判成同一侧才生效，之后要改判得连续异侧超过 `unlatch_s` 秒，
避免单帧误识别把炮塔甩出去。

---

## 7. 把你的程序接上转向（推荐，比赛用这个）

`src/vision_aim_main.cpp` 就是**你桌面 `main.txt` 的原文**，只加了 11 处 `// >>> AIM` 标记的代码，
你的识别流程、`w/s/a/d/b/n/t/y/k/l` 调参键、20 帧投票、30 秒超时，**一个都没动**。

编译与运行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)          # 目标名 vision_aim

./build/vision_aim                      # 接板子
MORTAT_SIM=1 ./build/vision_aim         # 仿真，不碰串口
```

新增的四个键（不占用你原来的键）：

| 键 | 作用 |
|---|---|
| `F` | 发射一次（手动） |
| `C` | 炮塔回中 90° |
| `X` | **急停**（注意：你的 `s` 是白 V_min −5，不是急停） |
| `R` | 炮塔控制器复位（清空侧别锁存） |

它怎么工作：

- 未按 `Enter` 前不驱动炮塔（免得调参时乱扫），只显示 `AIM` 状态
- 按 `Enter` 后，每帧把"这一帧判出哪一侧"喂给 `TurretAim`；20 帧投票出结论 → `finish()`
  强制锁死最终侧别 → 继续闭环对准到 `DONE` → 按 `fire.enabled` 决定是否发射
- 结算时自动存 `results/result_<时间>_<left|right>.png`（裁判要的那张单帧证据）

### 想只改自己现有文件（不用这个文件）

在你自己的 `main.cpp` 里加 6 处即可，`#include "turret_aim.h"` 之后：

```cpp
static mortat::TurretAim g_aim;                                  // ① 文件顶部

g_aim.init("config/aim.ini", std::getenv("MORTAT_SIM") != nullptr); // ② main() 开头

float board_cx = -1.0f;            // ③ 每帧：透视变换的四点均值
bool  frame_ok = logic_passed;
bool  frame_left = (current_result == "left");                     // ④ 判定之后

if (is_started)                                                    // ⑤ 每帧
    g_aim.observe(is_finished || frame_ok,
                  is_finished ? (final_result == "left") : frame_left,
                  board_cx, frame.cols);

g_aim.finish(final_result == "left");   // ⑥ 结算时（只调一次）
g_aim.handleKey(key);                   // ⑦ 按键：F/C/X/R 归它，其余放行
g_aim.close();                          // ⑧ 退出前
```

编译时把 `src/aim_controller.cpp src/aim_config.cpp src/serial_link.cpp` 一起编，
并加 `-Iinclude`。

---

## 8. 状态机

```
        ┌──────────► LOST（没看到标签，按节拍扫视）
        │              │ 看到标签
   SEARCH ─────────────┘
        │ 侧别连续 confirm_frames 帧一致
        ▼
     CONFIRM ──► ALIGN（误差 > tolerance，发 Turn，每次最多 max_steps_per_cmd 步）
                    │ 误差 ≤ tolerance 且连续 3 帧
                    ▼
                 SETTLE ──► DONE（可发射；默认不发射）
```

DONE 之后仍在闭环：目标动了（`mode = tag`）或侧别改判，会重新进入 ALIGN。

---

## 9. 和 ServoConsole.cs 的关系

* 协议完全一致：`0xAA 0x55 LEN CMD DATA CRC8`，命令号、状态帧 15 字节布局、`STOP_BASE_US = 1535`、
  `MEM_NONE = 0x7FFF` 全部照抄，`host_test` 里有逐字节对照。
* 用法建议：用 C# 的 `ServoConsole` 先手动标定步长/中位/平衡，确认机械没问题；再跑本程序做自动瞄准。
* **两个程序不能同时打开同一个串口**（会互相抢字节）。

---

## 10. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `打不开 /dev/ttyUSB0` | 权限：`sudo usermod -aG dialout $USER` 后重新登录；或口名不是 ttyUSB0 |
| 板子没反应 | 确认没被别的程序占用；`--sim` 能跑说明程序本身没问题 |
| 转向相反 | `invert_dir = 1` |
| 一直 CONFIRM 不动 | 侧别没确认：看窗口里 `high score:` 是否稳定；调 `confirm_frames` 或识别阈值 |
| 到某一侧就顶住 | 180° 舵机行程不够，改 `axis = 1` 用 360° 步进；或放宽 `servo_min/max_deg` |
| 在目标附近来回抖 | 加大 `deadband_deg`、减小 `max_steps_per_cmd` |
| 识别不到 | 先用 `detect_images` 在样图上调 `black_v_max / white_v_min / min_panel_area`，再到现场微调 |
| `cmake` 报找不到 OpenCV | `sudo apt install libopencv-dev`，或 `cmake -DOpenCV_DIR=...` |

---

## 11. 安全默认值

* `fire_enabled = 0`：默认**只转向、不发射**，确认场地安全后再改 1。
* 任何退出路径（`q`、Ctrl+C、SIGTERM）都会先发 `CMD_STOP`。
* 180° 舵机有软件限位，不会硬顶；`max_steps_per_cmd` 限制单条指令步数，防止一次甩太大。
* 建议第一次一定先 `--sim`，再接板子，人站在急停按钮旁。

---

## 12. 已验证 / 未验证

已验证（本机 Windows + MinGW 上实际跑过）：

* `host_test` 72 项全通过（协议、解析、判据、闭环、限位、扫视、仿真链路）。
* 识别判据用 `附件_识别高分区` 的 8 张样图逐张核算，8/8 与文件名 `左xx/右xx` 一致。

未验证：本机没有 Linux + OpenCV 环境，`hiscore_detector.cpp` / `main.cpp` 只用"OpenCV 4 接口桩"
做了语法与类型检查，没有真的链过 `libopencv`。第一次在 Ubuntu 上编译如果报错，大概率是
OpenCV 小版本 API 差异（例如 `cvRound`、`findContours` 的参数），按报错改一两行即可；
识别阈值建议按第 10 节的 `detect_images` 流程现场微调。
