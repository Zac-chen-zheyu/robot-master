#!/usr/bin/env bash
# ---------------------------------------------------------------------------
#  start_match.sh —— 比赛现场启动脚本
#
#  用法：
#     ./scripts/start_match.sh               启动（默认优先 vision_aim）
#     ./scripts/start_match.sh --sim         仿真，不碰真串口
#     ./scripts/start_match.sh --fire        允许自动发射（仅 mortat_aim 生效）
#     ./scripts/start_match.sh --no-gui      无显示器（仅 mortat_aim 生效）
#
#  说明：
#    vision_aim = 你的识别程序 + 炮塔转向（比赛用）
#    mortat_aim = 独立参考实现（备用）
#    vision_aim 的自动发射由 config/aim.ini 的 [fire] enabled 控制；
#    仿真通过 MORTAT_SIM=1 环境变量告知 vision_aim。
# ---------------------------------------------------------------------------
set -u

PROJ="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJ" || exit 1

BIN=""
if [ -x ./build/vision_aim ]; then
    BIN=./build/vision_aim
elif [ -x ./build/mortat_aim ]; then
    BIN=./build/mortat_aim
else
    echo "!! 还没编译。先执行："
    echo "   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j\$(nproc)"
    exit 1
fi

FIRE=0
SIM=0
for a in "$@"; do
    case "$a" in
        --fire)  FIRE=1 ;;
        --sim)   SIM=1 ;;
    esac
done

echo "== 工程目录: $PROJ"
echo "== 程序:     $BIN"
if [ "${BIN##*/}" = "vision_aim" ]; then
    echo "== 键位: P 预识别  Enter 开始  ESC 退出"
    echo "         w/s 白V  a/d 白S  b/n 蓝S  t/y 红S  k/l 黑V"
    echo "         F 发射  C 回中  X 急停  R 复位"
    [ "$SIM" = "1" ] && export MORTAT_SIM=1
    [ "$FIRE" = "1" ] && echo "== 提示：自动发射请改 config/aim.ini 的 [fire] enabled = 1"
    echo
    exec "$BIN"
else
    echo "== 键位: q 退出  a 自动转向  s 急停  c 回中  f 发射  r 复位  d 调试  t 调参"
    ARGS=()
    [ "$SIM" = "1" ]  && ARGS+=(--sim)
    [ "$FIRE" = "1" ] && ARGS+=(--fire)
    echo
    exec "$BIN" --config config/aim.ini "${ARGS[@]}"
fi
