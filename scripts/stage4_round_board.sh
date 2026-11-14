#!/bin/bash
# 板上轮壳（B3-lite，2026-11-14）：flock 单例 + 自动两步授权监听 + 编排器上板。
#
# 用法（由开发机 launch_round.py 经 setsid nohup 发起；也可以人手敲）：
#     bash stage4_round_board.sh <repo-dir> <编排器参数...>
#
# 职责与不变量：
# · **flock 单例**（D1）：并发第二次发起立即失败（退出 8）并指明持有者；
#   锁与编排器同一进程（末尾 exec），pid 文件 = 锁持有者 = kill 目标。
# · **授权监听**：real_motion_authorize --listen 在编排器之前起——观测语义与
#   手动两步（--step stop / --step play）逐条一致：只读 Dashboard、看到
#   STOPPED 记第 1 步、看到 PLAYING+RUNNING+NORMAL 才写令牌、**从不发送
#   play/stop**。人按示教器按键是唯一输入；本机接口造不出令牌。
#   没有 UR_ROBOT_IP / ip 文件 ⇒ 监听不起、authorize 段等不到令牌停轮
#   （fail-closed），而不是猜一个 IP。
# · 编排器固定 ``--where board``（LocalBoard）：令牌、run.sh、分析全在板上；
#   开发机断线只影响"看"，不影响"跑"。
set -u
REPO="${1:?用法: stage4_round_board.sh <repo-dir> <编排器参数...>}"
shift

LOCK="$HOME/precision/round_board.lock"
HOLDER="$HOME/precision/round_board.holder"
mkdir -p "$HOME/precision"
exec 9>"$LOCK"
flock -n 9 || {
    echo "!! 已有轮在跑（holder: $(cat "$HOLDER" 2>/dev/null || echo '?')）"
    echo "   并发发起拒绝（D1）。先 watch 确认状态，或等它结束。"
    exit 8
}
echo "$$ $*" > "$HOLDER"
# 换代：上一轮的孤儿授权监听（--timeout 1800）不许跨轮存活——多个监听会
# 重复观测同一批按键、把轮间的令牌文件写成中间态。本壳随后自己起新监听。
pkill -f 'real_motion_authorize.py --listen' 2>/dev/null || true
# ROS 环境：编排器在板上要碰 rclpy（A2 段级裁决的 /check_state_validity 客户端）
# —— b3demo-r3 实测：不 source 就是「本机没有 rclpy」秒退。
# ⚠️ setup.bash 引用未绑定变量，`set -u` 下直接炸（b3demo-r4 实测）⇒
# source 前后临时关掉 nounset。
export ROS_DOMAIN_ID=99
if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    . /opt/ros/humble/setup.bash
    set -u
fi
# 自报 pid 到轮目录（round.pid）：launch 侧的 `$!` 经多层 ssh/setsid 偶发为空
# （2026-11-14 验收实测），这里由轮壳自己写 $$ —— exec 之后 pid 不变，
# 永远是编排器本尊，watch / --kill-before-observe 的锚。
prev=""
for a in "$@"; do
    if [ "$prev" = "--out-dir" ] && [ -n "$a" ]; then
        mkdir -p "$a"
        echo "$$" > "$a/round.pid"
    fi
    prev="$a"
done

AUTH_IP="${UR_ROBOT_IP:-}"
IP_FILE="$HOME/.config/ur5-vision-grasp/ur-robot-ip"
if [ -z "$AUTH_IP" ] && [ -f "$IP_FILE" ]; then
    AUTH_IP=$(cat "$IP_FILE")
fi
if [ -n "$AUTH_IP" ]; then
    # ⚠️ 9<&-：监听**不许继承锁 fd**。b3demo 实测教训：编排器秒退后，孤儿
    # 监听（--timeout 1800）还攥着 flock，把下一次发起拒掉 30 分钟。
    python3 "$REPO/scripts/real_motion_authorize.py" --listen \
        --robot-ip "$AUTH_IP" --timeout 1800 9<&- &
    echo "授权监听已起（pid $!；robot-ip 不回显）。人在示教器按 STOP→PLAY。"
else
    echo "!! 没有 UR_ROBOT_IP 环境变量，也没有 $IP_FILE —— 授权监听未起。"
    echo "   authorize 段将等不到令牌停轮（fail-closed）；把 IP 放进 $IP_FILE 再发起。"
fi

cd "$REPO" || exit 2
# exec：pid 不变 ⇒ flock 与 round.pid 记的 pid 始终是编排器本尊
exec python3 scripts/stage4_round_run.py --where board "$@"
