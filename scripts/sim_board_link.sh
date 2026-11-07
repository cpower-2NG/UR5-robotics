#!/usr/bin/env bash
# 起「板子控制仿真」链路（开发机侧）。
#
#   板子 sim_board_agent.py ──TCP──> ssh -R 反向隧道 ──> 开发机 sim_bridge.py ──> JTC
#
# 反向隧道**必须由开发机发起**：板子 → 开发机方向被 NAT/防火墙挡住
# （2026-11-07 实测板子 ping 不通开发机；台账第三十一轮同样结论）。
#
# 用法：
#   scripts/sim_board_link.sh            # 起桥 + 建隧道（前台守护，Ctrl-C 退出）
#   scripts/sim_board_link.sh --check    # 只检查两端状态
#
# 环境变量：BOARD_ALIAS（默认 jetson-hotspot）、SIM_BRIDGE_PORT（默认 23500）
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../ros2_ws" && pwd)"
PORT="${SIM_BRIDGE_PORT:-23500}"
ALIAS="${BOARD_ALIAS:-}"
BOARD_DIR="~/simlink"
LOG_DIR="${SIM_BRIDGE_LOG_DIR:-$HOME/.local/state/sim_board_link}"
mkdir -p "$LOG_DIR"

# 板子别名：不写死某台机器的别名（公开仓），默认按常见别名顺序探活。
if [ -z "$ALIAS" ]; then
    for a in jetson jetson-hotspot; do
        if ssh -o BatchMode=yes -o ConnectTimeout=4 "$a" true 2>/dev/null; then
            ALIAS="$a"; break
        fi
    done
fi
if [ -z "$ALIAS" ]; then
    echo "!! 没有可达的板子别名；用 BOARD_ALIAS=<你的 ssh 别名> 指定" >&2
    exit 1
fi
echo "== 板子别名：$ALIAS"

ros_env() {
    # ⚠️ ROS 的 setup.bash 会引用未定义变量（AMENT_TRACE_SETUP_FILES），
    #    与本脚本的 set -u 冲突 —— source 期间必须关掉 nounset。
    set +u
    . /opt/ros/humble/setup.bash
    if [ -f "$WS/install/setup.bash" ]; then
        . "$WS/install/setup.bash"
    fi
    set -u
    export GZ_IP=127.0.0.1
}

bridge_up() {
    # 端口能连上就算起（不依赖 pgrep：pgrep 会匹配到自己的命令行）
    python3 - "$PORT" <<'PY' >/dev/null 2>&1
import socket, sys
try:
    socket.create_connection(('127.0.0.1', int(sys.argv[1])), timeout=1).close()
except OSError:
    raise SystemExit(1)
PY
}

start_bridge() {
    if bridge_up; then
        echo "== sim_bridge 已在 127.0.0.1:$PORT"
        return
    fi
    echo "== 起 sim_bridge（日志 $LOG_DIR/bridge.log）"
    ros_env
    nohup python3 "$HERE/sim_bridge.py" --port "$PORT" \
        --log "$LOG_DIR/bridge.jsonl" > "$LOG_DIR/bridge.log" 2>&1 &
    for _ in $(seq 1 50); do
        bridge_up && { echo "== sim_bridge 就绪"; return; }
        sleep 0.2
    done
    echo "!! sim_bridge 10s 没起来，看 $LOG_DIR/bridge.log" >&2
    exit 1
}

push_agent() {
    echo "== 推 sim_board_agent.py 到板子 $BOARD_DIR/"
    ssh "$ALIAS" "mkdir -p $BOARD_DIR" >/dev/null
    scp -q "$HERE/sim_board_agent.py" "$ALIAS:$BOARD_DIR/" >/dev/null
    # 路径**不加引号**交给远端 shell 展开 ~（Python 字符串里 ~ 不会展开）
    ssh "$ALIAS" "python3 -m py_compile $BOARD_DIR/sim_board_agent.py" \
        && echo "== 板上语法检查通过"
}

tunnel_loop() {
    echo "== 守护反向隧道 127.0.0.1:$PORT（板子本地同端口）"
    while true; do
        ssh -N -o BatchMode=yes -o ExitOnForwardFailure=yes \
            -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
            -R "127.0.0.1:$PORT:127.0.0.1:$PORT" "$ALIAS" \
            || echo "== 隧道断开，3s 后重连" >&2
        sleep 3
    done
}

check() {
    echo "-- 开发机 bridge --"
    bridge_up && echo "  端口 $PORT: 通" || echo "  端口 $PORT: 不通"
    echo "-- 板子 --"
    ssh "$ALIAS" "test -f $BOARD_DIR/sim_board_agent.py && echo '  agent: 已推' || echo '  agent: 未推'" 2>/dev/null | grep -v Warning || true
    echo "-- 隧道（在板子上 ping 桥）--"
    ssh "$ALIAS" "python3 $BOARD_DIR/sim_board_agent.py ping" 2>/dev/null | grep -v Warning || echo "  板上 ping 失败（隧道没起？）"
}

if [ "${1:-}" = "--check" ]; then
    check
    exit 0
fi

start_bridge
push_agent
tunnel_loop
