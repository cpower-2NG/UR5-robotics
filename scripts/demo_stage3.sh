#!/usr/bin/env bash
# 阶段 3 一键演示（开发机）。真正逻辑在 demo_stage3.py，这里只挑解释器。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${DEMO_PYTHON:-$HOME/robotics/venvs/robot-pose/bin/python}"

if [ ! -x "$PY" ]; then
  echo "找不到解释器：$PY" >&2
  echo "可以用 DEMO_PYTHON=/path/to/python 指一个装了 numpy/scipy/cv2 的环境。" >&2
  exit 2
fi

exec "$PY" -u "$HERE/demo_stage3.py" "$@"
