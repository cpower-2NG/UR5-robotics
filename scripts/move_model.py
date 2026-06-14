#!/usr/bin/env python3
"""移动Gazebo物体——模拟运动，展示实时视频流闪烁效果"""

import subprocess, time, math, argparse

parser = argparse.ArgumentParser()
parser.add_argument("--model", default="mover_block", help="要移动的模型名")
parser.add_argument("--radius", type=float, default=0.03, help="圆周运动半径(m)")
parser.add_argument("--speed", type=float, default=1.0, help="角速度(rad/s)")
args = parser.parse_args()

print(f"移动 {args.model}: r={args.radius}m, ω={args.speed}rad/s")
print("Ctrl+C 停止")

t0 = time.time()
try:
    while True:
        t = time.time() - t0
        angle = args.speed * t
        x = args.radius * math.cos(angle)
        y = args.radius * math.sin(angle)

        cmd = [
            "gz", "service", "-s", "/world/block_grid_world/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "500",
            "--req",
            f'name: "{args.model}", '
            f'position: {{x: {x:.4f}, y: {y:.4f}, z: 0.06}}'
        ]
        subprocess.run(cmd, capture_output=True, timeout=1)
        time.sleep(0.05)
except KeyboardInterrupt:
    print("停止")
