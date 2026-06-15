#!/usr/bin/env python3
"""移动 Gazebo 方阵方块（通过 set_pose 服务）

用法:
    # 列出所有方块
    python3 scripts/move_block.py --list

    # 移动方块到指定位置
    python3 scripts/move_block.py block_r0_c0 0.3 -0.1

    # 移动多个（随机打乱）
    python3 scripts/move_block.py --random
"""

import subprocess, sys, argparse, random

WORLD = "block_grid_world"

parser = argparse.ArgumentParser()
parser.add_argument("model", nargs="?", help="模型名")
parser.add_argument("x", nargs="?", type=float, help="X坐标")
parser.add_argument("y", nargs="?", type=float, help="Y坐标")
parser.add_argument("--list", action="store_true", help="列出所有方块")
parser.add_argument("--random", action="store_true", help="随机打乱所有方块位置")
parser.add_argument("--z", type=float, default=0.785, help="Z高度")
args = parser.parse_args()

BLOCKS = [f"block_r{r}_c{c}" for r in range(3) for c in range(3)]

if args.list:
    print("可移动方块:")
    for b in BLOCKS: print(f"  {b}")
    print(f"\n示例: python3 scripts/move_block.py {BLOCKS[0]} 0.3 0.1")
    sys.exit(0)

def move(name, x, y, z=0.785):
    cmd = ["gz", "service", "-s", f"/world/{WORLD}/set_pose",
           "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
           "--timeout", "1000",
           "--req", f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}']
    r = subprocess.run(cmd, capture_output=True, timeout=2)
    return r.returncode == 0

if args.random:
    for b in BLOCKS:
        x = random.uniform(0.2, 0.8)
        y = random.uniform(-0.25, 0.25)
        ok = move(b, x, y)
        print(f"  {b} → ({x:.2f}, {y:.2f}) {'✓' if ok else '✗'}")
    print("完成! YOLO 窗口应看到新位置")
elif args.model and args.x is not None:
    ok = move(args.model, args.x, args.y, args.z)
    print(f"{args.model} → ({args.x}, {args.y}) {'✓' if ok else '✗'}")
else:
    parser.print_help()
