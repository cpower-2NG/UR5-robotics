#!/usr/bin/env python3
"""板子侧客户端：板子是**控制方**，这条命令就由板子发出。

链路
----
    板子本进程  --TCP 127.0.0.1:23500-->  ssh -R 反向隧道  -->  开发机 sim_bridge

反向隧道必须由**开发机**发起（板子 → 开发机方向被 NAT 挡住，2026-11-07 实测）。
隧道的生命周期见 ``scripts/sim_board_link.sh``。

用法（在板子上跑）：
    python3 sim_board_agent.py ping
    python3 sim_board_agent.py joints
    python3 sim_board_agent.py home
    python3 sim_board_agent.py gripper --mm 50
    python3 sim_board_agent.py move --plan plan.json
    python3 sim_board_agent.py move --names j1,...,j6 --points '[[0,0,0,0,0,0],2.0]'

``--points`` 的每一项是 ``[positions, secs]``（也接受
``{"positions": [...], "secs": 2.0}``）。轨迹由**板子**给出，
``sim_bridge`` 只负责补 t=0 起点并转交 JTC，不做任何决策。
"""
import argparse
import json
import os
import socket
import sys

DEFAULT_HOST = os.environ.get("SIM_BRIDGE_HOST", "127.0.0.1")
DEFAULT_PORT = int(os.environ.get("SIM_BRIDGE_PORT", "23500"))
# 仿真世界（base_link 就在台面上）的折叠位姿，见 grasp_planner_sim.yaml。
SIM_HOME = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


class SimClient:
    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT, timeout=180.0):
        self.host, self.port, self.timeout = host, port, timeout

    def call(self, **req):
        s = socket.create_connection((self.host, self.port), timeout=self.timeout)
        try:
            f = s.makefile("rwb")
            f.write((json.dumps(req, ensure_ascii=False) + "\n").encode())
            f.flush()
            line = f.readline()
        finally:
            s.close()
        if not line:
            raise SystemExit("桥接端没有回包（隧道断了吗？）")
        return json.loads(line)

    # --- 便捷方法 ---
    def ping(self):
        return self.call(op="ping")

    def joints(self):
        return self.call(op="joints")

    def state(self):
        # 带 age（秒）：不重试、不阻塞，用来看仿真是不是还活着
        return self.call(op="state")

    def move(self, names, points, timeout=180.0):
        # 统一成 {positions, secs}：协议那头按 dict 解析
        norm = [p if isinstance(p, dict) else _norm(p) for p in points]
        return self.call(op="move", names=names, points=norm, timeout=timeout)

    def gripper(self, mm, secs=3.0):
        return self.call(op="gripper", mm=mm, secs=secs)

    def home(self, positions=None, secs=3.0):
        return self.move(ARM_JOINTS, [(positions or SIM_HOME, secs)])


def _norm(p):
    if isinstance(p, dict):
        return {"positions": p["positions"], "secs": p.get("secs", 2.0)}
    return {"positions": p[0], "secs": p[1] if len(p) > 1 else 2.0}


def parse_points(raw):
    """接受 ``[{"positions":…}]``、``[[positions, secs], …]``，以及单点简写
    ``[positions, secs]``。"""
    pts = json.loads(raw)
    if not pts:
        return []
    # 单点简写：[ [j1..j6], secs ] —— 首元素是"数的列表"，不是"点的列表"
    if isinstance(pts[0], list) and all(isinstance(x, (int, float)) for x in pts[0]):
        return [_norm(pts)]
    return [_norm(p) for p in pts]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ping")
    sub.add_parser("joints")
    sub.add_parser("state")
    sub.add_parser("home")
    g = sub.add_parser("gripper")
    g.add_argument("--mm", type=float, required=True)
    g.add_argument("--secs", type=float, default=3.0)
    m = sub.add_parser("move")
    m.add_argument("--plan", help="JSON 文件：{names, points}")
    m.add_argument("--names", help="逗号分隔的关节名")
    m.add_argument("--points", help='[[positions], secs] 或 [{"positions":…,"secs":…}]')
    m.add_argument("--timeout", type=float, default=180.0)
    args = ap.parse_args()

    c = SimClient(args.host, args.port)
    if args.cmd == "ping":
        r = c.ping()
    elif args.cmd == "joints":
        r = c.joints()
    elif args.cmd == "state":
        r = c.state()
    elif args.cmd == "home":
        r = c.home()
    elif args.cmd == "gripper":
        r = c.gripper(args.mm, args.secs)
    else:
        if args.plan:
            plan = json.load(open(args.plan))
            names, points = plan["names"], plan["points"]
        else:
            # 不给 --names 时默认六轴（2026-11-07 踩过：省略直接 AttributeError）
            names = args.names.split(",") if args.names else list(ARM_JOINTS)
            points = parse_points(args.points)
        r = c.move(names, points, args.timeout)
    print(json.dumps(r, ensure_ascii=False, indent=2))
    sys.exit(0 if r.get("ok") else 1)


if __name__ == "__main__":
    main()
