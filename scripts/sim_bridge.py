#!/usr/bin/env python3
"""仿真侧「手」：把板子的命令翻译成 ROS 动作。

为什么需要它
------------
板子与开发机**不同网段**，板子 → 开发机的方向被 NAT/防火墙挡住（2026-11-07
实测：板子 ping 不通开发机；台账第三十一轮的结论在当前拓扑下仍成立）。
⇒ 板子**不能**直接加入开发机的 DDS 图，任何「板上跑 ROS 节点去控仿真」的设计
都走不通。

唯一可用的方向是「开发机发起、板子回程」：开发机 ``ssh -R`` 建一条反向隧道，
板子在**自己的** 127.0.0.1 上就能连到本进程的 TCP 端口。于是：

    板子（轮逻辑 / 决策）  --TCP JSON-->  sim_bridge（开发机 ROS 图）  --> JTC

板子是**控制方**：每条轨迹都是板上算、板上发；本进程只做翻译和转交，不做任何
决策。这与 `AGENTS.md`「接缝 = SSH + 文件」的同一条思路，只是把文件换成了
一条常驻 TCP（轨迹只有几百字节，文件接缝每步要等一次往返，太慢）。

协议：一行一个 JSON（请求 → 一个响应），字段见 ``handle()``。

⚠️ ``move`` **必须补 t=0 起点**：2026-11-07 实测，只发终点的单点轨迹会让
JTC 以最大速度冲过去，闭爪时块被打飞（0/2）；补起点后 16/16。planner_node
已修，这里独立再实现一遍，因为板子发来的计划同样可能只有一个点。

用法：
    python3 scripts/sim_bridge.py --port 23500
"""
import argparse
import json
import os
import socket
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

DEFAULT_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
GRIPPER_JOINT = "ag95_left_finger_joint"
# 仿真里夹爪是 JTC 的第 7 个关节，q∈[0, 0.0475]，开口 = 2q（米）。
STATUS_NAME = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
               4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}


def wait_future(fut, timeout):
    """轮询等待 rclpy Future（executor 在另一个线程 spin）。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if fut.done():
            return fut
        time.sleep(0.01)
    return None


class SimBridge(Node):
    def __init__(self, action_name: str, log_path: str):
        super().__init__("sim_bridge")
        self._lock = threading.Lock()
        self._joints = {}
        self._stamp = 0.0
        self._log_path = log_path
        self._group = ReentrantCallbackGroup()
        self.create_subscription(JointState, "/joint_states", self._on_joint, 50,
                                 callback_group=self._group)
        self._ac = ActionClient(self, FollowJointTrajectory, action_name,
                                callback_group=self._group)
        self._action_name = action_name
        self.get_logger().info("sim_bridge 就绪，action=%s" % action_name)

    # ---------------- ROS 侧 ----------------
    def _on_joint(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._joints[name] = float(pos)
            self._stamp = time.time()

    def joints(self):
        with self._lock:
            return dict(self._joints), self._stamp

    def _wait_joints(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            j, _ = self.joints()
            if j:
                return j
            time.sleep(0.05)
        return None

    def _send(self, names, points, timeout):
        """发一条两点式轨迹。points = [(positions, secs), ...]。"""
        if not self._ac.wait_for_server(timeout_sec=5.0):
            return {"ok": False, "error": "action server 不在：%s（仿真起了吗？）"
                    % self._action_name}
        current, _ = self.joints()
        traj = JointTrajectory()
        traj.joint_names = list(names)
        # 起点：当前角。缺了它 JTC 会把整段时间压缩成一次冲刺（2026-11-07 实测）。
        start = [current.get(n) for n in names]
        msg_points = []
        if all(v is not None for v in start):
            p0 = JointTrajectoryPoint()
            p0.positions = [float(v) for v in start]
            p0.velocities = [0.0] * len(names)
            p0.time_from_start = Duration(sec=0, nanosec=0)
            msg_points.append(p0)
        t = 0.0
        for positions, secs in points:
            t += float(secs)
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in positions]
            p.velocities = [0.0] * len(names)
            whole = int(t)
            p.time_from_start = Duration(sec=whole, nanosec=int((t - whole) * 1e9))
            msg_points.append(p)
        traj.points = msg_points

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=2, nanosec=0)

        t0 = time.time()
        fut = self._ac.send_goal_async(goal)
        fut = wait_future(fut, 10.0)
        if fut is None:
            return {"ok": False, "error": "send_goal_async 超时"}
        gh = fut.result()
        if not gh.accepted:
            return {"ok": False, "error": "目标被 JTC 拒绝"}
        res_fut = gh.get_result_async()
        res_fut = wait_future(res_fut, timeout)
        if res_fut is None:
            gh.cancel_goal_async()
            return {"ok": False, "error": "执行超时 %.1fs（已取消）" % timeout}
        res = res_fut.result()
        status = int(res.status)
        err = int(res.result.error_code) if res.result is not None else -1
        ok = (status == GoalStatus.STATUS_SUCCEEDED and err == 0)
        after, _ = self.joints()
        return {
            "ok": ok,
            "status": STATUS_NAME.get(status, str(status)),
            "error_code": err,
            "elapsed": round(time.time() - t0, 3),
            "positions": {n: after.get(n) for n in names},
        }

    # ---------------- 协议 ----------------
    # ⚠️ 名字**不能**叫 handle：rclpy 的 Node.handle 是属性，Node.__init__ 里
    #    `with self.handle:` 用它做上下文管理；子类同名方法会把它覆盖成绑定方法，
    #    构造时直接 AttributeError: __enter__（2026-11-07 踩过）。
    def dispatch(self, req):
        op = req.get("op")
        if op == "ping":
            return {"ok": True, "host": "dev-sim", "action": self._action_name,
                    "action_ready": self._ac.server_is_ready()}
        if op == "state":
            j, stamp = self.joints()
            return {"ok": True, "joints": j, "age": round(time.time() - stamp, 3)
                    if stamp else None}
        if op == "joints":
            j = self._wait_joints()
            if j is None:
                return {"ok": False, "error": "5s 内没收到 /joint_states"}
            return {"ok": True, "joints": j}
        if op == "move":
            names = req["names"]
            points = [(p["positions"], p.get("secs", 2.0)) for p in req["points"]]
            if not points:
                return {"ok": False, "error": "points 为空"}
            return self._send(names, points, req.get("timeout", 120.0))
        if op == "gripper":
            # 开口(mm) → 单指半开 q(米)：开口 = 2q ⇒ q = mm/2000。
            # 与 grasp_planner/gripper.py 的 SimGripper.move_to 同一口径；
            # 2026-11-07 头一版写成 mm/(2*95)*0.0475（正好差一半），
            # 板上命令 95 mm 时关节只到 0.0237（= 开口 47.5 mm）。
            mm = float(req["mm"])
            q = mm / 2000.0
            return self._send([GRIPPER_JOINT], [([q], req.get("secs", 3.0))],
                              req.get("timeout", 30.0))
        return {"ok": False, "error": "未知 op：%r" % (op,)}

    def record(self, req, rep):
        try:
            with open(self._log_path, "a") as f:
                f.write(json.dumps({"t": time.time(), "req": req, "rep": rep},
                                   ensure_ascii=False) + "\n")
        except OSError:
            pass


def serve(node: SimBridge, host: str, port: int):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host, port))
    s.listen(4)
    node.get_logger().info("监听 %s:%d" % (host, port))

    def client(conn):
        with conn:
            f = conn.makefile("rwb")
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    req = json.loads(line)
                    rep = node.dispatch(req)
                except Exception as e:  # 单条命令出错不能打断链路
                    rep = {"ok": False, "error": "%s: %s" % (type(e).__name__, e)}
                node.record(req, rep)
                f.write((json.dumps(rep, ensure_ascii=False) + "\n").encode())
                f.flush()

    while True:
        conn, _ = s.accept()
        threading.Thread(target=client, args=(conn,), daemon=True).start()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=23500)
    ap.add_argument("--action", default=DEFAULT_ACTION)
    ap.add_argument("--log", default=os.path.expanduser("~/sim_bridge.jsonl"))
    args = ap.parse_args()

    rclpy.init()
    node = SimBridge(args.action, args.log)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        serve(node, args.host, args.port)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
