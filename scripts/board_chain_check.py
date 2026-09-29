#!/usr/bin/env python3
"""板端无臂全链路预检：真相机 → 真 YOLO → 投影 → 规划，只差机械臂驱动。

**在板子上跑。** 回答的问题是「这块板子能不能扛住真机部署的那条链路」——
不接机械臂、不进仿真器，用**实物俯视相机**把感知链路整条跑起来并测吞吐。

它起什么
--------
  · `real.launch.py start_ur_driver:=false` —— 即**真机部署配置**：
    `camera_source` 是硬编码的 `realsense`，所以 `ik_group` 自动取真机的
    `ur_manipulator`、`trajectory_action` 指向真机控制器、`max_sync_dt` 取 0.3。
    没有驱动 ⇒ 没有 controller_manager ⇒ **物理上没有任何东西可动**。
  · `robot_state_publisher`（ur_description 的**未标定** URDF）。
    缺了它 `base_link` 这个帧根本不存在，projector 会一直
    `TF 查询失败："base_link" ... does not exist`（实测踩过）。
  · 一个**假 /joint_states** 发布器（HOME 定频）。缺了它 move_group 的
    CurrentStateMonitor 拿不到状态，`/compute_ik` **挂住不回话**（实测踩过）。
  · 相机静态 TF `base_link → camera_link`。⚠️ **默认值不是标定结果**，
    只是「相机在上方一米朝下看」的占位，用来把 TF 树接通、让投影能算。
    **所以 /targets_3d 的数值在这一步不可信**，本脚本也不对它做数值断言；
    它断言的是**链路通不通、跑多快**。真外参要等手眼标定（见 W2/W5）。

测什么
------
逐级吞吐（相机 → 检测 → 三维目标），加一份资源采样。判据是「能不能持续」：
真机规划器只需要 1 Hz 级，所以这条链路有十几倍余量就算健康。

⚠️ 与另外两个板端脚本的分工
--------------------------
  · `check_board_deploy.py`  —— 静态就绪度（包装没装、模型路径解析到哪），不起进程
  · `moveit_only_check.py`   —— 只验 MoveIt 那一段（`/compute_ik` + 肘高探针），不吃相机
  · **本脚本**                —— 把感知那一段也拉起来，测**端到端吞吐**

⚠️ `planner_node` 会报「轨迹 action 服务不可用」并停住 —— 这是**预期的**：
没有驱动就没有控制器，规划器的状态机走不动。本条不影响吞吐测量，
但意味着**本脚本不覆盖运动**。那一步要真臂。

用法
----
    cd ~/yolo_ros_original && source /opt/ros/humble/setup.bash \\
        && source ros2_ws/install/setup.bash && python3 scripts/board_chain_check.py

退出码：0 = 吞吐达标且投影链路通；1 = 有判据不达标；2 = 环境不满足。
"""

import argparse
import os
import re
import signal
import subprocess
import sys
import time

# 脚本必须在**仓库根**跑：detector_node 的 `model` 默认值是相对路径
# （`block_best.pt`），按进程 cwd 解析。从别处起会 FileNotFoundError 崩掉，
# 而 /detections 只是「没有消息」——离原因很远。（check_board_deploy.py 的
# D67 就是为这个坑写的，本脚本直接在开头 chdir 掉它。）
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 残留守卫。⚠️ 排除 `bash -c`：经 ssh 跑时调用者那条命令行里带着同样的模式，
# 会把**自己**匹配上（同 moveit_only_check.py / board_ctrlc_check.py 的陷阱）。
LEFTOVER_RE = re.compile(
    r"ros2 launch yolo_bringup|move_group|robot_state_publisher"
    r"|detector_node|projector_node|planner_node|realsense2_camera_node"
    r"|fake_joint_states")

# 吞吐判据（FPS）。真机规划器只要 1 Hz 级；这里取的是「明显健康」的下限，
# 不是「卡在边缘」的值 —— 面板上掉到这些数以下就该查了。
GATE_CAMERA = 20.0
GATE_DETECTIONS = 15.0
GATE_TARGETS = 15.0


def leftovers():
    r = subprocess.run(["ps", "-eo", "pid,args", "--no-headers"],
                       capture_output=True, text=True)
    me = os.path.basename(__file__)
    return [l.strip() for l in r.stdout.splitlines()
            if "bash -c" not in l and me not in l and LEFTOVER_RE.search(l)]


def sh(cmd, timeout=60):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                              timeout=timeout, cwd=REPO)
    except subprocess.TimeoutExpired as e:
        out = e.stdout or ""
        if isinstance(out, bytes):
            out = out.decode(errors="replace")
        return subprocess.CompletedProcess(cmd, 124, out, "TIMEOUT")


def topic_rate(topic, seconds=12):
    """跑 `ros2 topic hz` 拿平均频率。返回 (fps 或 None, 原始输出)。"""
    r = sh(f"timeout {seconds} ros2 topic hz {topic} 2>&1", timeout=seconds + 12)
    rates = re.findall(r"average rate:\s*([0-9.]+)", r.stdout)
    if not rates:
        return None, r.stdout.strip()[:200]
    return float(rates[-1]), r.stdout.strip()[:200]


def sample_resources(seconds=5):
    """tegrastats 采样。⚠️ 必须 `stdbuf -oL`：不带行缓冲时管道里攒着，
    进程被 kill 后一个字节都读不到（实测踩过，空输出会伪装成「没这个工具」）。
    ⚠️ 也不能用 `pkill -f tegrastats` 收尾 —— 那个模式会匹配到**调用者自己的
    命令行**，把自己杀掉（实测踩过，ssh 直接 255）。用 PID。"""
    log = "/tmp/board_chain_tegrastats.log"
    p = subprocess.Popen(
        f"stdbuf -oL tegrastats --interval 1000 > {log} 2>&1",
        shell=True, cwd=REPO, start_new_session=True)
    time.sleep(seconds + 0.5)
    try:
        os.killpg(p.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    try:
        lines = [l for l in open(log, errors="replace").read().splitlines() if l.strip()]
    except OSError:
        return {}
    if not lines:
        return {}
    mid = lines[len(lines) // 2]          # 取中间一条，跳过启动/收尾的瞬时值
    out = {}
    for key, pat in [("RAM", r"RAM (\d+)/(\d+)MB"), ("GR3D", r"GR3D_FREQ (\d+)%"),
                     ("温度", r"tj@([\d.]+)C"), ("整板功耗", r"VDD_IN (\d+)mW"),
                     ("CPU+GPU 功耗", r"VDD_CPU_GPU_CV (\d+)mW")]:
        m = re.search(pat, mid)
        if m:
            out[key] = m.groups()
    m = re.search(r"CPU \[([^\]]+)\]", mid)
    if m:
        pcts = [int(x.split("%")[0]) for x in m.group(1).split(",")]
        out["CPU 各核"] = (pcts,)
    return out


def main():
    ap = argparse.ArgumentParser(description="板端无臂全链路预检（零运动风险）")
    ap.add_argument("--ur-type", default="ur5")
    ap.add_argument("--log", default="/tmp/board_chain_launch.log")
    # 相机占位外参：上方 1m、朝下看。⚠️ 不是标定结果，只为接通 TF 树。
    ap.add_argument("--camera-x", default="0.5")
    ap.add_argument("--camera-y", default="0.0")
    ap.add_argument("--camera-z", default="1.0")
    ap.add_argument("--camera-qx", default="-0.5")
    ap.add_argument("--camera-qy", default="0.5")
    ap.add_argument("--camera-qz", default="0.5")
    ap.add_argument("--camera-qw", default="0.5")
    args = ap.parse_args()

    if not os.environ.get("ROS_DISTRO"):
        print("环境未就绪：先 source /opt/ros/humble/setup.bash 和 "
              "ros2_ws/install/setup.bash（约定：环境不满足退 2）")
        return 2
    for pkg in ("ur_moveit_config", "ur_description", "robot_state_publisher"):
        if subprocess.run(["ros2", "pkg", "prefix", pkg],
                          capture_output=True).returncode != 0:
            print(f"缺 {pkg} —— ros-humble-moveit / ros-humble-ur-* 没装齐，退 2")
            return 2

    stale = leftovers()
    if stale:
        print("板上有残留进程，先清场再测（否则 TF/服务归属会乱）：")
        for s in stale:
            print("  " + s)
        return 2

    print(f"仓库根 {REPO}（已 chdir —— detector_node 的模型路径是相对的，D67）")
    procs = []

    def spawn(name, cmd, log):
        lf = open(log, "wb")
        procs.append((name, subprocess.Popen(
            cmd, shell=True, stdout=lf, stderr=subprocess.STDOUT,
            start_new_session=True, cwd=REPO)))

    try:
        # 1) 真机部署配置，但没有驱动 ⇒ 没有 controller_manager
        spawn("launch",
              "ros2 launch yolo_bringup real.launch.py start_ur_driver:=false "
              "publish_camera_tf:=true tf_required:=false "
              "gripper_backend:=none carry_objects:=false "
              f"camera_x:={args.camera_x} camera_y:={args.camera_y} "
              f"camera_z:={args.camera_z} camera_qx:={args.camera_qx} "
              f"camera_qy:={args.camera_qy} camera_qz:={args.camera_qz} "
              f"camera_qw:={args.camera_qw}",
              args.log)

        # 2) robot_state_publisher（未标定 URDF；⚠️ 口径见 moveit_only_check.py）
        xr = sh("xacro $(ros2 pkg prefix ur_description)/share/ur_description"
                f"/urdf/ur.urdf.xacro ur_type:={args.ur_type} name:=ur "
                "safety_limits:=true > /tmp/board_chain_ur5.urdf 2>/tmp/board_chain_xacro.err")
        if xr.returncode != 0 or os.path.getsize("/tmp/board_chain_ur5.urdf") == 0:
            print("xacro 失败：", xr.stderr
                  or open("/tmp/board_chain_xacro.err", errors="replace").read()[:400])
            return 2
        spawn("rsp",
              'ros2 run robot_state_publisher robot_state_publisher --ros-args '
              '-p "robot_description:=$(cat /tmp/board_chain_ur5.urdf)"',
              "/tmp/board_chain_rsp.log")

        # 3) 假 /joint_states
        spawn("fake_js", f"python3 {os.path.join(REPO, 'scripts/fake_joint_states.py')}",
              "/tmp/board_chain_js.log")

        # 4) 等链路就绪：/targets_3d 话题在 + /compute_ik 有服务
        ready = False
        for i in range(24):
            time.sleep(5)
            tl = sh("ros2 topic list").stdout
            if "/targets_3d" in tl and "/compute_ik" in sh("ros2 service list").stdout:
                print(f"链路就绪（约 {(i + 1) * 5} 秒）")
                ready = True
                break
        if not ready:
            print("120 秒内没就绪 —— 日志：", args.log)
            return 2
        time.sleep(8)                       # 让频率统计稳定

        # 5) 逐级吞吐
        print("\n=== 逐级吞吐（每级 12 秒采样）===")
        results, failed = [], []
        for topic, gate, label in [
                ("/camera/color/image_raw", GATE_CAMERA, "相机彩色"),
                ("/detections", GATE_DETECTIONS, "YOLO 检测"),
                ("/targets_3d", GATE_TARGETS, "三维目标")]:
            if topic not in sh("ros2 topic list").stdout:
                print(f"  {label:10s} {topic:32s} 话题不存在  ✗")
                failed.append(f"{label}：话题不存在")
                results.append((label, topic, None, gate))
                continue
            fps, _ = topic_rate(topic)
            ok = fps is not None and fps >= gate
            mark = "✓" if ok else "✗"
            shown = f"{fps:.1f} FPS" if fps is not None else "无消息"
            print(f"  {label:10s} {topic:32s} {shown:>12s}  （判据 ≥ {gate:.0f}）  {mark}")
            results.append((label, topic, fps, gate))
            if not ok:
                failed.append(f"{label}：{shown} < {gate:.0f}")

        # 6) 资源
        print("\n=== 运行中资源（5 秒采样）===")
        res = sample_resources(5)
        if not res:
            print("  （tegrastats 无输出）")
        else:
            if "RAM" in res:
                used, total = res["RAM"]
                print(f"  内存     {int(used)/1024:.1f} / {int(total)/1024:.1f} GiB")
            if "CPU 各核" in res:
                print(f"  CPU 各核 {res['CPU 各核'][0]}")
            if "GR3D" in res:
                print(f"  GPU 负载 {res['GR3D'][0]}%")
            if "温度" in res:
                print(f"  结温     {res['温度'][0]} °C")
            if "整板功耗" in res:
                print(f"  整板功耗 {int(res['整板功耗'][0])/1000:.1f} W"
                      + (f"（其中 CPU+GPU {int(res['CPU+GPU 功耗'][0])/1000:.1f} W）"
                         if "CPU+GPU 功耗" in res else ""))

        # 7) 预期内的报错要点名，免得下次被当成故障
        print("\n=== 日志要点 ===")
        log = open(args.log, errors="replace").read()
        for pat in ("使用 GPU", "Loading YOLO"):
            for line in log.splitlines():
                if pat in line:
                    print("  " + line.strip()[:150])
                    break
        expected = [l.strip() for l in log.splitlines()
                    if "轨迹 action 服务不可用" in l or "初始 HOME失败" in l]
        if expected:
            print(f"  （预期内，无臂所致）{expected[0][:130]}")
        hard = [l.strip() for l in log.splitlines()
                if "[ERROR]" in l and "occupancy_map_monitor" not in l
                and "轨迹 action" not in l and "HOME失败" not in l
                and "TF 查询失败" not in l]
        for line in hard[:5]:
            print("  ⚠️ " + line[:150])

        print("\n" + "=" * 60)
        if failed:
            print("判据未全部满足：")
            for f in failed:
                print("  ✗ " + f)
            return 1
        print("链路端到端通过：相机 → YOLO → 三维目标，且吞吐有余量。")
        print("⚠️ 不覆盖运动：没有驱动就没有控制器，规划器状态机走不动（预期）。")
        return 0
    finally:
        for name, p in reversed(procs):
            try:
                os.killpg(p.pid, signal.SIGTERM)   # start_new_session ⇒ pgid==pid
            except ProcessLookupError:
                pass
        # ⚠️ 别用固定短宽限：`ros2 launch` 收到 TERM 后要按顺序收掉每个子节点，
        # 3 秒时它自己先被 SIGKILL 了、projector_node 就成了孤儿（实测踩过）。
        # 判据用 leftovers() 本身：清干净了才算完。
        deadline = time.time() + 15
        while time.time() < deadline and leftovers():
            time.sleep(1)
        if leftovers():
            for name, p in reversed(procs):
                if p.poll() is None:
                    try:
                        os.killpg(p.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
            for line in leftovers():
                try:
                    os.kill(int(line.split()[0]), signal.SIGKILL)
                except (ValueError, ProcessLookupError):
                    pass
            time.sleep(2)
        for name, p in reversed(procs):
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        stale = leftovers()
        if stale:
            print("\n⚠️ 收尾后仍有残留（下次开跑会被守卫拦下）：")
            for s in stale:
                print("  " + s)


if __name__ == "__main__":
    sys.exit(main())
