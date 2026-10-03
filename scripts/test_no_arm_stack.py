#!/usr/bin/env python3
"""`no_arm_stack.py` 的自检（**不需要 ROS、不需要板子、不起任何进程**）。

这里只钉**三条契约**，都是「错了会静默地把人带到错误结论上」的那种：

1. **入口必须是 `start_ur_driver:=false`** —— 那是这套东西「物理上不可能动」
   的全部依据。改成 `true` 就等于起了一个能指挥真机的控制器。
2. **必须起 `robot_state_publisher`** —— 少了它，`/compute_ik` 照常被发现、
   照常被调用、**每次都回 `FRAME_TRANSFORM_FAILURE`**，而症状看起来像几何问题
   （包线会扫出「一个位姿都没解出来」）。这不是假想：这个脚本就是为它而写的。
3. **描述包必须从 `real.launch.py` 读**，不许抄一个值下来 —— 抄下来的那份迟早
   和真身分叉，后果是「同一张图两套几何」（第八十二→八十三轮的教训）。

真正的起停不在这里测 —— 那要板子（本脚本在板上实跑过一次，见 RESTRUCTURE_PLAN）。

跑法：
    python3 scripts/test_no_arm_stack.py
退出码 0 = 全通过。
"""

import os
import subprocess
import sys

# 见 test_table_envelope.py 同一行的注释：等长改动 + 同一秒 ⇒ (mtime, size) 未变，
# CPython 会**继续用旧字节码**，变异被静默吞掉、自检报绿。
sys.dont_write_bytecode = True

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import moveit_only_check as moc                                   # noqa: E402
import no_arm_stack as nas                                        # noqa: E402

N_PASS = N_FAIL = N_SKIP = 0
FAILS = []


def skip(name, why):
    """环境不满足 ⇒ **跳过**，既不算过也不算败。

    这里要的是「没 ROS 时不去测一条只有 ROS 才能答的题」，而不是把它判失败 ——
    判失败会让人以为代码坏了，判通过则是假绿。
    """
    global N_SKIP
    N_SKIP += 1
    print('  SKIP  %s（%s）' % (name, why))


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, ('\n        ' + detail) if detail else ''))


# ----------------------------------------------------------------------
# A. import 本模块**不许**把 rclpy 拉进来
# ----------------------------------------------------------------------
# rclpy 只在 wait_for_ik 里按需 import。拉到顶层就意味着「没 source ROS 时
# 连 --help 都跑不了」，而且本自检也得跟着要 ROS。
print("\n--- A. import 不拉 rclpy ---")
code = ("import sys; sys.path.insert(0, %r); import no_arm_stack; "
        "print('LOADED' if 'rclpy' in sys.modules else 'CLEAN')"
        % os.path.dirname(os.path.abspath(__file__)))
r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
check("子进程里 import 之后 rclpy **不在** sys.modules",
      r.stdout.strip().endswith("CLEAN"),
      f"stdout={r.stdout.strip()!r} stderr={r.stderr.strip()[-200:]!r}")


# ----------------------------------------------------------------------
# B. ★ 入口必须关掉驱动 —— 这是「不可能动」的全部依据
# ----------------------------------------------------------------------
print("\n--- B. 入口：驱动必须是关的 ---")
cmd = nas.launch_command("real.launch.py")
check("★ 命令里必须有 start_ur_driver:=false", "start_ur_driver:=false" in cmd,
      f"got {cmd!r}")
check("★ 不许出现 start_ur_driver:=true（改了就等于起了能指挥真机的控制器）",
      "start_ur_driver:=true" not in cmd, f"got {cmd!r}")
check("入口名可换（--entry 走的就是这个参数）",
      "bringup.launch.py" in nas.launch_command("bringup.launch.py"))
check("包名固定是 yolo_bringup", "yolo_bringup" in cmd, f"got {cmd!r}")


# ----------------------------------------------------------------------
# C. ★ 必须起 robot_state_publisher —— 缺了它就是 FRAME_TRANSFORM_FAILURE
# ----------------------------------------------------------------------
print("\n--- C. 必须起 RSP（缺了症状像几何问题）---")
rcmd = nas.rsp_command("/tmp/x.urdf")
check("★ 命令起的是 robot_state_publisher 这个可执行文件",
      "robot_state_publisher robot_state_publisher" in rcmd, f"got {rcmd!r}")
check("robot_description 从 URDF 文件读（不是内联别的什么）",
      "/tmp/x.urdf" in rcmd and "robot_description" in rcmd, f"got {rcmd!r}")
check("路径是可配的（不会被写死成某一个文件）",
      "/other/path.urdf" in nas.rsp_command("/other/path.urdf"))
check("★ 源码里真的**调用了**它（不是定义了不用）",
      "rsp_command(URDF_PATH)" in open(
          os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "no_arm_stack.py"), encoding="utf-8").read())


# ----------------------------------------------------------------------
# D. 描述包：读，不抄
# ----------------------------------------------------------------------
print("\n--- D. 描述包必须从 real.launch.py 读 ---")
check("★ 用的就是 moveit_only_check 那个实现（不是自己又抄一份正则）",
      nas.description_package is moc.description_package)
# ⚠️ 这条**只有板子上才答得了**：它走 `ros2 pkg prefix yolo_bringup` 去找
# launch 文件，而那个包是在板上 colcon build 装出来的（本机 WSL 没有）。
# 解不出来就**跳过** —— 把「这台机器上没装这个包」判成失败，会让人以为
# 代码坏了；判成通过就是假绿。
dp = nas.description_package()
if dp is None:
    skip("描述包读得出内容",
         "ros2 pkg prefix yolo_bringup 解析不出来（本机没构建；板上跑才有）")
    skip("读到的名字里带 calib", "同上")
else:
    check("它确实返回一个非空的名字（真源在 launch 文件里）",
          isinstance(dp, str) and len(dp) > 0, f"got {dp!r}")
    check("★ 读到的名字不是标称 ur_description（标定包才带得出 calib）",
          dp != "ur_description", f"got {dp!r}")
check("源码里没有硬编码的描述包名（'yolo_ur_description' 不该出现）",
      "yolo_ur_description" not in open(
          os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "no_arm_stack.py"), encoding="utf-8").read())


# ----------------------------------------------------------------------
# E. 清场路径存在（起得来还要收得掉）
# ----------------------------------------------------------------------
print("\n--- E. 收得掉 ---")
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "no_arm_stack.py"), encoding="utf-8").read()
check("有 --stop", '"--stop"' in src)
check("有 --check（只判可用性，不起东西）", '"--check"' in src)
check("★ 按**进程组**杀（`ros2 launch` 的子节点不按组杀会漏）",
      "killpg" in src and "getpgid" in src)
check("SIGTERM 之后还有 SIGKILL 兜底", "SIGTERM" in src and "SIGKILL" in src)
check("收到信号也走同一条清场路径",
      "signal.SIGTERM" in src and "stop_all()" in src)


print("\n通过 %d，失败 %d，跳过 %d" % (N_PASS, N_FAIL, N_SKIP))
if FAILS:
    print("失败项：")
    for f in FAILS:
        print("  - %s" % f)
sys.exit(1 if N_FAIL else 0)
