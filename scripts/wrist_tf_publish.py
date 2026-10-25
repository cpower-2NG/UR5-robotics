#!/usr/bin/env python3
"""把腕部手眼标定发成静态 TF：`tool0 → camera_link`。

**这条 TF 是腕部相机能不能用的开关。** 没有它，相机那棵树和机械臂那棵树是
**两棵断开的树**（`tf2_echo` 报 "two or more unconnected trees"），相机看到的
东西算出来的坐标落不进基座系，下游一步都走不了。

⚠️ 为什么是 `camera_link` 而不是 `camera_color_optical_frame`：RealSense 驱动
自己会发 `camera_link → camera_color_optical_frame`。直接发到光学帧会让它
**有两个父节点**，而且绕过驱动内部那一段（两块板的实测间距约 15 mm ——
`docs/当前状态.md` §3 记过它在俯视相机上造成的 17.46 mm 落地误差）。
所以标定文件存的是 `tool0 → camera_link`，见
[`docs/calib/wrist_eih_camlink-2026-10-24.json`](../docs/calib/wrist_eih_camlink-2026-10-24.json)。

三种用法（**都不碰机械臂、不写任何寄存器**）：

    # ⓪ 只看文件：帧名对不对、数是不是有限数、四元数模长是不是 1
    python3 scripts/wrist_tf_publish.py --check

    # ① 发 TF（前台常驻；静态 TF 的发布者一退，后来的订阅者就拿不到了）
    python3 scripts/wrist_tf_publish.py

    # ② 验收：直查 TF，不经过任何标定服务
    #     a. 查回 `tool0 → camera_link`，与存盘逐项比对（t / q，q 允许整体取负）
    #     b. 查 `base_link → camera_color_optical_frame` 是否可达
    python3 scripts/wrist_tf_publish.py --verify

退出码：0 = 通过；1 = `--verify` 对不上/查不到；2 = 标定文件或环境有问题
（**不用默认值硬跑** —— 发一条错的 TF 不会报错，只会让所有坐标静默偏掉）。

⚠️ 不要再起第二个发同一对帧的东西（例如 `real.launch.py` 的
`publish_camera_tf:=true`，或 easy_handeye2 的 `handeye_publisher`）：
给 `camera_link` 两个父节点，TF 树会**时对时错**。
"""

import argparse
import json
import math
import os
import sys
import time

# ⚠️ 必须在 import 被测模块之前设（本仓约定：.pyc 的失效判据是 mtime+size）
sys.dont_write_bytecode = True

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CALIB = os.path.join(
    REPO, "docs", "calib", "wrist_eih_camlink-2026-10-24.json")

#: 标定文件里**必须**是这一对帧。写死是判据的一部分：拿错文件（例如那份
#: 发到光学帧的旧标定）必须当场退 2，而不是把两条链悄悄接反。
WANT_PARENT = "tool0"
WANT_CHILD = "camera_link"

#: 四元数模长的容差。超过它说明这份数不是单位四元数 —— 归一化会**静默**
#: 改掉旋转，所以拒绝，让人回去查来源。
QUAT_NORM_TOL = 1e-3

OK = "✅"
BAD = "❌"
WARN = "⚠️ "


class CalibError(ValueError):
    """标定文件不能用。**消息就是给人看的**（含文件路径与字段名）。"""


# ----------------------------------------------------------------------
# 纯函数：解析与比对。**不碰 ROS**，所以自检够得着。
# ----------------------------------------------------------------------
def _number(doc, key, where):
    """取一个有限实数；缺项/非数/NaN/inf 一律抛 ``CalibError``。"""
    if not isinstance(doc, dict):
        raise CalibError("%s 必须是对象（有 x/y/z 或 w），现在是 %r"
                         % (where, type(doc).__name__))
    if key not in doc:
        raise CalibError("%s 缺字段 %r" % (where, key))
    v = doc[key]
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise CalibError("%s.%s 必须是数，现在是 %r" % (where, key, v))
    v = float(v)
    if not math.isfinite(v):
        raise CalibError("%s.%s 不是有限数：%r" % (where, key, v))
    return v


def load_calib(path):
    """读 JSON。文件不在/不是合法 JSON ⇒ ``CalibError``（**不退回默认值**）。"""
    if not path:
        raise CalibError("没给标定文件路径")
    if not os.path.exists(path):
        raise CalibError("标定文件不存在：%s" % path)
    try:
        with open(path, "r", encoding="utf-8") as fh:
            doc = json.load(fh)
    except OSError as exc:
        raise CalibError("读不了标定文件 %s：%s" % (path, exc))
    except ValueError as exc:
        raise CalibError("%s 不是合法 JSON：%s" % (path, exc))
    if not isinstance(doc, dict):
        raise CalibError("%s 顶层必须是对象" % path)
    return doc


def parse_calib(doc, want_parent=WANT_PARENT, want_child=WANT_CHILD):
    """把一份标定文档变成 ``{parent, child, t:[x,y,z], q:[x,y,z,w]}``。

    三条硬判据（**都是「错了不会报错」的那种**，所以必须在这里拦）：
      1. 帧名必须**恰好**是期望的那一对 —— 拿错文件（旧的光学帧版本）会
         给 `camera_link` 两个父节点；
      2. 7 个数必须都是有限数 —— NaN 会被 TF 静默发出去；
      3. 四元数模长必须是 1（容差 ``QUAT_NORM_TOL``）—— 模长 0 不是旋转，
         模长 ≠ 1 归一化会**改掉**标定值，都不许悄悄修。
    """
    if not isinstance(doc, dict):
        raise CalibError("标定文档顶层必须是对象")

    parent = doc.get("parent_frame")
    child = doc.get("child_frame")
    if parent != want_parent:
        raise CalibError(
            "parent_frame 是 %r，期望 %r —— 拿错标定文件了？"
            "（发到 camera_color_optical_frame 的那份会让光学帧有两个父节点，"
            "并且丢掉驱动内部那一段平移）" % (parent, want_parent))
    if child != want_child:
        raise CalibError("child_frame 是 %r，期望 %r —— 拿错标定文件了？"
                         % (child, want_child))

    t_doc = doc.get("translation")
    q_doc = doc.get("rotation")
    t = [_number(t_doc, k, "translation") for k in ("x", "y", "z")]
    q = [_number(q_doc, k, "rotation") for k in ("x", "y", "z", "w")]

    norm = math.sqrt(sum(v * v for v in q))
    if abs(norm - 1.0) > QUAT_NORM_TOL:
        raise CalibError(
            "rotation 四元数的模长是 %.6f，不是 1（容差 %.1e）。"
            "归一化会静默改掉标定值，所以这里拒绝；回去查这份文件怎么来的。"
            % (norm, QUAT_NORM_TOL))

    return {"parent": parent, "child": child, "t": t, "q": q}


def describe(cal):
    """一行摘要（打印给人看；不参与判据）。"""
    t, q = cal["t"], cal["q"]
    return ("%s → %s  t=(%+.6f, %+.6f, %+.6f) m  "
            "q=(%+.6f, %+.6f, %+.6f, %+.6f)"
            % (cal["parent"], cal["child"], t[0], t[1], t[2],
               q[0], q[1], q[2], q[3]))


def same_transform(got_t, got_q, want_t, want_q,
                   t_tol_mm=0.05, q_tol=1e-4):
    """两套 t/q 是不是同一个变换。→ ``(ok, 说明)``。**纯函数**。

    q 与 −q 表示同一个旋转，所以逐项比时两种符号都试。返回的说明里**同时**
    写两边的数值 —— 验收报告要能对着存盘文件逐位核。
    """
    dt = [abs(float(a) - float(b)) for a, b in zip(got_t, want_t)]
    dt_mm = max(dt) * 1000.0
    dq_same = max(abs(float(a) - float(b)) for a, b in zip(got_q, want_q))
    dq_flip = max(abs(float(a) + float(b)) for a, b in zip(got_q, want_q))
    dq = min(dq_same, dq_flip)
    detail = ("t 最大差 %.3f mm（容差 %.3f mm），q 最大差 %.1e（容差 %.1e）"
              % (dt_mm, t_tol_mm, dq, q_tol))
    if dq_flip < dq_same:
        detail += "；q 整体取负后一致（同一个旋转）"
    if dt_mm > t_tol_mm or dq > q_tol:
        return False, detail
    return True, detail


# ----------------------------------------------------------------------
# 发 TF / 查 TF（要 ROS）
# ----------------------------------------------------------------------
def _import_ros():
    try:
        import rclpy                                     # noqa: F401
        from tf2_ros import Buffer, TransformListener    # noqa: F401
        from geometry_msgs.msg import TransformStamped   # noqa: F401
    except ImportError as exc:                           # noqa: BLE001
        raise CalibError(
            "本机没有 ROS 2 Python 环境（%s）。\n"
            "  ⇒ 先 `source /opt/ros/humble/setup.bash`（板上还要 source "
            "ros2_ws/install/setup.bash）。\n"
            "  只验标定文件本身可以用 `--check`，它不需要 ROS。" % exc)


def publish(cal):
    """发静态 TF 并常驻。返回退出码（正常只会因 KeyboardInterrupt 回来）。"""
    _import_ros()
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    rclpy.init()
    node = rclpy.create_node("wrist_tf_publish")
    try:
        ts = TransformStamped()
        ts.header.stamp = node.get_clock().now().to_msg()
        ts.header.frame_id = cal["parent"]
        ts.child_frame_id = cal["child"]
        ts.transform.translation.x = cal["t"][0]
        ts.transform.translation.y = cal["t"][1]
        ts.transform.translation.z = cal["t"][2]
        ts.transform.rotation.x = cal["q"][0]
        ts.transform.rotation.y = cal["q"][1]
        ts.transform.rotation.z = cal["q"][2]
        ts.transform.rotation.w = cal["q"][3]

        # ⚠️ 必须留一个引用：广播器被 GC 掉时发布者也会没，transient-local 的
        # 那一份样本随发布者一起消失 —— 后起来的订阅者就再也拿不到这条 TF。
        broadcaster = StaticTransformBroadcaster(node)
        broadcaster.sendTransform(ts)
        node.get_logger().info("已发静态 TF：%s" % describe(cal))
        node.get_logger().info(
            "⚠️ 别再起第二个发同一对帧的东西（publish_camera_tf:=true / "
            "easy_handeye2 handeye_publisher）—— camera_link 会有两个父节点。")
        print("%s 已发静态 TF（Ctrl-C 结束）\n    %s" % (OK, describe(cal)))
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def verify(cal, base_frame, optical_frame, timeout):
    """直查 TF，不经过标定服务。→ 退出码（0 通过 / 1 对不上 / 2 环境）。"""
    try:
        _import_ros()
        import rclpy
        from rclpy.duration import Duration
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener
    except CalibError as exc:
        print(BAD, exc)
        return 2

    rclpy.init()
    node = rclpy.create_node("wrist_tf_verify")
    buf = Buffer(cache_time=Duration(seconds=10), node=node)
    TransformListener(buf, node, spin_thread=False)

    def look(target, source, wait_s):
        deadline = time.time() + wait_s
        last = None
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                return buf.lookup_transform(target, source, Time(),
                                            Duration(seconds=0.2)), None
            except Exception as exc:                    # noqa: BLE001
                last = exc
        return None, last

    rc = 1
    try:
        print("=" * 72)
        print("  腕部 TF 验收（只读：只查 TF）")
        print("=" * 72)
        print("  存盘：%s" % describe(cal))

        tf, err = look(cal["parent"], cal["child"], timeout)
        if tf is None:
            print("%s `%s → %s` **查不到**" % (BAD, cal["parent"], cal["child"]))
            print("    底层异常：%s: %s" % (type(err).__name__, err))
            print("    ⇒ 发 TF 的那个进程起了吗？`scripts/wrist_tf_publish.py`")
        else:
            tr = tf.transform.translation
            ro = tf.transform.rotation
            got_t = [tr.x, tr.y, tr.z]
            got_q = [ro.x, ro.y, ro.z, ro.w]
            ok, detail = same_transform(got_t, got_q, cal["t"], cal["q"])
            mark = OK if ok else BAD
            print("%s `%s → %s` 查到了：%s" % (mark, cal["parent"], cal["child"],
                                              detail))
            print("    查回 t=(%+.6f, %+.6f, %+.6f)" % tuple(got_t))
            print("    查回 q=(%+.6f, %+.6f, %+.6f, %+.6f)" % tuple(got_q))
            if not ok:
                print("    ⇒ 发出去的不是这份标定（或者有第二个发布者在抢）。")

            tf2, err2 = look(base_frame, optical_frame, timeout)
            if tf2 is None:
                print("%s `%s → %s` **查不到**" % (BAD, base_frame, optical_frame))
                print("    底层异常：%s: %s" % (type(err2).__name__, err2))
                print("    ⇒ 臂的 TF（UR 驱动）或相机的 TF（RealSense 节点）没起；"
                      "两条都在，这条链才通。")
            else:
                tr2 = tf2.transform.translation
                print("%s `%s → %s` 可达：t=(%+.4f, %+.4f, %+.4f) m"
                      % (OK, base_frame, optical_frame,
                         tr2.x, tr2.y, tr2.z))
                rc = 0 if ok else 1
        print("=" * 72)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


# ----------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="发/验腕部手眼静态 TF（`tool0 → camera_link`）。只读，不碰机械臂。",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", default=DEFAULT_CALIB,
                    help="标定 JSON 路径（默认仓内 2026-10-24 那份）")
    ap.add_argument("--parent", default=WANT_PARENT,
                    help="期望的父帧（文件里必须一致；默认 %(default)s）")
    ap.add_argument("--child", default=WANT_CHILD,
                    help="期望的子帧（文件里必须一致；默认 %(default)s）")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true",
                      help="只验标定文件，不碰 ROS")
    mode.add_argument("--verify", action="store_true",
                      help="直查 TF：比对存盘 t/q，并确认光学帧可达")
    ap.add_argument("--base-frame", default="base_link",
                    help="--verify 的起点（默认 %(default)s）")
    ap.add_argument("--optical-frame", default="camera_color_optical_frame",
                    help="--verify 的终点（默认 %(default)s）")
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="--verify 等 TF 的秒数（默认 %(default)s）")
    args = ap.parse_args(argv)

    try:
        cal = parse_calib(load_calib(os.path.expanduser(args.calib)),
                          want_parent=args.parent, want_child=args.child)
    except CalibError as exc:
        print("%s 标定文件不能用：%s" % (BAD, exc))
        print("   ⇒ 拒绝用默认值往下跑：发一条错的 TF **不会报错**，"
              "只会让所有坐标静默偏掉。")
        return 2

    if args.check:
        print("%s 标定文件可用：%s" % (OK, args.calib))
        print("    %s" % describe(cal))
        return 0
    if args.verify:
        return verify(cal, args.base_frame, args.optical_frame, args.timeout)
    return publish(cal)


if __name__ == "__main__":
    sys.exit(main())
