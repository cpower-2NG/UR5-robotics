#!/usr/bin/env python3
"""把 AG-95 驱动的「回调式异步」接口变成**同步、会报错**的调用。

为什么需要这一层（2026-10-26 真机实测）
--------------------------------------
``grasp_planner.gripper.Ag95Gripper.open()/close()`` 内部走 ``_async()``，而它的
第一句是::

    once = SimGripper._once(done_cb)
    if once is None:
        return

也就是说：**不给 ``done_cb`` 就什么都不做** —— 一个寄存器都不写、不抛异常、不打印。
``precision_exec.py`` 原先写的是 ``g.close()``，于是 2026-10-26 第一个真机循环里：

    {"event": "gripper", "where": "grasp.after", "action": "close",
     "position_mm": 95.0, ...}

闭合与张开全是**空操作**，而日志里「close → 回读 95.0 mm」读起来和成功一模一样。
机械臂照常下探、抬起、走到槽位、张开、撤退 —— 全程手里什么都没有，只有现场看
得出手指没动。这正是「状态/回读报的是它自己的事，不是你关心的行为」那一类陷阱：
回读 95.0 是真的（手指确实在 95.0），但它证明不了「闭合指令发出过」。

所以本模块做两件事：

1. ``call_sync(g, "open"|"close")``：给一个一次性回调、等它被调用，把驱动回的
   ``(status, result)`` 变成返回值或异常。**这是唯一允许的调用方式**。
2. ``check_moved(...)``：写完指令后比对回读 —— 手指几乎没动就判红。
   光「没报错」不算数，日志里的那行回读必须和动作对得上。

不 import ROS：状态码由调用方给（``GoalStatus.STATUS_SUCCEEDED`` 是 4）。
"""
import threading

#: ``action_msgs/msg/GoalStatus.STATUS_SUCCEEDED``。不 import ROS 就写死，
#: 调用方（precision_exec）有真实的 GoalStatus 时应当传进来。
STATUS_SUCCEEDED = 4

#: 指令下去以后手指至少要走这么多毫米，否则判「写了但没动」。
MOVED_TOL_MM = 5.0


class GripperActionError(RuntimeError):
    """夹爪动作没做成。**必须让整个循环停下来**，不能当作警告。"""


def call_sync(g, kind, timeout=20.0, status_ok=STATUS_SUCCEEDED):
    """调用 ``g.open(done_cb=...)`` / ``g.close(done_cb=...)`` 并等它真的做完。

    返回 ``(status, result)``。失败抛 :class:`GripperActionError`：

    - 回调在 ``timeout`` 秒内没来（线程没起来/卡在串口）；
    - ``status != status_ok`` 或 ``result.error_code != 0``（驱动把异常转成了
      ABORTED + error_code=-1，见 ``Ag95Gripper._async``）。

    ``timeout`` 必须大于驱动内部的 ``AG95_MOVE_TIMEOUT_S``（3 s）：手指被物体挡住
    时 ``_wait_until`` 会走满超时，那**不是**失败。
    """
    if kind not in ("open", "close"):
        raise GripperActionError("不认识的夹爪动作 %r" % (kind,))
    ev = threading.Event()
    box = {}

    def cb(status, result):
        box["status"] = status
        box["result"] = result
        ev.set()

    getattr(g, kind)(done_cb=cb)          # ← 不给 done_cb 就是空操作，见模块文档
    if not ev.wait(timeout):
        raise GripperActionError("AG-95 %s 等完成回调超时（%.0f s）" % (kind, timeout))
    status = box.get("status")
    result = box.get("result")
    code = getattr(result, "error_code", None)
    if status != status_ok or code != 0:
        raise GripperActionError(
            "AG-95 %s 失败：status=%s error_code=%s %s"
            % (kind, status, code, getattr(result, "error_string", "") or ""))
    return status, result


def check_moved(before, after, kind, target_mm, tol_mm=MOVED_TOL_MM):
    """指令写下去以后，回读**真的**变了吗？返回 ``(ok, 说明)``。

    三种合法情形：

    1. 本来就在目标附近（如 approach 前再张开一次，95.0 → 95.0）；
    2. 走到了目标（闭合到 0 mm，或张开到 95 mm）；
    3. 被物体挡住（闭合停在块宽 ~30 mm）—— 只要**朝目标方向**走了 ≥ ``tol_mm``。

    一种可疑情形：写了指令、回读纹丝不动（本次 bug 的指纹）。还有一种更糟的：
    朝反方向动（``open_is_high`` 猜反或链路错），那也不能放行。
    """
    if before is None or after is None:
        return False, "读不到位置（before=%s after=%s）—— 判不了动没动" % (before, after)
    if abs(before - target_mm) <= tol_mm and abs(after - target_mm) <= tol_mm:
        return True, "本来就在 %.1f mm（目标 %.1f mm），无需动" % (before, target_mm)
    toward = (before - after) if kind == "close" else (after - before)
    moved = abs(after - before)
    if moved < tol_mm:
        return False, ("写了 %s 指令但手指几乎没动（%.1f → %.1f mm，%.1f mm < 阈 %.1f）"
                       "—— 指令没生效" % (kind, before, after, moved, tol_mm))
    if toward <= 0:
        return False, ("手指朝**反方向**动了（%.1f → %.1f mm，%s 目标 %.1f）"
                       % (before, after, kind, target_mm))
    return True, "%.1f → %.1f mm（目标 %.1f，%s）" % (before, after, target_mm, kind)
