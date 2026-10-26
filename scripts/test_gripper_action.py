#!/usr/bin/env python3
"""``gripper_action`` 的纯逻辑测试 —— 不需要 ROS、不需要串口、不动臂。

    python3 scripts/test_gripper_action.py

为什么这个测试值得存在
----------------------
2026-10-26 第一个真机循环：夹爪四段动作全是空操作，臂完整走了一趟，日志里
「close → 回读 95.0 mm」看起来像成功。根因在驱动接口：``Ag95Gripper._async()``
在**没有 done_cb 时直接 return**，一个寄存器都不写。

所以这里的 ``FakeGripper`` **照着驱动的语义复刻**那条捷径（``test_fake_matches_
driver_semantics`` 就是拿不带回调的裸调用去证明它确实什么都不做）。这样一来：

- 若 ``call_sync`` 哪天退化成裸调用，第一个测试立刻变红；
- 若 ``check_moved`` 放宽到「没动也算过」，第二个、第三个测试立刻变红。
"""
import os
import sys
import types

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import gripper_action as ga                                       # noqa: E402

FAIL = []
N = [0]


def check(name, cond, detail=""):
    N[0] += 1
    if cond:
        print("PASS %s" % name)
    else:
        print("FAIL %s %s" % (name, detail))
        FAIL.append(name)


class FakeGripper:
    """复刻 ``Ag95Gripper`` 的对外契约，**包括那条静默捷径**。

    不给 ``done_cb`` ⇒ 什么都不写（和线上驱动一样）。给了就同步跑完再回调
    （线上是起线程；同步足够表达契约，测试也不必等）。
    """

    def __init__(self, start=95.0, blocked_at=None, fail=False, never_cb=False):
        self.pos = start
        self.written = []
        self.blocked_at = blocked_at
        self.fail = fail
        self.never_cb = never_cb

    @staticmethod
    def _once(cb):
        if cb is None:
            return None
        return cb

    def _async(self, work, done_cb, what):
        once = self._once(done_cb)
        if once is None:
            return                                   # ← 驱动的真实行为
        if self.never_cb:
            return                                   # 线程没起来 / 卡在串口
        result = types.SimpleNamespace(error_code=0, error_string="")
        if self.fail:
            work = lambda: (_ for _ in ()).throw(RuntimeError("串口断了"))
        try:
            work()
        except Exception as exc:                     # noqa: BLE001
            result = types.SimpleNamespace(error_code=-1, error_string=str(exc))
            once(6, result)                          # STATUS_ABORTED
            return
        once(ga.STATUS_SUCCEEDED, result)

    def _set(self, mm):
        self.written.append(mm)
        self.pos = self.blocked_at if self.blocked_at is not None else mm

    def open(self, done_cb=None):
        self._async(lambda: self._set(95.0), done_cb, "open")

    def close(self, done_cb=None):
        self._async(lambda: self._set(0.0), done_cb, "close")

    def current_position(self):
        return self.pos


# ---------------------------------------------------------------- 1 核心回归
def test_bare_call_is_a_no_op():
    """先证明测试替身复刻了驱动的那条捷径 —— 否则下面的测试是假绿。"""
    g = FakeGripper()
    g.close()                                        # 裸调用，正是 2026-10-26 的写法
    check("裸 close() 一个寄存器都不写", g.written == [], g.written)
    check("裸 close() 后回读仍是 95.0", g.pos == 95.0, g.pos)


def test_call_sync_actually_writes():
    g = FakeGripper()
    ga.call_sync(g, "close")
    check("call_sync(close) 写下了目标 0.0", g.written == [0.0], g.written)
    check("call_sync(close) 后回读 0.0", g.pos == 0.0, g.pos)
    ga.call_sync(g, "open")
    check("call_sync(open) 写下了目标 95.0", g.written == [0.0, 95.0], g.written)


def test_failures_raise():
    for name, g in (("驱动报错", FakeGripper(fail=True)),
                    ("回调不来", FakeGripper(never_cb=True))):
        try:
            ga.call_sync(g, "close", timeout=0.3)
            check("%s 抛 GripperActionError" % name, False, "没抛")
        except ga.GripperActionError:
            check("%s 抛 GripperActionError" % name, True)
    try:
        ga.call_sync(FakeGripper(), "twist")
        check("不认识的动作抛错", False)
    except ga.GripperActionError:
        check("不认识的动作抛错", True)


# ---------------------------------------------------------------- 2 效果判据
def test_check_moved():
    cases = [
        # (before, after, kind, target, 期望 ok, 说明)
        (95.0, 95.0, "close", 0.0, False, "本 bug 的指纹：写了没动"),
        (95.0, 0.0, "close", 0.0, True, "空夹全闭"),
        (95.0, 31.0, "close", 0.0, True, "夹住 31 mm 的块"),
        (95.0, 95.5, "close", 0.0, False, "只动了 0.5 mm"),
        (95.0, 95.0, "open", 95.0, True, "本来就在全开"),
        (30.0, 95.0, "open", 95.0, True, "放置时张开"),
        (30.0, 30.0, "open", 95.0, False, "该张开却没动"),
        (0.0, 95.0, "close", 0.0, False, "朝反方向动（open_is_high 猜反）"),
        (None, 95.0, "open", 95.0, False, "读不到位置"),
    ]
    for before, after, kind, target, want, why in cases:
        ok, note = ga.check_moved(before, after, kind, target)
        check("check_moved %-6s %.0f→%.0f %s" % (kind, before or 0, after or 0, why),
              ok is want, "得到 %s（%s）" % (ok, note))


def main():
    test_bare_call_is_a_no_op()
    test_call_sync_actually_writes()
    test_failures_raise()
    test_check_moved()
    print("\n%d PASS, %d FAIL" % (N[0] - len(FAIL), len(FAIL)))
    return 0 if not FAIL else 1


if __name__ == "__main__":
    sys.exit(main())
