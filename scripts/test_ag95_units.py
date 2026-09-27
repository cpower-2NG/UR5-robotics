#!/usr/bin/env python3
"""
AG-95 单位换算与判空语义自检 —— 不需要硬件、不需要 ROS 图（D56）
================================================================
`Ag95Gripper` 是**构造即报错的桩**（现场核实寄存器映射前不允许写夹爪），
所以它的 bug 今天一个都不会发作，但**实现者会照着它的文档与调用方式踩**。
本脚本测的正是那层「文档即契约」：

1. 模块文档承诺「参数面/返回值一律毫米，只有寄存器面是千分位」。
   AG-95 的 0x0103/0x0202 满量程是 **1000 千分位**，而 `ag95_open_mm = 95.0`。
   把寄存器原值当毫米返回，`is_empty()` 就会**恒为 False**（规划器永远以为
   夹爪里夹着东西，空夹也往放置区搬）。这里把换算钉死。
2. 千分位的**方向**（1000 是全开还是全闭）手册没写，因此两个换算函数把
   `open_is_high` 做成必填关键字参数——本脚本测两个方向都自洽，而不是替现场
   猜一个。
3. `is_empty()` 的判据方向：**走到了闭合位 = 空夹**（手指之间没东西挡着）。
   旧实现只判「回读 ≈ 张开位」，于是「夹到空气一路闭合到 0 mm」被判成夹住了。

用法:
    python3 scripts/test_ag95_units.py
退出码 0 = 全通过。
"""

import importlib.util
import sys
import types
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
GRIPPER_PY = (PROJECT_ROOT / "ros2_ws" / "src" / "grasp_planner"
              / "grasp_planner" / "gripper.py")

_failed = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ''))
    if not cond:
        _failed.append(name)


def _raises(fn, exc=ValueError):
    try:
        fn()
    except exc:
        return True
    except Exception:
        return False
    return False


def _stub(name, **attrs):
    """造一个假的 ROS 包模块树，缺哪个补哪个。

    只补 `find_spec` 查不到的包，所以**在真 ROS 环境里跑时用的仍是真包**。
    """
    pkg = types.ModuleType(name)
    sys.modules[name] = pkg
    for sub, members in attrs.items():
        full = f'{name}.{sub}'
        mod = types.ModuleType(full)
        for key, val in members.items():
            setattr(mod, key, val)
        setattr(pkg, sub, mod)
        sys.modules[full] = mod


def load_gripper():
    """按文件路径加载 gripper.py，并注入假的 ROS 消息类型。

    gripper.py 的三个 ROS 依赖都只是**类型**，换成哑元即可让本脚本在**没有
    ROS 环境**的机器上跑（与 test_detector_device.py 注入假 torch 是同一个
    套路）。真正的 ROS 环境里也能跑——先试真包，失败再注入。

    ⚠️ `action_msgs` / `control_msgs` 是随 `NullGripper`（gripper_backend:=none）
    一起进来的。**别删这两个桩**：删了本脚本在没有 ROS 的机器上会直接
    `ModuleNotFoundError` 崩掉——而它测的是纯换算函数，本就不该需要 ROS。
    """
    if importlib.util.find_spec('sensor_msgs') is None:
        class JointState:  # 只为通过注解/类型检查，本脚本不实例化它
            pass

        _stub('sensor_msgs', msg={'JointState': JointState})

    if importlib.util.find_spec('action_msgs') is None:
        class GoalStatus:
            STATUS_SUCCEEDED = 4

        _stub('action_msgs', msg={'GoalStatus': GoalStatus})

    if importlib.util.find_spec('control_msgs') is None:
        class Result:
            """哑元：只要求有 `error_code`，且默认 0（与真消息一致）。"""
            error_code = 0

        class FollowJointTrajectory:
            pass

        FollowJointTrajectory.Result = Result
        _stub('control_msgs', action={'FollowJointTrajectory': FollowJointTrajectory})

    spec = importlib.util.spec_from_file_location('ag95_gripper_probe', GRIPPER_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    g = load_gripper()

    # 参数文件里的实际取值（grasp_planner.yaml）
    STROKE, OPEN, CLOSE = 95.0, 95.0, 0.0

    print("\n--- 参数自洽校验 ---")
    check("yaml 现值 (open=95, close=0, stroke=95) 通过",
          not _raises(lambda: g.validate_ag95_geometry(OPEN, CLOSE, STROKE)))
    check("stroke <= 0 拒绝",
          _raises(lambda: g.validate_ag95_geometry(OPEN, CLOSE, 0.0)))
    check("close < 0 拒绝",
          _raises(lambda: g.validate_ag95_geometry(OPEN, -1.0, STROKE)))
    check("open > stroke 拒绝",
          _raises(lambda: g.validate_ag95_geometry(120.0, CLOSE, STROKE)))
    check("close > open 拒绝（开口越大越开，闭合不可能更宽）",
          _raises(lambda: g.validate_ag95_geometry(10.0, 20.0, STROKE)))

    print("\n--- 毫米 → 千分位 ---")
    hi = dict(open_is_high=True)
    lo = dict(open_is_high=False)
    check("全开 95mm → 1000 千分位 (open_is_high)", g.mm_to_permille(95.0, STROKE, **hi) == 1000)
    check("全闭  0mm →    0 千分位 (open_is_high)", g.mm_to_permille(0.0, STROKE, **hi) == 0)
    check("半程 47.5mm → 500 千分位", g.mm_to_permille(47.5, STROKE, **hi) == 500)
    check("方向反过来：全开 95mm → 0 千分位", g.mm_to_permille(95.0, STROKE, **lo) == 0)
    check("方向反过来：全闭  0mm → 1000 千分位", g.mm_to_permille(0.0, STROKE, **lo) == 1000)
    check("越界**报错**而不是静默饱和（100mm > 95mm）",
          _raises(lambda: g.mm_to_permille(100.0, STROKE, **hi)))
    check("负值报错", _raises(lambda: g.mm_to_permille(-1.0, STROKE, **hi)))

    print("\n--- 千分位 → 毫米 ---")
    check("1000 千分位 → 95mm (open_is_high)", abs(g.permille_to_mm(1000, STROKE, **hi) - 95.0) < 1e-9)
    check("0 千分位 → 0mm (open_is_high)", abs(g.permille_to_mm(0, STROKE, **hi)) < 1e-9)
    check("方向反过来：1000 千分位 → 0mm", abs(g.permille_to_mm(1000, STROKE, **lo)) < 1e-9)
    check("回读越界**夹到边界**（回读噪声不该让规划器崩）",
          g.permille_to_mm(1200, STROKE, **hi) == g.permille_to_mm(1000, STROKE, **hi)
          and g.permille_to_mm(-5, STROKE, **hi) == g.permille_to_mm(0, STROKE, **hi))

    print("\n--- 往返一致性 ---")
    ok = all(abs(g.permille_to_mm(g.mm_to_permille(mm, STROKE, **hi), STROKE, **hi) - mm) <= STROKE / 1000
             for mm in (0.0, 5.0, 23.7, 47.5, 90.0, 95.0))
    check("mm → 千分位 → mm 往返（±1 千分位）", ok)

    print("\n--- D56 回归：把寄存器原值当毫米会怎样 ---")
    raw = g.mm_to_permille(95.0, STROKE, **hi)          # 手指全开时的回读值 = 1000
    correct_mm = g.permille_to_mm(raw, STROKE, **hi)     # 正确换算 = 95.0 mm
    broken_is_empty = abs(raw - OPEN) <= g.AG95_EMPTY_TOLERANCE_MM      # 旧写法：raw 当 mm
    correct_is_empty = abs(correct_mm - OPEN) <= g.AG95_EMPTY_TOLERANCE_MM
    check("旧写法（raw 当 mm）判不出空夹 → 正是 D56 的症状",
          broken_is_empty is False and correct_is_empty is True,
          f"raw={raw} correct_mm={correct_mm}")
    check("量级差 ≥ 10 倍（1000 vs 95），不是小偏差",
          raw / max(correct_mm, 1e-9) > 10)

    print("\n--- is_empty() 判据方向 ---")

    class FakeAg95(g.Ag95Gripper):
        """绕开 __init__（它必然抛错），只装上 is_empty() 需要的属性。"""

        def __init__(self, readback):
            self._open_mm = OPEN
            self._close_mm = CLOSE
            self._stroke_mm = STROKE
            self._readback = readback

        def _read_position(self):
            return self._readback

    check("回读 0mm（夹到空气、一路闭合）→ 空夹",
          FakeAg95(0.0).is_empty() is True)
    check("回读 95mm（命令根本没生效、仍全开）→ 空夹",
          FakeAg95(95.0).is_empty() is True)
    check("回读 40mm（被物体挡住停在中途）→ 夹住了",
          FakeAg95(40.0).is_empty() is False)
    check("读不到（None）→ 保守判为「夹住了」，不半空松手",
          FakeAg95(None).is_empty() is False)

    print("\n--- 夹爪回调签名（D52 契约）---")
    seen = []
    cb = g.SimGripper._once(lambda status, result: seen.append((status, result)))
    cb('S', 'R')
    cb('S2', 'R2')          # 第二次必须被吞掉
    check("_once 透传 (status, result) 且只调一次", seen == [('S', 'R')], f"{seen}")
    check("done_cb 为 None 时不炸", g.SimGripper._once(None) is None)

    print("\n" + "=" * 52)
    if _failed:
        print(f"FAILED: {len(_failed)} 项 -> {_failed}")
        return 1
    print("全部通过。")
    return 0


if __name__ == '__main__':
    sys.exit(main())
