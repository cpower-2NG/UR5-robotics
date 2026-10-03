#!/usr/bin/env python3
"""`real_kinematics_fix.py` 的纯函数自检（**不碰 /opt、不碰 ROS**）。

要钉住的是三件事：

1. **改写是行级的** —— 注释、别的组、别的键都不许动。往系统文件里写东西，
   「顺手」改到别处是最贵的错。
2. **幂等** —— 跑第二遍必须 `changed=False` 且一个字节都不变（不然 `--apply`
   就成了每次都在改）。
3. **认不出结构就抛，不许猜** —— 找不到那个组、找不到锚点时 `LookupError`。
   往大概的位置插两行，正是本仓库反复在修的那一类。

另外用 `yaml.safe_load` 把改完的文本**再解析一次**：文本看着对、YAML 层次
错位（缩进多一格）是这里最容易犯又最难看出来的错。

跑法：
    python3 scripts/test_real_kinematics_fix.py
退出码 0 = 全通过。
"""

import collections
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import yaml                                                          # noqa: E402
from real_kinematics_fix import (                                    # noqa: E402
    WANT_LIMITS, patch_text, read_values)

N_PASS = N_FAIL = 0
FAILS = []


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, ('\n        ' + detail) if detail else ''))


def safe(fn, *a, **kw):
    """抛异常时返回一句「崩了」而不是中断整个套件（见 test_ik_repeat.py 的说明）。"""
    try:
        return fn(*a, **kw)
    except Exception as e:                       # noqa: BLE001
        return f"!! 崩了：{type(e).__name__}: {e}"


def patched(text, group="ur_manipulator"):
    """``patch_text`` 的安全版，**返回的一定是二元组**。

    ⚠️ 不能写成 `NEW, CHANGED = safe(patch_text, REAL)`：崩了的时候 `safe` 返回
    的是**字符串**，解包会抛 `ValueError: too many values to unpack`，整套检查
    当场中断 —— 于是「崩了」在只数 FAIL 的地方被读成「通过」。这个坑本项目栽过
    好几次（见 test_ik_repeat.py 的 `safe`），所以这里让它**一定可解包**，
    崩了就把 `CHANGED` 变成一句非 `True` 的字符串，由各条检查各自变红。
    """
    try:
        return patch_text(text, group)
    except Exception as e:                       # noqa: BLE001
        return text, f"!! 崩了：{type(e).__name__}: {e}"


# 板子上那份的**逐字**内容（第七十八轮 cat 出来的）
REAL = """\
/**:
  ros__parameters:
    robot_description_kinematics:
      ur_manipulator:
        kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
        kinematics_solver_search_resolution: 0.005
        kinematics_solver_timeout: 0.005
        kinematics_solver_attempts: 3
"""


def _dd():
    return collections.defaultdict(_dd)


def _deep(v):
    if isinstance(v, dict):
        return collections.defaultdict(_dd, {k: _deep(x) for k, x in v.items()})
    return v


def kin(text):
    """``ur_manipulator`` 那一层的映射，**取缺失的键返回空映射而不是抛**。

    ⚠️ 为什么不能直接 `yaml.safe_load(t)[...][组][键]`：被测代码一旦写错
    （键插到邻居组去了、缩进错位），这里就抛 `KeyError`，**整个套件当场中断**，
    后面的检查一条都不跑 —— 而只数 `FAIL` 的地方会把「崩了」读成「通过」。
    本项目在 `pick_zone_z_note(None,…)`、`test_ik_repeat` 的 `safe` 上栽过
    好几次，这次直接让取值**永不抛**：错的代码让各条检查各自变红。
    """
    try:
        d = yaml.safe_load(text)["/**"]["ros__parameters"][
            "robot_description_kinematics"]
    except Exception:                            # noqa: BLE001
        return _dd()
    return _deep(d) if isinstance(d, dict) else _dd()


# ----------------------------------------------------------------------
# A. 基本改写
# ----------------------------------------------------------------------
print("\n--- A. 基本改写 ---")
NEW, CHANGED = patched(REAL)
check("改之前 attempts 是 3", kin(REAL)["ur_manipulator"]["kinematics_solver_attempts"] == 3)
check("★ 改完 changed=True", CHANGED is True)
check("★ attempts 变成 1",
      kin(NEW)["ur_manipulator"]["kinematics_solver_attempts"] == 1)
check("★ 补上了 consistency_limits，且是 6 个 100.0",
      kin(NEW)["ur_manipulator"]["kinematics_solver_consistency_limits"]
      == [100.0] * 6)
check("  别的键一个都没动（solver / search_resolution / timeout）",
      kin(NEW)["ur_manipulator"]["kinematics_solver"]
      == "kdl_kinematics_plugin/KDLKinematicsPlugin"
      and kin(NEW)["ur_manipulator"]["kinematics_solver_search_resolution"] == 0.005
      and kin(NEW)["ur_manipulator"]["kinematics_solver_timeout"] == 0.005)
check("  组名与上层的 /** 都在（没有把结构改塌）",
      set(kin(NEW)) == {"ur_manipulator"})
check("★ 缩进对：新行是 8 个空格（与同级键齐）",
      "        kinematics_solver_attempts: 1" in NEW
      and "        kinematics_solver_consistency_limits: " in NEW)

# ----------------------------------------------------------------------
# B. 幂等
# ----------------------------------------------------------------------
print("\n--- B. 幂等 ---")
AGAIN, CHANGED2 = patched(NEW)
check("★ 已经是想要的 ⇒ changed=False", CHANGED2 is False)
check("★ 而且**逐字节**不变（不是「差不多」）", AGAIN == NEW)
check("  再跑第三遍还是一样", patched(AGAIN)[1] is False)
check("对原始文件跑两遍 == 跑一遍", patched(NEW)[0] == NEW)

# ----------------------------------------------------------------------
# C. 不许动到别处
# ----------------------------------------------------------------------
print("\n--- C. 别的组 / 注释 / 别的键 ---")
TWO_GROUPS = """\
/**:
  ros__parameters:
    robot_description_kinematics:
      # ← 这行注释必须活下来
      ur_manipulator:
        kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
        kinematics_solver_timeout: 0.005
        kinematics_solver_attempts: 3
      ur5_manipulator:
        kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
        kinematics_solver_timeout: 0.005
        kinematics_solver_attempts: 3
"""
OUT, _ = patched(TWO_GROUPS)
check("★ 注释活下来了", "# ← 这行注释必须活下来" in OUT)
check("★ 目标组被加固",
      kin(OUT)["ur_manipulator"]["kinematics_solver_attempts"] == 1
      and kin(OUT)["ur_manipulator"]["kinematics_solver_consistency_limits"]
      == [100.0] * 6)
check("★ **邻居组一个字节都没动**（还是 3，也没多出 limits）",
      kin(OUT)["ur5_manipulator"]["kinematics_solver_attempts"] == 3
      and "kinematics_solver_consistency_limits" not in kin(OUT)["ur5_manipulator"])
check("  组数没变（没有把邻居吃掉或复制一份）",
      set(kin(OUT)) == {"ur_manipulator", "ur5_manipulator"})
check("  行数只增加了 1（attempts 是替换、limits 是新增）",
      len(OUT.split("\n")) == len(TWO_GROUPS.split("\n")) + 1)

# ★ 上面那条**没有**覆盖块边界写错的情形：目标组和邻居**都有** attempts 时，
#   `_find` 取的是第一个匹配，仍然落在目标组里 —— 就算块延伸过去也看不出来。
#   真正的危险是**目标组缺这个键、邻居有**：块延伸过去就会把键插/改到**邻居**身上，
#   而邻居正是真机那个组名之外的另一份配置。这一条才判别块边界。
TARGET_LACKS = """\
/**:
  ros__parameters:
    robot_description_kinematics:
      ur_manipulator:
        kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
        kinematics_solver_timeout: 0.005
      ur5_manipulator:
        kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
        kinematics_solver_timeout: 0.005
        kinematics_solver_attempts: 3
"""
OUT3, _ = patched(TARGET_LACKS)
check("★ 目标组缺 attempts、邻居有 ⇒ 目标组补上 1",
      kin(OUT3)["ur_manipulator"]["kinematics_solver_attempts"] == 1)
check("★ 而**邻居那个 3 一动不动**（块边界写错就会把它改成 1）",
      kin(OUT3)["ur5_manipulator"]["kinematics_solver_attempts"] == 3)
check("★ 邻居也没有被塞进 limits",
      "kinematics_solver_consistency_limits" not in kin(OUT3)["ur5_manipulator"])

# ----------------------------------------------------------------------
# D. 认不出结构就抛，不许猜
# ----------------------------------------------------------------------
print("\n--- D. 认不出就不猜 ---")
NO_GROUP = "a:\n  b: 1\n"
r = safe(patch_text, NO_GROUP)
check("★ 没有那个组 ⇒ 抛 LookupError（不是默默插到一个地方）",
      isinstance(r, str) and r.startswith("!! 崩了：LookupError"), f"got {r!r}")
NO_SOLVER = ("/**:\n  ros__parameters:\n    robot_description_kinematics:\n"
             "      ur_manipulator:\n        foo: 1\n")
r = safe(patch_text, NO_SOLVER)
check("★ 组在但没有 kinematics_solver ⇒ 也抛（锚点不在不硬插）",
      isinstance(r, str) and r.startswith("!! 崩了：LookupError"), f"got {r!r}")
check("★ 抛的是 LookupError，不是别的异常（调用方照它判环境问题）",
      isinstance(r, str) and "LookupError" in r)

# ----------------------------------------------------------------------
# E. 缺键时插入的位置
# ----------------------------------------------------------------------
print("\n--- E. 从零插入 ---")
NO_ATTEMPTS = """\
/**:
  ros__parameters:
    robot_description_kinematics:
      ur_manipulator:
        kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
        kinematics_solver_timeout: 0.005
"""
OUT2, CH2 = patched(NO_ATTEMPTS)
check("没有 attempts ⇒ 插上，changed=True", CH2 is True)
check("★ 两项都插进去了，且值对",
      kin(OUT2)["ur_manipulator"]["kinematics_solver_attempts"] == 1
      and kin(OUT2)["ur_manipulator"]["kinematics_solver_consistency_limits"]
      == [100.0] * 6)
check("★ 插在**组块内部**（不是插到文件末尾去）",
      OUT2.rstrip("\n").split("\n")[-1].strip().startswith("kinematics_solver"))
check("  timeout 没被动", kin(OUT2)["ur_manipulator"]["kinematics_solver_timeout"]
      == 0.005)

# ----------------------------------------------------------------------
# F. read_values
# ----------------------------------------------------------------------
print("\n--- F. read_values ---")
cur, span = safe(read_values, REAL)
check("读得出 attempts=3", cur["kinematics_solver_attempts"] == "3")
check("★ 缺的键读成 None（不是 0、不是异常）",
      cur["kinematics_solver_consistency_limits"] is None)
check("  也读得出 timeout（报告里要印）",
      cur["kinematics_solver_timeout"] == "0.005")
check("★ 找不到组时返回 (None, None)，不抛",
      safe(read_values, NO_GROUP) == (None, None))
cur2, _ = safe(read_values, NEW)
check("  加固过的文件读出来是想要的值（验收判据的另一半）",
      cur2["kinematics_solver_attempts"] == "1"
      and cur2["kinematics_solver_consistency_limits"] == WANT_LIMITS)

print("\n通过 %d，失败 %d" % (N_PASS, N_FAIL))
if FAILS:
    print("失败项：")
    for f in FAILS:
        print("  - %s" % f)
sys.exit(1 if N_FAIL else 0)
