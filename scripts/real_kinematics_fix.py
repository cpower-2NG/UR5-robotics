#!/usr/bin/env python3
"""**把 D54 的加固补到真机那个规划组上**（台账 D60）—— 默认只看不改。

## 为什么要改这个文件

真机组的 KDL 配置来自系统自带的

    /opt/ros/humble/share/ur_moveit_config/config/kinematics.yaml

里面是 `kinematics_solver_attempts: 3` 且**没有** `kinematics_solver_consistency_limits`。
实测（第八十七轮，`scripts/ik_repeat_check.py`）：同一个位姿、同一个种子连问 20 次，
近基座带能回来 **17~20 个不同的解**（折回 (−π,π] 后 4~13 个位形）；而推荐矩形内
四层各只回来一个。

病根是**量出来的**：拿其中一个解当种子再问 20 次 → 20/20 差 `0.0000 rad`。
所以随机性来自「**种子解不出来时** KDL 拿随机种子重启」。仿真那份
`arm_grasp_sim/config/kinematics.yaml` 早就把这两项钉死了（`attempts: 1` +
`consistency_limits: [100.0]*6`），**真机这份没打上**。

## 为什么要动系统文件（而不是改 launch）

`ur_moveit_config/launch/ur_moveit.launch.py` **没有 kinematics 参数**：它把
`<moveit_config_package>/config/kinematics.yaml` 直接当 `--params-file` 传给
`move_group`（板上 `ps` 里能看到这一行）。唯一的旋钮是 `moveit_config_package`，
而 SRDF / joint_limits / ompl / rviz **也全从它取** —— 换掉它等于把整份 MoveIt
配置自己扛一遍，那正是「两套几何」的来源。所以最小且无分歧的改法就是改这一个文件。

⚠️ **它是系统文件**：`apt upgrade` 或重刷镜像会把它覆盖回去。所以本脚本
`--check` 随时可以再判一次；**验收判据**是重跑
`python3 scripts/ik_repeat_check.py --table-z 0.0 --x 0.20 --y 0.10`
—— 改之前是 3/4 个位姿不可复现，改之后应当全部只回来一个解。

## 跑法

    python3 scripts/real_kinematics_fix.py              # 只看（默认）
    python3 scripts/real_kinematics_fix.py --apply      # 写（要 sudo）

退出码：0 = 已经是加固过的 / 写成功；1 = 还没加固（`--check` 时）；
2 = 环境问题（文件不在、找不到那个组、YAML 结构不认识）。

**不写任何寄存器、不发轨迹、不碰机械臂** —— 只读写一个 yaml。
"""

import argparse
import os
import re
import shutil
import sys
import time

sys.dont_write_bytecode = True

# 想要的值。与仿真那份 `arm_grasp_sim/config/kinematics.yaml` **逐字相同** ——
# 两处不一样就成了两套语义，那种偏差最难查。
WANT_ATTEMPTS = "1"
WANT_LIMITS = "[100.0, 100.0, 100.0, 100.0, 100.0, 100.0]"
DEFAULT_GROUP = "ur_manipulator"


# ----------------------------------------------------------------------
# 纯函数：文本层面的改写。**不碰文件系统**，所以自检够得着。
# ----------------------------------------------------------------------
def _indent_of(line):
    return len(line) - len(line.lstrip(" "))


def _block_span(lines, group):
    """``(组名那一行的下标, 缩进, 块结束下标)``；找不到返回 ``None``。

    只看**严格更浅**的缩进作为块的结束 —— 同级（同缩进）的下一项也属于块外，
    所以用 ``<=`` 判下一项的缩进比它浅或相等时收尾。
    """
    for i, line in enumerate(lines):
        if re.match(rf"^\s*{re.escape(group)}\s*:\s*$", line):
            ind = _indent_of(line)
            for j in range(i + 1, len(lines)):
                s = lines[j]
                if not s.strip():                       # 空行不断块
                    continue
                if s.lstrip().startswith("#"):          # 注释不断块
                    continue
                if _indent_of(s) <= ind:
                    return i, ind, j
            return i, ind, len(lines)
    return None


def patch_text(text, group=DEFAULT_GROUP, attempts=WANT_ATTEMPTS, limits=WANT_LIMITS):
    """返回 ``(新文本, 是否改了)``。**行级改写**，所以注释与别的组都不动。

    值已经对 ⇒ 原样返回 ``(text, False)``（幂等：跑第二遍不写盘）。

    ⚠️ 找不到那个组就**抛 ``LookupError``**，绝不猜 —— 「往大概的位置插两行」
    正是本文件反复在修的「话说了没查过的事」。
    """
    lines = text.split("\n")
    span = _block_span(lines, group)
    if span is None:
        raise LookupError(f"找不到规划组 {group!r} 的块 —— 拒绝猜着改")
    start, ind, end = span
    key_ind = " " * (ind + 2)

    def _find(key):
        for k in range(start + 1, end):
            if re.match(rf"^{re.escape(key_ind)}{re.escape(key)}\s*:", lines[k]):
                return k
        return None

    changed = False

    k = _find("kinematics_solver_attempts")
    want_line = f"{key_ind}kinematics_solver_attempts: {attempts}"
    if k is None:
        # 插在 `kinematics_solver:` 后面（紧跟它最可读，也最不容易和政策冲突）
        anchor = _find("kinematics_solver")
        if anchor is None:
            raise LookupError(f"{group!r} 块里没有 kinematics_solver: —— 结构不认识")
        lines.insert(anchor + 1, want_line)
        end += 1
        changed = True
    elif lines[k] != want_line:
        lines[k] = want_line
        changed = True

    k = _find("kinematics_solver_consistency_limits")
    want_line = f"{key_ind}kinematics_solver_consistency_limits: {limits}"
    if k is None:
        anchor = _find("kinematics_solver_timeout")
        if anchor is None:
            anchor = _find("kinematics_solver")
        if anchor is None:
            raise LookupError(f"{group!r} 块里没有可用的锚点 —— 拒绝猜着插")
        lines.insert(anchor + 1, want_line)
        changed = True
    elif lines[k] != want_line:
        lines[k] = want_line
        changed = True

    return "\n".join(lines), changed


def read_values(text, group=DEFAULT_GROUP):
    """把当前那两项读出来（只报告用）。缺的记为 ``None``。"""
    lines = text.split("\n")
    span = _block_span(lines, group)
    if span is None:
        return None, None
    start, ind, end = span
    out = {}
    for key in ("kinematics_solver_attempts",
                "kinematics_solver_consistency_limits",
                "kinematics_solver_timeout"):
        val = None
        for k in range(start + 1, end):
            m = re.match(rf"^\s*{re.escape(key)}\s*:\s*(.*?)\s*$", lines[k])
            if m:
                val = m.group(1)
        out[key] = val
    return out, (start, ind, end)


# ----------------------------------------------------------------------
# 取数 / 改盘
# ----------------------------------------------------------------------
def default_path():
    """系统那份 `kinematics.yaml` 的路径（用 ament 索引取，不写死发行版路径）。"""
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return None
    return os.path.join(get_package_share_directory("ur_moveit_config"),
                        "config", "kinematics.yaml")


def main():
    ap = argparse.ArgumentParser(
        description="把 attempts:1 + consistency_limits 补到真机组的 KDL 配置上"
                    "（默认只看不改）")
    ap.add_argument("--path", default=None,
                    help="覆盖 yaml 路径；默认用 ament 索引找 ur_moveit_config")
    ap.add_argument("--group", default=DEFAULT_GROUP, help="规划组名")
    ap.add_argument("--apply", action="store_true",
                    help="**真的写**（要 sudo）。不给我就只打印将要做的改动")
    args = ap.parse_args()

    path = args.path or default_path()
    if not path:
        print("!! 找不到 ur_moveit_config —— 先 source /opt/ros/humble/setup.bash")
        return 2
    if not os.path.exists(path):
        print(f"!! 文件不在：{path}")
        return 2

    with open(path, encoding="utf-8") as f:
        text = f.read()

    cur, span = read_values(text, args.group)
    if cur is None:
        print(f"!! {path} 里没有规划组 {args.group!r} —— 认不出结构，拒绝改")
        return 2

    print("=" * 74)
    print(f"文件   {path}")
    print(f"规划组 {args.group}")
    print("=" * 74)
    for k, v in cur.items():
        want = {"kinematics_solver_attempts": WANT_ATTEMPTS,
                "kinematics_solver_consistency_limits": WANT_LIMITS,
                "kinematics_solver_timeout": None}[k]
        mark = ""
        if want is not None:
            mark = "  ✅ 已经是想要的" if v == want else f"  ← 要改成 {want}"
        print(f"  {k:<38} {v!r}{mark}")

    try:
        new, changed = patch_text(text, args.group)
    except LookupError as e:
        print(f"!! {e}")
        return 2

    if not changed:
        print("\n⇒ **已经是加固过的**，不用动。")
        print("   验收：python3 scripts/ik_repeat_check.py --table-z 0.0 "
              "--x 0.20 --y 0.10   （应当四层各只回来一个解）")
        return 0

    print("\n--- 改动（行级，注释与别的组不动）---")
    old_lines, new_lines = text.split("\n"), new.split("\n")
    import difflib
    for line in difflib.unified_diff(old_lines, new_lines,
                                     "before", "after", lineterm="", n=2):
        print("  " + line)

    if not args.apply:
        print("\n⇒ 这是**只看**模式，什么都没写。要写：加 `--apply`（需 sudo）。")
        return 1

    backup = f"{path}.bak-{time.strftime('%Y%m%d-%H%M%S')}"
    shutil.copy2(path, backup)
    with open(path, "w", encoding="utf-8") as f:
        f.write(new)
    # 写完**立刻回读**：确认落盘的真的是想要的样子，而不是「我写了所以它对」。
    with open(path, encoding="utf-8") as f:
        back = f.read()
    again, changed2 = patch_text(back, args.group)
    if changed2:
        print(f"\n!! 写完之后再判一次，**还是需要改** —— 落盘的内容不对。"
              f"原文件已备份在 {backup}，把它复制回去。")
        return 2
    print(f"\n✅ 已写入。备份：{backup}")
    print("   **要重启 move_group 才生效**（参数是启动时读的）。")
    print("   验收：python3 scripts/ik_repeat_check.py --table-z 0.0 "
          "--x 0.20 --y 0.10   （应当四层各只回来一个解）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
