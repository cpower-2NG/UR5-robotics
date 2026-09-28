#!/usr/bin/env python3
"""
板端部署前置检查 —— 在**板子上**跑，回答「这条链路今天能不能起来」（D69）
=========================================================================
起因：三个节点的模块级 `from vision_msgs.msg import ...` 在板上直接
`ModuleNotFoundError` —— 而 `vision_msgs` 从来没被列进板端清单（2026-09-28 实测）。
这类「缺一个 apt 包」的问题有两个特点，合起来就是**必须在开跑前一次问清**：

1. **症状离原因很远** —— 缺 `vision_msgs` 的表现是「三个节点全起不来」，
   看起来像项目坏了、像 Python 环境错了，唯独不像「少装一个消息包」；
2. **看文档看不出来** —— 依赖散在四个 `package.xml` 与若干个函数体内部的
   `import` 里，靠人肉比对必然漏（D69 就是这么漏的）。

所以本脚本**自己从代码里推导要检查什么**，而不是抄一份清单：

- ROS 包依赖：读四个包 `package.xml` 的 `depend`/`exec_depend`/`build_depend`
  （跳过 `test_depend`，跳过**板上刻意不装**的 `arm_grasp_sim`）；
- Python 模块：扫各节点源码里**所有** `import`（含函数体内的，比如 `detector_node`
  的 `__init__` 里才 `import torch`、`_load_model` 里才 `from ultralytics import YOLO`），
  滤掉标准库与包内自引用；
- 模型权重：调 `detector_node._resolve_model()` 本体（不重写它的逻辑），
  看默认的 `block_best.pt` 在**当前目录**解析到哪里 —— D67 的现场体检。

范围说明：这里只扫**四个节点源文件**，也就是 `bringup` 会启动的那些。现场工具
（`scripts/ag95_probe.py` 要 `pymodbus`）不在其中，它由 `docs/待执行命令.md` 的
甲-5 单独覆盖 —— 那一步在部署次序里是第 5 步，晚于本脚本要回答的「链路能不能起来」。

只读：不装包、不写文件、不碰串口、不启动任何节点。**不需要 sudo**。
缺什么会直接给出该装哪一条，装完再跑一遍即可。

用法（在板子上，或任何要验的环境里）:
    source /opt/ros/humble/setup.bash
    python3 scripts/check_board_deploy.py
    python3 scripts/check_board_deploy.py --self-test    # 不需要 ROS，本机即可
退出码 0 = 全部就绪；1 = 有缺失（逐条给出修法）。
"""

import re
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SRC = PROJECT_ROOT / "ros2_ws" / "src"

# 板端子集：**只编这四个**（见 README 的「真机部署」）。arm_grasp_sim 是工作站专用。
BOARD_PACKAGES = ["yolo_bringup", "perception_3d", "yolo_detector", "grasp_planner"]

# `package.xml` 里声明了、但**板上刻意不装**的：bringup 对缺它做了容错，
# 且整条真机分支不会走到仿真后端（README / test_real_launch.py 的 W2-15 钉住）。
BOARD_OPTIONAL = {"arm_grasp_sim"}

# 各节点的源码文件（用于推导 Python 侧依赖）
NODE_FILES = [
    "yolo_detector/yolo_detector/detector_node.py",
    "perception_3d/perception_3d/projector_node.py",
    "grasp_planner/grasp_planner/planner_node.py",
    "grasp_planner/grasp_planner/gripper.py",
]

# 少数几个模块的「修法」不能用通用套路，写死在这里 —— 通用的 `pip3 install torch`
# 在 Jetson(aarch64) 上是**错的建议**（PyPI 没有合适的 CUDA wheel，得用 NVIDIA 的）。
# 这几条都是踩过的，见 docs/待执行命令.md 的 甲-5 与 README 的「真机部署」。
FIX_HINTS = {
    "torch": "板子上 torch 随 JetPack/NGC 提供（实测 2.5.0a0+nv24.08）—— "
             "**不要** pip3 install torch；先确认解释器对不对",
    "ultralytics": "pip3 install ultralytics（依赖 torch，先确认 torch 在）",
    "pymodbus": 'pip3 install "pymodbus>=3.6" —— **不要**用 apt 的 python3-pymodbus（2.1.0，太老）',
}

_failed = []


def ok(msg):
    print(f"  [OK]   {msg}")


def bad(msg, fix):
    print(f"  [MISS] {msg}")
    print(f"         修法: {fix}")
    _failed.append(msg)


def ros_dep_to_deb(name):
    """ROS 包名 → apt 包名：下划线换连字符，加 `ros-humble-` 前缀。"""
    return "ros-humble-" + name.replace("_", "-")


def declared_ros_deps():
    """从四个包的 package.xml 里收集 ROS 包依赖，返回 {dep: [声明它的包]}。"""
    deps = {}
    for pkg in BOARD_PACKAGES:
        pkg_xml = SRC / pkg / "package.xml"
        if not pkg_xml.is_file():
            continue
        text = pkg_xml.read_text(encoding="utf-8")
        for tag in ("depend", "exec_depend", "build_depend", "buildtool_depend"):
            for m in re.finditer(rf"<{tag}>([^<]+)</{tag}>", text):
                name = m.group(1).strip()
                if name in BOARD_OPTIONAL or name in BOARD_PACKAGES:
                    continue
                deps.setdefault(name, []).append(pkg)
    return deps


def is_ros_provided(name, declared):
    """给一个缺失的模块选安装方式：True → apt 装 ROS 包，False → pip。

    ⚠️ **不能去问 ament 索引**「这是不是 ROS 包」—— 这个包**正因为缺失**才查不到，
    那样问必然得到「不是 ROS 包」，于是给出 `pip3 install vision_msgs` 这种
    **错误建议**。第一版就是这么写的，在板子上当场被抓到（与 D66 同一类错误：
    报错文案把方向指反）。

    改为静态判据：`package.xml` 里声明过，或者名字符合 ROS 生态的惯例 ——
    后者用于兜住 `action_msgs` / `shape_msgs` 这类**没被声明但确实来自 ROS** 的
    间接 import（planner_node 里就有）。
    """
    if name in declared:
        return True
    return (name.endswith("_msgs")
            or name in {"rclpy", "cv_bridge", "image_geometry", "rcl_interfaces"}
            or name.startswith(("tf2", "ament_", "launch", "rcl_", "rosidl")))


def declared_py_modules():
    """扫节点源码里**所有** import（含缩进的函数体内 import），返回第三方模块名。

    刻意不用固定清单：D69 的教训就是「清单会漏」。函数体内的
    `import torch`（`__init__` 里）、`from ultralytics import YOLO`
    （加载模型时才）都要算数 —— 它们和模块级 import 一样会让节点起不来，
    只是**晚一步**：模块级缺失是「节点起不来」，函数体内缺失是「起来了、
    YOLO 一加载就崩」，后者在现场更难归因。
    """
    mods = {}
    stdlib = getattr(sys, "stdlib_module_names", frozenset())
    local = set(BOARD_PACKAGES) | {"yolo_detector", "perception_3d", "grasp_planner"}
    for rel in NODE_FILES:
        f = SRC / rel
        if not f.is_file():
            continue
        for line in f.read_text(encoding="utf-8").splitlines():
            m = re.match(r"\s*(?:from\s+([A-Za-z_][\w.]*)\s+import|import\s+([A-Za-z_][\w.]*))",
                         line)
            if not m:
                continue
            name = (m.group(1) or m.group(2)).split(".")[0]
            if name in stdlib or name in local or name.startswith("_"):
                continue
            mods.setdefault(name, set()).add(rel.split("/")[0])
    return mods


def main():
    try:
        from ament_index_python.packages import (
            get_package_share_directory, PackageNotFoundError,
        )
    except ImportError:
        print("需要 ROS 环境：先 `source /opt/ros/humble/setup.bash`")
        return 2

    print("板端部署前置检查（只读；不装包、不启动节点、不需要 sudo）")
    print(f"仓库: {PROJECT_ROOT}")

    print("\n--- 1. ROS 包依赖（来自四个 package.xml）---")
    deps = declared_ros_deps()
    for name in sorted(deps):
        owners = "/".join(sorted(set(deps[name])))
        # `python3-*` 是 rosdep 键（不是 ROS 包名），ament 索引里当然找不到它 ——
        # 直接 `get_package_share_directory('python3-numpy')` 必然 MISS，那是**假警报**。
        # 这类键要按「它对应的 Python 模块在不在」来判。
        if name.startswith("python3-"):
            mod = name[len("python3-"):].replace("-", "_")
            try:
                __import__(mod)
                ok(f"{name} → 模块 {mod} 在  (被 {owners} 需要)")
            except Exception as exc:  # noqa: BLE001
                bad(f"{name} → 模块 {mod} 不在（{type(exc).__name__}）(被 {owners} 需要)",
                    FIX_HINTS.get(mod, f"pip3 install {mod}"))
            continue
        try:
            get_package_share_directory(name)
            ok(f"{name}  (被 {owners} 需要)")
        except PackageNotFoundError:
            bad(f"{name}  (被 {owners} 需要)",
                f"sudo apt install {ros_dep_to_deb(name)}")

    print("\n--- 2. Python 模块（从节点源码的 import 推导，含函数体内）---")
    mods = declared_py_modules()
    for name in sorted(mods):
        owners = "/".join(sorted(mods[name]))
        try:
            m = __import__(name)
            ver = getattr(m, "__version__", "")
            ok(f"{name} {ver}  (被 {owners} import)")
        except Exception as exc:  # noqa: BLE001 —— import 可能因任何原因失败
            # rclpy / cv_bridge / vision_msgs 这类既是 ROS 包又是 Python 模块：
            # 装法只有 apt 一条（见 is_ros_provided 的注释，第一版在这里给反了）。
            if is_ros_provided(name, deps):
                fix = f"sudo apt install {ros_dep_to_deb(name)}"
            else:
                fix = FIX_HINTS.get(name, f"pip3 install {name}")
            bad(f"{name}  (被 {owners} import) —— {type(exc).__name__}: {exc}", fix)

    print("\n--- 3. 模型权重解析（D67 的现场体检：默认参数在当前目录解析到哪）---")
    pkg_py = SRC / "yolo_detector"
    try:
        sys.path.insert(0, str(pkg_py))
        from yolo_detector.detector_node import YoloDetector
        got = YoloDetector._resolve_model(None, "block_best.pt")
    except Exception as exc:  # noqa: BLE001
        got = None
        print(f"  [SKIP] 读不到 _resolve_model（{type(exc).__name__}: {exc}）")
        print("         —— 上一条的缺失就是原因；先按第 2 段把依赖补齐")
    if got is not None:
        ok(f"block_best.pt → {got}")
        print("         启动日志里的 `Loading YOLO model:` 应当就是这一行")
    elif "YoloDetector" in dir():
        bad("block_best.pt 在当前目录解析不到（会以 FileNotFoundError 退出）",
            "export YOLO_MODEL_DIR=<放权重的目录>，或用绝对路径 -p model:=/abs/path.pt")

    print("\n" + "=" * 60)
    if _failed:
        print(f"有 {len(_failed)} 项缺失 —— 修完再跑一遍本脚本：")
        for f in _failed:
            print(f"  · {f}")
        return 1
    print("板端部署前置条件全部就绪。")
    return 0


def self_test():
    """`--self-test`：钉住「缺失的模块该 apt 还是 pip」（不需要 ROS、不需要板子）。

    这一段存在的唯一理由是它**真错过一次**：第一版靠 ament 索引判断「是不是 ROS 包」，
    而包正因为缺失才查不到 —— 于是在板子上给出 `pip3 install vision_msgs`。
    判据是纯函数，就在本机把表钉住；再犯就是 FAIL 而不是又要在板子上抓一遍。
    """
    declared = {"vision_msgs": ["grasp_planner"], "rclpy": ["yolo_detector"]}
    cases = [
        # (模块, 期望 apt?, 说明)
        ("vision_msgs", True, "声明过的 ROS 消息包 —— 正是被抓到的那条"),
        ("rclpy", True, "包.xml 里声明过"),
        ("action_msgs", True, "没声明过，但 _msgs 命名 ⇒ ROS 提供（planner_node 间接 import）"),
        ("tf2_geometry_msgs", True, "_msgs 后缀 + tf2 前缀"),
        ("cv_bridge", True, "ROS 生态的已知模块"),
        ("ultralytics", False, "pip 包 —— 给 apt 建议会白忙"),
        ("numpy", False, "pip/apt 都有，但 pip 建议不会指错方向"),
        ("pymodbus", False, "pip 包（且 FIX_HINTS 里另有更强的话要说）"),
        ("serial", False, "pyserial 的模块名，pip 包"),
    ]
    bad_cases = []
    for name, want_apt, why in cases:
        got = is_ros_provided(name, declared)
        print(f"  [{'PASS' if got == want_apt else 'FAIL'}] {name} → "
              f"{'apt' if got else 'pip'}")
        if got != want_apt:
            bad_cases.append(f"{name}: 期望 {'apt' if want_apt else 'pip'}（{why}）")
    if bad_cases:
        print("FAILED: " + "; ".join(bad_cases))
        return 1
    print("全部通过。")
    return 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    sys.exit(main())
