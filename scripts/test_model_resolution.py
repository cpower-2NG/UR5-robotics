#!/usr/bin/env python3
"""
模型路径解析自检 —— 钉住「哪一个模型会被加载」（D67）
=====================================================
`detector_node` 的 `_resolve_model()` 决定**真机上到底加载了哪个权重**。
它存在的原因是一次真实事故（D6）：旧实现把 `model` 参数原样交给 `YOLO()`，
于是（a）行为随进程 CWD 改变，（b）两处都找不到时 ultralytics 会**静默联网
下载官方 COCO 模型** —— 真机上表现为「什么都检测不到，且不报错」。

D67 是同一类问题在**部署路径**上的复发：`block_best.pt` 这个文件名下同时存在
**两个不同的训练产物**，加载到哪一个取决于启动时的 CWD：

| 启动目录 | 解析到 | 来源 run | epoch | mAP50-95 |
|---|---|---|---|---|
| 仓库根目录 | `<repo>/block_best.pt` | `runs/block_detect` | 100 | **0.966** |
| `~` / `/tmp` / 任何仓库外目录 | 包内 `models/block_best.pt` | `runs/detect/block_grid_demo` | 50 | 0.906 |

**而 `detector_node` 那条「用错模型就告警」的类别自检看不见这个差别** ——
两个模型的 `names` 都是 `['block']`（实测），只有一行 INFO 里的**路径**能看出
来。所以「两个同名模型」这件事不能靠运行时自检兜底，只能靠**结构**：本节第 2 段
断言「同名必须同源」，不同源直接 FAIL。

处置（2026-09-28，用户选定「删掉包内那份」）：包内那份已删除。于是
从仓库根启动 → 模型 A（与仿真 9/9 那套一致）；从别处启动 → 返回 `None`
→ **明确报错**，而不是静默换一个更差的模型。包内那份可随时从
`runs/detect/block_grid_demo/weights/best.pt` 复制回来（该文件被跟踪）。

本脚本不需要 torch/ultralytics、不需要 ROS 图、不需要模型文件真的存在
（只有第 2 段需要仓库根那份权重，缺了就 SKIP 并说明）。

用法:
    source /opt/ros/humble/setup.bash      # 只需要 ROS 基础包（import rclpy 等）
    python3 scripts/test_model_resolution.py
退出码 0 = 全通过。
"""

import hashlib
import os
import sys
import tempfile
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DETECTOR_SRC = PROJECT_ROOT / "ros2_ws" / "src" / "yolo_detector"
# ⚠️ 是**包根**下的 models/，不是内层 python 包 `yolo_detector/` 下的 ——
# setup.py 的 `glob('models/*.pt')` 也是相对包根。写错这一层，第 2 段会去
# 一个不存在的目录里 glob，于是**恒为空、恒 PASS**（这个错真发生过，靠
# 「把模型 B 放回去看断言会不会响」的变异测试抓到的）。
MODELS_DIR = DETECTOR_SRC / "models"

_failed = []
_skipped = []


def check(name, cond, detail=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    if not cond:
        _failed.append(name)


def skip(name, why):
    print(f"  [SKIP] {name}  {why}")
    _skipped.append(name)


def sha256(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    try:
        sys.path.insert(0, str(DETECTOR_SRC))
        from yolo_detector.detector_node import YoloDetector
    except ImportError as exc:
        print(f"需要 ROS 环境（import 失败：{exc}）。先 `source /opt/ros/humble/setup.bash`")
        return 2

    # `_resolve_model` 不读任何 self 属性，故用一个占位 self 调用它
    resolve = lambda name: YoloDetector._resolve_model(None, name)  # noqa: E731

    # 用一个**唯一的**临时名，免得被 /tmp、/ 之类的父目录里的同名文件干扰
    probe = f"__probe_model_{os.getpid()}.pt"

    print("\n--- 1. 解析顺序（绝对 → $YOLO_MODEL_DIR → CWD 及父目录 → 包内）---")
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        deep = td / "a" / "b" / "c"
        deep.mkdir(parents=True)
        env_dir = td / "envdir"
        env_dir.mkdir()

        # 1a 绝对路径
        abs_hit = td / probe
        abs_hit.write_bytes(b"x")
        check("绝对路径存在 → 原样返回",
              resolve(str(abs_hit)) == abs_hit.resolve(),
              f"got {resolve(str(abs_hit))}")
        check("绝对路径**不存在** → None（不退回按名字猜）",
              resolve(str(td / "nope.pt")) is None)

        # 1b $YOLO_MODEL_DIR 优先于 CWD 及父目录
        env_hit = env_dir / probe
        env_hit.write_bytes(b"x")
        old_env = os.environ.get("YOLO_MODEL_DIR")
        old_cwd = os.getcwd()
        try:
            os.environ["YOLO_MODEL_DIR"] = str(env_dir)
            os.chdir(deep)
            check("$YOLO_MODEL_DIR 命中时，优先于 CWD 及父目录",
                  resolve(probe) == env_hit.resolve(),
                  f"got {resolve(probe)}")

            # 1c 父目录逐级向上
            del os.environ["YOLO_MODEL_DIR"]
            # 注意用**另一个**名字：`probe` 在 1a 里已经落在 td/（也就是本目录的
            # 祖父）了，拿它断言「找不到」会立刻失败 —— 那不是代码的问题。
            missing = f"__probe_missing_{os.getpid()}.pt"
            check("没有 env 时，哪一级都没有 → None（不猜、不下载）",
                  resolve(missing) is None, f"got {resolve(missing)}")
            root_hit = td / probe
            check("放在**祖父目录**里的能被找到（逐级向上，不是只看 CWD）",
                  resolve(probe) == root_hit.resolve(),
                  f"got {resolve(probe)}")
        finally:
            os.chdir(old_cwd)
            if old_env is None:
                os.environ.pop("YOLO_MODEL_DIR", None)
            else:
                os.environ["YOLO_MODEL_DIR"] = old_env

    print("\n--- 2. D67：同名必须同源（不同源就是「两个模型共用一个名字」）---")
    canonical = PROJECT_ROOT / "block_best.pt"
    if not canonical.is_file():
        skip("同名同源规则", f"仓库根没有 {canonical.name}（权重是构建产物，未随仓库分发）")
    else:
        others = sorted(MODELS_DIR.glob("*.pt")) if MODELS_DIR.is_dir() else []
        # 包内那份**允许存在**（有人可能刻意让模型随包安装），但必须与仓库根那份
        # 逐字节相同 —— 否则同一个名字会按 CWD 解析成两个不同的模型（D67）。
        # 报错要能直接指向来源 run，因为「哪份是哪份」正是当年查不出来的东西。
        KNOWN = {
            "4916bd5e48f5d98557b1d1caad61a85f": "runs/block_detect（模型 A，100 epoch，mAP50-95 0.966）",
            "87fb70359c644ec6ab83faf309b2538e": "runs/detect/block_grid_demo（模型 B，50 epoch，mAP50-95 0.906）",
        }
        clashes = []
        for f in others:
            same_name_root = PROJECT_ROOT / f.name
            if not same_name_root.is_file():
                clashes.append(f"{f} 在仓库根没有同名对照物 —— 无法判断它是什么")
            elif sha256(f) != sha256(same_name_root):
                a = hashlib.md5(same_name_root.read_bytes()).hexdigest()
                b = hashlib.md5(f.read_bytes()).hexdigest()
                clashes.append(
                    f"{f.name}: 仓库根={KNOWN.get(a, a[:8])} vs 包内={KNOWN.get(b, b[:8])}")
        if others:
            check(f"包内 {len(others)} 个权重都与仓库根同名文件逐字节相同",
                  not clashes, "; ".join(clashes))
        else:
            check("包内没有权重（从仓库外启动会明确报错，而不是换一个模型）",
                  not clashes,
                  "这就是「删掉包内那份」之后应有的状态")
            print("       注：包内没有模型**不是缺陷** —— `_resolve_model` 会在"
                  "全找不到时抛出可操作的 FileNotFoundError（见第 3 段）。")

        # 两个已知来源的哈希不该再出现在「包内」这个位置
        present = {hashlib.md5(f.read_bytes()).hexdigest() for f in others}
        check("包内**没有**模型 B（runs/detect/block_grid_demo 那份）",
              "87fb70359c644ec6ab83faf309b2538e" not in present,
              "它还在这儿的话，从仓库外启动就会静默换模型")

    print("\n--- 3. 找不到时**必须报错**，不许联网兜底 ---")
    src = (DETECTOR_SRC / "yolo_detector" / "detector_node.py").read_text(encoding="utf-8")
    check("找不到 → 抛 FileNotFoundError（不是交给 YOLO() 去猜/去下载）",
          "raise FileNotFoundError" in src and "_resolve_model(raw_model)" in src)
    check("报错里给出可操作的两条路（绝对路径 / 环境变量）",
          "YOLO_MODEL_DIR" in src and "绝对路径" in src)
    check("**先解析、后加载**：YOLO() 拿到的一定是已解析的路径",
          "self.model = YOLO(str(model_path))" in src)
    check("找不到的身影不进 YOLO()：没有裸的 YOLO(raw_model) 调用",
          "YOLO(raw_model)" not in src and "YOLO(model)" not in src)
    check("类别自检仍在（它挡不住同名不同物，但能挡住 COCO 权重）",
          "names != ['block']" in src)

    print("\n" + "=" * 56)
    if _skipped:
        print(f"跳过 {len(_skipped)} 项: {_skipped}")
    if _failed:
        print(f"FAILED: {len(_failed)} 项 -> {_failed}")
        return 1
    print("全部通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
