#!/usr/bin/env python3
"""
detector_node.resolve_device 自检 —— 不需要 torch、不需要 ROS 图（P2-4）
=====================================================================
`resolve_device()` 决定 YOLO 推理跑在哪个设备上。旧实现直接
`self.model.to('cuda:0')`，在没有 CUDA 的机器上会让**整个节点崩溃**；
现在的行为是「不可用则告警并退回 cpu」，除非显式禁止 fallback。

本脚本覆盖该函数的全部分支，**不导入 torch/ultralytics、不实例化 ROS 节点**
（用假 logger 收集日志，用 sys.modules 注入假的 torch 来模拟有/无 GPU）。

用法:
    python3 scripts/test_detector_device.py
退出码 0 = 全通过。
"""

import sys
import types
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DETECTOR_SRC = PROJECT_ROOT / "ros2_ws" / "src" / "yolo_detector"

sys.path.insert(0, str(DETECTOR_SRC))


class FakeLogger:
    """收集日志，避免依赖 rclpy。"""

    def __init__(self):
        self.infos, self.warns, self.errors = [], [], []

    def info(self, m): self.infos.append(m)
    def warn(self, m): self.warns.append(m)
    def error(self, m): self.errors.append(m)


def fake_torch(available, count=1, name='Fake GPU'):
    """构造一个假的 torch 模块，用来模拟 GPU 有无。"""
    m = types.ModuleType('torch')
    cuda = types.SimpleNamespace()
    cuda.is_available = lambda: available
    cuda.device_count = lambda: count
    cuda.get_device_name = lambda i: name
    m.cuda = cuda
    return m


_failed = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  {detail}" if detail else ''))
    if not cond:
        _failed.append(name)


def main():
    # 导入模块：detector_node 顶层不 import torch/ultralytics（它们在 __init__ 内），
    # 但会 import rclpy —— 若环境没有 ROS，跳过而不是报错。
    try:
        from yolo_detector.detector_node import resolve_device
    except ImportError:
        # 退一步：直接按文件路径加载，绕开包结构
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            'detector_node_probe',
            DETECTOR_SRC / 'yolo_detector' / 'detector_node.py',
        )
        mod = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(mod)
        except ImportError as e:
            print(f"[SKIP] 需要 ROS2 Python 环境（{e}）。")
            print("       请先 source /opt/ros/humble/setup.bash 再运行本脚本。")
            return 0
        resolve_device = mod.resolve_device

    real_torch = sys.modules.get('torch')

    def with_torch(mod, fn):
        if mod is None:
            # 模拟「torch 真的没装」。**不能**用 sys.modules.pop('torch')：
            # 本仓库的 conda 环境里 torch 是**真装着**的，pop 掉之后
            # `import torch` 会重新把它导进来，于是「torch 未安装」这条
            # 分支永远测不到（在装了 torch 的机器上表现为假失败）。
            # sys.modules[name] = None 才是让 import 抛 ImportError 的正规做法。
            sys.modules['torch'] = None
        else:
            sys.modules['torch'] = mod
        try:
            return fn()
        finally:
            if real_torch is not None:
                sys.modules['torch'] = real_torch
            else:
                sys.modules.pop('torch', None)

    print("\n--- 显式 cpu / 非法值 ---")
    log = FakeLogger()
    check("'cpu' 原样返回", resolve_device('cpu', True, log) == 'cpu')
    check("  → 无任何告警", not log.warns)

    log = FakeLogger()
    check("空串按 cpu 处理", resolve_device('', True, log) == 'cpu')

    log = FakeLogger()
    check("'mps' 无法识别 → cpu", resolve_device('mps', True, log) == 'cpu')
    check("  → 有告警提示", any('无法识别' in w for w in log.warns))

    print("\n--- GPU 可用 ---")
    log = FakeLogger()
    check("'cuda:0' 可用 → 原样返回",
          with_torch(fake_torch(True, count=2), lambda: resolve_device('cuda:0', True, log)) == 'cuda:0')
    check("  → 日志含 GPU 名", any('Fake GPU' in i for i in log.infos), f"{log.infos}")

    log = FakeLogger()
    check("'cuda' 无序号 → cuda:0",
          with_torch(fake_torch(True, count=1), lambda: resolve_device('cuda', True, log)) == 'cuda:0')

    log = FakeLogger()
    check("'cuda:1' 存在 → cuda:1",
          with_torch(fake_torch(True, count=2), lambda: resolve_device('cuda:1', True, log)) == 'cuda:1')

    print("\n--- GPU 不可用（旧实现会在此崩溃）---")
    log = FakeLogger()
    check("无 CUDA + 允许 fallback → cpu",
          with_torch(fake_torch(False), lambda: resolve_device('cuda:0', True, log)) == 'cpu')
    check("  → 告警含「已退回 cpu」", any('已退回 cpu' in w for w in log.warns))
    check("  → 告警提醒 Orin CUDA/torch 匹配",
          any('JetPack' in w for w in log.warns))

    log = FakeLogger()
    check("无 CUDA + 禁止 fallback → 抛 RuntimeError",
          _raises(lambda: with_torch(fake_torch(False),
                                     lambda: resolve_device('cuda:0', False, log))))

    print("\n--- 序号越界 / torch 缺失 / 序号非法 ---")
    log = FakeLogger()
    check("cuda:5 越界(count=1) → cpu",
          with_torch(fake_torch(True, count=1), lambda: resolve_device('cuda:5', True, log)) == 'cpu')
    check("  → 告警说明可见 GPU 数", any('可见 GPU 数' in w for w in log.warns))

    log = FakeLogger()
    check("torch 未安装 → cpu（不是崩溃）",
          with_torch(None, lambda: resolve_device('cuda:0', True, log)) == 'cpu')
    check("  → 告警含「torch 未安装」", any('torch 未安装' in w for w in log.warns))

    log = FakeLogger()
    check("'cuda:x' 序号非法 → 按 cuda:0 并告警",
          with_torch(fake_torch(True, count=1), lambda: resolve_device('cuda:x', True, log)) == 'cuda:0')
    check("  → 有序号告警", any('序号' in w for w in log.warns))

    print("\n" + "=" * 52)
    if _failed:
        print(f"FAILED: {len(_failed)} 项 -> {_failed}")
        return 1
    print("全部通过。")
    return 0


def _raises(fn):
    try:
        fn()
    except RuntimeError:
        return True
    except Exception:
        return False
    return False


if __name__ == '__main__':
    sys.exit(main())
