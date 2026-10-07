#!/usr/bin/env python3
"""
最小验证测试 — 验证 YOLO 检测 + 3D 投影流水线（无需 ROS2/Gazebo）

这个脚本仅需要:
  - Conda 环境 (yolo_ros)
  - ultralytics (YOLOv8)
  - OpenCV

运行:
  python scripts/test_minimal.py

退出码（本仓约定，与 `state_check.py` 同源）：
  0 = 跑到的都通过；1 = 有断言失败；**2 = 一项都没验到**（环境不满足，不是失败）
⚠ **`SKIP` 不是通过**：本机没叠 conda 环境时 YOLO 那一段**没验到**，
  它在总结里印 `SKIP` 而不是 `✓ PASS` —— 两者混在一起的话，人就学会无视它了
  （台账里这道红被逐轮手写「与本轮无关」跳过了 5 次）。
"""

import sys
import os
from pathlib import Path

# 添加项目根目录
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import numpy as np
    import cv2
except ImportError as exc:
    # 缺 numpy/cv2 ⇒ **退 2（环境不满足），不是退 1**。两者的区别见文件头。
    # 这道分支意味着 main() 里不必再有「一项都没验到」的判断：numpy/cv2 是硬依赖，
    # 进得来这个模块就说明它俩在，测试 2/3 必然跑得成。
    print("⏭  环境不满足：%s —— 本机没有 numpy/cv2，这道自检跑不了（**没验到**）" % exc)
    sys.exit(2)


def test_yolo():
    """测试 YOLOv8 模型加载和推理。

    **三态**：`True` 通过 / `False` 断言失败 / **`None` 没验到**（环境不满足）。
    缺 ultralytics 或缺模型文件都是 `None` —— 它们是「这台机器上跑不了这一段」，
    不是「这一段坏了」。两者都退成 `False` 的话，本机一跑就是红的、而红得没有
    信息量，人就学会无视它（台账里这道红正是这么被跳过 5 次的）。
    """
    print("=" * 60)
    print("[Test 1] YOLOv8 模型加载与推理")
    print("=" * 60)

    try:
        from ultralytics import YOLO
    except ImportError:
        print("SKIP: ultralytics 未安装（本机没叠 conda 环境）。"
              "要跑这一段: PYTHONPATH=<venv>/lib/python3.10/site-packages，"
              "见 README「跑脚本」一节")
        return None

    # 加载模型 —— 本项目实际部署的方块检测器（1 类 block）。
    # 旧版此处写的是 YOLO("yolov8n.pt") 却打印「COCO 类别示例」：那个名字
    # 在仓库根其实是方块模型，真 COCO 在 *_backup.pt 里，且该名字已不再
    # 存在——继续用会触发 ultralytics 静默联网下载（D6/D24）。
    model_path = Path(__file__).resolve().parent.parent / "block_best.pt"
    if not model_path.is_file():
        # ⚠️ 这里**曾经 `return True`** —— 模型文件根本不在，却报「通过」。
        # 没验到就是没验到，不是「绿」。同族的坑本仓踩过：`0x0100` 只写寄存器
        # 被读来判状态，读回来永远是 0，于是一直「通过」（AGENTS.md §2 红线 2）。
        print(f"SKIP: 未找到 {model_path}，模型加载与推理**没验到**。")
        return None
    print(f"加载 {model_path.name}...")
    model = YOLO(str(model_path))
    print(f"模型加载成功。类别数: {len(model.names)}")
    print(f"类别: {list(model.names.values())[:10]}")

    # 生成测试图像（模拟彩色方块）
    img = np.ones((480, 640, 3), dtype=np.uint8) * 200  # 灰色背景

    # 红色方块
    cv2.rectangle(img, (200, 150), (280, 230), (0, 0, 255), -1)
    # 蓝色方块
    cv2.rectangle(img, (350, 200), (430, 280), (255, 0, 0), -1)
    # 绿色圆形
    cv2.circle(img, (500, 100), 40, (0, 255, 0), -1)

    # 推理
    results = model(img, verbose=False)
    result = results[0]

    if result.boxes is None or len(result.boxes) == 0:
        print("WARNING: 未检测到目标。阈值可能太高或模拟图像不匹配。")
        print("  尝试降低 confidence_threshold 或使用真实图像。")
        # 仍算通过：**模型加载与推理这两件事确实跑了**，只是合成图没检出东西。
        # 与上面「模型文件不在」不同 —— 那一个连推理都没发生。（这一条留 `True`
        # 是作者的判断，不是漏改。）
        return True

    boxes = result.boxes.xyxy.cpu().numpy()
    confs = result.boxes.conf.cpu().numpy()
    clss = result.boxes.cls.cpu().numpy()

    print(f"\n检测到 {len(boxes)} 个目标:")
    for i, (box, conf, cls_id) in enumerate(zip(boxes, confs, clss)):
        x1, y1, x2, y2 = box
        class_name = model.names.get(int(cls_id), f'class_{int(cls_id)}')
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        print(f"  [{i}] {class_name}: conf={conf:.3f}, "
              f"bbox=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}), "
              f"center=({cx:.1f},{cy:.1f})")

    print("\nYOLO 检测测试通过 ✓")
    return True


def test_3d_projection():
    """测试 2D→3D 投影数学逻辑。"""
    print("\n" + "=" * 60)
    print("[Test 2] 2D→3D 投影逻辑验证")
    print("=" * 60)

    # 模拟 RealSense D435 内参
    fx, fy = 600.0, 600.0
    cx, cy = 320.0, 240.0
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]])

    # 模拟检测框中心像素
    u, v = 400.0, 300.0

    # 模拟深度值（1.0 米）
    Z = 1.0

    # 投影公式: [X, Y, Z]^T = Z * K^{-1} * [u, v, 1]^T
    uv1 = np.array([u, v, 1.0])
    K_inv = np.linalg.inv(K)
    XYZ_cam = Z * K_inv @ uv1

    print(f"相机内参: fx={fx}, fy={fy}, cx={cx}, cy={cy}")
    print(f"检测框中心: ({u}, {v})")
    print(f"深度: Z = {Z} m")
    print(f"→ 相机坐标系 3D: ({XYZ_cam[0]:.3f}, {XYZ_cam[1]:.3f}, {XYZ_cam[2]:.3f}) m")

    # 验证公式
    expected_x = (u - cx) * Z / fx
    expected_y = (v - cy) * Z / fy
    print(f"  公式验证: X = (u-cx)*Z/fx = ({u}-{cx})*{Z}/{fx} = {expected_x:.3f}")
    print(f"             Y = (v-cy)*Z/fy = ({v}-{cy})*{Z}/{fy} = {expected_y:.3f}")

    assert abs(XYZ_cam[0] - expected_x) < 1e-6, "X 计算不匹配!"
    assert abs(XYZ_cam[1] - expected_y) < 1e-6, "Y 计算不匹配!"

    print("2D→3D 投影验证通过 ✓")
    return True


def test_depth_median():
    """测试深度中值滤波。"""
    print("\n" + "=" * 60)
    print("[Test 3] 深度中值滤波")
    print("=" * 60)

    # 模拟深度图 (带噪声)
    depth = np.array([
        [0.50, 0.52, 0.0,  0.51, 0.53],
        [0.51, 0.99, 0.52, 0.50, 0.49],   # 中心 (2,2) = 0.52, 有一个 0.99 噪声
        [0.53, 0.51, 9.99, 0.52, 0.50],   # 中心偏右有 NaN-like 9.99
        [0.49, 0.48, 0.51, 0.52, 0.48],
        [0.50, 0.51, 0.0,  0.49, 0.50],
    ], dtype=np.float32)

    # 邻域中值滤波实现
    def get_median_depth(depth_img, u, v, win=3):
        h, w = depth_img.shape
        half = win // 2
        r_min = max(0, v - half)
        r_max = min(h, v + half + 1)
        c_min = max(0, u - half)
        c_max = min(w, u + half + 1)
        patch = depth_img[r_min:r_max, c_min:c_max]
        valid = patch[(patch > 0) & (~np.isnan(patch)) & (patch < 9.0)]  # 添加合理范围过滤
        if len(valid) == 0:
            return None
        return float(np.median(valid))

    # 测试中心点
    center_depth = get_median_depth(depth, 2, 2, win=3)
    print(f"深度图中心点 (2,2) 邻域中值: {center_depth:.3f}")
    assert abs(center_depth - 0.51) < 0.01, f"期望约 0.51，得到 {center_depth}"

    # 测试边界点
    edge_depth = get_median_depth(depth, 0, 0, win=3)
    print(f"深度图角点 (0,0) 邻域中值: {edge_depth:.3f}")

    print("深度中值滤波验证通过 ✓")
    return True


def main():
    print("\n" + "█" * 60)
    print("  YOLO + 3D Perception 最小验证测试")
    print("█" * 60)

    results = []
    results.append(("YOLO 检测", test_yolo()))
    results.append(("3D 投影", test_3d_projection()))
    results.append(("深度滤波", test_depth_median()))

    print("\n" + "=" * 60)
    print("  测试总结")
    print("=" * 60)
    all_pass = True
    skipped = []
    for name, ok in results:
        if ok is None:
            status = "— SKIP（没验到）"
            skipped.append(name)
        elif ok:
            status = "✓ PASS"
        else:
            status = "✗ FAIL"
            all_pass = False
        print(f"  {name:.<20} {status}")

    print()
    if not all_pass:
        print("  部分测试失败，请检查环境配置。")
    elif skipped:
        print("  跑到的都通过，但**没验到的**：%s —— SKIP 不算通过。" % "、".join(skipped))
    else:
        print("  全部测试通过! 可以继续部署 ROS2 环境。")

    # 退出码：1 = 有断言失败；0 = 跑到的都通过（**允许有 SKIP** —— 与
    # `test_ag95_probe.py` 同一约定：局部没验到不把整道判红）。
    # 「一项都验不到」在**导入期**就退 2 了（numpy/cv2 是硬依赖），所以这里没有
    # 那个分支 —— 写了也永远不亮，等于假装有覆盖。
    return 1 if not all_pass else 0


if __name__ == '__main__':
    sys.exit(main())
