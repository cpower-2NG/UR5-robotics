#!/usr/bin/env python3
"""
YOLO 实时检测 Gazebo 方阵
=========================
订阅 Gazebo 相机话题 → YOLOv8 推理 → OpenCV 显示检测框。

前提:
    source /opt/ros/jazzy/setup.bash
    conda activate yolo_ros
    Gazebo 已启动方阵世界 (带相机)

用法:
    # 终端1: 启动 Gazebo
    gz sim -r ros2_ws/src/yolo_bringup/worlds/block_grid_world.sdf

    # 终端2: 运行检测
    python3 scripts/yolo_live_detect.py

    # 用训练好的模型 (mAP 99.5%)
    python3 scripts/yolo_live_detect.py --model runs/detect/block_grid_demo/weights/best.pt
"""

import os
import sys
import time
import signal
import subprocess
import threading
import argparse
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent

parser = argparse.ArgumentParser(description="YOLO 实时检测 Gazebo 相机")
parser.add_argument("--model", default="yolov8n.pt", help="YOLO 模型权重")
parser.add_argument("--topic", default="/camera/color/image_raw", help="相机 topic")
parser.add_argument("--conf", type=float, default=0.5, help="置信度阈值")
parser.add_argument("--no-bridge", action="store_true", help="不自动启动 ros_gz_bridge")
args = parser.parse_args()


def start_bridge():
    subprocess.run(["pkill", "-f", "parameter_bridge.*camera"], capture_output=True)
    time.sleep(0.5)
    cmd = [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        f"{args.topic}@sensor_msgs/msg/Image[gz.msgs.Image",
        "--ros-args", "-r", "__node:=bridge_camera"
    ]
    print(f"[Bridge] {' '.join(cmd)}")
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
    from ultralytics import YOLO

    bridge_proc = None
    if not args.no_bridge:
        bridge_proc = start_bridge()
        time.sleep(3)

    # 模型
    model_path = args.model
    if not os.path.exists(model_path):
        alt = PROJECT_ROOT / "runs" / "detect" / "block_grid_demo" / "weights" / "best.pt"
        model_path = str(alt) if alt.exists() else "yolov8n.pt"

    print(f"[YOLO] {model_path}")
    model = YOLO(model_path)
    print(f"[YOLO] conf={args.conf}")

    rclpy.init()
    node = Node("yolo_live_detect")
    bridge = CvBridge()
    latest = [None]
    lock = threading.Lock()

    def cb(msg: Image):
        try:
            with lock:
                latest[0] = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    node.create_subscription(Image, args.topic, cb, 10)
    print(f"[订阅] {args.topic}")
    print("[操作] Q/ESC 退出\n")

    running = True
    signal.signal(signal.SIGINT, lambda *_: setattr(sys.modules[__name__], 'running', False))

    times = []
    try:
        while running:
            rclpy.spin_once(node, timeout_sec=0.01)
            with lock:
                frame, latest[0] = latest[0], None
            if frame is None:
                cv2.waitKey(30)
                continue

            t0 = time.time()
            results = model(frame, conf=args.conf, verbose=False)
            t_ms = (time.time() - t0) * 1000
            times.append(t_ms)
            if len(times) > 30:
                times.pop(0)

            annotated = results[0].plot()
            n = len(results[0].boxes)
            fps = 1000 / (sum(times) / len(times)) if times else 0

            for i, txt in enumerate([
                f"Detections: {n}", f"Inference: {t_ms:.0f}ms",
                f"FPS: {fps:.0f}", f"Model: {Path(model_path).name}", "Q/ESC: Quit"
            ]):
                cv2.putText(annotated, txt, (8, 20 + i * 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 0) if n > 0 else (0, 0, 255), 1, cv2.LINE_AA)

            cv2.imshow("YOLO Live - 方阵", annotated)
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        if bridge_proc:
            bridge_proc.terminate()
        print("\n[退出]")


if __name__ == "__main__":
    main()
