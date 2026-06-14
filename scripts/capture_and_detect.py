#!/usr/bin/env python3
"""采集Gazebo相机一帧 → YOLO检测 → 保存标注图像 → 退出

用法:
source /opt/ros/jazzy/setup.bash && conda activate yolo_ros && python3 scripts/capture_and_detect.py
"""

import time, sys, os
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

rclpy.init()
node = Node("capture_once")
bridge = CvBridge()
result = [None]

def cb(msg):
    if result[0] is None:
        result[0] = msg

node.create_subscription(Image, "/camera/color/image_raw", cb, 10)

# 等一帧 (最多10秒)
print("等待相机帧...")
t0 = time.time()
while result[0] is None and time.time() - t0 < 10:
    rclpy.spin_once(node, timeout_sec=0.1)

if result[0] is None:
    print("超时! bridge未运行?")
    sys.exit(1)

msg = result[0]
print(f"收到: {msg.width}x{msg.height}, {msg.encoding}")

img = bridge.imgmsg_to_cv2(msg, "bgr8")
print(f"像素: [{img.min()},{img.max()}], mean={img.mean():.0f}")

# YOLO
mpath = str(PROJECT_ROOT / "runs/detect/block_grid_demo/weights/best.pt")
if not os.path.exists(mpath):
    mpath = "yolov8n.pt"
model = YOLO(mpath)
results = model(img, conf=0.5, verbose=False)

n = len(results[0].boxes)
annotated = results[0].plot()

out = str(PROJECT_ROOT / "runs/block_detect/capture_detect.jpg")
os.makedirs(os.path.dirname(out), exist_ok=True)
cv2.imwrite(out, annotated)
print(f"检测: {n} 个物体 → 已保存 {out}")

node.destroy_node()
rclpy.shutdown()
