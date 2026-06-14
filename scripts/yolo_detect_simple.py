#!/usr/bin/env python3
"""YOLO实时检测Gazebo方阵 - matplotlib版 (WSL2兼容)

一行启动:
source /opt/ros/jazzy/setup.bash && conda activate yolo_ros && pkill -f parameter_bridge 2>/dev/null; ros2 run ros_gz_bridge parameter_bridge /camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image & sleep 3 && python3 scripts/yolo_detect_simple.py --model runs/detect/block_grid_demo/weights/best.pt
"""

import sys, time, threading, argparse
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
parser = argparse.ArgumentParser()
parser.add_argument("--model", default="yolov8n.pt")
parser.add_argument("--conf", type=float, default=0.5)
args = parser.parse_args()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

# 模型
mpath = args.model
if not Path(mpath).exists():
    alt = PROJECT_ROOT / "runs/detect/block_grid_demo/weights/best.pt"
    mpath = str(alt) if alt.exists() else "yolov8n.pt"
print(f"[YOLO] {mpath}")
model = YOLO(mpath)

rclpy.init()
node = Node("yolo_detect")
bridge_cv = CvBridge()
frame = [None]
lock = threading.Lock()
logged = False

def cb(msg):
    global logged
    try:
        if not logged:
            print(f"[编码] {msg.encoding}, {msg.width}x{msg.height}")
            logged = True
        img = bridge_cv.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        if img is not None:
            with lock:
                frame[0] = img
    except Exception as e:
        print(f"[CB] {e}")

node.create_subscription(Image, "/camera/color/image_raw", cb, 10)

plt.ion()
fig, ax = plt.subplots(figsize=(8, 6))
fig.canvas.manager.set_window_title("YOLO 方阵检测")
img_disp = ax.imshow([[0]])
ax.set_xticks([]); ax.set_yticks([])
plt.tight_layout()

print("[等待] 相机数据... (关闭窗口退出)")
fc = 0
try:
    while plt.fignum_exists(fig.number):
        rclpy.spin_once(node, timeout_sec=0.01)
        with lock:
            f, frame[0] = frame[0], None
        if f is None:
            plt.pause(0.03)
            continue
        if fc == 0:
            print(f"[首帧] {f.shape}, range=[{f.min()},{f.max()}], mean={f.mean():.0f}")
            cv2.imwrite("/tmp/yolo_debug_frame.jpg", cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
            print("[调试] 已保存 /tmp/yolo_debug_frame.jpg")

        results = model(f, conf=args.conf, verbose=False)
        ann = cv2.cvtColor(results[0].plot(), cv2.COLOR_BGR2RGB)
        n = len(results[0].boxes)
        t_ms = results[0].speed.get("inference", 0)

        img_disp.set_data(ann)
        ax.set_title(f"Detections: {n} | {t_ms:.0f}ms | F{fc}", fontsize=11,
                     color='green' if n else 'red')
        fig.canvas.draw_idle()
        plt.pause(0.02)
        fc += 1
except KeyboardInterrupt:
    pass
finally:
    plt.close('all')
    node.destroy_node()
    try: rclpy.shutdown()
    except: pass
    print(f"\n[退出] {fc} 帧")
