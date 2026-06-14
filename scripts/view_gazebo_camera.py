#!/usr/bin/env python3
"""
Gazebo 方阵实时相机预览
=======================
启动 Gazebo 无头模式，订阅相机 topic 并用 OpenCV 实时显示。
不依赖 GUI 3D 视口渲染，WSL2 下完美工作。

用法:
    source /opt/ros/jazzy/setup.bash
    python3 scripts/view_gazebo_camera.py

    # 或指定世界文件
    python3 scripts/view_gazebo_camera.py --world grasping_world.sdf
"""

import os
import sys
import time
import signal
import subprocess
import argparse
import threading
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
WORLDS_DIR = PROJECT_ROOT / "ros2_ws" / "src" / "yolo_bringup" / "worlds"

# ---- 尝试导入 Gazebo Transport ----
try:
    from gz.msgs.image_pb2 import Image
    from gz.transport import SubscribeOptions
    from gz.transport import Node
    HAVE_GZ = True
except ImportError:
    HAVE_GZ = False

parser = argparse.ArgumentParser(description="Gazebo 方阵实时相机预览")
parser.add_argument("--world", default="block_grid_world.sdf",
                    help="世界文件名 (位于 ros2_ws/src/yolo_bringup/worlds/)")
parser.add_argument("--topic", default="/camera/color/image_raw",
                    help="相机 topic")
args = parser.parse_args()


def find_world(name):
    """查找世界文件"""
    path = WORLDS_DIR / name
    if path.exists():
        return str(path)
    if os.path.exists(name):
        return name
    print(f"[ERROR] 找不到世界文件: {name}")
    print(f"  搜索路径: {WORLDS_DIR}")
    sys.exit(1)


def start_gazebo(world_path):
    """启动 Gazebo 无头模式"""
    cmd = ["gz", "sim", "-r", "-s", world_path]
    print(f"[启动] {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(4)
    return proc


def gazebo_native_viewer(world_path):
    """方案A: 用 Gazebo Transport 原生 SDK 订阅相机图像并用 OpenCV 显示"""
    import cv2
    import numpy as np

    gz_proc = start_gazebo(world_path)
    running = True

    def on_shutdown(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, on_shutdown)

    # 创建 Gazebo Transport 节点
    node = Node()
    latest_img = [None]
    lock = threading.Lock()

    def on_image(msg: Image):
        if msg.width > 0 and msg.height > 0:
            # 将 gz.msgs.Image 转为 numpy array
            data = np.frombuffer(msg.data, dtype=np.uint8)
            expected = msg.width * msg.height * 3  # RGB 3 channels
            if len(data) == expected:
                img = data.reshape((msg.height, msg.width, 3))
                # RGB -> BGR for OpenCV
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                with lock:
                    latest_img[0] = img

    # 订阅
    node.subscribe(args.topic, on_image, SubscribeOptions())

    print(f"[相机] 已订阅 {args.topic}")
    print(f"[操作] 按 Q 或 Ctrl+C 退出")
    print()

    try:
        while running:
            with lock:
                frame = latest_img[0]
            if frame is not None:
                # 添加文字标注
                h, w = frame.shape[:2]
                cv2.putText(frame, f"Gazebo Camera | {w}x{h} | Q=Quit",
                            (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow("Gazebo - 方阵俯瞰相机", frame)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:  # Q or ESC
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        gz_proc.terminate()
        gz_proc.wait()
        print("[退出] Gazebo 已停止")


def ros2_bridge_viewer(world_path):
    """方案B: 用 ROS2 subscriber 显示（需要 ros2 run + colcon build）"""
    import cv2
    import numpy as np

    gz_proc = start_gazebo(world_path)

    # 启动 ros_gz_bridge
    bridge_cmd = [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        f"{args.topic}@sensor_msgs/msg/Image@gz.msgs.Image"
    ]
    print(f"[Bridge] {' '.join(bridge_cmd)}")
    bridge_proc = subprocess.Popen(bridge_cmd, stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL)
    time.sleep(3)

    # ROS2 订阅
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    rclpy.init()
    ros_node = Node("camera_viewer")
    bridge = CvBridge()
    latest_frame = [None]
    lock = threading.Lock()

    def ros_callback(msg: Image):
        try:
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            with lock:
                latest_frame[0] = frame
        except Exception as e:
            pass

    ros_node.create_subscription(Image, args.topic, ros_callback, 10)

    print(f"[相机] 已订阅 ROS2 topic: {args.topic}")
    print(f"[操作] 按 Q 或 Ctrl+C 退出")

    running = True
    try:
        while running and rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.03)
            with lock:
                frame = latest_frame[0]
            if frame is not None:
                h, w = frame.shape[:2]
                cv2.putText(frame, f"ROS2 Bridge | {w}x{h} | Q=Quit",
                            (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow("Gazebo - 方阵俯瞰相机 (ROS2)", frame)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        ros_node.destroy_node()
        rclpy.shutdown()
        bridge_proc.terminate()
        gz_proc.terminate()
        gz_proc.wait()
        print("[退出] 所有进程已停止")


def main():
    world_path = find_world(args.world)
    print(f"世界文件: {world_path}")
    print()

    if HAVE_GZ:
        print("[模式] Gazebo Transport 原生模式 (无需 ROS2)")
        gazebo_native_viewer(world_path)
    else:
        print("[模式] ROS2 Bridge 模式")
        print("[提示] 请先 source /opt/ros/jazzy/setup.bash 并 conda activate yolo_ros")
        ros2_bridge_viewer(world_path)


if __name__ == "__main__":
    main()
