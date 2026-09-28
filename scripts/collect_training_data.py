#!/usr/bin/env python3
"""
训练数据采集脚本
================
互动式 Gazebo 截图工具：在仿真中摆放方块，按键采集图像。

用法:
    # 使用本项目的相机话题
    python3 scripts/collect_training_data.py

    # 使用参照项目的相机话题
    python3 scripts/collect_training_data.py --topic /image_raw

操作:
    Space  — 保存当前帧
    S      — 连续模式: 每 0.5 秒自动保存（再按一次停止）
    Q/ESC  — 退出

输出: block_dataset/train/images/img_0000.jpg ...
"""

import os
import sys
import time
import threading
import cv2
import argparse

# ---- 路径设置 ----
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "block_dataset", "train", "images")

# ---- 命令行参数 ----
parser = argparse.ArgumentParser(description="YOLO 训练数据采集工具")
parser.add_argument("--topic", default="/camera/color/image_raw",
                    help="相机 ROS2 topic（默认: /camera/color/image_raw）")
args = parser.parse_args()

# ---- 检测 ROS2 可用性 ----
USE_ROS = True
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except ImportError:
    USE_ROS = False


# ============================================================
# ROS2 模式：独立线程订阅 Gazebo 相机
# ============================================================
class ImageCollector(Node):
    def __init__(self):
        super().__init__("image_collector")
        self.bridge = CvBridge()
        self.latest_frame = None
        self._lock = threading.Lock()
        self._running = True

        topic = args.topic
        self.sub = self.create_subscription(
            Image, topic, self.callback, 10
        )
        self.get_logger().info(f"订阅中: {topic}")

    def callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self.latest_frame = frame
        except Exception:
            pass

    def get_frame(self):
        with self._lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def stop(self):
        self._running = False


def _ros_spin(node: ImageCollector):
    """ROS2 独立 spin 线程，避免阻塞 OpenCV。"""
    while node._running and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)


# ============================================================
# OpenCV 模式：使用本地摄像头（无 ROS2 时回退）
# ============================================================
class WebcamCollector:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("无法打开摄像头")
        self._lock = threading.Lock()
        self.latest_frame = None

    def get_frame(self):
        ret, frame = self.cap.read()
        return frame if ret else None

    def stop(self):
        self.cap.release()


# ============================================================
# 采集主循环
# ============================================================
def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # 确定起始编号
    existing = [f for f in os.listdir(OUTPUT_DIR) if f.endswith(".jpg")]
    count = len(existing)

    # 初始化采集器
    spin_thread = None
    if USE_ROS:
        rclpy.init(args=None)
        collector = ImageCollector()
        spin_thread = threading.Thread(target=_ros_spin, args=(collector,), daemon=True)
        spin_thread.start()
    else:
        print("[WARN] ROS2 不可用，使用本地摄像头。")
        print("       如需从 Gazebo 采集，请先 source ROS2 环境。")
        collector = WebcamCollector()

    print(f"""
╔══════════════════════════════════════════╗
║      YOLO 训练数据采集工具               ║
╠══════════════════════════════════════════╣
║  相机话题: {args.topic}
║  输出目录: {OUTPUT_DIR}
║  已有图像: {count} 张
║  推荐采集: 200-500 张                     ║
╠══════════════════════════════════════════╣
║  [Space] 保存当前帧                      ║
║  [S]     连续自动采集（0.5s/张）         ║
║  [Q/ESC] 退出                            ║
╚══════════════════════════════════════════╝
""")

    auto_mode = False
    last_capture = 0.0
    stall_count = 0
    window_name = "Collect"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    try:
        while True:
            frame = collector.get_frame()

            if frame is None:
                # 无帧：等待，但允许退出
                stall_count += 1
                key = cv2.waitKey(50) & 0xFF
                if key == ord("q") or key == ord("Q") or key == 27:
                    break
                if stall_count % 20 == 0:
                    print(f"  ⏳ 等待相机数据... (已等 {stall_count * 0.05:.0f}s, 按 Q 退出)")
                continue

            stall_count = 0

            # 显示
            display = frame.copy()
            status = f"Saved: {count} | {'AUTO' if auto_mode else 'MANUAL'}"
            color = (0, 255, 0) if auto_mode else (255, 255, 255)
            cv2.putText(display, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            try:
                cv2.imshow(window_name, display)
            except cv2.error:
                # 窗口被关闭
                print("\n窗口已关闭，退出。")
                break

            key = cv2.waitKey(10) & 0xFF

            # 自动模式
            if auto_mode and time.time() - last_capture > 0.5:
                key = ord(" ")

            # 保存
            if key == ord(" "):
                filename = f"img_{count:04d}.jpg"
                filepath = os.path.join(OUTPUT_DIR, filename)
                cv2.imwrite(filepath, frame)
                count += 1
                last_capture = time.time()
                print(f"  ✅ [{count}] {filename}")

            # 切换自动模式
            elif key == ord("s") or key == ord("S"):
                auto_mode = not auto_mode
                print(f"  {'🔄 连续采集' if auto_mode else '⏸ 手动模式'}")

            # 退出
            elif key == ord("q") or key == ord("Q") or key == 27:
                break

    except KeyboardInterrupt:
        pass
    finally:
        print("\n清理中...")
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        collector.stop()
        if USE_ROS:
            # ⚠️ 收工清理要扛得住**第二次 SIGINT**。`ros2 launch` 收到 SIGINT 后会把
            # SIGINT **转发**给每个子进程（日志里的 `sending signal 'SIGINT' to
            # process[...]`），加上终端/进程组那一次，子进程一共收到两次。实测第二次
            # 落在 `destroy_node()` 里面（rclpy 的 `client.destroy_when_not_in_use`），
            # 抛 `KeyboardInterrupt` 从 finally 逃出去 ⇒ 一段 traceback ＋
            # `process has died ... exit code -2`。2026-09-28 板端：`set -m` ＋ 整组
            # SIGINT 可稳定复现（pty 真 Ctrl-C 那次反倒是干净的 —— 这个面只在
            # 「launch 转发得比子进程退得早」的时候露头）。
            # 收工阶段没有「失败要处理」这回事，信号更不是错误 ⇒ 只吞
            # KeyboardInterrupt，别的异常照旧让它现形。
            try:
                collector.destroy_node()
            except KeyboardInterrupt:
                pass
            # 已关过就别再关：`rclpy.init()` 装的信号处理器在 SIGINT 时
            # 已经调过一次 `rcl_shutdown`，再无条件调一次会抛
            #     RCLError: failed to shutdown: rcl_shutdown already called
            # —— 一个「正常退出」于是看起来像崩了。采集是本仓库在**真机**上
            # 要用的脚本（W4），别让它每次都留一段 traceback。
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except KeyboardInterrupt:
                    pass
        if spin_thread:
            spin_thread.join(timeout=1.0)

    print(f"\n采集完成。共 {count} 张图像保存在:")
    print(f"  {OUTPUT_DIR}")
    print(f"\n下一步: python3 scripts/label_blocks.py 标注 → train_yolo_block.py 训练")


if __name__ == "__main__":
    main()
