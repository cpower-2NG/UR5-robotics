#!/usr/bin/env python3
"""
测试图像发布节点 — 模拟 Gazebo 相机输出，用于不依赖仿真的 YOLO 流水线验证。

发布:
  /camera/color/image_raw   (sensor_msgs/Image) — 包含彩色方块/圆柱的测试图像
  /camera/depth/image_raw   (sensor_msgs/Image) — 模拟深度图
  /camera/color/camera_info (sensor_msgs/CameraInfo) — 模拟 RealSense D435 内参
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2


class TestImagePublisher(Node):
    """发布测试图像和相机内参，模拟 Gazebo RGBD 相机。"""

    def __init__(self):
        super().__init__('test_image_publisher')

        self.declare_parameter('rate', 10.0)
        rate = self.get_parameter('rate').value

        self.bridge = CvBridge()
        self.frame_id = 0

        # ---- 发布者 ----
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)

        # ---- 相机内参（模拟 RealSense D435） ----
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = 'camera_color_optical_frame'
        self.camera_info_msg.height = 480
        self.camera_info_msg.width = 640
        self.camera_info_msg.distortion_model = 'plumb_bob'
        # D435 典型内参 (640x480)
        self.camera_info_msg.k = [
            610.0, 0.0, 320.0,   # fx,  0, cx
            0.0, 610.0, 240.0,   #  0, fy, cy
            0.0,   0.0,   1.0,
        ]
        self.camera_info_msg.p = [
            610.0, 0.0, 320.0, 0.0,
            0.0, 610.0, 240.0, 0.0,
            0.0,   0.0,   1.0, 0.0,
        ]

        # ---- 定时器 ----
        self.timer = self.create_timer(1.0 / rate, self.publish_frame)

        self.get_logger().info(f'Test image publisher started at {rate} Hz')

    def publish_frame(self):
        """生成测试图像并发布。"""
        stamp = self.get_clock().now().to_msg()
        self.frame_id += 1

        # 生成 RGB 测试图像：灰色背景 + 彩色方块
        rgb = self._generate_test_image()
        depth = self._generate_depth_image()

        # 发布 RGB
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = 'camera_color_optical_frame'
        self.rgb_pub.publish(rgb_msg)

        # 发布深度图（32FC1, 单位: 米）
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = 'camera_color_optical_frame'
        self.depth_pub.publish(depth_msg)

        # 发布相机内参
        self.camera_info_msg.header.stamp = stamp
        self.info_pub.publish(self.camera_info_msg)

        self.get_logger().debug(f'Frame {self.frame_id} published')

    def _generate_test_image(self):
        """
        生成测试图像：灰色背景上绘制彩色方块和圆柱。
        模拟俯视 Gazebo 场景的视角。
        """
        h, w = 480, 640
        img = np.ones((h, w, 3), dtype=np.uint8) * 180  # 灰色背景

        # 红色方块（对应 SDF 中 red_cube 位置映射到图像）
        cv2.rectangle(img, (280, 200), (340, 260), (0, 0, 220), -1)
        cv2.putText(img, 'red cube', (280, 190),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

        # 蓝色方块
        cv2.rectangle(img, (380, 270), (440, 330), (200, 0, 0), -1)
        cv2.putText(img, 'blue cube', (380, 260),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # 绿色圆柱
        cv2.circle(img, (230, 230), 30, (0, 180, 0), -1)
        cv2.putText(img, 'green cylinder', (200, 210),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

        # 添加网格背景帮助可视化
        for i in range(0, w, 80):
            cv2.line(img, (i, 0), (i, h), (200, 200, 200), 1)
        for i in range(0, h, 60):
            cv2.line(img, (0, i), (w, i), (200, 200, 200), 1)

        return img

    def _generate_depth_image(self):
        """
        生成模拟深度图。
        假设相机在场景上方 1.5m 处向下看，物体在地面 (z=0) 处。
        物体区域深度约 1.5m，其他区域约 3m。
        """
        h, w = 480, 640
        # 基础深度 ~3m（地面以下，或远处）
        depth = np.ones((h, w), dtype=np.float32) * 3.0

        # 红色方块区域深度 ~1.45m（方块高 5cm）
        depth[195:265, 275:345] = 1.45

        # 蓝色方块区域
        depth[265:335, 375:445] = 1.45

        # 绿色圆柱区域
        cy, cx = 230, 230
        y, x = np.ogrid[:h, :w]
        mask = (x - cx)**2 + (y - cy)**2 < 32**2
        depth[mask] = 1.45

        return depth


def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 下 `spin()` 抛的是 `ExternalShutdownException`，不是
        # `KeyboardInterrupt`（rclpy 自己的信号处理器先关了 context）。
        pass
    except Exception:
        # D70 的第二个面：SIGINT 落在**定时器回调执行到 `publish` 那一行**时，
        # rclpy 会把回调里的异常从 `spin_once` 重新抛出
        # （`RCLError: Failed to publish: publisher's context is invalid`），
        # 它绕过上面那个 except ⇒ 「正常收工」报成 `process has died`。
        # 用 `rclpy.ok()` 而不是异常类型来判：rclpy 没有公开的 `RCLError` 别名。
        # context 还活着就照抛，别把真正的发布失败一起藏起来。
        if rclpy.ok():
            raise
    finally:
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
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        # 已关过就别再关：无条件调会抛 RCLError，把「正常收工」变成
        # `process has died ... exit code 1`。理由与闸门写法同
        # `detector_node.py` / `projector_node.py`。
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
