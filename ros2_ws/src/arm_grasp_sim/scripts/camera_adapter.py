#!/usr/bin/env python3
"""把 Gazebo 相机输出适配成**真机同款**的 ROS 图像契约。

Gazebo 与真机的三处差异，全部在这一层抹平（算法代码不许知道自己在仿真里）：

1. **深度编码**：Gazebo depth_camera 出 ``32FC1``，单位米、0 表示无回波；
   真机 RealSense 出 ``16UC1``，单位**毫米**、0 表示无回波。
   这里做 m→mm 并截断到 65535，NaN/负值一律写 0。
2. **frame_id**：Gazebo 发 ``model::link::sensor``，这里改写成 TF 里存在的
   光学帧（x 右 / y 下 / z 前）。
3. **camera_info**：两路 sensor（彩色 + 深度）会把各自的信息挤到同一个
   Gazebo 话题上；这里用参数里的实测内参**重新发布**，保证与真机一致。

参数（默认值对应真机实测，见 arm_grasp_sim/launch/sim.launch.py）：
  in_color / in_depth / in_info : 输入话题
  out_color / out_depth / out_info : 输出话题
  optical_frame : 输出 header.frame_id
  width / height / fx / fy / cx / cy : 输出 CameraInfo 的内参
  depth_scale_mm : 深度缩放（真实相机 1000.0：米→毫米）
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

_HDR = "sensor_msgs/msg/Header"

_DTYPES = {
    "32FC1": np.dtype(np.float32),
    "16UC1": np.dtype(np.uint16),
    "8UC1": np.dtype(np.uint8),
    "rgb8": np.dtype(np.uint8),
    "bgr8": np.dtype(np.uint8),
    "rgba8": np.dtype(np.uint8),
}


class CameraAdapter(Node):
    def __init__(self):
        super().__init__("camera_adapter")

        # 每路相机一个实例；参数用前缀区分（wrist_, topcam_）
        self.declare_parameter("camera_name", "camera")
        name = self.get_parameter("camera_name").value
        pre = f"{name}_"
        self.declare_parameter(pre + "in_color", f"/{name}/color/image_raw")
        self.declare_parameter(pre + "in_depth", f"/{name}/depth/image_raw")
        self.declare_parameter(pre + "in_info", f"/{name}/color/camera_info")
        self.declare_parameter(pre + "out_color", f"/{name}/color/image_raw")
        self.declare_parameter(pre + "out_depth", f"/{name}/depth/image_raw")
        self.declare_parameter(pre + "out_info", f"/{name}/color/camera_info")
        self.declare_parameter(pre + "optical_frame", f"{name}_optical_frame")
        self.declare_parameter(pre + "width", 640)
        self.declare_parameter(pre + "height", 480)
        self.declare_parameter(pre + "fx", 554.383)
        self.declare_parameter(pre + "fy", 554.383)
        self.declare_parameter(pre + "cx", 320.0)
        self.declare_parameter(pre + "cy", 240.0)
        self.declare_parameter(pre + "depth_scale_mm", 1000.0)
        self.declare_parameter(pre + "publish_info", True)

        g = lambda k: self.get_parameter(pre + k).value  # noqa: E731
        self._in_color = g("in_color")
        self.frame = g("optical_frame")
        self.scale = float(g("depth_scale_mm"))
        self.publish_info = bool(g("publish_info"))

        self.info = CameraInfo()
        self.info.width = int(g("width"))
        self.info.height = int(g("height"))
        k = [float(g("fx")), 0.0, float(g("cx")),
             0.0, float(g("fy")), float(g("cy")),
             0.0, 0.0, 1.0]
        self.info.k = k
        # P 必须是 12 个元素：每行 K 后补一项（Tx/Ty/Tz=0）
        self.info.p = (k[0:3] + [0.0] +
                       k[3:6] + [0.0] +
                       k[6:9] + [0.0])
        self.info.distortion_model = "plumb_bob"
        self.info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.info.header.frame_id = self.frame

        self._n_color = 0
        self._n_depth = 0
        self._n_info = 0

        self.pub_color = self.create_publisher(Image, g("out_color"), qos_profile_sensor_data)
        self.pub_depth = self.create_publisher(Image, g("out_depth"), qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, g("out_info"), qos_profile_sensor_data)

        self.create_subscription(Image, g("in_color"), self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, g("in_depth"), self.on_depth, qos_profile_sensor_data)
        if g("in_info"):
            self.create_subscription(CameraInfo, g("in_info"), self.on_info,
                                     qos_profile_sensor_data)

        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"[{name}] {g('in_color')} | {g('in_depth')} → {g('out_depth')} "
            f"(16UC1 mm, frame={self.frame})")

    # ── 彩色：只改 frame_id ────────────────────────────────────────────
    def on_color(self, msg: Image):
        # in 与 out 同名会自激（自己订自己），启动时就该避免；这里兜底
        if self.pub_color.topic_name == self._in_color:
            return
        out = Image()
        out.header = msg.header
        out.header.frame_id = self.frame
        out.height, out.width = msg.height, msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = msg.data
        self.pub_color.publish(out)
        self._n_color += 1
        if self.publish_info:
            # 与真机一致：camera_info 与彩色帧同戳、同帧号
            self.info.header.stamp = msg.header.stamp
            self.pub_info.publish(self.info)

    # ── 深度：32FC1 m → 16UC1 mm ───────────────────────────────────────
    def on_depth(self, msg: Image):
        dt = _DTYPES.get(msg.encoding)
        if dt is None:
            self.get_logger().warn(f"未知深度编码 {msg.encoding}，原样转发")
            out = msg
        else:
            arr = np.frombuffer(bytes(msg.data), dtype=dt)
            n = msg.height * msg.width
            if arr.size < n:
                self.get_logger().warn("深度数据不足一帧，丢弃")
                return
            a = arr[:n].astype(np.float32)
            if msg.encoding == "32FC1":
                a = np.nan_to_num(a, nan=0.0, posinf=0.0, neginf=0.0)
                a = np.clip(a, 0.0, 65.535) * self.scale
            elif msg.encoding == "16UC1":
                pass
            out = Image()
            out.height, out.width = msg.height, msg.width
            out.encoding = "16UC1"
            out.step = msg.width * 2
            out.data = a.astype(np.uint16).tobytes()
        out.header = msg.header
        out.header.frame_id = self.frame
        out.is_bigendian = 0
        self.pub_depth.publish(out)
        self._n_depth += 1

    def on_info(self, msg: CameraInfo):
        # Gazebo 的 info 只用来看「有没有出帧」；实际发布的是参数里的实测内参，
        # 因为彩色/深度两路 sensor 的 info 会串到同一个话题上。
        self._n_info += 1

    def report(self):
        if self._n_color or self._n_depth or self._n_info:
            self.get_logger().info(
                f"color={self._n_color} depth={self._n_depth} info={self._n_info}")


def main():
    rclpy.init()
    n = CameraAdapter()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
