"""
2D→3D 投影与坐标变换节点 (perception_3d)

订阅:
  /detections             (vision_msgs/Detection2DArray)
  /camera/depth/image_raw (sensor_msgs/Image)
  /camera/color/camera_info (sensor_msgs/CameraInfo)

发布:
  /target_3d  (geometry_msgs/PoseStamped) — base_link 坐标系

工作流程:
  1. 从 YOLO 检测框中心取出像素坐标 (u, v)
  2. 在深度图中取邻域中位数作为 Z
  3. 针孔投影: P_camera = Z * K⁻¹ * [u, v, 1]ᵀ
  4. 通过 TF 查询 camera→base_link 变换
  5. 发布 base_link 下的 3D 坐标
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion

from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel

import tf2_ros
import tf2_geometry_msgs
import numpy as np


class Perception3D(Node):
    """2D 检测 → 3D 投影节点"""

    def __init__(self):
        super().__init__('perception_3d')

        # ---- 参数 ----
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('depth_median_window', 5)  # 邻域窗口大小

        self.target_frame = self.get_parameter('target_frame').value
        self.median_win = self.get_parameter('depth_median_window').value

        # ---- 相机模型 ----
        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False

        # ---- 深度图缓存 ----
        self.latest_depth = None
        self.bridge = CvBridge()

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- 发布者 ----
        self.target_pub = self.create_publisher(
            PoseStamped, '/target_3d', 10
        )

        # ---- 订阅者 ----
        qos_best_effort = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self.camera_info_callback, qos_best_effort
        )

        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw',
            self.depth_callback, qos_best_effort
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections',
            self.detection_callback, 10
        )

        self.get_logger().info('Perception3D node started.')

    def camera_info_callback(self, msg: CameraInfo):
        """接收相机内参，初始化 PinholeCameraModel。"""
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True
            self.get_logger().info(
                f'Camera info received. '
                f'fx={self.camera_model.fx():.1f}, '
                f'fy={self.camera_model.fy():.1f}, '
                f'cx={self.camera_model.cx():.1f}, '
                f'cy={self.camera_model.cy():.1f}'
            )

    def depth_callback(self, msg: Image):
        """缓存最新深度图。"""
        try:
            # 深度图编码可能是 16UC1 或 32FC1
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')

    def detection_callback(self, msg: Detection2DArray):
        """收到检测结果 → 投影到 3D → TF 变换 → 发布。"""
        if not self.camera_info_received:
            self.get_logger().warn('Camera info not yet received, skipping.')
            return

        if self.latest_depth is None:
            self.get_logger().warn('No depth image received yet, skipping.')
            return

        if len(msg.detections) == 0:
            return

        camera_frame = msg.header.frame_id  # e.g. camera_color_optical_frame
        stamp = msg.header.stamp

        for detection in msg.detections:
            # ---- Step 1: 提取检测框中心像素坐标 ----
            u = detection.bbox.center.x
            v = detection.bbox.center.y

            # ---- Step 2: 取深度邻域中位数 ----
            depth_val = self._get_median_depth(int(u), int(v))

            if depth_val is None or depth_val <= 0.0:
                self.get_logger().debug(
                    f'Invalid depth at ({u:.1f}, {v:.1f}), skipping.'
                )
                continue

            # ---- Step 3: 针孔投影 (u,v,Z) → (X_c, Y_c, Z_c) ----
            # X = (u - cx) * Z / fx
            # Y = (v - cy) * Z / fy
            x = (u - self.camera_model.cx()) * depth_val / self.camera_model.fx()
            y = (v - self.camera_model.cy()) * depth_val / self.camera_model.fy()
            z = depth_val

            # ---- Step 4: TF 变换 camera_frame → target_frame ----
            try:
                # 构建相机坐标系下的 Pose
                cam_pose = PoseStamped()
                cam_pose.header.stamp = stamp
                cam_pose.header.frame_id = camera_frame
                cam_pose.pose.position = Point(x=x, y=y, z=z)
                cam_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

                # 查询 TF 变换
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    camera_frame,
                    stamp,
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )

                # 变换
                base_pose = tf2_geometry_msgs.do_transform_pose(cam_pose, transform)
                base_pose.header.stamp = self.get_clock().now().to_msg()
                base_pose.header.frame_id = self.target_frame

                self.target_pub.publish(base_pose)

                class_name = ''
                if len(detection.results) > 0:
                    class_name = detection.results[0].hypothesis.class_id

                self.get_logger().info(
                    f'Target [{class_name}] projected: '
                    f'cam=({x:.3f},{y:.3f},{z:.3f}) → '
                    f'base=({base_pose.pose.position.x:.3f},'
                    f'{base_pose.pose.position.y:.3f},'
                    f'{base_pose.pose.position.z:.3f})'
                )

            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f'TF lookup failed: {e}')
                continue

    def _get_median_depth(self, u: int, v: int) -> float:
        """取深度图中 (u,v) 邻域的中位数，抗噪声。"""
        h, w = self.latest_depth.shape[:2]
        half = self.median_win // 2

        r_min = max(0, v - half)
        r_max = min(h, v + half + 1)
        c_min = max(0, u - half)
        c_max = min(w, u + half + 1)

        patch = self.latest_depth[r_min:r_max, c_min:c_max]
        # 过滤掉 0 和 NaN
        valid = patch[(patch > 0) & (~np.isnan(patch))]

        if len(valid) == 0:
            return None

        return float(np.median(valid))


def main(args=None):
    rclpy.init(args=args)
    node = Perception3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
