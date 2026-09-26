"""
2D→3D 投影与坐标变换节点 (perception_3d)

订阅:
  /detections             (vision_msgs/Detection2DArray)
  /camera/depth/image_raw (sensor_msgs/Image)
  /camera/color/camera_info (sensor_msgs/CameraInfo)

发布:
  /target_3d  (geometry_msgs/PoseStamped) — target_frame 坐标系（默认 base_link）

工作流程:
  1. 深度图按 encoding 换算成**米**后存入带时间戳的环形缓冲
  2. 收到检测 → 按时间戳就近配对深度帧（超过 max_sync_dt 则丢弃并告警）
  3. 在检测框内多区域采样，取**最近簇**的中位数作为 Z（抗背景平面干扰）
  4. 针孔投影: P_camera = Z * K⁻¹ * [u, v, 1]ᵀ
  5. TF: camera_frame → target_frame
  6. 发布 target_frame 下的 3D 坐标

输入约定（**依赖 yolo_detector 冻结的像素约定**，见 P2-5 / D7）:
  ``/detections`` 的 ``bbox`` 必须是**像素**坐标、原点在图像左上角；
  本节点把 ``center.position.x/y`` 直接当作像素 (u, v) 使用，并与
  ``camera_info`` 的 fx/fy/cx/cy 配套。**若上游改成归一化坐标，这里会静默算错**
  ——这正是 D7 记录过的陷阱。``size_x``/``size_y``（像素）用于框内采样；
  若上游未填（≤1），本节点退回单点邻域并在需要时告警。

本文件针对真机（RealSense D435i）修了四处会让链路直接失败的问题，
每条都在下面各自的代码处以「D1/D2/D8/D33」标注了原因，详见
docs/RESTRUCTURE_PLAN.md。
"""

import math
from collections import deque

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


# 深度编码 → 「像素值 × 系数 = 米」。
# D1: 原实现把深度值**直接当成米**用，且不判断编码。仿真里深度是 32FC1（米），
#     所以蒙对了；而真机 D435i 的 16UC1 是**毫米**，直接用会放大 1000 倍。
#     这里按编码换算，两种后端都正确。
DEPTH_SCALE_BY_ENCODING = {
    '16UC1': 0.001,   # RealSense D435i 默认深度：毫米
    'mono16': 0.001,
    '32FC1': 1.0,     # Gazebo / ros_gz 仿真深度：米
    '64FC1': 1.0,
}


class Perception3D(Node):
    """2D 检测 → 3D 投影节点"""

    def __init__(self):
        super().__init__('perception_3d')

        # ---- 参数 ----
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('depth_median_window', 5)   # 每个采样点的邻域窗口
        self.declare_parameter('depth_grid', 5)            # 框内采样网格边长（5 → 5×5）
        self.declare_parameter('depth_cluster_tol', 0.03)  # 最近簇容差（米）
        # -1 = 按 msg.encoding 自动判定；显式给值则强制覆盖（单位：米/像素）
        self.declare_parameter('depth_scale', -1.0)
        self.declare_parameter('max_sync_dt', 0.1)         # 检测↔深度 最大时间差（秒）
        self.declare_parameter('depth_buffer_len', 10)     # 深度帧环形缓冲长度
        self.declare_parameter('tf_required', False)       # True = 查不到 TF 就退出

        # 话题名做成参数（P2-7）。原因很具体：真机上 realsense2_camera 打开
        # align_depth.enable:=true 后，**对齐后的深度发布在
        # /camera/aligned_depth_to_color/image_raw**，而 /camera/depth/image_raw
        # 保持**未对齐**。若把话题写死成后者，真机会静默用未对齐深度——
        # 正是 D2 警告的视差偏差。仿真侧由 ros_gz_bridge 直接发到
        # /camera/depth/image_raw，故默认值保持向后兼容。
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('detections_topic', '/detections')

        self.target_frame = self.get_parameter('target_frame').value
        self.median_win = self.get_parameter('depth_median_window').value
        self.depth_grid = max(1, self.get_parameter('depth_grid').value)
        self.cluster_tol = self.get_parameter('depth_cluster_tol').value
        self.depth_scale_param = self.get_parameter('depth_scale').value
        self.max_sync_dt = self.get_parameter('max_sync_dt').value
        self.tf_required = self.get_parameter('tf_required').value

        # ---- 相机模型 ----
        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False

        # ---- 深度缓冲：[(stamp_sec, depth_in_metres), ...] ----
        # D2: 原实现只存「最新一帧」深度，与检测之间没有任何时间关联；
        #     改为带时间戳的环形缓冲，按 stamp 就近配对。
        self.depth_buffer = deque(maxlen=self.get_parameter('depth_buffer_len').value)
        self.depth_scale = None          # 首次收到深度图时确定
        self.bridge = CvBridge()
        self._warned_normalized = False  # D7 护栏：归一化坐标只提示一次

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_fail_streak = 0          # D8: 连续失败计数

        # ---- 发布者 ----
        self.target_pub = self.create_publisher(PoseStamped, '/target_3d', 10)

        # ---- 订阅者 ----
        qos_best_effort = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.camera_info_callback, qos_best_effort
        )

        self.depth_sub = self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, qos_best_effort
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray, self.get_parameter('detections_topic').value,
            self.detection_callback, 10
        )

        if self.depth_scale_param > 0.0:
            self.depth_scale = float(self.depth_scale_param)
            self.get_logger().info(
                f'深度尺度被参数强制指定为 {self.depth_scale} m/px'
                f'（自动判定已禁用）'
            )

        self.get_logger().info(
            f'Perception3D node started. target_frame={self.target_frame}, '
            f'max_sync_dt={self.max_sync_dt}s\n'
            f'  订阅: {self.get_parameter("detections_topic").value}, '
            f'{self.get_parameter("depth_topic").value}, '
            f'{self.get_parameter("camera_info_topic").value}'
        )

        # D2: 深度必须与彩色对齐，否则用彩色内参配深度像素会有视差偏差。
        # 真机上「对齐」不是一句提醒就能保证的——realsense2_camera 开启
        # align_depth.enable:=true 后，对齐深度发在**另一个话题**上，所以这里
        # 既提示参数、也把当前实际订阅的话题打出来，便于当场核对。
        depth_topic = self.get_parameter('depth_topic').value
        if 'aligned' not in depth_topic:
            self.get_logger().info(
                f'提示：当前深度话题为 {depth_topic}。真机上请确认它是**与彩色对齐**的'
                f'（realsense2_camera 开 align_depth.enable:=true 时，对齐深度在 '
                f'/camera/aligned_depth_to_color/image_raw；/camera/depth/image_raw '
                f'是**未对齐**的）。用未对齐深度配彩色内参会有视差偏差。'
                f'仿真侧 ros_gz_bridge 直接发到 /camera/depth/image_raw，属正常。'
            )

    # ------------------------------------------------------------------
    # 相机内参
    # ------------------------------------------------------------------
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
                f'cy={self.camera_model.cy():.1f}, '
                f'frame={msg.header.frame_id}'
            )

    # ------------------------------------------------------------------
    # 深度
    # ------------------------------------------------------------------
    def _resolve_depth_scale(self, encoding: str):
        """按编码确定「像素值 → 米」的系数（D1）。"""
        if self.depth_scale is not None:
            return self.depth_scale

        if encoding in DEPTH_SCALE_BY_ENCODING:
            self.depth_scale = DEPTH_SCALE_BY_ENCODING[encoding]
            self.get_logger().info(
                f'深度编码 {encoding} → 尺度 {self.depth_scale} m/px'
                f'（{"毫米" if self.depth_scale == 0.001 else "米"}）'
            )
        else:
            self.get_logger().error(
                f'未知深度编码 {encoding!r}，无法确定尺度。'
                f'请用 depth_scale 参数显式指定（单位：米/像素）。'
                f'已支持：{sorted(DEPTH_SCALE_BY_ENCODING)}。'
                f'本帧按「米」处理，结果可能相差 1000 倍。'
            )
            self.depth_scale = 1.0
        return self.depth_scale

    def depth_callback(self, msg: Image):
        """把深度图换算成米后连同时间戳一起入缓冲。"""
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')
            return

        scale = self._resolve_depth_scale(msg.encoding)
        # 统一在入口换算成 float32 的米，下游不必再关心编码。
        depth_m = raw.astype(np.float32) * scale

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.depth_buffer.append((stamp, depth_m))

    def _nearest_depth(self, stamp_sec: float):
        """取与给定时间戳最接近的深度帧（D2）。

        返回 (depth_m, dt)；缓冲为空时返回 (None, None)。
        超过 max_sync_dt 由调用方决定是否丢弃。
        """
        if not self.depth_buffer:
            return None, None
        best = min(self.depth_buffer, key=lambda item: abs(item[0] - stamp_sec))
        return best[1], abs(best[0] - stamp_sec)

    def _depth_at(self, depth, u: float, v: float):
        """取 (u,v) 邻域内的有效深度中位数（米），无有效值返回 None。"""
        h, w = depth.shape[:2]
        half = self.median_win // 2

        r_min, r_max = max(0, int(v) - half), min(h, int(v) + half + 1)
        c_min, c_max = max(0, int(u) - half), min(w, int(u) + half + 1)
        if r_max <= r_min or c_max <= c_min:
            return None

        patch = depth[r_min:r_max, c_min:c_max]
        valid = patch[(patch > 0) & np.isfinite(patch)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _robust_depth(self, depth, u: float, v: float, size_x: float, size_y: float):
        """在检测框内多区域采样，取**最近簇**的中位数（P2-6）。

        俯视场景里物体顶面一定比它身后的桌面更近，所以「最近的一簇」
        就是物体本身；这样能剔除背景平面点，比单点邻域稳得多。
        """
        samples = []

        if size_x > 1.0 and size_y > 1.0:
            # 在框内 ±60% 半宽内均匀取样，避开框边缘（边缘常混入背景）
            half_x, half_y = size_x / 2.0 * 0.6, size_y / 2.0 * 0.6
            n = self.depth_grid
            for du in np.linspace(-half_x, half_x, n):
                for dv in np.linspace(-half_y, half_y, n):
                    d = self._depth_at(depth, u + du, v + dv)
                    if d is not None:
                        samples.append(d)
        else:
            # 框尺寸缺失（上游没填 size_x/size_y）→ 退回单点邻域
            d = self._depth_at(depth, u, v)
            if d is not None:
                samples.append(d)

        if not samples:
            return None

        vals = np.sort(np.asarray(samples))
        nearest = vals[vals <= vals[0] + self.cluster_tol]
        return float(np.median(nearest))

    # ------------------------------------------------------------------
    # TF
    # ------------------------------------------------------------------
    def _transform_pose(self, pose_stamped: PoseStamped, transform):
        """PoseStamped 的 TF 变换（D33）。

        D33: 原实现调用 `tf2_geometry_msgs.do_transform_pose(cam_pose, tf)`，
        但该函数在 Humble 里的签名是 `do_transform_pose(pose: Pose, ...)`——
        要的是 **Pose**，传 PoseStamped 会抛
        `AttributeError: 'PoseStamped' object has no attribute 'position'`。
        而外层的 except 只捕获 TF 三类异常，**AttributeError 会逃出回调**，
        于是**每次检测都报错、/target_3d 从未被发布过**。
        这里改用 `do_transform_pose_stamped`（Humble 已提供，专为 PoseStamped）。
        """
        if hasattr(tf2_geometry_msgs, 'do_transform_pose_stamped'):
            return tf2_geometry_msgs.do_transform_pose_stamped(pose_stamped, transform)
        # 兜底：老版本没有 *_stamped 时，自己拆出 Pose、再装回 header
        out = PoseStamped()
        out.pose = tf2_geometry_msgs.do_transform_pose(pose_stamped.pose, transform)
        out.header = pose_stamped.header
        return out

    # ------------------------------------------------------------------
    # 主回调
    # ------------------------------------------------------------------
    def detection_callback(self, msg: Detection2DArray):
        """收到检测结果 → 投影到 3D → TF 变换 → 发布。"""
        if not self.camera_info_received:
            self.get_logger().warn(
                'Camera info not yet received, skipping.', throttle_duration_sec=5.0
            )
            return

        if len(msg.detections) == 0:
            return

        camera_frame = msg.header.frame_id  # e.g. camera_color_optical_frame
        stamp = msg.header.stamp
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9

        # ---- 配对深度帧（D2）----
        depth_m, dt = self._nearest_depth(stamp_sec)
        if depth_m is None:
            self.get_logger().warn(
                'No depth image received yet, skipping.', throttle_duration_sec=5.0
            )
            return

        if stamp_sec > 0.0 and dt > self.max_sync_dt:
            # 深度与检测差得太远：宁可不发，也不要发出一个基于错误时刻深度的坐标
            self.get_logger().warn(
                f'深度与检测时间差 {dt*1000:.0f} ms 超过 max_sync_dt'
                f'（{self.max_sync_dt*1000:.0f} ms），丢弃本批检测。',
                throttle_duration_sec=5.0
            )
            return

        for detection in msg.detections:
            # ---- Step 1: 提取检测框 ----
            u = detection.bbox.center.position.x
            v = detection.bbox.center.position.y
            size_x = detection.bbox.size_x
            size_y = detection.bbox.size_y

            # D7 运行时护栏：上游若误发**归一化**坐标，投影会静默算错
            # （x/y 全落在 0~1，深度取到图像左上角而非目标）。历史上真有过
            # 一份这样的实现（已删的 yolo_vision），接口一样、语义不同。
            # 这里只提示一次，避免刷屏。
            if (not self._warned_normalized
                    and 0.0 < u < 1.0 and 0.0 < v < 1.0
                    and 0.0 < size_x <= 1.0 and 0.0 < size_y <= 1.0):
                self._warned_normalized = True
                self.get_logger().error(
                    f'检测框看起来是**归一化**坐标（center=({u:.3f},{v:.3f}), '
                    f'size=({size_x:.3f},{size_y:.3f})），而本节点按**像素**处理。'
                    f'上游若确实发了归一化坐标，3D 投影会静默算错（D7）。'
                    f'请确认 /detections 的发布者遵守像素约定。'
                )

            # ---- Step 2: 取稳健深度（最近簇中位数）----
            depth_val = self._robust_depth(depth_m, u, v, size_x, size_y)

            # 注意 `not math.isfinite` 这一条：NaN 比较恒为 False，所以单写
            # `depth_val <= 0.0` 是拦不住 NaN 的，NaN 会一路投影成 NaN 位姿发布
            # 出去。目前 `_robust_depth()` 的采样已用 np.isfinite 过滤，**不会**
            # 返回 NaN（已实测），这条属于发布边界的兜底：真机上下游拿到 NaN
            # 目标位姿，轻则 MoveIt 报错，重则规划出一条 NaN 轨迹。
            if depth_val is None or not math.isfinite(depth_val) or depth_val <= 0.0:
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
                cam_pose = PoseStamped()
                cam_pose.header.stamp = stamp
                cam_pose.header.frame_id = camera_frame
                cam_pose.pose.position = Point(x=x, y=y, z=z)
                cam_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    camera_frame,
                    stamp,
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )

                base_pose = self._transform_pose(cam_pose, transform)
                base_pose.header.stamp = self.get_clock().now().to_msg()
                base_pose.header.frame_id = self.target_frame

                self.target_pub.publish(base_pose)
                self.tf_fail_streak = 0

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
                # D8: 原实现只 warn 一句就 continue——真机上 TF 缺失时**静默失效**，
                #     表现为「什么都检测不到」，极难定位。改为 ERROR + 可操作提示，
                #     并用连续失败计数把问题顶到明面上。
                self.tf_fail_streak += 1
                self.get_logger().error(
                    f'TF 查询失败（连续 {self.tf_fail_streak} 次）：{e} | '
                    f'需要 {self.target_frame} → {camera_frame} 的变换。'
                    f'真机请先完成手眼标定并发布静态 TF；仿真下由 URDF/SDF 提供。',
                    throttle_duration_sec=5.0
                )
                if self.tf_required:
                    raise RuntimeError(
                        f'TF {self.target_frame} → {camera_frame} 不可用，'
                        f'tf_required=True 故直接失败'
                    ) from e
                continue


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
