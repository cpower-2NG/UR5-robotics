"""
YOLOv8 目标检测 ROS2 节点 (yolo_detector)

订阅:  /camera/color/image_raw  (sensor_msgs/Image)
发布:  /detections              (vision_msgs/Detection2DArray)
       /detections_image        (sensor_msgs/Image, 标注可视化)

通过 ultralytics YOLOv8 进行推理，输出 2D 检测框。

======================================================================
坐标约定（**冻结**，改动即破坏下游，见 P2-5 / D7）
======================================================================
本节点发布的 ``Detection2D.bbox`` 一律使用 **像素坐标**，原点在图像**左上角**：

  bbox.center.position.x = cx   像素，向右为正
  bbox.center.position.y = cy   像素，向下为正
  bbox.size_x             = w    像素宽度（x2 - x1）
  bbox.size_y             = h    像素高度（y2 - y1）

``center`` 是 ``Pose2D``，其中 **``theta`` 保持未使用**（朝向见 D27：当前物体集
全部朝向对称，不需要）。**没有归一化坐标**——不要假设值落在 [0,1]。

为何要写死：仓库历史上存在另一份实现（已删除的教学包 ``yolo_vision``）
发布的是**归一化** bbox，两者接口长得一样、语义不同，混用则 3D 投影全错，
且**不会报错**（D7）。现在只剩这一份实现，故把约定冻结在此处。

下游依赖此约定的地方：
  - ``perception_3d/projector_node.py``：把 center 当作像素，配合 camera_info
    的 fx/fy/cx/cy 做针孔投影。若这里改成归一化，那里会静默算错。
  - ``/detections_image``：把 bbox 直接画回图像像素坐标系。
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge

import numpy as np
import os
import time
from pathlib import Path


def resolve_device(requested: str, allow_fallback: bool, logger) -> str:
    """把 `device` 参数解析成**当前环境真正可用**的设备（P2-4）。

    旧实现直接 `self.model.to(device)`：在没有 CUDA 的机器上（或 GPU 被占用、
    驱动/CUDA 版本不匹配时）会抛异常并让**整个节点崩溃**。真机部署时这类环境
    差异很常见——「因为显卡不可用所以感知整个不启动」不是想要的失败模式；
    退到 CPU 慢一点但链路是通的，而且**明确告警**，问题不会被藏起来。

    行为：
      - ``cpu``                 → 原样返回
      - ``cuda`` / ``cuda:N``   → 检查 torch.cuda 是否可用、序号是否存在
      - 不可用 + allow_fallback → 告警并返回 ``cpu``
      - 不可用 + 不允许 fallback → 抛 RuntimeError（真机上用来暴露环境问题）

    刻意做成模块级函数（而非 Node 方法），这样**不依赖 torch/ultralytics**
    也能单独测试，见 scripts/test_detector_device.py。
    """
    req = (requested or 'cpu').strip()

    if req == 'cpu':
        return 'cpu'

    if not req.startswith('cuda'):
        logger.warn(
            f'无法识别的 device={req!r}（期望 "cpu" 或 "cuda:N"），按 cpu 处理。'
        )
        return 'cpu'

    # 解析序号：'cuda' → 0，'cuda:1' → 1
    index = 0
    if ':' in req:
        _, _, idx = req.partition(':')
        if not idx.isdigit():
            logger.warn(f'device={req!r} 的序号不是整数，按 cuda:0 处理。')
        else:
            index = int(idx)

    try:
        import torch
    except ImportError:
        reason = 'torch 未安装'
    else:
        if not torch.cuda.is_available():
            reason = 'torch.cuda.is_available() 为 False（无 GPU，或驱动/CUDA 版本不匹配）'
        elif index >= torch.cuda.device_count():
            reason = f'cuda:{index} 不存在（本机可见 GPU 数为 {torch.cuda.device_count()}）'
        else:
            logger.info(f'使用 GPU cuda:{index} — {torch.cuda.get_device_name(index)}')
            return f'cuda:{index}'

    if not allow_fallback:
        raise RuntimeError(
            f'device={req!r} 不可用：{reason}。（device_fallback=false，故不退回 cpu）'
        )

    logger.warn(
        f'device={req!r} 不可用：{reason}。**已退回 cpu**——功能正常，但推理会慢很多。'
        f'Orin 上若出现这条警告，通常说明 JetPack 的 CUDA/torch 不匹配，值得排查。'
    )
    return 'cpu'


class YoloDetector(Node):
    """YOLOv8 检测节点：订阅 RGB 图像，推理后发布 2D 检测框。"""

    def __init__(self):
        super().__init__('yolo_detector')

        # ---- 参数 ----
        self.declare_parameter('model', 'block_best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cuda:0')  # 'cpu' / 'cuda:0'
        # 请求的 device 不可用时是否退回 cpu，而不是直接崩溃（P2-4）。
        # 置 False 可强制「要么用 GPU，要么失败」，便于在真机上暴露环境问题。
        self.declare_parameter('device_fallback', True)
        self.declare_parameter('publish_annotated', True)

        raw_model = self.get_parameter('model').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        device = self._resolve_device(
            self.get_parameter('device').value,
            self.get_parameter('device_fallback').value,
        )

        # ---- 解析模型路径（D6 / D24）----
        # 旧实现把参数原样交给 YOLO()，于是行为随进程 CWD 改变；而且当两处
        # 都找不到时 ultralytics 会**静默联网下载官方 COCO 模型**——真机上
        # 表现为「什么都检测不到，且不报错」。这里改为显式查找，找不到就
        # 直接报错退出，绝不允许联网兜底。
        model_path = self._resolve_model(raw_model)
        if model_path is None:
            self.get_logger().error(
                f'找不到模型文件 {raw_model!r}。已搜索：CWD 及其所有父目录、'
                f'$YOLO_MODEL_DIR、yolo_detector/models/。'
                f'请改用绝对路径，例如 -p model:=$HOME/yolo_ros/block_best.pt'
            )
            raise FileNotFoundError(f'YOLO model not found: {raw_model}')

        # ---- 加载 YOLOv8 模型 ----
        self.get_logger().info(f'Loading YOLO model: {model_path} on {device}')
        try:
            from ultralytics import YOLO
            self.model = YOLO(str(model_path))
            self.model.to(device)
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise

        # 类别自检：方块检测器应为 1 类 block。用错模型是 D6 的原始症状，
        # 这里让它至少**可见**。
        names = list(self.model.names.values())
        self.get_logger().info(
            f'Model loaded: {len(names)} class(es) — {names[:8]}'
        )
        if names != ['block']:
            self.get_logger().warn(
                f'期望 1 类方块模型 [\'block\']，实际 {len(names)} 类。'
                f'若这不是本意，请检查 model 参数是否指到了 COCO 权重。'
            )

        self.bridge = CvBridge()

        # ---- 发布者 ----
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/detections', 10
        )

        publish_annotated = self.get_parameter('publish_annotated').value
        if publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image, '/detections_image', 10
            )

        # ---- 订阅者 ----
        qos_profile = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, qos_profile
        )

        self.get_logger().info('YOLO Detector node started.')

    def _resolve_model(self, model: str):
        """把 `model` 参数解析成真实存在的文件路径，找不到返回 None。

        查找顺序（D6 / D24 修复）：
          1. 绝对路径 → 直接判定
          2. ``$YOLO_MODEL_DIR/<name>``  → 真机部署推荐用法
          3. CWD 及其所有父目录 ``/<name>`` → 兼容「从仓库任意子目录启动」
          4. ``yolo_detector/models/<name>``（若模型随包安装）

        刻意**不做**的事：不把名字交给 YOLO() 让它联网下载。宁可启动失败，
        也不要静默地用官方 COCO 模型去检测方块。
        """
        p = Path(model).expanduser()

        if p.is_absolute():
            return p if p.is_file() else None

        candidates = []

        env_dir = os.environ.get('YOLO_MODEL_DIR')
        if env_dir:
            candidates.append(Path(env_dir).expanduser() / p)

        cwd = Path.cwd().resolve()
        for d in (cwd, *cwd.parents):
            candidates.append(d / p)

        try:
            from ament_index_python.packages import get_package_share_directory
            share = Path(get_package_share_directory('yolo_detector'))
            candidates.append(share / 'models' / p)
        except Exception:
            pass

        for c in candidates:
            if c.is_file():
                return c.resolve()
        return None

    def _resolve_device(self, requested: str, allow_fallback: bool) -> str:
        """见模块级 `resolve_device()`。此处只做日志转发。"""
        return resolve_device(requested, allow_fallback, self.get_logger())

    def image_callback(self, msg: Image):
        """RGB 图像回调：运行 YOLO 推理，发布结果。"""
        # 收工途中到达的帧直接丢：此时 context 已经无效，往下走必然在
        # `publish` 上抛 `RCLError`（D70 的第二个面，见 `main()` 里那段说明）。
        # 顺带省掉一次无用的推理。
        if not rclpy.ok():
            return

        try:
            # ROS Image → OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # 运行 YOLO 推理
        t0 = time.time()
        results = self.model(cv_image, verbose=False)
        dt = time.time() - t0

        if len(results) == 0:
            return

        result = results[0]
        if result.boxes is None:
            return

        # 构建 Detection2DArray 消息
        detections_msg = Detection2DArray()
        detections_msg.header.stamp = msg.header.stamp
        detections_msg.header.frame_id = msg.header.frame_id

        boxes = result.boxes.xyxy.cpu().numpy()    # [N, 4]  x1,y1,x2,y2
        confs = result.boxes.conf.cpu().numpy()     # [N]
        clss  = result.boxes.cls.cpu().numpy()      # [N]

        annotated = cv_image.copy()

        for box, conf, cls_id in zip(boxes, confs, clss):
            if conf < self.conf_thresh:
                continue

            # ⚠️ 必须显式转成 Python float。`boxes` 来自 `.cpu().numpy()`，是
            # float32，`x1..y2` 就是 **np.float32** 标量；而 vision_msgs 的纯
            # Python 消息类在 setter 里做了严格断言
            #   assert isinstance(value, float), "The 'size_x' field must be of type 'float'"
            # `isinstance(np.float32(1.0), float)` 是 **False**（np.float64 才是
            # float 的子类），于是消息一构造就 AssertionError，节点在**第一帧
            # 图像**上直接崩掉。
            #
            # 这里曾经只在下面把 score 包了 float()，bbox 的四个量没包，所以
            # cx/cy 侥幸能过（np.float32 与 Python float 运算在 NumPy 1.x 的
            # 值域规则下被提升成 np.float64），而 w/h 是 np.float32 相减、仍是
            # np.float32，必崩——报错只指向 size_x，很容易让人以为只错了一个字段。
            #
            # 更要命的是这依赖 NumPy 版本：NumPy 2 起（NEP 50）np.float32 与
            # float 运算**不再**提升，cx/cy 也会一起崩。Orin 上装的若是 NumPy 2，
            # 症状会与这里不同。所以四个量全部显式 float()。
            x1, y1, x2, y2 = (float(v) for v in box)
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            w = x2 - x1
            h = y2 - y1

            # 获取类别名
            class_name = self.model.names.get(int(cls_id), f'class_{int(cls_id)}')

            # 构建 Detection2D
            detection = Detection2D()
            detection.header.stamp = msg.header.stamp
            detection.header.frame_id = msg.header.frame_id

            # bbox（vision_msgs Pose2D 有 API 兼容问题，手动构造）
            from vision_msgs.msg import BoundingBox2D
            bbox = BoundingBox2D()
            bbox.center.position.x = cx
            bbox.center.position.y = cy
            bbox.size_x = w
            bbox.size_y = h
            detection.bbox = bbox

            # 类别 + 置信度
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(class_name)
            hyp.hypothesis.score = float(conf)
            detection.results.append(hyp)

            # ID (用 class_id 填充，可通过 track 扩展)
            detection.id = str(class_name)

            detections_msg.detections.append(detection)

            # 标注可视化
            if self.get_parameter('publish_annotated').value:
                import cv2
                cv2.rectangle(annotated, (int(x1), int(y1)),
                              (int(x2), int(y2)), (0, 255, 0), 2)
                label = f'{class_name}: {conf:.2f}'
                cv2.putText(annotated, label, (int(x1), int(y1) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        self.detection_pub.publish(detections_msg)
        self.get_logger().debug(
            f'Detected {len(detections_msg.detections)} objects '
            f'({dt*1000:.1f} ms)'
        )

        # 发布标注图像
        if self.get_parameter('publish_annotated').value:
            try:
                anno_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                anno_msg.header = msg.header
                self.annotated_pub.publish(anno_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish annotated image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # 两个都要接：`ros2 launch` 下的 Ctrl-C 走的是 rclpy 自己的信号处理器，
        # 它关掉 context 之后 `spin()` 抛的是 **`ExternalShutdownException`**，
        # 不是 `KeyboardInterrupt`。只接后者的话它一路逃出 main()。
        pass
    except Exception:
        # D70 的**第二个面**（板端实测抓到的，第一个面修完才露出来）：
        # SIGINT 落在**回调执行到 `publish` 那一行时**，rclpy 会把回调里的异常
        # 从 `spin_once` 重新抛出（`executors.py:736 raise handler.exception()`）：
        #
        #     RCLError: Failed to publish: publisher's context is invalid
        #
        # 它既不是 `KeyboardInterrupt` 也不是 `ExternalShutdownException`，
        # 于是绕过上面那个 except 逃出 main() —— 又是一个「正常收工看起来像崩了」。
        # 判据用 `rclpy.ok()` 而**不是异常类型**：rclpy 没有公开的 `RCLError`
        # 别名（实测 `rclpy.exceptions` 里没有它，只有私有的
        # `rclpy._rclpy_pybind11.RCLError`），而「context 还活着吗」正是我们要
        # 区分的那件事 —— context 活着就照抛，免得把真正的发布失败一起藏起来。
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        # ⚠️ 必须先 `rclpy.ok()`：信号处理器已经调过一次 `rcl_shutdown`，
        # 再无条件调一次会抛
        #     RCLError: failed to shutdown: rcl_shutdown already called
        # 而**在 finally 里抛出的异常会顶掉原本那个**，于是
        # `ros2 launch` 报「process has died ... exit code 1」——功能上无害，
        # 但**每次正常收工都像崩了**（2026-09-28 板端实测：同一份 launch 里
        # 有这道闸的 planner_node 报 `process has finished cleanly`，
        # 没有它的本节点报 `process has died ... exit code 1`）。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
