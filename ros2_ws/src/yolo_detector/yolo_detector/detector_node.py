"""
YOLOv8 目标检测 ROS2 节点 (yolo_detector)

订阅:  /camera/color/image_raw  (sensor_msgs/Image)
发布:  /detections              (vision_msgs/Detection2DArray)
       /detections_image        (sensor_msgs/Image, 标注可视化)

通过 ultralytics YOLOv8 进行推理，输出 2D 检测框。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge

import numpy as np
import time


class YoloDetector(Node):
    """YOLOv8 检测节点：订阅 RGB 图像，推理后发布 2D 检测框。"""

    def __init__(self):
        super().__init__('yolo_detector')

        # ---- 参数 ----
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cuda:0')  # 'cpu' / 'cuda:0'
        self.declare_parameter('publish_annotated', True)

        model_path = self.get_parameter('model').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        device = self.get_parameter('device').value

        # ---- 加载 YOLOv8 模型 ----
        self.get_logger().info(f'Loading YOLO model: {model_path} on {device}')
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.model.to(device)
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise

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

    def image_callback(self, msg: Image):
        """RGB 图像回调：运行 YOLO 推理，发布结果。"""
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

            x1, y1, x2, y2 = box
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
