"""
抓取规划节点 (grasp_planner)

订阅:
  /target_3d  (geometry_msgs/PoseStamped) — 目标在 base_link 坐标系下的位置

提供:
  /trigger_grasp 服务 (std_srvs/Trigger) — 手动触发抓取

工作流程:
  1. 接收 target_3d 目标位置
  2. 调用 MoveIt2 进行 IK 求解 + 轨迹规划  
  3. 执行抓取轨迹
  4. 控制夹爪闭合

注: 完整 MoveIt2 集成需要:
  - 机器人 URDF + MoveIt 配置包
  - MoveGroupInterface Python API

当前实现提供两种模式:
  - mock: 仅打印日志的测试模式（无需 MoveIt2）
  - moveit: 完整 MoveIt2 控制模式（需 MoveIt2+机器人配置）
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from std_msgs.msg import String


class GraspPlanner(Node):
    """抓取规划节点：IK 求解 + 轨迹执行"""

    def __init__(self):
        super().__init__('grasp_planner')

        # ---- 参数 ----
        self.declare_parameter('mode', 'mock')  # 'mock' | 'moveit'
        self.declare_parameter('gripper_close_service', '/gripper/close')
        self.declare_parameter('gripper_open_service', '/gripper/open')
        self.declare_parameter('approach_distance', 0.10)  # 预接近距离 (m)
        self.declare_parameter('grasp_offset_z', 0.0)  # 抓取 Z 偏移

        self.mode = self.get_parameter('mode').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.grasp_offset_z = self.get_parameter('grasp_offset_z').value

        # ---- 状态 ----
        self.latest_target = None  # 最新目标位姿

        # ---- 回调组 ----
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        # ---- 订阅者 ----
        self.target_sub = self.create_subscription(
            PoseStamped, '/target_3d', self.target_callback, 10
        )

        # ---- 服务 ----
        self.grasp_srv = self.create_service(
            Trigger, '/trigger_grasp', self.grasp_handler,
            callback_group=self.service_cb_group
        )

        # ---- 日志发布 ----
        self.status_pub = self.create_publisher(String, '/grasp_status', 10)

        # ---- MoveIt 初始化（moveit 模式） ----
        if self.mode == 'moveit':
            self._init_moveit()

        self.get_logger().info(f'GraspPlanner node started (mode={self.mode}).')

    def _init_moveit(self):
        """初始化 MoveIt2 MoveGroupInterface。"""
        try:
            from moveit_msgs.msg import MoveItErrorCodes
            # 注意: moveit_py 需要特定版本，这里用 rclpy + action client 替代
            # 完整 MoveIt2 集成请参考 MoveIt2 官方 Python API
            self.get_logger().warn(
                'MoveIt2 full Python API not yet integrated. '
                'Use moveit_servo or action-based interface for now.'
            )
            self.moveit_available = False
        except ImportError:
            self.get_logger().warn('moveit_msgs not found, falling back to mock.')
            self.moveit_available = False

    def target_callback(self, msg: PoseStamped):
        """缓存最新目标 3D 位姿。"""
        self.latest_target = msg
        self.get_logger().debug(
            f'Target updated: ({msg.pose.position.x:.3f}, '
            f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})'
        )

    def grasp_handler(self, request, response):
        """处理 /trigger_grasp 服务请求。"""
        if self.latest_target is None:
            response.success = False
            response.message = 'No target available. Wait for detection.'
            self.get_logger().warn(response.message)
            return response

        self.get_logger().info(
            f'Grasp triggered! Target: '
            f'({self.latest_target.pose.position.x:.3f}, '
            f'{self.latest_target.pose.position.y:.3f}, '
            f'{self.latest_target.pose.position.z:.3f})'
        )

        if self.mode == 'moveit':
            success, msg = self._execute_moveit_grasp()
        else:
            success, msg = self._execute_mock_grasp()

        response.success = success
        response.message = msg

        status = String()
        status.data = f'{"OK" if success else "FAIL"}: {msg}'
        self.status_pub.publish(status)

        return response

    def _execute_mock_grasp(self):
        """Mock 抓取：仅打印日志和模拟步骤，用于测试流水线。"""
        target = self.latest_target
        px, py, pz = target.pose.position.x, target.pose.position.y, target.pose.position.z

        self.get_logger().info('=== Mock Grasp Sequence ===')

        # Step 1: 预接近
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = target.header.frame_id
        approach_pose.pose.position.x = px
        approach_pose.pose.position.y = py
        approach_pose.pose.position.z = pz + self.approach_distance
        approach_pose.pose.orientation.w = 1.0
        self.get_logger().info(
            f'[1/4] Approach pose: '
            f'({approach_pose.pose.position.x:.3f}, '
            f'{approach_pose.pose.position.y:.3f}, '
            f'{approach_pose.pose.position.z:.3f})'
        )

        # Step 2: 下降抓取
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = target.header.frame_id
        grasp_pose.pose.position.x = px
        grasp_pose.pose.position.y = py
        grasp_pose.pose.position.z = pz + self.grasp_offset_z
        grasp_pose.pose.orientation.w = 1.0
        self.get_logger().info(
            f'[2/4] Grasp pose: '
            f'({grasp_pose.pose.position.x:.3f}, '
            f'{grasp_pose.pose.position.y:.3f}, '
            f'{grasp_pose.pose.position.z:.3f})'
        )

        # Step 3: 闭合夹爪
        self.get_logger().info('[3/4] Close gripper')

        # Step 4: 抬起
        lift_pose = PoseStamped()
        lift_pose.header.frame_id = target.header.frame_id
        lift_pose.pose.position.x = px
        lift_pose.pose.position.y = py
        lift_pose.pose.position.z = pz + self.approach_distance + 0.05
        lift_pose.pose.orientation.w = 1.0
        self.get_logger().info(
            f'[4/4] Lift pose: '
            f'({lift_pose.pose.position.x:.3f}, '
            f'{lift_pose.pose.position.y:.3f}, '
            f'{lift_pose.pose.position.z:.3f})'
        )

        self.get_logger().info('=== Mock Grasp Complete ===')
        return True, 'Mock grasp sequence completed successfully.'

    def _execute_moveit_grasp(self):
        """MoveIt2 模式抓取（需要完整机器人配置）。

        注意: 完整实现需要:
        1. MoveIt2 配置包 (moveit_config)
        2. MoveGroupInterface Python 绑定
        3. 机器人 controller 激活
        """
        self.get_logger().error(
            'MoveIt2 mode not yet fully implemented. '
            'Please configure robot-specific MoveIt parameters.'
        )
        return False, 'MoveIt2 mode requires robot configuration.'


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
