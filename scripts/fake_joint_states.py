#!/usr/bin/env python3
"""假 /joint_states —— 无臂预检（moveit_only_check.py）的垫片之一。

move_group 的 CurrentStateMonitor 没有 /joint_states 就拿不到「当前状态」，
而 /compute_ik 的表现是：服务在、能列出，**但调用永远不完成**（挂住，不是
报错）。真机上 UR 驱动负责发这个话题；无臂预检时由本脚本补上 HOME 姿态。

只发话题，不连任何硬件、没有任何执行通道 —— 物理上不可能让臂动。
单独用没意义：配合 moveit_only_check.py。
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
# 与 grasp_planner.yaml 的 home_joints 一致（臂的 HOME 姿态，不是全零）。
HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]


def main():
    rclpy.init()
    node = Node("ur5_fake_joint_states")
    pub = node.create_publisher(JointState, "/joint_states", 10)
    try:
        while rclpy.ok():
            m = JointState()
            m.header.stamp = node.get_clock().now().to_msg()
            m.name = NAMES
            m.position = HOME
            pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.03)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    import sys
    sys.exit(main())
