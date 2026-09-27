cmake_minimum_required(VERSION 3.8)
project(arm_grasp_sim)

find_package(ament_cmake REQUIRED)

# 世界 / 模型 / 配置 / launch / 工具脚本
install(DIRECTORY launch worlds models config scripts
  DESTINATION share/${PROJECT_NAME}
)

# 说明：抓取规划器不再装在本包里。
# 参考项目把它作为 scripts/grasp_planner.py 装到 lib/grasp_sim/ 下（ros2 run grasp_sim
# grasp_planner）。迁入主线后它归 grasp_planner 包，因为那块代码要在仿真与真机之间
# 复用，且需要单独的夹爪抽象层与单元测试。附件原版保留在
# docs/reference/ur5-yolo-grasp/grasp_sim/scripts/grasp_planner.py 供对照。

ament_package()
