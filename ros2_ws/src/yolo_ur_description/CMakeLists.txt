cmake_minimum_required(VERSION 3.8)
project(yolo_ur_description)

find_package(ament_cmake REQUIRED)
# 不是可选依赖：下面整个包的内容都是从它镜像出来的。
find_package(ur_description REQUIRED)

# find_package 给的 XXX_DIR 是 <prefix>/share/<pkg>/cmake，上一层才是包的 share 目录。
get_filename_component(ur_description_share "${ur_description_DIR}/.." ABSOLUTE)
message(STATUS "yolo_ur_description: 镜像上游自 ${ur_description_share}")

# ---------------------------------------------------------------------------
# 1) 镜像上游：xacro + config/（**排除**运动学标定）
#
# 为什么要镜像而不是在仓库里存一份副本：存副本就会「上游升级、我们的副本
# 悄悄过期」，而这里镜像的每一个文件都必须与上游逐字节相同 —— 驱动与
# move_group 各拿一份 robot_description，两份只要差一个字节，
# move_group 的 FK/IK 与驱动发布的 TF 就是两套几何，而两边都不报错。
# 构建期镜像 = 永远跟着板上装的 ur_description 走，没有需要人工同步的副本。
#
# 排除 default_kinematics.yaml：那是唯一一处**必须**不一样的东西，
# 由下面第 2 步提供这台机器人的出厂标定。
# ---------------------------------------------------------------------------
install(DIRECTORY "${ur_description_share}/config/"
        DESTINATION share/${PROJECT_NAME}/config
        PATTERN "default_kinematics.yaml" EXCLUDE)
install(FILES "${ur_description_share}/urdf/ur.urdf.xacro"
        DESTINATION share/${PROJECT_NAME}/urdf)
# 两个官方 launch 都会用 description_package 拼 rviz 配置的路径
# （ur_control.launch.py:220、ur_moveit.launch.py:236 —— 后者其实用
# moveit_config_package，但一并镜像掉就不必去分辨）。本仓库 launch_rviz:=false，
# 正常路径下这个 substitution 根本不会被求值；镜像它只是不想让「哪天有人开了
# RViz」变成一次缺文件报错。
install(DIRECTORY "${ur_description_share}/rviz/"
        DESTINATION share/${PROJECT_NAME}/rviz)

# ---------------------------------------------------------------------------
# 2) 本包唯一自有的内容：本机 UR5 的出厂标定（从机器人上抓下来的，见 README）
# ---------------------------------------------------------------------------
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

ament_package()
