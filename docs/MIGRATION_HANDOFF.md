# YOLO-ROS 迁移对接说明（供 WSL Codex CLI 使用）

最后核对：2026-09-26

## 1. 任务目标

将教学示范项目中的可复用 ROS 2 / YOLO 感知代码迁移到 Jetson Orin Nano Super（Ubuntu 22.04、ROS 2 Humble），接入 RealSense D435i，并在需求、机械臂接口和安全方案确认后，逐步完成 UR5 真机感知与抓取闭环验证。

这不是把旧环境原样搬过去：原项目主要围绕 YOLO 教学；用户回忆早期曾有 CoppeliaSim + MoveIt 的仿真抓取实现，之后删减为仅 YOLO 识别。当前 README 则描述 ROS 2 Jazzy + Gazebo Harmonic，并将 grasp planner 描述为 mock。**这些说法有待通过 Git 历史、代码、教学附件和参考仓库交叉核实**，不得据 README 或旧迁移笔记直接断定抓取实现是否存在。

## 2. 唯一源码基线与环境

- WSL 发行版：`Ubuntu-22.04`；主机当前时间的核对结果以本文件日期为准。
- 原仓库克隆路径：`/home/<user>/robotics/repos/yolo_ros_original`
- Windows / Codex Desktop 查看路径：`\\wsl.localhost\Ubuntu-22.04\home\<user>\robotics\repos\yolo_ros_original\`
- 导入来源：`C:\Users\<winuser>\Downloads\yolo_ros.bundle`
- Bundle SHA-256：`b8b6f8e837ff3d07e888ef9ba19587ad2ece79205a7682ea083858a9aeeb7c18`
- 导入基线：`main`，提交 `f250af11b77b1629b4e867eb7820912db66dbc2c`；共 8 个提交、约 251,146 个跟踪文件。
- 导入后执行 `git fsck --full --strict` 通过；克隆时工作区干净。当前新增的本文件是有意添加的迁移对接文档。
- 原仓库工作树约 18 GB；不要把整个仓库、训练数据或 COCO 数据集复制到 Jetson。模型/数据按实验需要单独选择。

原始仓库当前 `main` 只作为导入基线。**审计阶段不要改写、重置或清理 `main`**。完成只读审计并经用户确认迁移方案后，再从基线创建迁移分支；不要擅自提交或推送。

## 3. 重要边界与已知缺口

- 现有 WSL 项目 `/home/<user>/robotics/repos/arm-grasping` 是另一个独立仓库，有自己的 `AGENTS.md` 与 `PROJECT_ENVIRONMENT.md`。未经用户确认，不要在本任务中修改、合并或覆盖它。
- Windows 工作区 `C:\Users\<winuser>\Documents\ChatGPT\基于YOLO+Moveit的机械臂感知抓取闭环项目` 是早期整理副本；其中“源码缺失 / bundle 损坏”等旧记录已过时。原项目内容以本文件所指 WSL 克隆和导入 bundle 为准。
- `Moveit2YoloObb` 在 Git 树中是 gitlink，指向 `ce16f5a51235371a780671943e635ba80dfd42c0`；bundle 根目录没有 `.gitmodules`，因此子模块实际源码和 URL 尚未恢复。先从提交历史、文档、脚本和教学附件寻找来源；不得假称子模块已完整导入。
- 教学附件中可能包含 CoppeliaSim / UR5 / YOLO 抓取说明、压缩包和演示视频。盘点附件前先列目录/清单，不要解压覆盖现有文件。
- UR5 当前如何接入控制板尚未确认。用户曾怀疑 USB，但 UR 驱动常见方案为网络 RTDE；必须现场核实控制器型号、网络/IP、示教器配置及夹爪接口，不能写死连接方式。

## 4. 目标硬件（需上板复核）

根据此前用户提供的信息和板端盘点记录，目标为 Jetson Orin Nano Super 8GB、Ubuntu 22.04 aarch64、JetPack 6.2、ROS 2 Humble、RealSense D435i 和 UR5。上板软件、网络、相机连接、UR5 地址以及驱动状态均可能变化；部署前通过 SSH 只读核验，不要假定旧盘点仍准确。

不要把密码、私钥、SSH 配置或其他凭据复制进 Git、此文档、日志或命令历史。机械臂动作、控制器切换和真机抓取属于有风险操作：未经用户当次明确授权及安全条件确认，不得启动真实运动。

## 5. 给 Codex CLI 的工作流程

### 第一步：只读基线审计（当前允许）

1. 读取本文件、原仓库 `README.md`、`docs/` 设计文档、`ros2_ws/src/` 和相关安装/启动脚本。
2. 查看全部 8 个提交及 `61703a46d`（提交说明为“github参照项目验证通过”）涉及的文件，追溯参考项目、CoppeliaSim / MoveIt / UR5 / 抓取代码的真实情况。
3. 清点教学附件压缩包内容（只列清单或解压到新建隔离目录），区分源码、参考代码、演示视频、训练数据和生成产物。
4. 汇报当前可复用包、缺失/失效依赖、Jazzy→Humble API 差异、子模块缺口，以及哪些真机信息仍需用户确认。
5. 提交一份迁移方案和验收清单供用户确认；此阶段不改源代码、不安装/升级系统依赖、不向 Jetson 部署。

### 第二步：经用户确认后的迁移

1. 保留导入 `main` 为只读基线，另建明确命名的迁移分支。
2. 优先在 WSL Ubuntu 22.04 做 ROS 包分析、Python/接口迁移和可重复的非硬件测试；先确认本机是否已安装 ROS 2 Humble、MoveIt 2 和所需依赖。
3. 采用分阶段、小范围的修改与验证；保留仿真、假相机和真机配置的边界，不把 mock 执行误报为真实抓取。
4. 本机 WSL 若为 x86_64，不能以本机通过构建代替 Jetson aarch64 / CUDA / TensorRT 验证。硬件相关包和板端推理需在 Jetson 核验；TensorRT engine 应按 Jetson 软件栈生成/验证。
5. 先完成不驱动机械臂的感知链路，再单独做 UR5 连通、关节状态、MoveIt 规划和低速安全动作验收。每个真机动作都需要用户明确授权、急停可用、工作区隔离和低速/小范围策略。

## 6. Codex CLI 启动提示

本机 WSL 检查时 `codex` 命令未在 PATH 中，Linux `node` 也未找到；现有 `npm` 指向 Windows 安装，不要误用 Windows Node/npm 在 WSL 仓库内安装或构建。先确认用户已安装 Linux 原生 Codex CLI；未安装时说明需求并等待用户决定安装方式。

CLI 安装并可用后，在 WSL 执行：

```bash
cd /home/<user>/robotics/repos/yolo_ros_original
codex
```

建议首条任务指令：

> 请先完整阅读 `docs/MIGRATION_HANDOFF.md`，并严格执行其中的“第一步：只读基线审计”。只读检查 Git 历史、源码、教学附件和依赖；不要修改/删除文件，不要安装依赖，不要连接或驱动真实机械臂。完成后汇报证据、未确认项、迁移建议和分阶段验收方案，等待我确认后再改代码。

## 7. 本轮已经完成与尚未完成

已完成：验证 bundle SHA-256；从 bundle 克隆完整主仓库到 WSL；确认 `main` 提交与 8 提交历史；执行 `git fsck --full --strict` 并通过。

尚未完成：历史抓取代码审计、附件/压缩包盘点、GitHub 参考项目定位、Moveit2YoloObb 子模块来源恢复、Humble 迁移方案确认、任何代码迁移、板端部署及真机实验。