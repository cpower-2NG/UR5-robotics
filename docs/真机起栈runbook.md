# 真机起栈 runbook —— ROS 管线的唯一权威启动序列

> **层级**：意图 · 这里**没有待办**，只有一套真机跑通过的动作序列；
> 「现在做到哪」见 [`当前状态.md`](当前状态.md)。
>
> **出身**：2026-10-05 A1 验收（4/4 + 槽位用尽收工）实际敲进终端的序列，
> 从阶段 1 的 `A1现场runbook.md` 泛化而来（A1 专属判据与录像流程已随阶段冻结进
> [`阶段/阶段1-点对点抓放/`](阶段/阶段1-点对点抓放/)；那次的完整原文在本仓 git 历史与仓外留档）。
>
> ⚠️ **A3 精密链不走这份入口**：它由 `scripts/precision_round.py` 驱动
> （开发机推理 + 板上抓帧/执行，接缝 = SSH + 文件），见
> [`阶段/阶段1-点对点抓放/A3精密抓放-实验记录.md`](阶段/阶段1-点对点抓放/A3精密抓放-实验记录.md)。
>
> 🔴 **唯一权威**：README / 技术文档里出现的启动命令，凡与本文件参数集或次序不一致的，**以本文件为准**。
> 🔴 **本文件不复述任何一版相机外参数字**。外参唯一真值在板上站点配置
> `~/.config/ur5-vision-grasp/site.yaml` 的 `topcam.extrinsics`（**仓外**）；现行值见
> [`当前状态.md`](当前状态.md) §3。照抄旧数字 = 候选落到臂够不到的地方，读起来像检测坏了。

---

## 第一步：控制箱接线并上电（这步不做，后面全是白搭）

| # | 做什么 | 怎么算通过 |
|---|---|---|
| 1 | 控制箱背面 LAN 口 → 板子 `eno1` 网线插回（点对点直连，无交换机） | RJ45 座**链路灯**亮 |
| 2 | 控制箱上电 | — |
| 3 | 板上 `ip -br a show eno1` | `UP` + `carrier=1` + 地址 `<板子IP>/24` |
| 4 | `ping -c3 <ur-controller-ip>` | 0% 丢包 |

> ⚠️ **别把 `carrier=0` 当板子的毛病查**（驱动/PCI/MAC 都查过，全正常）。
> 直连时对端断电 ⇒ 它的 PHY 断电 ⇒ 必然 `carrier=0`。查对端电源和线。

## 第二步：拆掉板上残留的旧栈（别跳）

```bash
# 板上 —— 先看清楚，再动手
pgrep -af 'ros2 launch'
pgrep -cf controller_stopper
# 确认无误后拆掉（会一并断开 URCap 的 External Control，属预期）
pkill -f 'ros2 launch yolo_bringup'
```

**通过**：`pgrep -af 'ros2 launch|controller_stopper'` 输出为空。

> ⚠️ **顺序别反**：拆栈会停掉示教器上的 `ur5ros.urp`。合法次序只有一条：
> **起栈（驱动先监听）→ 示教器按 PLAY → 连接后把控制器拉回来 → 规划器才起**。
> 没有驱动监听时按 PLAY，示教器直接「连接被拒绝」，程序停在 STOPPED（实测守 5 分钟也一样）。

## 第三步：分两段起 —— 先「不动手」，预检全绿再「动手」

```bash
# 板上 —— 第一段：驱动 + MoveIt + 感知，但不自动抓
export ROS_DOMAIN_ID=99            # ⚠️ 现场跑在 99，不是默认 0
cd ~/yolo_ros_original/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
# ⚠️ 七个 camera_* 从站点配置现读（site.yaml 的 topcam.extrinsics），别手拼、别照抄旧文
ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense \
    robot_ip:=<ur-controller-ip> \
    planner_params_file:=$HOME/site_real.yaml \
    gripper_backend:=ag95 \
    carry_objects:=false \
    tf_required:=true \
    use_target_plane:=true  target_plane_z:=0.0285 \
    publish_camera_tf:=true \
    camera_x:=<现读> camera_y:=<现读> camera_z:=<现读> \
    camera_qx:=<现读> camera_qy:=<现读> camera_qz:=<现读> camera_qw:=<现读>
```

| 预检项 | 命令 | 通过判据 |
|---|---|---|
| 轨迹控制器活着 | `ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {}` | `scaled_joint_trajectory_controller` = **active** |
| 相机在出帧 | `ros2 topic hz /camera/color/image_raw` | 稳定频率（实测 22.3 Hz）。⚠️ 实际话题是**单层** `/camera/...`，不是 `/camera/camera/...` |
| 检测器看到方块 | `ros2 topic echo --once /targets_3d` | ≥ 4 个候选、落在 `pick_zone` 内 |
| 规划器就绪 | `ros2 topic echo --once /grasp_planner/telemetry` | `state` 不是 `FINISHED`、`slots_free` 等于槽位数 |

> ⚠️ `ROS_DOMAIN_ID=99` 漏了会看到「话题列表是空的」——是**查询侧**问题，不是板子没起。
> ⚠️ 板子没装 `ros2 control` CLI，只能走上面的服务调用；`ros2 node list` 过滤关键字用 `control` 别用 `controller`。

**预检全绿后起第二段（臂开始动）**——Ctrl-C 掉第一段，同一条命令只改两处：
`carry_objects:=true` + 补 `start_delay:=120`，起完**立刻**去示教器按 PLAY（2 分钟窗口）。

> ✅ **`start_delay:=120` 的语义**：`pipeline.launch.py` 的 TimerAction 只包
> `[realsense, detector, perception, planner]`、不含 UR 驱动 ⇒ 驱动立刻监听，
> 留 2 分钟按 PLAY，规划器后起 —— 启动 HOME 撞在活连接上，不会被状态容差拒掉。
> 实测：不加时开局 `连续未放置 2/10`；加了之后零次。

**脚本替代（起栈 + 盯日志 + 判分）**：

```bash
# 板上
export UR_ROBOT_IP=<ur-controller-ip>
ROS_DOMAIN_ID=99 python3 scripts/board_a1.py --params $HOME/site_real.yaml --start-delay 120
```

退出码 `0` = 通过 / `1` = 判据没过 / `2` = 前置不满足（不起栈）。复核旧日志加 `--no-launch --log <文件>`。

**按完 PLAY 后立刻跑控制器闸门**（守 100 s，BEST_EFFORT 必须用 `strictness: 1`；
板上没有 `ros2 control` CLI）——完整命令块见仓外留档「☑ 队列清账 2026-10-26」的 A1 节。

**臂动之前 5 秒健康自检**：

```bash
grep -c "Aborting, no controller is switched" <日志>   # 必须是 0
grep -c "State tolerances failed"               <日志>   # 最好是 0
```

## 第四步：摆件（通用要求，成败多在这里不在代码）

- **抓取区里必须摆 槽位数+1 块**，才能触发「槽位用尽」收工 —— 4 块配 4 槽时收工走空闲看门狗，
  槽位闸门那行日志**永远不会打**（2026-10-05 真机实测；机理见阶段 1 契约 §3）。
- **重跑前必须清空放置槽位**：`place_z` 是全局标量，不知道槽上已有东西，会照同一高度直直压下去。
- 候选要**整体落在 `pick_zone` 内且离边界留余量**（别贴线）；投影结果用预检第 3 行核对。
- 摆好退开、手离开台面，再说「摆好了」。

---

## 已知坑速查（详细证据在台账）

| 症状 | 先查 |
|---|---|
| 发目标臂不动、`goal_status=<未知 None>` | `Aborting, no controller is switched` —— URCap 连上后的 STRICT 切换把控制器留在 not-running；按 PLAY 后立刻跑控制器闸门 |
| 话题列表是空的 | `ROS_DOMAIN_ID` 没设成 99（查询侧问题） |
| 候选落到臂够不到的地方、检测却高分 | 相机外参照抄了旧数字 / 用了 `extrinsics-2026-10-03` 那份的 `q`（⛔ 不可用，见当前状态 §3） |
| 相机没数据、日志 30 秒刷 7 万行 | 有孤儿进程占着相机（`ros2 topic info --verbose` 看 Publishers，别信「话题还在」） |
