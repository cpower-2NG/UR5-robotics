# A1 现场 runbook —— 真机「抓 4 放 4 + 槽位用尽收工」

> **层级**：意图 · 阶段 1 的出口验收 runbook。**阶段 1 一结束就该退役** ——
> 结论进 [`当前状态.md`](当前状态.md)，过程进仓外留档（去处见
> [`文档地图.md`](文档地图.md) §7）。
>
> ⚠️ **这一份不是队列**：里面没有待办，只有一套真机跑通过的动作序列。
> 「现在做到哪一步」见 [`当前状态.md`](当前状态.md)。
>
> **为什么单独一份**：它 390 行，原来长在 [`待执行命令.md`](待执行命令.md) 里，
> 占那份文件六分之一，把队列顶得离它自己第 11 行写的「两屏」很远。2026-10-08
> 拆出来，**搬运时正文一字未改**（只把内部标题降了一级 `###` → `##`）；当天同一批
> 里另改了三处：「为什么」段补判决、三处命令块前加点明 `camera_*` 必须换的注释
> （原文写「整条照抄」，是错的）、零散的「本节」改「本文件」。


> ## 🔴 这一份是全仓唯一权威的「真机启动序列」
>
> **起栈 → 按 PLAY → 把控制器拉回来 → 预检四项 → 摆件 → 收工判据**，
> 全部以本文件为准。README / 技术文档 / 板卡清单里出现的启动命令，凡与本文件的
> 参数集或次序不一致的，**一律以本文件为准**（那几处已改成指向这里的指针）。
>
> **为什么是这一份**：它是**唯一一次真机跑通过的序列**（2026-10-05 A1 验收 4/4）。
> 本文件里的参数值不是「建议值」，是**那一次实际敲进终端的值**。
>
> **它由机器把守**：`python3 scripts/test_startup_docs.py` 会把本文件三处命令块的
> **参数集**与 `scripts/board_a1.py::build_cmd()` **实际下发的参数集**逐项对，
> 缺一项 / 多一项就**报红**。改了本文件就要同步改 `board_a1.py`，反之亦然。
> 另外它还会检查两类**照做会失败**的写法有没有复活：
> 「真机命令里没写 `robot_ip`」与「把 `pipeline.launch.py` 当成真机入口」（D45）。
>
> ---
>
> ### 🔴 本文件三处命令块里的 `camera_*` **已作废，别照抄**
>
> 相机在 2026-10-06 被**物理挪过两次**（白天抬高下俯 207.6 mm / 7.93°；晚上又是一次
> 整体位移与转角）。**下面三处 `camera_x:=0.34694477757210807 …` 是 2026-10-05 那次
> 验收时敲的值**，现在会算错。
>
> 🔴 **本文件不再复述任何一版外参的具体数字。** 这不是省事，是这条提醒自己换来的教训：
> 本框**原来**写着一组「现行值」（`0.284688 …`），而**当天晚上那组就又被重标换掉了** ——
> 复述一次就多一份会烂的真值。**外参只许有一份真值，在板上站点配置里**：
> `~/.config/ur5-vision-grasp/site.yaml` 的 `topcam.extrinsics`（**仓库外**）。
> 现行值见 [`docs/当前状态.md`](当前状态.md) §3 那一行（列了日期、来源与判据）。
>
> **别手拼外参**：用 `scripts/board_run.py`，它读站点配置，报告里把每个参数的
> **来源**逐项印出来。同理 `scripts/board_a1.py` 现在**没有 `--site` 就退 2**
> （2026-10-06 定），仓库里那份 `CAM` 已降级成**作废的孤本记录**、不作兜底 ——
> 所以「不带站点配置拿旧几何起栈」这条路上已经没有出口了。
>
> ⚠️ 另记：`顶视外参` 的**漂移**闸门 `scripts/topcam_drift_check.py` 目前**没有任何调用方**
> （P1 只跑日期闸门）。已经两次证明日期闸门不够 —— 白天那次**相机挪了 207 mm，日期闸门
> 照样放行**；晚上 R0 第 3 次又是**日期报 PASS、外参实际歪到让 `P4.targets` 挂掉**。

**为什么**：**2026-10-05 真机已经跑通了**（第二轮：`已放置 4/4` **且**收工原因是
`放置槽位已用尽（4 个）`—— 证据在下面「怎么算通过」那一节）。

> ⚠️ **这一句原来是这么写的**：「真机证据目前只到 `已放置 1/4`，而且那次是靠
> 20 秒空闲看门狗收的工，不是走到『槽位用尽』—— 那条收工路径只在仿真里验过」。
> 它写于**第一轮之前**，当天下午就被**本文件自己的记录**推翻，却留了三天没人回头改
> （2026-10-08 查出）。**这种句子比数字错了更坏**：它让人以为一条已经走通的路还没走通。
> 历史判据（技术文档`:355` 的触发条件、实验记录`:194` 的「真机上尚未触发」）也随之作废。

A1 判的就是这一条：真机连续抓放 → 槽位耗尽 → 正常收工（判据全文见「怎么算通过」）。

> ✅ **这一步不需要改任何代码，也不需要重新 `colcon build`。**
> 槽位数 = `place_slots` 的**值的对数**（现场 yaml 里正好 4 对 = 4 个槽；
> ⚠️ 它是**扁平**的 `[x0,y0, x1,y1, …]` —— 不是审美选择：**ROS 2 的参数类型系统
> 没有嵌套数组**，`[[x,y], …]` 会让节点**直接起不来**（`planner_node.py:203-210`）。
> 所以 yaml 里 `len()` 是 **8** 不是 4，节点自己在 `__init__` 里
> `zip(flat[0::2], flat[1::2])` 解成 4 对 —— 2026-10-06 启动器就是直接数 `len()`
> 而把 4 个槽报成 8 个，见台账第九十九轮）。日志里的「N/4」和「槽位用尽」判据
> 全部走**解出来的** `len(self.place_slots)`、没有硬编码；抓放循环由持续的
> 检测流 + busy 复位自然驱动。**板子现在这份检出就能跑。**

## 第一步：把控制箱接回来并上电（**这步不做，后面全是白搭**）

| # | 做什么 | 怎么算通过 |
|---|---|---|
| 1 | **控制箱背面 LAN 口 → 板子 `eno1`** 那根网线插回去（点对点直连，无交换机） | RJ45 座上的**链路灯**亮 |
| 2 | **控制箱上电** | —— |
| 3 | 板上 `ip -br a show eno1` | `UP` + **`carrier=1`** + 地址 `<板子IP>/24` |
| 4 | `ping -c3 <控制箱IP>` | 0% 丢包（历史实测 ~0.26 ms） |

> ✅ **2026-10-05 15:20：第 1~4 步已全部实测通过** —— `eno1` UP / `carrier=1` /
> `<板子IP>/24`，`ping <控制箱IP>` 3/3 **0% 丢包**（avg 0.201 ms）；Dashboard 只读查询
> `Robotmode: RUNNING`、`Safetystatus: NORMAL`、`programState: PLAYING ur5ros.urp`、
> 机型 `UR5`。**这一段不必重做，直接从第二步往下。**

> ⚠️ **别再把 `carrier=0` 当板子的毛病查。** 10-01 已经查透：`r8168` 驱动、PCI、
> MAC、IRQ 全正常，板子只有一个 RJ45、不存在插错口。`carrier=0` 是**物理层**判据 ⇒
> 线没插实 / **对端（控制箱）没电** / 线坏；直连时对端断电 ⇒ 它的 PHY 也断电 ⇒ 必然 `carrier=0`。
> （05 日上午实测过一次 `carrier=0`，当日下午复通。）

## 第二步：拆掉板上那套「僵尸栈」（**别跳，而且它现在是带电的**）

2026-10-05 15:20 复查：板子上挂的是**昨晚留下的整套栈**，而且**链路一通它就活过来了**。

- `real.launch.py` 起于 **10-04 21:49**（已 17.5 h）、`pipeline.launch.py` 起于
  **10-04 23:26**（已 15.9 h）—— **都不是本次实验起的**；
- 卡在 SYN-SENT 十七个小时的 `ur_ros2_control`，在 `eno1` 起来后**立刻建连成功**
  （对 30001/30003/30004 全 ESTAB），`scaled_joint_trajectory_controller` 现在是
  **`active`**（不再是 inactive）；
- ⚠️ **那条 `pipeline.launch.py` 带的是显式 `carry_objects:=true`** ⇒ 只要相机一出图、
  检测到方块，**它会自己发起抓取**。现在没动，**唯一原因**是相机哑了：
  - 两台 D435i 都在（Bus 002 Device 003 / **045**），**045 这个高编号说明它重新枚举过**；
  - 而相机节点是 15 小时前起的，**早就放弃、不会重试** ⇒
    `/camera/camera/color/image_raw` 无数据。**这条只能靠重启栈恢复，等不来。**
- 仍残留 **3 个 `controller_stopper_node`**、2×`move_group`、2×`dashboard_client`
  —— 两次 launch 叠加出来的。

```bash
# 板上 —— 先看清楚，再动手
pgrep -af 'ros2 launch'
pgrep -cf controller_stopper
```

```bash
# 确认无误后拆掉（会一并断开 URCap 的 External Control，属预期，见下面的⚠️）
pkill -f 'ros2 launch yolo_bringup'
```

**怎么算通过**：`pgrep -af 'ros2 launch|controller_stopper'` **输出为空**
（若仍有残留，按上一条列出的 PID 逐个 `kill`）。

> ⚠️ **顺序别反**：拆栈会断开 URCap 的 External Control ⇒ 示教器上那个
> `ur5ros.urp` **会停**。正确顺序是 **先起驱动、再在示教器按播放**（即第三步做完之后）。

## 第三步：分两段起 —— 先起「不动手」的那一半

`carry_objects` 的**默认值就是 `false`**（`real.launch.py:343`），所以可以**分两段**：
先起不带它的，臂这时**只连不抓**，能安心做完全部预检；预检四项全绿，再换第二段。

**⚠️ 现场值必须走显式命令行**，不能靠默认值 —— 因为现场 yaml 里写的是
`carry_objects: true`（`site_real.yaml:60`）和 `gripper_backend: sim`（`:90`），
正好都是危险/错的那一侧。

```bash
# 板上 —— 第一段：驱动 + MoveIt + 感知，但不自动抓
#   ⚠️ 七个 `camera_*` —— 上面那组是 **2026-10-05 的旧几何**，照抄＝把外参搞错，
#      症状是「候选落到臂够不到的地方」，读起来像检测坏了。现读当前值（只读）：
#          python3 scripts/board_run.py --task a1 --dry-run
export ROS_DOMAIN_ID=99            # ⚠️ 现场跑在 99，不是默认 0
cd ~/yolo_ros_original/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense \
    robot_ip:=<控制箱IP> \
    planner_params_file:=$HOME/site_real.yaml \
    gripper_backend:=ag95 \
    carry_objects:=false \
    tf_required:=true \
    use_target_plane:=true  target_plane_z:=0.0285 \
    grasp_point_mode:=top_face_center  table_plane_z:=0.0 \
    publish_camera_tf:=true \
    camera_x:=0.34694477757210807 camera_y:=0.8693043936314018 camera_z:=0.6454397556505314 \
    camera_qx:=-0.32913450910630504 camera_qy:=-0.316374655430605 \
    camera_qz:=0.6185281368827056 camera_qw:=-0.6395314661538762
```

> ### 🔴 相机外参：**别照抄 `extrinsics-2026-10-03-base_link.json` 里的 `q`**
>
> 2026-10-05 实测查实：那份 JSON 的 **`q` 字段和它自己的 `R` / `t` 不一致**。
> 它同时把 `t` 和 `R` 绕 z 转了 180°（从 UR 的 `base` 帧换到 URDF 的 `base_link`），
> **唯独 `q` 没跟着改** —— 由文件里的 `q` 算出的旋转矩阵，与文件里的 `R`
> 差一个 `diag(-1,-1,1)`。上面那串 `camera_q*` 就是**按文件里的 `R` 反解出来的正确值**。
>
> 三组候选的**实机 A/B**（同一帧、4 个方块、`carry_objects:=false`，只换静态 TF）：
>
> | 用哪组 | 投到的 `base_link` 坐标 | 判读 |
> |---|---|---|
> | 文件里的 `q` + `t=(+0.347,+0.869,+0.645)` | y ≈ **1.38~1.57** | 1.5 m 外，**臂够不到** ✗ |
> | 文件里的 `q` + `t=(−0.347,−0.869,+0.645)` | x ≈ **−0.39~−0.52** | 镜像回 UR `base` 帧 ✗ |
> | **文件里的 `R` 对应 `q` + `t=(+0.347,+0.869,+0.645)`** | **(0.522,0.172) (0.471,0.281) (0.385,0.204) (0.394,0.360)** | 臂展内、贴着 `pick_zone` ✓ |
>
> ⚠️ **填错的后果是无声的**：投影照样出数、检测照样 0.96 分，只是点落在臂够不到的地方
> ——预检第 3 行是**唯一**能发现它的地方。摆方块时务必核对。

> - 入口用 `bringup.launch.py camera_source:=realsense`（`real.launch.py` 文件头写明
>   这是**唯一入口**，它自己把 `camera_source` 写死成 realsense 再 include
>   `pipeline.launch.py`）。
> - `planner_params_file` **只由 `pipeline.launch.py:482` 声明**，`real.launch.py`
>   include 时**没有传它**；靠仓库注释（`real.launch.py:390`）说的
>   「未声明的参数会直接进 launch_configurations」透传下去。
> - **起来后到示教器按播放**，让 External Control 重新握手，再做下面的预检。

| 预检项 | 命令 | 通过判据 |
|---|---|---|
| 轨迹控制器活着 | `ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {}` | `scaled_joint_trajectory_controller` = **active** |
| 相机在出帧 | `ros2 topic hz /camera/color/image_raw` | 稳定频率（实测 **22.3 Hz**）。⚠️ **不是** `/camera/camera/color/image_raw` —— 那是相机节点的**重映射前**名字，实际发的是 `/camera/color/…`；查错名会误判成「相机没起来」 |
| 检测器看到方块 | `ros2 topic echo --once /targets_3d` | **≥ 4** 个候选、且落在 `pick_zone` 内 |
| 规划器就绪 | `ros2 topic echo --once /grasp_planner/telemetry` | `state` 不是 `FINISHED`、`slots_free=4` |

> ⚠️ **`ROS_DOMAIN_ID=99` 别漏。** 漏了会看到「话题列表是空的」，极容易误判成
> 「栈没起来」——那是**查询侧**的问题，不是板子的问题。2026-10-05 实测踩过这个坑。
> 另：板子没装 `ros2controlcli`，`ros2 control list_controllers` **用不了**，只能走上面的服务调用。
> 再另：`ros2 node list` 里 `ur_ros2_control` 之类的驱动节点**过滤关键字别用 `controller`**
> —— 它拼的是 `control`，会漏掉（05 日踩过，一度误判成「节点都没起来」）。

**预检四项全绿之后**，才起第二段 —— **这一步臂开始动**。

> 🚨 **起第二段之前两件事（缺一则验收第二条拿不到）**：
> 1. **抓取区摆 5 块**（不是 4 块）—— 理由见下面「怎么算通过」；
> 2. **把上一轮放在 4 个槽位上的方块拿走**（槽位 z 是全局标量，不会自己避让）。

> ✅ **再加一个 `start_delay:=120`，先起栈、再按播放**（2026-10-05 验证，强烈建议）：
> `start_delay` 是 `pipeline.launch.py:737-740` 的 `TimerAction`，包的是
> `[realsense, detector, perception, planner]` —— **不含 UR 驱动**。所以它的语义
> 正好是「驱动立刻起（5000x 开始监听）→ 给你 2 分钟在示教器上按 PLAY →
> 规划器才启动」。这样规划器的**启动 HOME 撞在活连接上**，不会像上次那样被
> 状态容差拒掉、白烧「连续未放置」配额。
> 实测对比：不加时开局 `连续未放置 2/10` + 两次 `State tolerances failed`；
> 加 `start_delay:=120` 后 **一次都没有**，开局直接 `已就绪` + `连续未放置 0/10`。

```bash
# 板上 —— Ctrl-C 掉第一段，把上面那条照抄。**要改两处**：
#   ① carry_objects → true，并补 start_delay:=120（起完立刻去示教器按 PLAY，2 分钟窗口）
#   ② 七个 `camera_*` —— 上面那组是 **2026-10-05 的旧几何**，照抄＝把外参搞错，
#      症状是「候选落到臂够不到的地方」，读起来像检测坏了。现读当前值（只读）：
#          python3 scripts/board_run.py --task a1 --dry-run
ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense \
    robot_ip:=<控制箱IP> \
    planner_params_file:=$HOME/site_real.yaml \
    gripper_backend:=ag95 \
    carry_objects:=true  start_delay:=120 \
    tf_required:=true \
    use_target_plane:=true  target_plane_z:=0.0285 \
    grasp_point_mode:=top_face_center  table_plane_z:=0.0 \
    publish_camera_tf:=true \
    camera_x:=0.34694477757210807 camera_y:=0.8693043936314018 camera_z:=0.6454397556505314 \
    camera_qx:=-0.32913450910630504 camera_qy:=-0.316374655430605 \
    camera_qz:=0.6185281368827056 camera_qw:=-0.6395314661538762
```

**更省事的办法：让脚本起栈 + 盯日志 + 判分**（2026-10-05 新加）：

```bash
# 板上 —— 先把 scripts/board_a1.py 同步过去（板子是另一份检出，别指望它自己就有）
export UR_ROBOT_IP=<控制箱IP>          # 公开仓不写现场 IP，所以这个没有默认值
ROS_DOMAIN_ID=99 python3 scripts/board_a1.py --params $HOME/site_real.yaml \
    --start-delay 120                  # 与上面手敲的两段取同一个值（脚本默认 25）
```

它做四件事：**前置检查**（无残留 / 串口空闲 / AG-95 已初始化 / 示教器在 PLAYING）
→ 起 `carry_objects:=true` 的整栈 → 盯到「收工：」→ 按两条判据判分。
退出码 `0` = 通过，`1` = 判据没过，`2` = 前置不满足（**不会起栈**）。
只想复核已有日志：`--no-launch --log ~/a1_board.log`（不起栈、不动臂）。

对应自检 `scripts/test_board_a1.py`（30 项断言 + 3 个变异测试，纯离线）。

> ⚠️ **`start_delay` 的两个数**：上面手敲的两段用 **120 s**（2026-10-05 实测
> 「加了之后一次 `State tolerances failed` 都没有」）；而 `board_a1.py` 的
> `--start-delay` **默认是 25 s**。25 是脚本的保守下限（够按 PLAY），
> **120 是建议值**；想照录像是那一轮的数，就显式加 `--start-delay 120`。
> 两个都跑通过，别把 25 当成「跑不了的旧值」。

## 第四步：摆件（**A1 成败其实在这里，不在代码**）

- **抓取区**：现场 yaml 的 `pick_zone` = x∈[0.4, 0.6]、y∈[0, 0.45]。要抓 4 块，
  就得让**至少 4 块同时**落在里面、且**被检测器看到**（看预检第 3 行）。
  - 📌 **2026-10-05 实测（两次，中间用户挪过一次）**：按常规摆位放 4 块，投影出来是
    `(0.522,0.172) (0.471,0.281) (0.385,0.204) (0.394,0.360)` —— **只有 2 块在区里**，
    另两块卡在 **x≈0.385~0.394**，就差 `x≥0.4` 这条线一点点。
    ⇒ 摆的时候**整体往 +x 挪 ~2 cm**（或把方块聚拢些），别贴着 x=0.4 放。
  - 📌 **同日 15:45 复核（用 `scripts/draw_zone_on_frame.py` 把 `pick_zone` 画回实拍图）**：
    挪过之后变成 `(0.476,0.185) (0.369,0.220) (0.476,0.298) (0.372,0.337)` ——
    **仍是 2 在区里 / 2 在区外**，区外那两块还是卡在 x≈0.37，只差 `x≥0.4`。
    ⇒ **用户上次是把方块往 −x 挪了约 5 cm，方向反了**（x 变小 = 离基座更近 = 更出区）。
    要**往 +x（远离基座）推**，让四块 `x ≥ 0.42`（留 2 cm 余量，别贴线）。
    y 全在 `[0.185, 0.337]`，都在 `[0, 0.45]` 里，**不用动**。
    复核图留在本机 `~/a1_shots/pick_zone_overlay.png`（**不进仓**，含现场实拍）。
  - ✅ **同日 15:52 第三次复核：四块全进区了** ——
    `(+0.574,+0.318) (+0.572,+0.181) (+0.462,+0.180) (+0.463,+0.304)`，
    分数 0.88~0.92，判定 `区内 4 / 区外 0`。
    ⇒ 摆位这一步**已达标**；以后重跑只要保持这个摆法（x 全 ≥ 0.46）。
- 🚨 **要触发「槽位用尽」收工，抓取区里必须摆 5 块**（**这条是 2026-10-05 真机实测
  才发现的，不是设计文档里写的**）。见下面「怎么算通过」第一条的说明。
- **放置区**：4 个槽位在 x∈{0.45, 0.55}、y∈{−0.25, −0.35} —— **在俯视相机视野之外**
  （B1 要解决的正是这个）。A1 的放置是**开环按坐标下发**、不需要看见，
  但意味着**落点没人复核**：摆件时把槽位附近清空，别堆东西。
  - ⚠️ **重跑之前必须先把槽位上那几块拿走。** 槽位 z 是全局标量
    （`place_z=0.029`），它**不知道槽上已经有东西** —— 上一轮放下的方块还在原位时，
    下一轮会**照着同一个高度直直压下去**。2026-10-05 第一轮放的 4 块
    收工时仍留在 4 个槽位上。

## 怎么算通过

- 规划器日志出现 **`已放置 4/4`**；
- 收工原因是 **`放置槽位已用尽（4 个）—— 没有地方可放了`**（`ok=True` 的正常收工），
  **不是** `空闲 21s…`（`idle_finish_timeout`）、也**不是** `连续 10 次没放下`
  （`max_attempts_without_place`）；
- 录像留档。

> 🚨 **上面第二条有前置条件：抓取区里的方块数必须 > 槽位数（即 ≥ 5 块）。**
> **2026-10-05 真机实测暴露的**：4 块方块配 4 个槽位，第 4 块放下的**同一刻**
> 抓取区也空了，于是收工走的是 `idle_finish_timeout` 那条路，
> **`放置槽位已用尽` 这条 log 永远不会打出来**。
>
> 原因在 `planner_node.py:1339-1348`（`_select_and_approach`）：那道槽位闸门
> 排在**选中候选之后** ——
> ```python
> if best is None:      # ← 4 块 / 4 槽 时先命中这里
>     ...; return       #    区里没目标 → 交空闲看门狗
> # ③ 槽位闸门：起步之前先确认还有地方可放
> if self._next_place_slot() is None:
>     self._finish(f"放置槽位已用尽（{len(self.place_slots)} 个）—— 没有地方可放了")
> ```
> 它要求「**有一个够分的新候选**」与「**没有空槽位**」**同时**成立。
> ⇒ 摆 **5 块**、槽位仍留 4 个，第 5 块就会在放完第 4 块之后撞上这道闸门。
> （顺带：这道闸门在 `_attempts_since_place += 1` **之前**，所以不会被
> 「连续 10 次」抢先。）
>
> 📌 **2026-10-05 16:0x 第一轮真机结果（4 块 / 4 槽）**：`已放置 4/4` ✅、
> 四次夹爪闭合回读 25.08 / 25.94 / 34.20 / 26.60 mm（**全部 ≫ 0，是真夹住了**）✅、
> 落点 `(0.463,0.180) (0.573,0.181) (0.463,0.304) (0.574,0.319)`；
> 收工原因 ❌ = `空闲 20s …（上限 idle_finish_timeout=20s）`。
> 完整日志在板上 `~/a1_stage2.log`。
>
> ⚠️ 起栈时回 HOME 的轨迹被 `ur_ros2_control_node` 的**状态容差**拒了两次
> （joint 5 误差 `0.200263` vs 容差 `0.200000`，只超 `0.000263 rad ≈ 0.015°`，
> 日志 `State tolerances failed for joint 5` + `error_code=-4` = `CONTROL_FAILED`）。
> 规划器**自己重试后成功**，但这 2 次算进了 `连续未放置 2/10` ——
> 容差再紧一点就会走成「连续 10 次没放下」。
> ✅ **已根治**：改用 `start_delay:=120`（先起栈、按 PLAY、规划器后起）。
> 见上面第二段启动块前的说明。

> ✅✅ **2026-10-05 第二轮真机结果（5 块 / 4 槽）—— A1 通过**：
> ```
> 候选 5 个 | 已放置 0/4 | 连续未放置 0/10
> 已放置 1/4 → 2/4 → 3/4 → 4/4          （每循环约 23 s）
> 候选 1 个 ['block'] | 已放置 4/4 | 已抓 4 个
> 收工：放置槽位已用尽（4 个）—— 没有地方可放了
>   已成功放置 4 个 / 槽位共 4 个；已抓取记账 4 个
> ```
> 候选数轨迹 `5(0) → 4(1) → 3(2) → 2(3) → 1(4)` —— 放完第 4 块后**区里还剩 1 块**，
> 这正是闸门能触发的关键。夹爪回读 25.08 / 25.46 / 26.13 mm（全部 ≫ 0 = 真夹住），
> `连续未放置 0/10`（**零次多余重试**）。
> `python3 scripts/board_a1.py --no-launch --log ~/a1_board.log --slots 4` → 退出码 0。
> 完整日志在板上 `~/a1_board.log`。
> ⏳ 这一轮没录 —— 后来在第九十轮补上了（见下）。

## 补录像（✅ 2026-10-05 已完成；下面是验证过的标准流程，可重跑）

**摆件（人做，约 1 分钟）** —— 上一轮的 4 块现在压在 4 个槽位上：

1. **把槽位上那 4 块拿回抓取区**（否则下一轮照着 `place_z=0.029` 直直压下去）；
2. 抓取区凑够 **5 块**，`x ≥ 0.46`（见上面「第四步：摆件」的复核结论）。

**次序（别改成「先按 PLAY 再起栈」）**：URCap 的 External Control 程序**必须先有对端**
才起得来 —— 没有驱动监听时按 PLAY，示教器直接显示「连接被拒绝」、程序停在 STOPPED
（实测守着轮询 5 分钟，`programState` 一直是 `STOPPED ur5ros.urp`）。所以只有这一条
合法次序：**起栈 → 按 PLAY → 连接后把控制器拉回来 → 规划器才起**。

最后一步**不能省**：驱动在 URCap 连上时会发一次 **STRICT** 切换，而 `friction_model_controller`
已经在 active ⇒ 整批 `Aborting, no controller is switched!` ⇒
`scaled_joint_trajectory_controller` 被留在 not-running ⇒ 规划器一发目标就当场收工
（报 `goal_status=<未知 None>`，**臂一下都不会动**）。第九十轮实测踩过。

**终端 1：起栈 + 录制开始**（`start_delay:=120`：驱动立刻监听，规划器 120 s 后才起，
中间这段时间留给按 PLAY 和下面的闸门）：

```bash
#   ⚠️ 七个 `camera_*` —— 上面那组是 **2026-10-05 的旧几何**，照抄＝把外参搞错，
#      症状是「候选落到臂够不到的地方」，读起来像检测坏了。现读当前值（只读）：
#          python3 scripts/board_run.py --task a1 --dry-run
export UR_ROBOT_IP=<控制箱IP>
ros2 launch yolo_bringup bringup.launch.py camera_source:=realsense \
    robot_ip:=$UR_ROBOT_IP planner_params_file:=$HOME/site_real.yaml \
    gripper_backend:=ag95 carry_objects:=true start_delay:=120 \
    tf_required:=true use_target_plane:=true target_plane_z:=0.0285 \
    grasp_point_mode:=top_face_center table_plane_z:=0.0 \
    publish_camera_tf:=true \
    camera_x:=0.34694477757210807 camera_y:=0.8693043936314018 camera_z:=0.6454397556505314 \
    camera_qx:=-0.32913450910630504 camera_qy:=-0.316374655430605 \
    camera_qz:=0.6185281368827056 camera_qw:=-0.6395314661538762 \
    2>&1 | tee ~/a1_video.log
```

**你（示教器）**：起栈约 12 s 后按**播放** —— 驱动已经在监听，这时按一定连得上。

**终端 2：闸门** —— 按完播放**立刻**跑下面这段（守 100 s，覆盖到规划器启动为止）：

```bash
end=$(( $(date +%s) + 100 ))
while [ "$(date +%s)" -lt "$end" ]; do
  ros2 service call /controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: [scaled_joint_trajectory_controller, friction_model_controller], \
      deactivate_controllers: [], strictness: 1, activate_asap: true, \
      timeout: {sec: 4, nanosec: 0}}" >/dev/null 2>&1
  sleep 4
done
```

> `strictness: 1`（BEST_EFFORT）是**必须**的：已经在 active 的控制器只会**告警**、
> 不会中止整批。板上**没装 `ros2 control` CLI**（`invalid choice: 'control'`），
> 所以只能走这个服务。

**臂开始动之前，先花 5 秒确认这一轮起栈是健康的**：

```bash
grep -c "Aborting, no controller is switched" ~/a1_video.log   # 必须是 0
grep -c "State tolerances failed"               ~/a1_video.log   # 最好是 0
```

> ⚠️ 别拿 `grep -E "STRICT|not inactive"` 的**计数**当判据 —— **通过的那一轮里它也有 20 处命中**，
> 全是 BEST_EFFORT 每拍跳过的良性告警。要看的是 `Aborting, no controller is switched!` 这一行**在不在**。

**录完判分**（不动臂）：
```bash
python3 scripts/board_a1.py --no-launch --log ~/a1_video.log --slots 4
```
退出码 0 = 通过。把 `~/a1_video.log` 和视频一起留档。

> ⚠️ 录像**不要**录到示教器屏幕上的控制箱 IP、也不要录到实验室可识别的门牌/人员
> ——本仓是公开的，脱敏自检**扫不了图片像素**，只能靠人眼。

---

