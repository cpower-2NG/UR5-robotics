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

> ⚠️ **同名节点必须只剩一个**：`pgrep -cf dashboard_client` —— 拆栈后应为 **0**，新栈起来后应为 **1**。
> 多于 1 说明有孤儿（实测：整栈重启后 2026-11-05 的旧节点 PPID=1 还活着），两个同名节点抢同一个服务，
> `programState` 查询会**随机时通时不通**，阶段 3 的前置闸门会因此拒跑（fail-closed，不会误发轨迹）。

> ⚠️ **控制箱重新上电后先重连 dashboard**：栈没动过也会失联（TCP 随控制箱断，节点进程还在）。
> `ros2 service call /dashboard_client/connect std_srvs/srv/Trigger "{}"`；不重连的话执行闸门读到的是
> `Did not receive answer from dashboard server in time.(Configured timeout: 20 sec)`，整轮在发第一条轨迹前停（2026-11-12 实测）。
>
> ⚠️ **顺序别反**：拆栈会停掉示教器上的 `ur5ros.urp`。合法次序只有一条：
> **起栈（驱动先监听）→ 示教器按 PLAY → 连接后把控制器拉回来 → 规划器才起**。
> 没有驱动监听时按 PLAY，示教器直接「连接被拒绝」，程序停在 STOPPED（实测守 5 分钟也一样）。

## 第三步：分两段起 —— 先「不动手」，预检全绿再「动手」

**起栈前先锁 GPU 最高频**（2026-11-08 用户拍板，见入口实验 E6）：

```bash
sudo jetson_clocks          # GPU 306 → 1173 MHz；RNGNet 板载 114.3 → 57.6 ms
```

> ⚠️ **重启即失效**，每次起栈前重跑。默认 governor（`nvhost_podgov`）下短帧跑不到满频，
> RNGNet 板载延迟会从 57.6 ms 涨到 114.3 ms，**超 E6 的 100 ms 门**。
> 想恢复默认：`sudo jetson_clocks --restore`。

```bash
# 板上 —— 第一段：驱动 + MoveIt + 感知，但不自动抓
export ROS_DOMAIN_ID=99            # ⚠️ 现场跑在 99，不是默认 0
cd ~/yolo_ros_original/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
# ⚠️ 七个 camera_* 从站点配置现读（site.yaml 的 topcam.extrinsics），别手拼、别照抄旧文
#    本栈里 `/camera/` 就是俯视相机（所以参数叫 camera_*）；阶段 4 的 real.launch.py 里叫
#    topcam_*，读的是**同一块** site.yaml —— 起栈脚本 `~/precision/restart_stack3.sh`
#    已改成现读（2026-11-08），不再硬编码，两个入口从此只有一个数。
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

## 第五步：跑阶段 3 堆叠（已完成，2026-11-06 出口）

**阶段 3 六轮验收已达成**（3 块×3 + 5 块×3 全部 `verdict=ok`），当前**没有要跑的轮次**，
所以队列里不再有条目。需要复跑某一轮时，命令从仓外
`~/robotics/notes/ur5-vision-grasp/待执行命令-留档.md` 里「最终版第 NN 轮」那一节原样取，
**并按当次重解垫位、重核夹爪、重新走逐次授权**。
开跑前的四条通用前置见本文件第三步与 `待执行命令.md` 顶部表；`--go` 前的当次授权
按 `AGENTS.md` §1 红线 6。

## 第六步：阶段 4 正式轮（S0，整轮编排）

**一轮 = 一个布局的全部指定块，一次授权跑完「观察 → 计算 → 执行 → 判分」**。
唯一执行入口是 `scripts/stage4_round_run.py`（开发机跑，经 SSH 驱动板子）；
它不复制运动几何，只做阶段编排与闸门。契约见
[`阶段/阶段4-杂乱指定抓取/契约.md`](阶段/阶段4-杂乱指定抓取/契约.md) §4.3–§4.7。

### 1. 桌面布置（人）

- 定稿布局与块位（base 毫米 / yaw）：唯一出处是 `scripts/stage4_layouts.py`
  的 `S0_SELECTED`；**别抄文档里的表**（会漂）。
- **摆位引导 = 腕部相机高位照片**（2026-11-11 用户定，不再打印 1:1 图 —— 现场只有 A4，
  1:1 放不下 325×250 mm 的拾取区）。分两步，都在摆块**之前**：

  > ⚠️ **腕部 TF 要单独起**：阶段 4 的起栈脚本 `~/precision/restart_stack3.sh` 传
  > `publish_camera_tf:=false`，**不发布** `tool0 → camera_link`；不起它，
  > `precision_capture --burst` 会 fail-closed 报 `TF 链缺 ['tool0:camera_link']`
  > （2026-11-11 实测，臂已到位但 burst 失败，臂停在观察位）。
  > 板上常驻 + 验收（只读，不碰臂）：
  >
  > ```bash
  > setsid nohup ~/precision/run.sh ~/stage4-entry/scripts/wrist_tf_publish.py \
  >   > ~/precision/logs/wrist-tf.log 2>&1 < /dev/null &
  > ~/precision/run.sh ~/stage4-entry/scripts/wrist_tf_publish.py --verify   # rc=0 才算
  > ```
  >
  > `--verify` 会逐位比对 `tool0 → camera_link` 与仓内标定
  > `docs/calib/wrist_eih_camlink-2026-10-24.json`，并确认 `base_link →
  > camera_color_optical_frame` 可达。**别**另起 `publish_camera_tf:=true` 或
  > easy_handeye2 的 handeye_publisher —— `camera_link` 会有两个父节点。

  ```bash
  # ① 臂到拾取区上方的 survey 机位、拍一组腕部 burst、回原位（真机运动，要当次授权）
  python3 scripts/stage4_placement_view.py --plan \
    --urdf ~/robotics/notes/ur5-vision-grasp/runs/2026-10-25-precision/robot.urdf \
    --from-frame <run>/joints-now --out-dir <run>/setup-view \
    --board-dir ~/precision/plans/e9/setup-view \
    --board-log-dir ~/precision/logs/e9/setup-view
  #   然后单独做一次 STOP → PLAY 授权（这一步也是真机运动，红线 6 逐次有效）：
  python3 scripts/real_motion_authorize.py --step stop
  #      >>> 需要人动手：示教器 STOP
  python3 scripts/real_motion_authorize.py --step play
  #      >>> 需要人动手：示教器 PLAY（两步都核验到才写令牌）
  ssh jetson-hotspot 'bash ~/precision/plans/e9/setup-view/setup-view.sh <token>'
  #   setup-view.sh = 到机位 → burst 20 帧 → 回原起点；令牌剩余 <120 s 在开头就停
  #   把 burst 拉回开发机（照片叠加在离线跑）：
  rsync -a jetson-hotspot:precision/e9/setup-view/wrist-pick/ <run>/setup-view/wrist-pick/

  # ② 把本轮布局的目标足迹叠到那张照片上（离线，不动臂）
  python3 scripts/stage4_placement_view.py --batch S0 --layout 1 \
    --burst <run>/setup-view/wrist-pick --out <run>/setup-view/s0-1.png
  ```

  PNG 上：青线 = 拾取区边框、白线 = 100 mm 网格、彩色框 + 对角线 = 目标块足迹与朝向、
  白色十字 = 块心、左下 = 50 mm 比例尺。人**照着屏幕**把块摆到框里即可。
  照片是**引导不是量具**（500 mm 高、约 1.82 px/mm，肉眼对位就够）：手摆目标本来就是
  ±3–5 mm / ±10°，不需要 1:1。
- 某块出画幅（照片上打 `!! 目标出画幅`、命令 rc=1）⇒ 那张照片引导不了它，先查相机位姿 /
  拾取区参数，别照着猜。
- 照片拍的是**空台面**；标完不留任何纸/胶带在块下（会污染深度与彩色）。
- 手摆目标 **±3–5 mm / ±10°**；**不跑前实测、不阻断**，偏差跑后由
  `stage4_placement_deviation.py` 用跑前 pick burst 真值量、只归档。
- 摆好退开、手离开台面，再说「摆好了」；摆场景不算人工干预。

### 2. 每轮预检（编排器阶段 0，只读）

板子身份（模组序列号）/ 板上全树哈希 / 栈（驱动、MoveIt、腕部相机 USB3）/
夹爪 `0x0200 == 1` / 急停与顶灯。任一红 ⇒ 编排器不授权、不运动。
预检期望值由人先写一份 JSON（`--expect`），不给就判红 —— fail-closed。
部署树一变（**新增一份文档也算**），旧基线必然判红：用
`scripts/stage4_preflight_expect.py --repo <worktree> --from <旧基线> --out <新基线>`
只重算 `tree` 两项，`serial` / `stack` 原样继承（别手改 JSON，容易把「认亲」判据改宽）。
板上预检与本机重算共用同一份哈希代码（`PREFLIGHT_TREE_CODE`），不是两个口径。
**部署树里不许写日志 / 产物**（会让 `tree` 哈希假红）：日志一律写
`~/precision/logs/`，板上留档放 `~/stage4-entry-local/`（2026-11-12 清理过一次：
5 个历史文件从部署树移出）。

### 3. Round 0 计时干跑（S0 六轮之前，只做一次，不计入六轮）

**先离线排练（不碰板子、不碰臂、不要授权）** —— 机械臂关机也能跑，用的是上一轮
观察段落的 burst 与一份已跑过的探针报告：

```bash
python3 scripts/stage4_round_run.py --batch S0 --layout 1 --run 0 \
  --out-dir ~/robotics/notes/ur5-vision-grasp/runs/<date>-stage4-s0/round0 \
  --urdf <robot.urdf> --board jetson-hotspot --board-repo '~/stage4-entry' \
  --pad-spec ~/.config/ur5-vision-grasp/pad-spec.json \
  --bias-seed -2.791,-0.248 \
  --probe-report <round>/probe.json \
  --dry-run --from compute --to compute
```

判据：`解出 20/20 帧`、五个目标各出一份计划、8 份计划（5 目标 + 3 机位）、
`timing.json` 落盘。**干跑不执行、不动臂 ⇒ 不要令牌**（`timing.r1.skipped = "dry_run"`）；
带 `--probe-report` 时连板子都不碰。先在这一步把代码缺陷修干净，再去要真机授权。

真机计时（**要授权**：观察段会让臂走到 pick / pad 机位）—— 授权后只跑到阶段 4
（观察 / 解垫 / 探针 / 规划），**不执行**：

```bash
python3 scripts/stage4_round_run.py --batch S0 --layout 1 --run 0 \
  --out-dir ~/robotics/notes/ur5-vision-grasp/runs/<date>-stage4-s0/round0 \
  --urdf <robot.urdf> --board jetson-hotspot --board-repo '~/stage4-entry' \
  --expect <preflight-expect.json> --pad-spec ~/.config/ur5-vision-grasp/pad-spec.json \
  --bias-seed -2.791,-0.248 --dry-run
```

产物 `timing.json` 给出 `T_observe / T_pad / T_probe / T_plan / T_exec_est`；
后续轮用 `--timing-prior <round0>/timing.json` 标定 R1/R2。
**2026-11-11 离线实测**（S0:1，复用同一批 burst 与探针报告）：解垫 0.8 s、规划 8.5 s、
整段 compute 9.3 s、`T_exec_est` 206 s；探针单独一趟 6.1 s。
（2026-11-12 起解垫改在板上算：3.1 s；多出的 ~2 s 换掉轮内 20 MB 搬运。）

### 4. 授权（人，一轮一次）

编排器阶段 2 打印本轮摘要（布局 / 5 个目标 / 5 个格位 / 跑前两机位 / 跑后三机位 /
预计时长）后：

> **>>> 需要人动手：确认两指整个行程内无手无遮挡、急停在手，然后示教器 STOP → PLAY**

AI 只读核验两步（Dashboard 29999），都看到才写 600 s 令牌；没看到 ⇒ 不发（fail-closed）。
授权的范围覆盖本轮**观察 → 执行 → 终态评分**三段；轮内任何偏离（重规划 / 重试 /
换目标）⇒ 停、重新授权。

### 5. 两道时间闸门（600 s 硬窗口）

- **R1**（观察后、探针前）：`剩余 ≥ C_est + T_exec_est + 60 s`；
- **R2**（执行起手）：`剩余 ≥ T_exec_est + max(45 s, 15%·T_exec_est)`；
- 板端 `run.sh` 首行 `--min-remaining` 守卫；执行中过期 ⇒ `aborted_token`
  （本轮 void、不计 streak、不记抓取失败）。
- 不过闸门 ⇒ 停在执行前，`timing.json` 落盘，重做 STOP→PLAY。**不自行改 TTL**。

### 6. 执行与失败处理

`run.sh` 一条脚本跑完全部目标 + 跑后三机位 burst，Agent 不介入、**不自动重试**。
任一 rc≠0 / 碰撞 / 掉落 / 指定块丢失 / 夹爪未初始化 / 程序自停 ⇒ 停轮。
执行后编排器自动判分并归档：`round.json` / `round-verify.json` / `score.json/md` /
`bias.json` / `placement-deviation.json`。缺证据 ⇒ `complete=false` + 退出码 2。

**分析段在板上算**（2026-11-12 起，`--analysis board` 缺省）：解垫 / 跑前真值 /
跑后三机位真值 / 彩色质心全由板上 `scripts/stage4_board_analysis.py` 跑，轮内只回传
KB 级 JSON；板上实测解垫 3.1 s、20 帧真值 7.8 s（`--jobs 6`，线程钉 1）。
**轮内不拉任何 burst** —— 五组图像（约 44 MB）在归档段（整轮结束、令牌窗口之外）
才 rsync 回开发机。板上分析失败 = 停轮，不许回退本机算。

### 7. 连续判分

S0 六轮全部落盘后，只喂 S0 的 `round.json`（`aborted_token` 的轮先剔除），
按时间顺序算连续 3 轮：

```bash
python3 scripts/stage4_score.py --exit --need 3 <s0-*-r*/round.json ...>
```

---

## 已知坑速查（详细证据在台账）

| 症状 | 先查 |
|---|---|
| 发目标臂不动、`goal_status=<未知 None>` | `Aborting, no controller is switched` —— URCap 连上后的 STRICT 切换把控制器留在 not-running；按 PLAY 后立刻跑控制器闸门 |
| 话题列表是空的 | `ROS_DOMAIN_ID` 没设成 99（查询侧问题） |
| 候选落到臂够不到的地方、检测却高分 | 相机外参照抄了旧数字 / 用了 `extrinsics-2026-10-03` 那份的 `q`（⛔ 不可用，见当前状态 §3） |
| 相机没数据、日志 30 秒刷 7 万行 | 有孤儿进程占着相机（`ros2 topic info --verbose` 看 Publishers，别信「话题还在」） |
| burst 报 `TF 链缺 ['tool0:camera_link']` | 腕部 TF 发布者没起（`pgrep -af wrist_tf_publish`）；板上起 `~/stage4-entry/scripts/wrist_tf_publish.py`，`--verify` rc=0 再重拍 |
| `programState` 时通时不通（`Failed to send request to dashboard server`） | `pgrep -cf dashboard_client` 是不是 >1 —— 孤儿与新节点同名抢服务（2026-11-06 实测） |
| 控制箱**重新上电**后 `programState` 报 `Did not receive answer from dashboard server in time`（节点进程活着、`node list` 也有它） | dashboard_client 的 TCP 连接随控制箱断掉、不会自己重连。**不要重起整栈**：`ros2 service call /dashboard_client/connect std_srvs/srv/Trigger "{}"`，回 `success: True` 后 raw_request 立刻恢复（2026-11-12 实测）。**2026-11-13 补充**：控制箱**长时间离线 + 栈重启后**，dashboard_client **进程可能直接不在**（当日实测 `pgrep -cf dashboard_client` = 0）——此时 connect 无处可调，**重启整栈**（`restart_stack3.sh`），并先按「同名节点必须只剩一个」清孤儿（`SIGTERM` 杀不死的旧实例用 `kill -9`） |
| 抓帧等不到 TF，报缺整段 `base_link→…→tool0`（robot_state_publisher 进程在、ELAPSED 在涨） | 它的 **DDS 参与者**掉了：`ros2 topic info /tf` 的 publisher 数掉到 1（正常 2）。`bash ~/precision/restart_stack3.sh` 重起（会核对每个节点恰好一份）；这是机器人侧故障，**不是相机/参数问题**（2026-11-12 实测） |
| 判分把示教器/台外杂物当成一块积木，「全桌终态」假红 | 真值工具的保留区是「本机位 zone 外扩 25 mm」（`truth_zone_for`，2026-11-12 改）；被喂整张台（x 到 0.80 m）时会把示教器当 42 mm 高的青块（785, 509 mm）。看 `--zone` 是否被改回大区 |
