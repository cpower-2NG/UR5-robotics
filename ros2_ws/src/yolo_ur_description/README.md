# yolo_ur_description

UR5 描述的**覆盖包**。存在的唯一理由是：把 `ur_description` 里那份**标称**
运动学换成**这台机器人**的出厂标定。

它在 `real.launch.py` 里通过 `description_package:=yolo_ur_description`
同时喂给驱动和 MoveIt，见下。

---

## 为什么需要它

### 症状

驱动启动时会拿 URDF 里的 `kinematics/hash` 去和机器人通过主接口报上来的哈希比对，
不等就打一条 ERROR：

```
[URPositionHardwareInterface]: Calibration checksum: 'calib_209549117540498681'.
The calibration parameters of the connected robot don't match the ones from the given
kinematics config file. Please be aware that this can lead to critical inaccuracies of
tcp positions.
```

那条 checksum 是 **URDF 的**，不是机器人的（`hardware_interface.cpp:800` 打印的是
从参数里读到的 `kinematics_hash`，`:804` 的 `checkCalibration()` 才拿它去和机器人比）。
机器人那份是 `calib_2353869974907916917`。

### 机制

```
ur_description/urdf/ur.urdf.xacro:15
   kinematics_params 默认 → config/<ur_type>/default_kinematics.yaml
        ↓  ur_common.xacro:241  kinematics_hash = sec_kinematics['hash']
        ↓  ur_macro.xacro:126   → ur.ros2_control.xacro:60
   <param name="kinematics/hash">…</param>
        ↓  hardware_interface.cpp:644 读参数、:800 打印
        ↓  :804 checkCalibration()  与 PrimaryClient::checkCalibration
                                    从机器人读到的 KinematicsInfo::toHash() 比字符串
```

所以「URDF 对不上」= **URDF 用的是 UR5 的标称 DH，机器人用的是出厂实测 DH**。
出厂标定在 PolyScope 里能看到，但不会自动进 ROS 侧的模型。

### 到底差多少（实测，不是估计）

把同一份 xacro 只换 `kinematics_params` 生成两份 URDF，在同一批随机关节角下做 FK 比
`tool0`（`scripts/ur_calibration_delta.py`，3000 组 ±180° 均匀采样，结果存档在
`docs/calib/ur5_calibration_delta.json`）：

| 量 | 均值 | 中位 | p95 | **最大** |
|---|---|---|---|---|
| `tool0` 位置偏差 | 1.6161 mm | 1.5643 mm | 2.9649 mm | **3.5945 mm** |
| `tool0` 姿态偏差 | 0.1894° | 0.1934° | 0.3137° | **0.3923°** |

单关节 origin 里最大的是 `wrist_1_joint` 的 **Δz = +1.4200 mm**（其次 Δy = −0.2165 mm），
`elbow_joint` Δx = −0.2179 mm，`shoulder_pan_joint` Δz = +0.1555 mm。

判据用**最大**，不要用均值 —— 位置偏差是各连杆长度误差的矢量和，随关节角变化，
工作区边缘最大。

### 后果：不致命，但半修等于没修

- **不致命**：驱动打完这条 ERROR **继续正常工作**（紧接着就是 `System successfully
  started!`）。它不是安全/碰撞问题，是精度偏差。
- **但是**：`tool0` 往下的一切（IK 目标、抓取高度、手眼投影）都带着这 1.6~3.6 mm。

**关键一点：只修驱动侧或只修 MoveIt 侧，等于什么都没修。**

驱动和 move_group 各拿一份 `robot_description`。两边都标称时，IK 按标称解、真机按标称
的逆解去动 —— 末端落在目标 δ 外。只把驱动换成标定，TF 准了，但 IK 还是按标称解，末端
**照样**偏 δ。只有两份**都**换成标定，IK 与 TF 才是同一套几何，δ 才真的消掉。

所以这里不是"驱动侧修一下就行"，而是必须让两个 launch 用同一份描述。
这正是 `real.launch.py` 的 `_moveit` 注释里那条警告（"两份 description 不一致时，
move_group 的 FK/IK 与驱动实际发布的 TF 是**两套几何**"）在说的事。

---

## 这个包是怎么做的

```
urdf/ur.urdf.xacro          ← 构建时从已安装的 ur_description 镜像
config/                     ← 构建时从已安装的 ur_description 镜像，
                              但**排除**所有 default_kinematics.yaml
config/ur5/
  default_kinematics.yaml   ← 本包唯一自有的内容：这台 UR5 的出厂标定
```

镜像在 `CMakeLists.txt` 里用 `install(DIRECTORY … PATTERN "default_kinematics.yaml"
EXCLUDE)` 做，**仓库里不存上游文件的副本** —— 存副本就会「上游升级、我们的副本悄悄
过期」，而这里镜像的每一个文件都必须与上游逐字节相同：两份 description 只要差一个
字节，就是上面说的两套几何，且两边都不报错。

被复制过去的 `ur.urdf.xacro` 内部所有 `xacro:include` 都是
`$(find ur_description)/…` 的绝对形式（`:6`、`ur_macro.xacro:53,54,112`），所以它照样
引用真正的 `ur_description`，这份副本本身很薄。

### 只支持 ur5

`config/` 里只有 `ur5` 的标定。用别的 `ur_type` 会在 xacro 阶段
「找不到 default_kinematics.yaml」直接失败 —— 这是刻意的：**失败比静默用标称值好**。

---

## 标定是怎么抓出来的

1. `ur_calibration` **2.14.0 / 2.15.0 都跑不起来**：`run()` 调了 `pipeline.run()` 却漏了
   `pipeline.init()`，而 `setupProducer()`（唯一调 `stream_.connect()` 的地方）只被
   `init()` 调用。结果是 stream 永远没连上，生产者线程无限刷
   `Stream is invalid. Connect it before trying to read!`。上游 `humble` 分支 HEAD 已修
   （同一行），但**没有任何已发布版本包含它**，`apt upgrade` 也拿不到。
2. 板上 `~/ur_calib_ws` 是 tag `2.15.0` 的检出 + 那一行补丁，
   `colcon build --packages-select ur_calibration` 得到能用的 `calibration_correction`。
3. 用它的 ROS 参数接口（**不是** `--robot_ip` 那种命令行旗标）跑一次只读抓取：

   ```bash
   ros2 run ur_calibration calibration_correction --ros-args \
       -p robot_ip:=<ur-controller-ip> -p target_filename:=/tmp/ur5_calibration.yaml
   ```

   它只读机器人（主接口 30001 上的一次标定查询），臂不动。

那一次读回来的原始数据（在工具日志里，是**另一套参数化**，不等于上面的 yaml）：

```
dh_theta: [4.0e-05, 0.4132, -0.2376, -0.1755, 5.3e-05, -2.6e-05]
dh_a:     [8.0e-05, -0.3894, -0.3862, 1.2e-04, -2.2e-05, 0]
dh_d:     [0.0893, 64.2024, -29.7635, -34.3286, 0.0948, 0.0824]
dh_alpha: [1.57006, 0.00266, 0.00199, 1.57019, -1.56980, 0]
calibration_status: 2
```

`dh_d` 里的 64.2 / −29.76 / −34.33 看着吓人，但那是**原始 DH**；
`Calibration::correctChain()` 会把它转成 URDF 约定下的连杆变换，才是上面那份 yaml。
yaml 里所有连杆偏差都 ≤1.42 mm、角度 ≤0.14°，是合理的出厂量级。

---

## 手眼标定不用重做

`docs/calib/外参-B路-臂点法.md` 里那四个角点的 `base_link` 系坐标是**从示教器上读的**，
即来自 **URControl 自己的（已标定的）FK**，不是我们的 URDF。所以修 URDF 不影响那份手眼
结果 —— 这是这次能放心改的前提。

---

## 维护

- **上游 `ur_description` 升级后**：不用做任何事，重新 `colcon build` 就会镜像新版。
- **换机器人 / 重做标定后**：重新跑一次上面的 `calibration_correction`，把
  `config/ur5/default_kinematics.yaml` 整个替换掉（新文件的 `hash:` 会跟着变）。
- **怎么确认接上了**：驱动启动日志里那条 ERROR 应该消失。不用连真机也能先验一半 ——
  生成一次 description，看 `kinematics/hash` 是不是 `calib_2353869974907916917`：
  ```bash
  xacro $(ros2 pkg prefix yolo_ur_description)/share/yolo_ur_description/urdf/ur.urdf.xacro \
      ur_type:=ur5 name:=ur5 \
      kinematics_params:=$(ros2 pkg prefix yolo_ur_description)/share/yolo_ur_description/config/ur5/default_kinematics.yaml \
      joint_limit_params:=… physical_params:=… visual_params:=… | grep -o 'calib_[0-9]*'
  ```
