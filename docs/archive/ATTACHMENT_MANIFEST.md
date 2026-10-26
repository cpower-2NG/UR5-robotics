# 教学附件清单与哈希

建立日期：2026-09-26（P0 阶段）
基线提交：`f250af11b77b1629b4e867eb7820912db66dbc2c`（`main`）
当前分支：`migrate/humble-orin`

> **用途**：冻结附件与模型权重的身份，防止后续整理时误改、误删或张冠李戴。
> 解压副本位于 `docs/reference/`，已在 `.gitignore` 中排除，**不得提交入库**。

---

## 1. 附件文件哈希

路径前缀：`04-Chapter（参考）+官方统一要求/`

| 文件 | 大小 | SHA-256 |
|---|---|---|
| `ur5-yolo-grasp最终提交包/ur5-yolo-grasp.zip` | 5.7 MB | `612abcbb0d2e637a22750a7e7302d15a9d9bec124b9e23e30a8d04c314c3c49f` |
| `ur5-yolo-grasp最终提交包/UR5 YOLO+IK抓取仿真教材.docx` | 590 KB | `512a270cb8e4f545d69c0f58f2fe79ef879f0320f6348f6e53f4b1940b1aa916` |
| `ur5-yolo-grasp最终提交包/UR5 YOLO+IK抓取仿真系统.pptx` | 1.35 MB | `884e4c23e0c08baf97f6fe1883368b5a77a771cfcb7cfe97e9b56b05ce3a1834` |
| `ur5-yolo-grasp最终提交包/ur5-yolo抓取演示视频.mp4` | 64 MB | `cc43e744dde4661c378849da6169d67c1340bc0c6f41d7e612892e9b1a036153` |

对应 Git blob（`git rev-parse HEAD:<path>` 可复核）：

| 文件 | Git blob hash |
|---|---|
| `ur5-yolo-grasp.zip` | `16513c2f94de6b88dc3c895cf7e949f38aa830a2` |
| `ur5-yolo抓取演示视频.mp4` | `b9dea70cbcfe9990869ee1b4dcd8593bdfae82f6` |

**入库时间**：以上附件**全部由 `f250af11b`（"标准化"，最后一个提交）新增**。此前 7 个提交中不存在。这意味着抓取实现从未出现在工作区的提交历史里，只以附件形式存档。

---

## 2. 解压副本

```
docs/reference/ur5-yolo-grasp/          （27 个文件，已在 .gitignore 中排除）
├── grasp_sim/                          ament_cmake 包：UR5 + Robotiq 2F-85 + MoveIt2
│   ├── config/    ros2_control.yaml · kinematics.yaml · ur5_gripper.srdf.xacro
│   │              gz_bridge.yaml · hardware_plugins.xml
│   ├── models/    ur5_with_gripper.urdf.xacro
│   ├── worlds/    grasp_world.sdf
│   ├── launch/    sim.launch.py · full.launch.py
│   └── scripts/   grasp_planner.py (591 行) · view_camera.py
├── yolo_vision/                        ament_python 包：检测 + 自带 3D 投影
└── LAB.md · TEXTBOOK.md · README.md
```

---

## 3. 模型权重：存在两个不同版本（重要）

对 5 处方块模型做 SHA-256 后，确认**不是同一份文件的多副本，而是两个不同训练产物**：

| 模型 | SHA-256 | 大小 | 出现位置 |
|---|---|---|---|
| **A（较新，域随机化）** | `a926d959b5aa9f9e2435c2deb9666803a32040dae1177cd85dc2cff96e29e467` | 6,242,538 | `block_best.pt`<br>`yolo_vision/models/best.pt`<br>`runs/block_detect/weights/best.pt` |
| **B（较旧，方阵演示）** | `b9baf6f168dceebd54560cd9fdc9a54ad2f5a62e7d8f6acff323394942743b87` | 6,236,074 | `runs/detect/block_grid_demo/weights/best.pt`<br>**附件 `docs/reference/ur5-yolo-grasp/yolo_vision/models/best.pt`** |

**推断依据**：模型 A 出自 `runs/block_detect/`，该目录对应 `a2dbfdeec 完成域随机化` 与 `7fce73a83 完成域随机化改造` 两个提交，即主线后期训练的域随机化版本；模型 B 出自 `runs/detect/block_grid_demo/`，对应更早的方阵演示阶段。

**影响**：附件抓取栈自带的是**较旧的模型 B**，而主线有**较新的域随机化模型 A**。P3 接入抓取时**应采用模型 A**（域随机化对 sim-to-real 更有价值），不要因为「跟附件保持一致」而回退到 B。

> 待补：`runs/block_detect/args.yaml` 与 `results.csv` 可用于补全模型 A 的训练配置（epochs / 增强参数），P1-3 时一并记录。

---

## 4. 两套 `yolo_vision` 同名不同内容（重要）

仓库根 `yolo_vision/` 与附件的 `yolo_vision/` **包名相同、内容不同**：

| | 仓库根 `yolo_vision/` | 附件 `yolo_vision/` |
|---|---|---|
| 定位 | 纯 2D 检测 | 检测 **+ 3D 投影**（`/detections` 的 `ObjectHypothesisWithPose.pose` 携带相机系 3D 坐标） |
| 订阅 | `/camera/color/image_raw` | RGB **+ 深度 + camera_info** |
| bbox 约定 | **归一化**（`/w_img`, `/h_img`，行 98-100） | **归一化**（行 190-193，与仓库版同） |
| 图像转换 | `cv_bridge` | **手工 numpy 转换，不依赖 cv_bridge** |
| 行数 | 约 230 | 约 250 |

> **同名冲突**：两者包名都是 `yolo_vision`，**不能共存于同一工作区**。P3 解压附件时必须先处理仓库根的同名包。
> 已确认用户决定删除仓库根 `yolo_vision/`，冲突自然消解。
>
> 附件的「不依赖 cv_bridge」值得保留借鉴——Jetson 上 cv_bridge 的 numpy ABI 问题（numpy 1.x/2.x）是常见坑，手工转换可绕开。

---

## 5. 关键发现：附件的 3D 投影有与主线相同的缺陷

附件 `yolo_vision/detector_node.py` **自称**做 3D 定位，但核对后确认它**同样没有处理深度尺度**：

```python
# docs/reference/ur5-yolo-grasp/yolo_vision/yolo_vision/detector_node.py
125:    def _depth_callback(self, msg: Image):
126:        self._latest_depth = _msg_to_numpy(msg)      # 原样存，不看 encoding、不换算
...
233:        zc = float(self._latest_depth[vi, ui])       # 直接当米用
```

与主线 `perception_3d/projector_node.py` 的 D1/D2 缺陷**完全同类**：深度独立缓存（无时间同步）、raw 值直接当米。

**结论**：换用附件实现**不能规避** P2-1 / P2-2 的修复，两条路都必须修。这进一步支持选择路线 **(B) 抽出统一 `perception_3d`**——把修复集中做一次，而不是在两处各修一遍。

---

## 6. 待办

- [ ] P1-3：用本文件的哈希收敛 13 组权重，只保留模型 A + 一份 COCO 基线
- [ ] P1-3：补全模型 A 的训练配置（读 `runs/block_detect/args.yaml`）
- [ ] P3：解压附件 `grasp_sim` 入库时，**不要**连带解压附件 `yolo_vision`（与仓库根同名，且其 3D 逻辑将由统一 `perception_3d` 取代）
- [ ] 确认 `runs/block_detect/weights/epoch*.pt`（20 份 × 24 MB）是否可弃
