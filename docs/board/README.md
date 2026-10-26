# board/ —— Orin 板端资产（原样留档）

这里的文件**不是本仓库开发的代码**，而是本项目在 Orin 板子上做的部署准备。
它们原先**只存在于板子家目录**，板子一重装就全没了，故在此留档。

**来源**：2026-09-27 通过 `ssh jetson` **只读**取回（`scp`），内容未做任何修改。
连接方式见 [`../JETSON_SSH.md`](../JETSON_SSH.md)。

## 文件

| 文件 | 是什么 | 要不要 sudo |
| --- | --- | --- |
| `arm-ready.sh` | 把 Orin 调到「机械臂控制」高性能态：CPU 调度器锁 `performance`（开机持久）、`swappiness=10` | **要** |
| `arm-check.sh` | 链路快速体检：CPU/温度/内存/磁盘/串口/ROS 环境，还探一个内部 LLM 端点 | 不要 |
| `arm-isolation-sudo.sh` | 资源隔离（体检清单 **D4**）：控制栈绑 4-5 号核 + 内存保底 1G；桌面压到 0-3 核 | **要** |
| `arm-headless-check.sh` | 无头运行（只接电源+网线）可行性体检：自动登录 / DRM 状态 / 分辨率 / 远程入口 | 不要 |
| `arm-cleanup-sudo.sh` | 关停无关服务、启 ufw、清 apt、swap 瘦身 8G→2G、根分区扩容、透明大页改 madvise | **要** |
| `arm-grow-root.sh` | 单独把根分区扩到整块 NVMe（清单 **B8**），含分区表备份与失败回滚 | **要** |
| `机械臂板卡-体检与改造清单.md` | **母文档**（305 行）。上面几个脚本逐条对应它的 A/B/C/D 项；`arm-isolation-sudo.sh` 头部「体检清单 D4」指的就是它 | — |
| `UR5接入方案.md` | UR5 真机接入方案：网络两种接法、板端要装什么、示教器侧做什么、联调顺序、**WSL2 侧 DDS 发现不到板子的坑** | — |

## ⚠️ 安全须知

- **四个 `*-sudo.sh` / `arm-ready.sh` 会改系统状态**（CPU 调度器、cgroup 隔离、ufw 规则、
  swap 文件、**分区表**）。它们**不是**本项目构建/运行的一环，**不要顺手执行**。
- `arm-cleanup-sudo.sh` 与 `arm-grow-root.sh` 会动 **`/dev/nvme0n1` 的分区表**。
  两者都自带备份（前者备份 `fstab`，后者备份分区表到 `/root`）与失败回滚，
  但**动分区仍有断电风险**，执行前须确认供电稳定。
- `arm-isolation-sudo.sh apply` 会把你**正在用的桌面**压到 4 个核。脚本自己的注释就写了
  「等真正开始跑 UR5 控制节点时再执行更合适」——**别在开发时跑**。
- 与机械臂运动相关的部分：**未经当次明确授权且急停就绪，不得触发真实运动**。
- 本目录所有文件均已扫描确认**不含密码、私钥、令牌等凭据**。

## 与本仓库的关系

- 这些是**板端运维脚本**，不进 `ros2_ws/`，不参与 `colcon build`，不影响仿真。
- 与迁移直接相关的是 `UR5接入方案.md` 与 `机械臂板卡-体检与改造清单.md` 的
  §3（UR5 接入）与 §5-C（逐项确认）。
- 板子的版本事实（22.04 / Humble / aarch64）是**本项目全线用 Humble 的依据**，
  见 [`../archive/MIGRATION_HANDOFF.md`](../MIGRATION_HANDOFF.md) 与 [`../RESTRUCTURE_PLAN.md`](../RESTRUCTURE_PLAN.md) §3 P4。
