# Jetson 板子 SSH 访问（WSL 侧）

核对时间：2026-09-27。本文供 WSL 下的工具直接使用。

> 2026-09-27 补充核验：`ssh jetson` **实测连通**（返回 `CONNECT_OK` / `yahboom` / `aarch64`）。
> 同日在板端做了一次**只读**资产取回，见文末「板端已有资产」。

## 一句话

在 WSL 里直接 `ssh jetson` 即可免密登录，别名已配好，等价于：

```bash
ssh -i ~/.ssh/board_ed25519 jetson@<host-ip>
```

## 连接信息

| 项 | 值 |
| --- | --- |
| 主机名 | `yahboom` |
| IP | `<host-ip>` |
| 登录用户 | `jetson` |
| 密码 | 仅在密钥失效时使用。**本仓库不记录**（凭据只放本机未跟踪位置 —— 这条是硬规矩：别把口令贴进 issue / 提交信息 / 本仓任何文件） |
| 系统 | Ubuntu 22.04.5 LTS aarch64 / JetPack 6.2 / ROS 2 Humble |

## WSL 侧已就绪的状态

- WSL 与板子同网段直通：WSL 本机地址 `<ip>/23`，`ping <host-ip>` 约 1 ms。
- 私钥 `~/.ssh/board_ed25519`（权限 `600`）已在 WSL 内，免密登录实测通过。
- `~/.ssh/config` 已写入 `Host jetson` 别名；配置模板见 `docs/jetson_ssh_config.txt`。

## 常用命令

```bash
# 登录
ssh jetson

# 远程执行一条命令（板子 .bashrc 已 source ROS 环境）
ssh jetson 'source /opt/ros/humble/setup.bash && ros2 pkg list | wc -l'

# 查看板端资源与相机
ssh jetson 'tegrastats --interval 1000 | head -5'
ssh jetson 'source /opt/ros/humble/setup.bash && rs-enumerate-devices -s'

# 传文件 / 传目录
scp some_file.py jetson:~/target/
scp -r ./some_dir jetson:~/
rsync -av --exclude build --exclude install ./pkg/ jetson:~/pkg/
```

## 别名丢失时如何重建

配置模板已随本仓库跟踪（`docs/jetson_ssh_config.txt`），推荐直接用仓库里的这份：

```bash
mkdir -p ~/.ssh && chmod 700 ~/.ssh
cp docs/jetson_ssh_config.txt ~/.ssh/config      # 在仓库根执行
chmod 600 ~/.ssh/config
```

（原先记的 Windows 侧副本路径是
`/mnt/c/Users/<winuser>/Documents/ChatGPT/基于YOLO+Moveit的机械臂感知抓取闭环项目/docs/jetson_ssh_config.txt`，
现在仓库里就有一份，不必再依赖那条路径。）

## 注意事项

- 不要直接用 Windows 路径下的私钥（`/mnt/c/Users/<winuser>/.ssh/...`）：`/mnt/c` 权限是 `777`，SSH 会以 `permissions too open` 拒绝加载，必须在 WSL 内保留一份 `600` 权限的副本。
- 本仓库是 Git 跟踪目录，不要把板子密码、私钥内容或 SSH 配置中的凭据写进任何被跟踪文件；凭据只放本机未跟踪位置。
- 板子 `~/.bashrc` 里设了 `ROS_DOMAIN_ID=99`。如果要在 WSL 和板子之间互通 ROS 2 话题，WSL 侧也要设成同一个 `ROS_DOMAIN_ID`。
- 机械臂、控制器切换、真机运动属于高风险操作：未经用户当次明确授权且急停就绪，不得通过 SSH 触发真实运动。
- 通过 SSH 做部署或安装前先确认目标路径，不要覆盖板子上已有的 `~/yahboom_ws`、`~/ultralytics` 等目录。

### 补充三条（2026-09-27 核实）

- **只对齐 `ROS_DOMAIN_ID` 还不够。** WSL2 默认 NAT 网络下多播出不去，DDS 根本发现不到板上的
  节点。三种解法（镜像网络模式 / Fast DDS Discovery Server / CycloneDDS 单播）见
  [`board/UR5接入方案.md`](board/UR5接入方案.md) §7.1。这是本项目从 WSL 连板子时**最先撞上的坑**。
- **架构不同，二进制不能互拷。** WSL 是 `x86_64`，板子是 `aarch64`。源码同步过去**在板上编译**，
  或做交叉编译。纯 Python 包理论上可直拷，但只要有 `.msg/.srv` 或 C++ 就得重编。
- **板端资产已留档。** 2026-09-27 只读取回了 6 个 `arm-*.sh`、体检与改造清单、UR5 接入方案，
  存于 [`board/`](board/README.md)。其中四个 `*-sudo.sh` 会改系统状态（含**分区表**），
  **不要顺手执行**。

## 板端已有资产（只读留档）

| 位置 | 内容 | 留档于 |
| --- | --- | --- |
| `~/arm-*.sh`（6 个） | 性能调优 / 体检 / 资源隔离 / 无头检查 / 清理 / 扩容 | [`board/`](board/README.md) |
| `~/Documents/Codex/2026-09-25/…/outputs/` | 体检与改造清单（母文档）、UR5 接入方案 | [`board/`](board/README.md) |
