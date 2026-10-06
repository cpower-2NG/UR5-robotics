#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真机侧公共件：站点配置、进程残留、串口/夹爪/Dashboard 只读查询。

**顶层不 import rclpy，也不 import launch。** 本机（WSL）的 `/opt/ros/humble`
下根本没装 rclpy，顶层一旦 import，`test_board_lib.py` 就在离线层跑不起来 ——
而离线层是这套东西**唯一**能成测的地方。所以：判定逻辑一律**纯函数**，I/O 收在
薄适配器里并留注入口（`leftovers(ps_out=…)`），变异测试要能只喂数据、不起进程、
不开串口。

本文件里**没有一处**会写设备或发运动命令。`gripper_state` / `dashboard` /
`port_busy` 全是只读查询。

用法（自查「现在生效的站点配置是哪份、合不合法」）：

    python3 scripts/board_lib.py [--site PATH]

退出码：0 = 站点配置合法且外参没过期；1 = 有毛病（逐条列出）；2 = 读不到文件。
"""

import datetime as dt
import os
import re
import signal
import socket
import subprocess
import sys
import time

PASS, FAIL, SKIP = "PASS", "FAIL", "SKIP"

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

# ⚠️ 排除 `bash -c`：经 ssh 跑时调用者那条命令行里带着同样的模式，会把自己匹配上
#    （board_chain_check.py / board_ctrlc_check.py 都踩过，ssh 直接 255）。
LEFTOVER_RE = re.compile(
    r"ros2 launch yolo_bringup|move_group|robot_state_publisher"
    r"|detector_node|projector_node|planner_node|realsense2_camera_node"
    r"|static_transform_publisher|ur_ros2_control_node"
    r"|ur_robot_driver/(dashboard_client|robot_state_helper|controller_stopper)")


# ============================================================ 进程残留

def _int(s):
    try:
        return int(s)
    except (TypeError, ValueError):
        return None


def _ps_lines():
    r = subprocess.run(["ps", "-eo", "pid,args", "--no-headers"],
                       capture_output=True, text=True)
    return r.stdout.splitlines() if r.returncode == 0 else []


def _ppid(pid):
    """父 pid。`/proc/<pid>/stat` 的 comm 字段可能含空格与括号，所以从**最后**一个
    `)` 往右切（`rsplit`），再取 state 之后的第一个数。"""
    try:
        with open(f"/proc/{int(pid)}/stat", "rb") as fh:
            tail = fh.read().rsplit(b")", 1)[1].split()
        return int(tail[1])
    except (OSError, IndexError, ValueError):
        return None


def own_pids(pid=None):
    """本进程 + 全部祖先的 pid（从自己往上追到 init）。

    为什么需要：残留检查必须**排除自己这一族**，否则「谁在跑检查」这件事会被
    当成残留。
    """
    out = []
    pid = os.getpid() if pid is None else pid
    while pid and pid > 1 and pid not in out and len(out) < 64:
        out.append(pid)
        pid = _ppid(pid)
    return out


def _me_default():
    """「调用方脚本名」= `basename(sys.argv[0])`（最外层入口）。

    ⚠️ 老判据是 `os.path.basename(__file__)`：它当时写在 `board_a1.py` 里，
    于是**碰巧**等于调用方脚本名 —— 因为一条 `ps` 记录里必然含有它自己的脚本名。
    这个等价是偶然的：函数一搬进本文件，`__file__` 就变成 `board_lib.py`，
    而调用方那条记录里带的是 `board_run.py`，于是**排除不掉**。后果是：只要
    调用方命令行里恰好含一个残留模式（例如 `--log ~/planner_node.log`），前置
    检查就报「有残留」并拒绝起栈 —— 同一族的坑本项目已经踩过（`pkill -f`
    把执行它的那条 shell 自己杀掉，记了三次）。
    `sys.argv[0]` 指向最外层入口，**与函数住在哪个文件里无关**。
    """
    return os.path.basename(sys.argv[0] or "")


def is_own_process(args, me, own, pid=None):
    """纯函数：这条 `ps` 记录是不是「我们自己人」。

    `me` 太短（`-c`、`py` 这种）时**不按子串排除** —— 否则 `"" in args` 恒真，
    整条残留检查会被一个空名字关掉，而且关得无声无息。
    """
    if "bash -c" in args:
        return True
    if me and len(me) >= 4 and me in args:
        return True
    return pid is not None and pid in own


def leftovers(ps_out=None, me=None, own=None, pattern=None):
    """残留的栈进程。返回 `ps` 原始行（未裁剪，便于人肉辨认）。

    注入口：`ps_out` 直接喂 `ps -eo pid,args --no-headers` 形状的行，
    `me` / `own` 覆盖默认的自我排除 —— 变异测试靠这三个口子离线成测。

    `pattern` 覆盖「什么算残留」：**默认的 `LEFTOVER_RE` 是整车的**（含
    `ur_ros2_control_node` / `ur_robot_driver/*`）。分阶段类检查
    （`moveit_only_check` / `no_arm_stack` / `board_chain_check` /
    `board_state_machine_check`）跑在**驱动还没起**的时候，它们等的是
    `fake_joint_states` 而不是真驱动 —— 硬套整车模式会让这些脚本**漏报**
    自己那一阶段的残留，而漏报的表现是**静默变绿**。
    所以「哪一类算残留」按阶段给，「谁是自己人」**只有这一份**。
    """
    lines = _ps_lines() if ps_out is None else ps_out
    me = _me_default() if me is None else me
    own = set(own_pids()) if own is None else set(own)
    pat = LEFTOVER_RE if pattern is None else pattern
    out = []
    for ln in lines:
        s = ln.strip()
        if not s:
            continue
        pid_s, _, args = s.partition(" ")
        if not pat.search(args):
            continue
        if is_own_process(args, me, own, _int(pid_s)):
            continue
        out.append(s)
    return out


def teardown(proc):
    """收栈。⚠️ 只 killpg **父进程组**；`ros2 launch` 的子节点里有一批
    （`move_group` / `robot_state_publisher` / `static_transform_publisher` /
    `ur_ros2_control_node`）在父进程被收后会**重新挂到 init**，
    所以必须再按模式扫一遍（2026-10-05 实测：漏扫会留下 PPID=1 的孤儿）。
    """
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    deadline = time.time() + 15
    while time.time() < deadline and leftovers():
        time.sleep(1)
    if leftovers():
        for line in leftovers():
            try:
                os.kill(int(line.split()[0]), signal.SIGKILL)
            except (ValueError, ProcessLookupError):
                pass
        time.sleep(2)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    stale = leftovers()
    if stale:
        print("\n⚠️ 收尾后仍有残留（下次会被前置检查拦下）：")
        for s in stale:
            print("  " + s[:120])


# ============================================================ 只读查询

def port_busy(path):
    """串口有没有被占。`fuser` 会跟随 by-id 符号链接，所以传 by-id 路径也准。"""
    r = subprocess.run(["fuser", path], capture_output=True, text=True)
    return r.returncode == 0


def gripper_state(port=None):
    """读 AG-95 的 `0x0200`。返回 `(状态, 说明)`；读不到返回 `(None, 原因)`。

    **只读** —— 复用 `ag95_init_close.py` 的 `Gripper`，它唯一会写的地方是
    `write()`，本函数一次都不调。
    """
    try:
        import ag95_init_close as ag
    except Exception as exc:                      # noqa: BLE001
        return None, f"导入 ag95_init_close 失败：{exc}"
    old = ag.PORT
    if port:
        ag.PORT = port
    try:
        g = ag.Gripper.open_port()
        if g is None:
            return None, f"串口打不开（{ag.PORT}）"
        try:
            # 0 = 未初始化 / 2 = 进行中 / 1 = 已完成
            return g.read(ag.REG_INIT_STATUS), "ok"
        finally:
            g.close_port()
    finally:
        ag.PORT = old


def dashboard(ip, timeout=5.0):
    """**只读**问一句控制柜。返回 `{命令: 回复}`（**原文，未解析**）；连不上返回 None。

    走 29999 只读查询，**不发 play/power/stop、不碰 30001–30004**
    （红线 1：手搓 RTDE 包把 URControl 打死过）。

    ⚠️ 返回的是**线上原文**，不是纯值 —— 别拿它跟 `'RUNNING'` / `'PLAYING'`
    精确比，见 :func:`dash_mode` / :func:`dash_program_state`。
    """
    try:
        s = socket.create_connection((ip, 29999), timeout=timeout)
    except OSError:
        return None
    try:
        f = s.makefile("rw")
        f.readline()                              # 问候行，别拿来判状态
        out = {}
        for c in ("robotmode", "programState", "safetystatus"):
            f.write(c + "\n")
            f.flush()
            out[c] = f.readline().strip()
        return out
    finally:
        s.close()


# ------------------------------------------------- 控制柜回复的**线上格式**
#
# 🔴 2026-10-06 实测（29999 只读，一台正常在跑的控制柜）：
#
#     robotmode      'Robotmode: RUNNING'      ← 带**标签前缀**
#     programState   'STOPPED ur5ros.urp'      ← 无标签，但缀着**程序名**
#     safetystatus   'Safetystatus: NORMAL'    ← 带标签前缀
#
# 三条各自点自己的名 ⇒ 回复没有错行，这就是原样。
#
# 在此之前，`board_run` 是拿**原文**与纯值精确比的：
#   · `judge_cabinet` 的 `!= 'RUNNING'`  ⇒ 在任何正常控制柜上**恒 FAIL**；
#   · `is_playing` 的 `== 'PLAYING'`     ⇒ 播放时原文是 `'PLAYING ur5ros.urp'`，
#     **永不相等** ⇒ P3 永远等不到播放，300 s 超时退 2，而现场表现是
#     「人明明按了播放」。
# 病根是**夹具里写的是猜的格式**（`{'robotmode': 'RUNNING'}`），猜错了，
# 于是 1294 行离线自检全绿而真机恒红。夹具现已换成上面这三条**原文**。
#
# 所以：判定一律过下面这两个解析器，**永不**拿 `dashboard()` 的原文比。

#: UR 自己给回复加的标签词（只认这几个；别的冒号一律当**值的一部分**，不剥）。
#: 不剥的理由：`programState` 的程序名里可能有冒号（如 Windows 路径），
#: 靠标签白名单才不会把值切坏。
_DASH_LABELS = {"robotmode", "programstate", "program state",
                "safetystatus", "safety status"}


def dash_field(raw):
    """Dashboard 一行回复 → 纯值（剥掉 UR 的标签前缀；无标签的原样返回）。"""
    s = str(raw or "").strip()
    head, sep, tail = s.partition(":")
    if sep and head.strip().lower() in _DASH_LABELS:
        s = tail.strip()
    return s


def dash_mode(d):
    """`robotmode` → `'RUNNING'` / `'POWER_OFF'` / …（**首词**，大写）。

    ⚠️ 取**首词**而不是整串：实测已证 UR 会往这些行后面缀东西
    （`programState` 就缀了程序名）。首词不可能**造出**假绿 ——
    `RUNNING` 仍必须是这行的第一个词。
    """
    f = dash_field((d or {}).get("robotmode"))
    return f.split()[0].upper() if f.split() else ""


def dash_program_state(d):
    """`programState` → `'STOPPED'` / `'PLAYING'` / `'PAUSED'`（**首词**，大写）。

    ⚠️ 后缀是实测的，不是防御性编程：原文 `'STOPPED ur5ros.urp'` 里
    `ur5ros.urp` 是**当前载入的程序名**，播放时就是 `'PLAYING ur5ros.urp'`。
    （播放那一行**尚未实测** —— 等 R1 自然产生时抓一次核对；判据不依赖它。）
    """
    f = dash_field((d or {}).get("programState"))
    return f.split()[0].upper() if f.split() else ""


def is_playing(d):
    """示教器在 PLAYING 吗。**全仓唯一**一份判定，`board_run` 与 `board_a1` 都用它。

    （曾有两份：`board_run` 精确比、`board_a1` 用子串 `"PLAYING" in …`。
    子串那份**碰巧**是对的，精确比那份恒 False —— 两份并存正是缺陷能活下来的原因。）
    """
    return dash_program_state(d) == "PLAYING"


# ============================================================ 站点配置

SITE_ENV = "UR5VG_SITE"
SITE_DEFAULT = "~/.config/ur5-vision-grasp/site.yaml"   # 仓库外；仓内只有 *.example.yaml
DOMAIN_DEFAULT = 99
CAM_KEYS = ("camera_x", "camera_y", "camera_z",
            "camera_qx", "camera_qy", "camera_qz", "camera_qw")


class SiteError(Exception):
    """站点配置读不到 / 不合 YAML 形状。**不**表示「内容不合法」——
    内容问题由 `validate_site` 逐条列出（要一次看全，不要抛第一个就停）。"""


def site_path(explicit=None):
    """站点配置路径：`--site` > `$UR5VG_SITE` > `~/.config/…/site.yaml`。
    三个都在仓库外 —— 仓内**没有**任何指向真值的默认值。"""
    return os.path.expanduser(explicit or os.environ.get(SITE_ENV) or SITE_DEFAULT)


class Site(object):
    """站点配置。取值一律经属性，**缺项返回 None，不静默回落**。
    合不合法让 `validate_site` 说，调用方按三态处置。
    """

    __slots__ = ("raw", "path")

    def __init__(self, raw, path="<memory>"):
        if raw is not None and not isinstance(raw, dict):
            raise SiteError(f"{path} 的顶层不是映射（是 {type(raw).__name__}）")
        self.raw = raw or {}
        self.path = path

    # ---- 基本项 ----
    @property
    def robot_ip(self):
        return self.raw.get("robot_ip") or os.environ.get("UR_ROBOT_IP")

    @property
    def planner_params(self):
        p = self.raw.get("planner_params")
        return os.path.expanduser(str(p)) if p else None

    @property
    def ros_domain_id(self):
        v = self.raw.get("ros_domain_id")
        return DOMAIN_DEFAULT if v is None else int(v)

    @property
    def gripper_port(self):
        g = self.raw.get("gripper")
        g = g if isinstance(g, dict) else {}
        return g.get("port") or self.raw.get("gripper_port")

    # ---- 俯视相机外参（含时效）----
    @property
    def topcam(self):
        t = self.raw.get("topcam")
        return t if isinstance(t, dict) else {}

    @property
    def verified(self):
        return self.topcam.get("verified")

    @property
    def ttl_days(self):
        return self.topcam.get("ttl_days")

    def extrinsics(self):
        """`{'camera_x': '0.34…', …}` —— 值一律**字符串**（launch 参数就是字符串）。"""
        e = self.topcam.get("extrinsics")
        e = e if isinstance(e, dict) else {}
        return {k: str(e[k]) for k in CAM_KEYS if e.get(k) is not None}

    def missing_keys(self):
        """外参缺哪几个键（按 `CAM_KEYS` 顺序）。"""
        e = self.topcam.get("extrinsics")
        e = e if isinstance(e, dict) else {}
        return [k for k in CAM_KEYS if e.get(k) is None]


def parse_site(raw, path="<memory>"):
    """纯函数：把已经读进内存的 dict 包成 Site（好让离线测试不碰文件）。"""
    return Site(raw, path=path)


def load_site(explicit=None, path=None):
    """读站点配置。读不到 ⇒ 抛 `SiteError`（fail-closed，绝不退回默认值）。

    `yaml` 在这里才 import —— 本机的 PyYAML 有没有都不影响 `parse_site` 的离线测试。
    """
    p = path or site_path(explicit)
    if not os.path.exists(p):
        raise SiteError(
            f"读不到站点配置：{p}\n"
            f"   公开仓不写现场真值。配一次：\n"
            f"     mkdir -p ~/.config/ur5-vision-grasp\n"
            f"     cp {REPO}/config/site.example.yaml ~/.config/ur5-vision-grasp/site.yaml\n"
            f"     # 然后填真实值；或用 --site PATH / $UR5VG_SITE 指定别处")
    try:
        import yaml
    except ImportError as exc:
        raise SiteError(f"缺 PyYAML（{exc}）—— pip install pyyaml")
    try:
        with open(p, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh)
    except Exception as exc:                      # noqa: BLE001
        raise SiteError(f"{p} 不是合法 YAML：{exc}")
    return Site(raw, path=p)


def validate_site(site, env=None):
    """逐条列出站点配置的毛病（**纯函数**，返回 list；空 list = 没有毛病）。

    一次列全，不要抛第一个就停 —— 现场来回改五行比来回改一行省事得多。
    """
    env = os.environ if env is None else env
    p = []

    if not site.robot_ip:
        p.append("缺 robot_ip（站点文件里没写，环境变量 UR_ROBOT_IP 也没设）")
    if not site.planner_params:
        p.append("缺 planner_params —— 不给它会在真机上**静默用仿真几何**"
                 "（table_plane_z=-0.39），现象是臂完全不动")
    if not site.gripper_port:
        p.append("缺 gripper.port —— 前置检查要用它查串口占用、读 0x0200；"
                 "现场值形如 /dev/serial/by-id/usb-<厂商>_<型号>_<序列号>-if00-port0")

    miss = site.missing_keys()
    if miss:
        p.append("外参缺键：" + "、".join(miss))
    else:
        for k, v in site.extrinsics().items():
            try:
                float(v)
            except ValueError:
                p.append(f"外参 {k} 不是数：{v!r}")

    for k, why in (("topcam.verified", "没有它这一行**永远不会被判过期**，等于没写"),
                   ("topcam.ttl_days", "没有它就无法判过期")):
        if site.topcam.get(k.split(".")[1]) is None:
            p.append(f"缺 {k} —— {why}")

    d = site.ros_domain_id
    want = _int(env.get("ROS_DOMAIN_ID"))
    if want is not None and want != d:
        p.append(f"ROS_DOMAIN_ID 不一致：站点说 {d}，环境里是 {want} "
                 f"—— 两边不同的域号 = 两个互不可见的 DDS 网络，"
                 f"现象是「栈起着但什么话题都看不到」")
    return p


def as_date(v):
    """`date` / `datetime` / `'2026-10-05'` / `'2026-10-05 记'` → `date`；否则 None。"""
    if isinstance(v, dt.datetime):
        return v.date()
    if isinstance(v, dt.date):
        return v
    if isinstance(v, str):
        m = re.match(r"\s*(\d{4})-(\d{1,2})-(\d{1,2})", v)
        if m:
            try:
                return dt.date(int(m.group(1)), int(m.group(2)), int(m.group(3)))
            except ValueError:
                return None
    return None


MISSING_TTL = ("站点配置没写 topcam.ttl_days —— 缺了它这一行永远无法被判过期，"
               "按不新鲜处置")


def judge_freshness(verified, ttl_days, today=None):
    """俯视外参的时效。返回 `(PASS|FAIL, 说明)`。**纯函数**。

    **与 `state_check.py` 同一条线**：`age > 保质期` 才算超期，`age == 保质期`
    当天仍算数。两处判据必须一致，否则同一份文件两个守卫各说各话。

    fail-closed：字段缺失、类型不对、日期解析不了 **一律 FAIL** ——
    「不知道」不许当成「新鲜」。俯视外参实测一天能漂 0.945°，
    过期外参的症状是点云整体偏、`/targets_3d` 的候选落到臂够不到的地方。
    """
    today = today or dt.date.today()
    if ttl_days is None or isinstance(ttl_days, bool):
        return FAIL, MISSING_TTL
    try:
        ttl = int(ttl_days)
    except (TypeError, ValueError):
        return FAIL, f"topcam.ttl_days 不是整数：{ttl_days!r}"
    if ttl < 0:
        return FAIL, f"topcam.ttl_days 是负数：{ttl}"
    d = as_date(verified)
    if d is None:
        return FAIL, f"topcam.verified 缺失或解析不出日期：{verified!r}"
    age = (today - d).days
    if age > ttl:
        return FAIL, (f"外参核对于 {d}（{age} 天前），保质期 {ttl} 天 —— **已超期**。"
                      f"先跑 python3 scripts/topcam_drift_check.py，再填新日期")
    return PASS, f"外参核对于 {d}（{age} 天前），保质期 {ttl} 天，剩 {ttl - age} 天"


def freshness_gate(site, today=None, override=None):
    """站点外参时效闸门。`override` = 人写的破例理由（`--override-freshness`）。

    ⚠️ 破例**必须写下理由**且会被原样记进报告 —— 空理由拒绝。
    """
    if override is not None:
        if not str(override).strip():
            return FAIL, "--override-freshness 给了空理由 —— 破例必须写下为什么"
        return PASS, f"⚠️ 已由人显式破例，理由：{str(override).strip()}"
    return judge_freshness(site.verified, site.ttl_days, today=today)


# ============================================================ 自查入口

def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(
        description="站点配置自查：路径、合法性、外参时效（全离线，不碰设备）")
    ap.add_argument("--site", default=None, help=f"站点配置路径（默认 ${SITE_ENV} 或 {SITE_DEFAULT}）")
    ap.add_argument("--today", default=None, help="当成哪一天（YYYY-MM-DD），用于演练时效判定")
    a = ap.parse_args(argv)

    p = site_path(a.site)
    print(f"站点配置：{p}")
    print(f"  （来源优先级：--site > ${SITE_ENV} > {SITE_DEFAULT}）")
    try:
        site = load_site(path=p)
    except SiteError as exc:
        print(f"❌ {exc}")
        return 2

    problems = validate_site(site)
    today = as_date(a.today) if a.today else None
    if a.today and today is None:
        print(f"❌ --today 解析不出日期：{a.today!r}")
        return 2
    st, detail = freshness_gate(site, today=today)
    print(f"  外参时效：[{st}] {detail}")

    if problems:
        print(f"\n❌ {len(problems)} 条毛病：")
        for i, x in enumerate(problems, 1):
            print(f"  {i}. {x}")
        return 1
    print("  ✓ 逐项合法")
    if st != PASS:
        return 1
    print("\n结果：✅ 站点配置可用")
    return 0


if __name__ == "__main__":
    sys.exit(main())
