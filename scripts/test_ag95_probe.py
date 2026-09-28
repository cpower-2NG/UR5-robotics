#!/usr/bin/env python3
"""`scripts/ag95_probe.py` 的自检（离线，不需要真夹爪、不需要 Gazebo、不需要 colcon build）。

为什么这个自检里最重的一段是「扫源码」而不是「跑功能」
========================================================
这个探针是**现场第一个接触真夹爪的东西**，而它能造成的最大伤害不是「读不到」，
而是「偷偷写了一个寄存器」—— 夹爪会当场动起来，而现场的人以为它只是只读的。

所以本文件用三段不同性质的手段钉住「只读」这一条，**从强到弱**：

1. **扫源码找写调用**（结构性、最强）：源码里出现 `write_register` / `write_coils`
   等任何写 API 就是 FAIL。这条不依赖我的记性 —— 以后谁加了写路径都会被拦下。
2. **pty 回环**：用一个**手写的**最小 Modbus RTU 从站（`_RtuSlave`，只用
   `os.read`/`os.write` + 自己算的 CRC，**不依赖 pymodbus**）接上真实串口设备节点，
   让探针的 `--scan-slaves` / `--dump` 真的跑一遍。被测的是探针，从站是量具。
   顺带覆盖了三个真机上最容易翻车的分支：**地址不是我的就静默**（表现为超时）、
   **非法地址回异常码**、**FC3 与 FC4 都要试**。
3. **换算不重复实现**：断言探针里的 `permille_to_mm` **就是**
   `grasp_planner.gripper` 里那一个对象（`is` 比较）。两份换算迟早会分叉。

⚠️ 第 2 段需要 `pymodbus`；没装时报 **SKIP 并打印原因**（不算 PASS）。板上装了
就能全跑。第 3 段需要 ROS 环境（`gripper.py` 会 import `sensor_msgs`），同理处理。

用法：`python3 scripts/test_ag95_probe.py`（建议先 `source /opt/ros/humble/setup.bash`）
"""

import fcntl
import importlib.util
import io
import os
import pty
import re
import signal
import subprocess
import sys
import threading
import time
from contextlib import redirect_stdout
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PROBE_PATH = REPO / "scripts" / "ag95_probe.py"
YAML_PATH = (REPO / "ros2_ws" / "src" / "grasp_planner" / "config"
             / "grasp_planner.yaml")

PASS = SKIP = FAIL = 0
FAILURES = []
PROBE = None                      # 由 main() 装载（见 load_probe）


def check(name, cond, detail="", *, skip=False):
    global PASS, SKIP, FAIL
    if skip:
        SKIP += 1
        print(f"  SKIP  {name}" + (f"  —— {detail}" if detail else ""))
        return
    if cond:
        PASS += 1
        print(f"  PASS  {name}")
    else:
        FAIL += 1
        FAILURES.append(name)
        print(f"  FAIL  {name}" + (f"  —— {detail}" if detail else ""))


def load_probe():
    """按路径加载探针模块（`scripts/` 不是包，只能这么导）。"""
    spec = importlib.util.spec_from_file_location("ag95_probe", PROBE_PATH)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["ag95_probe"] = mod
    spec.loader.exec_module(mod)
    return mod


# ------------------------------------------------------------------ 手写从站
def _crc16(data: bytes) -> int:
    """Modbus RTU 的 CRC-16（poly 0xA001，初值 0xFFFF）。自己算，不借库。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _reply(slave: int, fc: int, payload: bytes) -> bytes:
    body = bytes([slave, fc]) + payload
    crc = _crc16(body)
    return body + bytes([crc & 0xFF, crc >> 8])      # Modbus 是**低字节在前**


class _RtuSlave:
    """手写的最小 Modbus RTU 从站，跑在 pty 的 **master** 端。

    实现两种请求，都是 **8 字节定长**，所以不需要通用解析：

      · **读 1 个寄存器**（FC3/FC4）—— 探针用
      · **写 1 个寄存器**（FC6，`write_register`）—— `Ag95Gripper` 的写路径用，
        应答按规范**原样回显**，写过的值记在 `self.writes` 里供断言

    寄存器值由 `regs` 给定；不在表里的地址回**异常码 2（非法地址）**，
    `slave_id` 之外的一律**静默**（真夹爪也是这样：地址不对就没人应答，表现为超时）。

    可选的**运动模拟**（`target_reg` / `actual_reg` / `step`）：给了这两个寄存器
    地址之后，读 `actual_reg` 时会让它朝 `target_reg` 里的值走一步（每读一次走
    `step` 千分位）。`Ag95Gripper._wait_until()` 是「轮询 0x0202 直到到位」，
    没有这一步模拟，写路径的自检要**真等满 3 秒超时**才能跑完（测试会慢得没有意义），
    也测不出「到位」这一分支。默认不给 ⇒ 行为与从前完全一样（探针的读测试不受影响）。
    """

    def __init__(self, master_fd: int, regs: dict, slave_id: int = 1, *,
                 target_reg: int = None, actual_reg: int = None,
                 step: int = 250):
        self.fd = master_fd
        self.regs = dict(regs)
        self.slave_id = slave_id
        self.seen = []                                # 收到的有效请求，供断言
        self.writes = []                              # (地址, 值)，FC6
        self.target_reg = target_reg
        self.actual_reg = actual_reg
        self.step = step
        self._stop = threading.Event()
        flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        self._t = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._t.start()
        return self

    def stop(self):
        """停从站线程，并**等它真的退出**。

        ⚠️ 只 ``set`` 事件是不够的，必须 ``join``。``_loop`` 把 ``OSError`` 当成
        「暂时没数据」**吞掉继续轮询**，所以线程不会因为 fd 被关而退出 —— 一旦
        调用方 ``pair.close()`` 之后又建了新的 pty，**这个线程就会开始读新的
        master fd**：两个从站抢同一个 fd，请求被僵尸从站吃掉、回的是上一段的
        寄存器值。症状极具迷惑性，看起来像「被测代码写丢了」「读到了旧值」。

        ``join`` 之后「停止」是一个确定的时点：返回时保证没有任何一次 ``os.read``
        在这个 fd 上还悬着。本文件原先只有一处从站、又只建一对 pty，所以侥幸
        没事；`test_ag95_gripper.py` 有九段、每段一对 pty，当场踩到了。
        """
        self._stop.set()
        self._t.join(timeout=1.0)

    def _loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = os.read(self.fd, 256)
            except (BlockingIOError, OSError):
                time.sleep(0.005)
                continue
            if not chunk:
                time.sleep(0.005)
                continue
            buf += chunk
            while len(buf) >= 8:
                req, buf = buf[:8], buf[8:]
                self._handle(req)

    def _step_towards_target(self):
        """让「实时位置」朝「目标位置」走一步（模拟手指运动）。"""
        if self.target_reg is None or self.actual_reg is None:
            return
        cur = self.regs.get(self.actual_reg, 0)
        want = self.regs.get(self.target_reg, cur)
        if cur < want:
            self.regs[self.actual_reg] = min(want, cur + self.step)
        elif cur > want:
            self.regs[self.actual_reg] = max(want, cur - self.step)

    def _handle(self, req: bytes):
        sid, fc = req[0], req[1]
        addr = (req[2] << 8) | req[3]
        if _crc16(req[:6]) != (req[7] << 8) | req[6]:
            return                                    # CRC 不过就当一个字节都没收到
        if sid != self.slave_id:
            return                                    # 不是我的地址：静默
        self.seen.append((fc, addr))

        if fc == 6:
            # 写单个寄存器：写进表里，应答是**原样回显**（Modbus RTU 规范）。
            value = (req[4] << 8) | req[5]
            self.regs[addr] = value
            self.writes.append((addr, value))
            os.write(self.fd, req)
            return

        if addr == self.actual_reg:
            self._step_towards_target()
        val = self.regs.get(addr)
        if val is None:
            os.write(self.fd, _reply(sid, fc | 0x80, bytes([0x02])))
        else:
            os.write(self.fd, _reply(sid, fc, bytes([2, val >> 8, val & 0xFF])))


class _PtyPair:
    """一对 pty 组成的「虚拟串口线」：从站占 master 端，探针用 `/dev/pts/N`。"""

    def __init__(self):
        self.master_fd, self.slave_fd = pty.openpty()
        self.port = os.ttyname(self.slave_fd)

    def close(self):
        for fd in (self.master_fd, self.slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass


def _run_probe(argv):
    """跑探针的 `main()`，捕获 stdout 与返回码（不起子进程，省一层环境问题）。"""
    buf = io.StringIO()
    with redirect_stdout(buf):
        rc = PROBE.main(argv)
    return rc, buf.getvalue()


def _run_probe_cli(argv):
    """同 `_run_probe`，但真的起子进程 —— 用来验「命令行入口」本身是通的。"""
    p = subprocess.run([sys.executable, str(PROBE_PATH)] + argv,
                       capture_output=True, text=True, timeout=120)
    return p.returncode, p.stdout + p.stderr


# 手写的从站寄存器表：637‰ 让两种方向的解释**不相等**（60.5 / 34.5），
# 用 500 的话两个方向都算出 47.5 mm，断言就分不出方向错了没。
FAKE_REGS = {0x0202: 637, 0x0100: 1, 0x0103: 500, 0x0101: 50}


def main() -> int:
    print("AG-95 只读探针自检")
    print("=" * 72)
    src = PROBE_PATH.read_text(encoding="utf-8")
    global PROBE
    PROBE = load_probe()

    # ---- 1. 只读契约（扫源码）-------------------------------------------
    print("\n[1] 只读契约：源码里不得出现任何写 API")
    for api in ("write_register", "write_registers", "write_coil",
                "write_coils", "write_discrete", "mask_write"):
        hits = [i + 1 for i, ln in enumerate(src.splitlines()) if api in ln]
        check(f"没有 {api}", not hits, f"出现在第 {hits} 行")

    # 「不写」还有一层：不碰那个**有写能力**的类。探针只用纯换算函数，
    # 所以文档里**提到**它没问题（要解释为什么不用写路径），**导入或实例化**才是问题。
    check("没有实例化 Ag95Gripper（写路径在那边）",
          "Ag95Gripper(" not in src and not hasattr(PROBE, "Ag95Gripper"))
    check("探针模块里没有写路径的类",
          not any(n for n in dir(PROBE) if "Gripper" in n),
          f"多了 {[n for n in dir(PROBE) if 'Gripper' in n]}")
    # 只读功能码是 3/4；6（写单个）与 16（写多个）一旦出现就是写路径的痕迹。
    check("源码里没有功能码 6 / 16 的常量",
          not re.search(r"\b0x0?06\b|\b0x10\b", src))

    # ---- 1b. 从站号参数名不得写死（2026-09-28 的真坑）---------------------
    # 板子上实测是 pymodbus **3.15.0**，用 `device_id=`；而 3.6/3.7 是 `slave=`、
    # 2.x 是 `unit=` —— **3.x 自己就有两代 API**。写死任何一个，在另一代上就是
    # `TypeError`。这里钉住两处读调用**必须**用探测出来的名字。
    read_calls = [ln for ln in src.splitlines()
                  if "read_holding_registers(" in ln or "read_input_registers(" in ln]
    check("两处读调用都用探测出来的从站号参数名（**{kw: slave}）",
          len(read_calls) == 2 and all("**{kw: slave}" in ln for ln in read_calls),
          f"实际 {len(read_calls)} 处：{[ln.strip() for ln in read_calls]}")

    # ---- 2. 换算：不重复实现 --------------------------------------------
    print("\n[2] 换算复用（不重复实现）")
    gripper_mod = None
    try:
        sys.path.insert(0, str(REPO / "ros2_ws" / "src" / "grasp_planner"))
        from grasp_planner import gripper as gripper_mod
    except ImportError as exc:
        why = f"没有 ROS 环境（{exc}）—— 先 source /opt/ros/humble/setup.bash"
    if gripper_mod is None:
        check("permille_to_mm 就是 gripper.py 里那一个", False, why, skip=True)
        check("千分位方向在两种解释下不同", False, why, skip=True)
    else:
        # 前提：探针确实导入成功（没成功的话下面这个断言会以「None is not ...」失败，
        # 那正是我们要的 —— 「没导入」本身就是缺陷，不能被静默算过）
        check("permille_to_mm 就是 gripper.py 里那一个",
              PROBE.permille_to_mm is gripper_mod.permille_to_mm,
              f"探针里是 {PROBE.permille_to_mm!r}")
        hi = gripper_mod.permille_to_mm(1000, 95.0, open_is_high=True)
        lo = gripper_mod.permille_to_mm(1000, 95.0, open_is_high=False)
        check("1000‰ 两种方向 = 95.0 / 0.0 mm", (hi, lo) == (95.0, 0.0), f"{hi} / {lo}")

    # ---- 3. interpret()：两种方向都印出来 --------------------------------
    print("\n[3] interpret()：两种方向都印（现场靠它判方向）")
    text = PROBE.interpret(FAKE_REGS[0x0202], 95.0)
    if gripper_mod is None:
        check("印出两种方向", False, why, skip=True)
    else:
        check("印出两种方向", "若 1000=全开" in text and "若 1000=全闭" in text, text)
        check("637‰ ⇒ 60.5 / 34.5 mm（两值不等，方向看得见）",
              "60.5" in text and "34.5" in text, text)

    # ---- 4. 默认值只有一个来源（yaml）-----------------------------------
    print("\n[4] 串口默认值取自 grasp_planner.yaml（不另写一份）")
    if not YAML_PATH.exists():
        check("yaml 存在", False, str(YAML_PATH), skip=True)
    else:
        import yaml
        cfg = yaml.safe_load(YAML_PATH.read_text(encoding="utf-8"))
        cfg = cfg["grasp_planner"]["ros__parameters"]
        d = PROBE._yaml_defaults()
        check("port 与 yaml 一致", d["port"] == cfg["ag95_port"],
              f"{d['port']} vs {cfg['ag95_port']}")
        check("baudrate 与 yaml 一致", d["baudrate"] == cfg["ag95_baudrate"],
              f"{d['baudrate']} vs {cfg['ag95_baudrate']}")
        check("slave_id 与 yaml 一致", d["slave_id"] == cfg["ag95_slave_id"],
              f"{d['slave_id']} vs {cfg['ag95_slave_id']}")
        check("行程与 yaml 一致", d["stroke_mm"] == cfg["ag95_stroke_mm"],
              f"{d['stroke_mm']} vs {cfg['ag95_stroke_mm']}")
        check("命令行默认值 = yaml 值",
              PROBE.build_parser().parse_args([]).port == cfg["ag95_port"])

    # 没有 PyYAML / 没有 yaml 文件时**必须退到默认值而不是抛异常** —— 这个函数在
    # 建解析器时就被调用，一抛异常连 `--frames` 都打不开，而 `--frames` 正是
    # 「环境不全时还能看一眼」的那条路。用 `sys.modules['yaml'] = None` 模拟缺模块
    # （CPython 里这会让 `import yaml` 抛 ImportError）。
    saved = sys.modules.get("yaml", "缺失")
    try:
        sys.modules["yaml"] = None
        try:
            d2 = PROBE._yaml_defaults()
            check("没有 PyYAML 时退到默认值（不抛异常）",
                  d2["baudrate"] == 115200 and "PyYAML" in d2["source"], str(d2))
        except Exception as exc:                       # noqa: BLE001
            check("没有 PyYAML 时退到默认值（不抛异常）", False, f"{type(exc).__name__}: {exc}")
    finally:
        if saved == "缺失":
            sys.modules.pop("yaml", None)
        else:
            sys.modules["yaml"] = saved

    # ---- 5. 命令行界面：模式互斥 + 默认模式不碰串口 ----------------------
    print("\n[5] 命令行：模式互斥，默认模式最安全")
    rc, _ = _run_probe(["--dump", "--watch"])
    check("两个模式同时给 → 退出码 2", rc == 2, f"rc={rc}")
    rc, out = _run_probe_cli(["--frames"])
    check("子进程跑 --frames → 退出码 0", rc == 0, f"rc={rc}")
    check("--frames 印出寄存器表", "0x0202" in out and "FC3" in out)
    check("--frames 说明「全程不写」", "不写" in out)
    # 关键：默认（不给模式）走的是最安全的那条，且**不打开串口**。
    # 用一个必然打不开的端口当默认值也照样成功，就证明它没碰端口。
    rc, out = _run_probe(["--port", "/dev/definitely-not-a-port"])
    check("默认模式不碰串口（给个不存在的口也能跑）", rc == 0, f"rc={rc}")
    check("默认模式就是 --frames", "只读探测" in out)

    # ---- 6. pty 回环：真的读一遍 ----------------------------------------
    print("\n[6] pty 回环：接一个手写从站，真的把这几个模式跑一遍")
    try:
        import pymodbus                                     # noqa: F401
        has_pymodbus = True
        pymodbus_why = ""
    except ImportError:
        has_pymodbus = False
        pymodbus_why = ('没有 pymodbus —— pip3 install "pymodbus>=3.6"'
                        '（板上装了就能全跑）')

    if not has_pymodbus:
        for name in ("扫从站找到 1", "扫从站跳过非本机地址", "dump 读到 637‰",
                     "dump 对非法地址报错", "FC4 也试", "全失败时给排查顺序"):
            check(name, False, pymodbus_why, skip=True)
    else:
        from pymodbus.client import ModbusSerialClient as _ClientCls
        kw = PROBE._slave_kwarg(_ClientCls)
        check(f"探测出的从站号参数名是已知的一代（本机 = {kw}）",
              kw in PROBE._SLAVE_KWARGS, f"{kw}（签名：{PROBE._signature_of(_ClientCls)}）")

        pair = _PtyPair()
        slave = _RtuSlave(pair.master_fd, FAKE_REGS, slave_id=1).start()
        try:
            rc, out = _run_probe(["--port", pair.port, "--scan-slaves", "1", "4"])
            check("扫从站：找到 1 号并读到 637‰", "从站 1" in out and "637" in out, out)
            check("扫从站：2..4 号静默（地址不对就没人应答）",
                  out.count("无响应") == 3, out)
            check("扫从站：退出码 0（有设备就该成功）", rc == 0, f"rc={rc}")

            rc, out = _run_probe(["--port", pair.port, "--slave", "1", "--dump"])
            check("dump：FC3 读到 0x0202 = 637‰（含两种方向解释）",
                  "[OK ] 0x0202" in out and "60.5" in out, out)
            check("dump：非法地址 0x0300 报错（不是崩溃）",
                  "[--] 0x0300" in out, out)
            check("dump：FC3 与 FC4 都试过",
                  {fc for fc, _ in slave.seen} == {3, 4},
                  f"实际只收到 FC{sorted({fc for fc, _ in slave.seen})}")
            check("dump：退出码 0", rc == 0, f"rc={rc}")

            # 一个真·负例：从站地址对不上时必须**明确失败**，不能假装成功。
            rc, out = _run_probe(["--port", pair.port, "--slave", "7", "--dump"])
            check("从站号给错 → 全部无应答且退出码 3（不静默成功）",
                  rc == 3 and "有应答的读取项：0" in out, f"rc={rc}\n{out}")
            check("全失败时给出排查顺序（接线/波特率/地址/上电）",
                  "按这个顺序查" in out and "A/B" in out and "24V" in out, out)

            # --watch 是现场**判方向**用的那条（人工掰手指，看原始值落在 0 还是 1000），
            # 所以它必须满足两件事：只读地持续读、且**值的每一次变化都看得见**。
            # 这里在子进程跑 --watch 的同时**改从站的寄存器**，模拟「手指被掰动」。
            proc = subprocess.Popen(
                [sys.executable, str(PROBE_PATH), "--port", pair.port,
                 "--slave", "1", "--watch", "--interval", "0.05"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            try:
                time.sleep(0.6)                     # 先读到 637
                slave.regs[0x0202] = 1000           # 「掰到全开」
                time.sleep(0.6)
                proc.send_signal(signal.SIGINT)
                wout, _ = proc.communicate(timeout=15)
            except subprocess.TimeoutExpired:        # pragma: no cover
                proc.kill()
                wout, _ = proc.communicate()
                wout += "\n（超时被杀：--watch 没有响应 Ctrl-C）"
            check("watch：变化前后的原始值都打印出来",
                  "637" in wout and "1000" in wout, wout)
            check("watch：被中断时干净退出（退出码 0）", proc.returncode == 0,
                  f"rc={proc.returncode}\n{wout}")
            check("watch：结束时提示怎么判方向",
                  "open_is_high" in wout, wout)
            check("watch：全程只有读请求（从站没收到过别的功能码）",
                  {fc for fc, _ in slave.seen} <= {3, 4},
                  f"收到过 FC{sorted({fc for fc, _ in slave.seen})}")
        finally:
            slave.stop()
            pair.close()

    print("\n" + "=" * 72)
    print(f"PASS {PASS}   SKIP {SKIP}   FAIL {FAIL}")
    if FAILURES:
        print("失败项：")
        for f in FAILURES:
            print(f"  - {f}")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
