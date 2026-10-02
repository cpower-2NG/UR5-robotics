#!/usr/bin/env python3
"""`scripts/ag95_init_close.py` 的自检（离线，不需要真夹爪、不需要 Gazebo、不需要 colcon build）。

这个脚本是全仓库**唯一会主动驱动夹爪**的东西，所以自检的重点不是「能不能初始化」，
而是**「不给 `--go` 时它到底写没写」** —— 写错了夹爪当场就动，而现场的人
以为它在预演。这条用一个**手写的最小 Modbus RTU 从站**（跑在 pty 上）真的量：
从站把收到的每一次 FC6 记在 `slave.writes` 里，断言它是空的。

⚠️ 从站类直接复用 `test_ag95_probe.py` 里的那一个（按路径加载），
**不复制一份** —— 复制出来的量具迟早会和原件分叉，而分叉的量具会给出假的绿。

用法：`python3 scripts/test_ag95_init_close.py`（需要 pymodbus；没装则 SKIP）
"""

import importlib.util
import io
import sys
from contextlib import redirect_stdout
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SCRIPT_PATH = REPO / "scripts" / "ag95_init_close.py"
SIBLING = REPO / "scripts" / "test_ag95_probe.py"

PASS = SKIP = FAIL = 0
FAILURES = []

SCRIPT = None
SLAVE_MOD = None


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


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def run_script(argv):
    """跑被测脚本的 `main()`，捕获 stdout 与返回码（不起子进程，省一层环境问题）。"""
    buf = io.StringIO()
    with redirect_stdout(buf):
        rc = SCRIPT.main(argv)
    return rc, buf.getvalue()


def with_slave(regs, *, actual_reg=None, target_reg=None, step=1000):
    """起一个 pty + 手写从站，把被测脚本的 PORT 指过去。

    用 ``with`` 语句进出；退出时**先停从站线程再关 pty**（顺序反了会有僵尸从站
    抢 fd —— `_RtuSlave.stop()` 的文档里记了那个坑）。
    """
    class _Ctx:
        def __enter__(self):
            self.pair = SLAVE_MOD._PtyPair()
            self.slave = SLAVE_MOD._RtuSlave(
                self.pair.master_fd, regs, slave_id=1,
                actual_reg=actual_reg, target_reg=target_reg, step=step).start()
            self._old_port = SCRIPT.PORT
            SCRIPT.PORT = self.pair.port
            return self.slave

        def __exit__(self, *exc):
            SCRIPT.PORT = self._old_port
            self.slave.stop()
            self.pair.close()
            return False

    return _Ctx()


def main() -> int:
    global SCRIPT, SLAVE_MOD
    print("AG-95 初始化/合拢脚本自检")
    print("=" * 72)
    SCRIPT = _load(SCRIPT_PATH, "ag95_init_close")

    print("\n[1] 结构：源码里写寄存器的入口只有一个")
    src = SCRIPT_PATH.read_text(encoding="utf-8")
    check("源码里只有一处 write_register（在 Gripper.write 里）",
          src.count("write_register") == 1, f"实际 {src.count('write_register')} 处")

    try:
        import pymodbus                                     # noqa: F401
        has_pymodbus, why = True, ""
    except ImportError:
        has_pymodbus = False
        why = '没有 pymodbus —— pip3 install "pymodbus>=3.6"（板上装了就能全跑）'

    names = ["预演：一个寄存器都不写", "预演：打印本来会写什么", "预演：退出码 0",
             "预演 + --close 也不写", "--go：只写了初始化寄存器",
             "--go：初始化完成即停（不发位置）", "--go --close：写了力与位置",
             "--go --close：合拢到 0", "--go：没走到 1000 时告警", "退出码 2：口打不开"]
    if not has_pymodbus:
        for n in names:
            check(n, False, why, skip=True)
    else:
        SLAVE_MOD = _load(SIBLING, "test_ag95_probe")

        # ---- 2. 预演：**核心安全断言** ----------------------------------
        print("\n[2] 预演（不给 --go）：**从站的 writes 必须空**")
        regs = {0x0100: 0, 0x0200: 0, 0x0201: 0, 0x0202: 0, 0x0101: 50, 0x0103: 1000}
        with with_slave(regs) as slave:
            rc, out = run_script([])
            check("预演：一个寄存器都不写", slave.writes == [], f"写了 {slave.writes}")
            check("预演：打印本来会写什么",
                  "0x0100 = 0x01" in out and "全开" in out, out)
            check("预演：退出码 0", rc == 0, f"rc={rc}")
            before = list(slave.writes)
            rc, out = run_script(["--close"])
            check("预演 + --close 也不写（--close 不能绕开 --go 的闸门）",
                  slave.writes == before, f"写了 {slave.writes}")

        # ---- 3. --go：只初始化 ------------------------------------------
        print("\n[3] --go（只初始化）：写一个寄存器，然后停在**全开**")
        regs = {0x0100: 0, 0x0200: 0, 0x0201: 0, 0x0202: 1000,
                0x0101: 50, 0x0103: 1000}
        with with_slave(regs) as slave:
            rc, out = run_script(["--go"])
            check("--go：只写了初始化寄存器",
                  slave.writes == [(0x0100, 1)], f"写了 {slave.writes}")
            check("--go：初始化完成即停（不发位置指令，不写 0x0103）",
                  0x0103 not in [a for a, _ in slave.writes], f"写了 {slave.writes}")
            check("--go：退出码 0", rc == 0, f"rc={rc}")

        # ---- 4. --go --close：初始化 + 合拢 ------------------------------
        print("\n[4] --go --close：初始化之后再写力与位置，合拢到 0")
        # 0x0202 从 1000 起步、朝 0x0103 走 —— 位置写下去之后手指「动」到 0。
        regs = {0x0100: 0, 0x0200: 1, 0x0201: 1, 0x0202: 1000,
                0x0101: 50, 0x0103: 1000}
        with with_slave(regs, actual_reg=0x0202, target_reg=0x0103, step=1000) as slave:
            rc, out = run_script(["--go", "--close"])
            check("--go --close：写了力与位置",
                  (0x0101, SCRIPT.FORCE_N) in slave.writes
                  and (0x0103, SCRIPT.TARGET_PERMILLE) in slave.writes,
                  f"写了 {slave.writes}")
            check("--go --close：合拢到 0（判据取自读到的位置，不是自报）",
                  slave.regs[0x0202] == 0 and "已合拢到 0.0 mm" in out,
                  f"0x0202={slave.regs[0x0202]}\n{out}")
            check("--go --close：退出码 0", rc == 0, f"rc={rc}")

        # ---- 5. 一个真·负例：初始化没走到 1000 要**出声** ----------------
        print("\n[5] 负例：初始化之后位置**不是** 1000（卡住/夹住了）")
        regs = {0x0100: 0, 0x0200: 0, 0x0201: 0, 0x0202: 400,
                0x0101: 50, 0x0103: 1000}
        with with_slave(regs) as slave:
            rc, out = run_script(["--go"])
            check("--go：没走到 1000 时告警（不静默当成功）",
                  "没走到 1000" in out, out)

        # ---- 6. 口打不开 → 退出码 2（不是 0）-----------------------------
        SCRIPT.PORT = "/dev/definitely-not-a-port"
        rc, out = run_script(["--go"])
        check("退出码 2：口打不开时明确失败（2 是「环境不满足」，不是 0）",
              rc == 2, f"rc={rc}\n{out}")

    print("\n" + "=" * 72)
    print(f"{PASS} PASS / {SKIP} SKIP / {FAIL} FAIL")
    if FAILURES:
        print("失败项：")
        for n in FAILURES:
            print(f"  · {n}")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
