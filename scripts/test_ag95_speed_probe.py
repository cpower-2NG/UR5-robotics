#!/usr/bin/env python3
"""`ag95_speed_probe.py` 的自检 —— **不需要夹爪**，全部用假从站。

把守三件事：
  1. 不带 `--go` 时**一个字节都不写**（闸门是结构性的，不靠自觉）；
  2. `0x0200 != 1`（未初始化）时**拒绝动**，退出码 2；
  3. `judge_ratio` 的三档判据边界（生效 / 没生效 / 说不清 / 样本不足）。

判据口径来自 2026-10-27 的实测：写 0x0104 只回 ACK 不生效，所以判决
必须看**耗时比**，不能看回读 —— 这个测试把「拿回读当判据」的写法钉死在门外。
"""
import io
import contextlib
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, __file__.rsplit("/", 1)[0])

import ag95_speed_probe as sp


class FakeGripper:
    """假从站：记录每一次 write，读走一张预置表。"""

    def __init__(self, regs):
        self.regs = dict(regs)
        self.writes = []

    def read(self, addr):
        return self.regs.get(addr)

    def write(self, addr, value):
        self.writes.append((addr, value))
        self.regs[addr] = value
        return True

    def move_and_time(self, target_pm, **_kw):
        self.writes.append((sp.REG_TARGET, target_pm))
        self.regs[sp.REG_ACTUAL] = target_pm
        return target_pm, 0.4, True

    def settle(self, *_a, **_k):
        return self.regs.get(sp.REG_ACTUAL), 0.0

    def wait_init_done(self, *_a, **_k):
        return 1, 0.1

    def close_port(self):
        pass


RESULTS = []


def check(name, cond, detail=""):
    RESULTS.append((name, bool(cond), detail))
    print("%-4s %s%s" % ("PASS" if cond else "FAIL", name,
                         "" if cond else "  ← " + detail))


def run_main(args, fake):
    """跑 main()，把 open_port 换成假的。返回 (rc, stdout文本)。"""
    old = sp.Gripper.open_port
    sp.Gripper.open_port = classmethod(lambda cls, port=sp.PORT: fake)
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            rc = sp.main(args)
    finally:
        sp.Gripper.open_port = old
    return rc, buf.getvalue()


def main():
    print("=" * 70)
    print("  ag95_speed_probe 自检（假从站，不碰硬件）")
    print("=" * 70)

    # --- 1. judge_ratio 三档 ---
    r, v = sp.judge_ratio([1.0, 1.1], [5.0, 5.2])
    check("20% 若生效（慢约 5 倍）判「生效」", r is not None and r > 1.5 and "生效" in v, v)
    r, v = sp.judge_ratio([0.42, 0.42], [0.42, 0.43])
    check("耗时不变判「没生效」", r is not None and r <= sp.RATIO_IGNORED and "没生效" in v, v)
    r, v = sp.judge_ratio([1.0], [1.3])
    check("1.2–1.5 之间判「说不清」", r is not None and "说不清" in v, v)
    r, v = sp.judge_ratio([], [1.0])
    check("空样本判「无法判决」", r is None, v)

    # --- 2. 不带 --go：只读，一个字节都不写 ---
    regs = {sp.REG_INIT_STATUS: 1, sp.REG_ACTUAL: 526, sp.REG_SPEED: 100,
            sp.REG_FORCE: 50, sp.REG_TARGET: 526}
    fake = FakeGripper(regs)
    rc, out = run_main([], fake)
    check("不带 --go 退出码 0", rc == 0, str(rc))
    check("不带 --go **零写入**", fake.writes == [], str(fake.writes))
    check("不带 --go 打印了只读快照", "0x0104 速度设定" in out)

    # --- 3. --go 但未初始化：拒绝 ---
    regs = dict(regs)
    regs[sp.REG_INIT_STATUS] = 0
    fake = FakeGripper(regs)
    rc, out = run_main(["--go"], fake)
    check("0x0200=0 时 --go 退出码 2", rc == 2, str(rc))
    check("0x0200=0 时**零写入**", fake.writes == [], str(fake.writes))

    # --- 4. --go 正常：写的就是 0x0104，且最后写回 100 ---
    regs = {sp.REG_INIT_STATUS: 1, sp.REG_ACTUAL: 526, sp.REG_SPEED: 100,
            sp.REG_FORCE: 50, sp.REG_TARGET: 526}
    fake = FakeGripper(regs)
    rc, out = run_main(["--go", "--speed-pct", "20", "--repeat", "1"], fake)
    check("--go 单点退出码 0", rc == 0, str(rc))
    speed_writes = [v for (a, v) in fake.writes if a == sp.REG_SPEED]
    check("速度写的是 20 并最终收回 100", speed_writes[:1] == [20] and speed_writes[-1] == 100,
          str(speed_writes))
    check("没有把**回读**当判据（判决行按耗时比给）", "耗时比" in out, out[-200:])

    # --- 5. 扫值模式：每一档都写一次，收尾回到 100 ---
    fake = FakeGripper({sp.REG_INIT_STATUS: 1, sp.REG_ACTUAL: 526, sp.REG_SPEED: 100,
                        sp.REG_FORCE: 50, sp.REG_TARGET: 526})
    rc, out = run_main(["--go", "--sweep", "1,20,100", "--repeat", "1"], fake)
    speed_writes = [v for (a, v) in fake.writes if a == sp.REG_SPEED]
    check("--sweep 退出码 0", rc == 0, str(rc))
    check("--sweep 逐档都写了", speed_writes[:3] == [1, 20, 100], str(speed_writes))
    check("--sweep 收尾写回 100", speed_writes[-1] == 100, str(speed_writes))

    # --- 6. 越界速度在碰硬件之前就被拒 ---
    fake = FakeGripper({sp.REG_INIT_STATUS: 1})
    rc, out = run_main(["--go", "--speed-pct", "0"], fake)
    check("速度 0 拒绝（退出码 2）", rc == 2, str(rc))
    check("速度 0 时零写入", fake.writes == [], str(fake.writes))

    # --- 7. 冷启动路径（掉电后未初始化 → 先写速度再初始化）---
    cold = {sp.REG_INIT_STATUS: 0, sp.REG_ACTUAL: 0, sp.REG_SPEED: 100,
            sp.REG_FORCE: 0, sp.REG_TARGET: 1000}
    fake = FakeGripper(cold)
    rc, out = run_main(["--preinit-speed", "20"], fake)
    check("冷启动**不带 --go** 退出码 0 且零写入", rc == 0 and fake.writes == [],
          "%s %s" % (rc, fake.writes))
    check("冷启动不带 --go 打印了会做什么", "0x0100" in out and "0x0104" in out, out[-200:])

    fake = FakeGripper(dict(cold))
    rc, out = run_main(["--go", "--preinit-speed", "20"], fake)
    kinds = [(a, v) for (a, v) in fake.writes]
    check("冷启动 --go 退出码 0", rc == 0, str(rc))
    check("冷启动**先写速度、再写初始化**",
          kinds[:2] == [(sp.REG_SPEED, 20), (sp.REG_INIT, 1)], str(kinds[:3]))
    check("冷启动最后把速度收回 100",
          [v for (a, v) in kinds if a == sp.REG_SPEED][-1] == 100, str(kinds))
    check("冷启动没把回读当判据（按耗时比判决）", "耗时比" in out, out[-300:])

    # 已经初始化过 ⇒ 冷启动路径没了，必须拒（且零写入）
    fake = FakeGripper({sp.REG_INIT_STATUS: 1, sp.REG_ACTUAL: 1000, sp.REG_SPEED: 100,
                        sp.REG_FORCE: 100, sp.REG_TARGET: 1000})
    rc, out = run_main(["--go", "--preinit-speed", "20"], fake)
    check("0x0200=1 时冷启动退出码 2", rc == 2, str(rc))
    check("0x0200=1 时冷启动零写入", fake.writes == [], str(fake.writes))

    # 越界速度在碰硬件之前就拒（冷启动路径同样）
    fake = FakeGripper(dict(cold))
    rc, out = run_main(["--go", "--preinit-speed", "0"], fake)
    check("冷启动速度 0 拒绝且零写入", rc == 2 and fake.writes == [],
          "%s %s" % (rc, fake.writes))

    print("=" * 70)
    bad = [r for r in RESULTS if not r[1]]
    print("PASS %d   FAIL %d" % (len(RESULTS) - len(bad), len(bad)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
