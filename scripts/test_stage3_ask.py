#!/usr/bin/env python3
"""``stage3_entry.ask`` 的放行方式测试：enter / file / auto。

跑法：
  python3 scripts/test_stage3_ask.py

为什么不测「能跑通」就完事
──────────────────────────
``ask`` 是**「需要人动手」那一步的闸门**：③④ 的节奏是人搭塔 → 机器量。
非交互管道下老实现抓到 ``EOFError`` 就打印「直接继续」往下走 —— 后台一跑，
塔还没搭就去量、接着就去抓块。这里的用例就是这条的哨：
``file`` 必须真的**卡住**到 ``ack`` 出现；``ack`` 编号错了不许放行；
``auto`` 不许配 ``--go``。

反向变异（把下面任一处改掉，对应用例必须变红）：
· 去掉 ``if os.path.exists(ack): os.remove(ack)``  ⇒ test_stale_ack_not_honoured 红
· 把等待循环改成空转一遍就返回          ⇒ test_file_mode_blocks_until_ack 红
· 去掉 ``--go`` + ``auto`` 的拒绝         ⇒ test_auto_refused_with_go 红
"""
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage3_entry as se                                         # noqa: E402

CASES = []
TMP = tempfile.mkdtemp(prefix="stage3-ask-test-")


def case(fn):
    CASES.append(fn)
    return fn


def _fresh():
    d = tempfile.mkdtemp(dir=TMP)
    return d


def _touch(path):
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("ok\n")


def _set(**kw):
    se._ASK.update({"mode": "enter", "dir": None, "timeout_s": 0.0,
                    "seq": 0})
    se._ASK.update(kw)


# ---------------------------------------------------------------- auto
@case
def test_auto_does_not_block():
    d = _fresh()
    _set(mode="auto", dir=d)
    t0 = time.time()
    se.ask("这一步不该停")
    assert time.time() - t0 < 0.2, "auto 模式停住了"
    assert not os.listdir(d), "auto 模式不该写哨兵文件"


# ---------------------------------------------------------------- file
@case
def test_file_mode_blocks_until_ack():
    d = _fresh()
    _set(mode="file", dir=d)

    def ack_later():
        time.sleep(1.0)
        _touch(os.path.join(d, "ack-001"))

    th = threading.Thread(target=ack_later)
    th.start()
    t0 = time.time()
    se.ask("在塔位搭好 3 层直塔，然后离开工作区")
    dt = time.time() - t0
    th.join()
    assert dt >= 0.9, "没等 ack 就往下走了（%.2f s）" % dt
    need = os.path.join(d, "ask-001.txt")
    assert os.path.exists(need), "没写 ask-001.txt"
    txt = open(need, encoding="utf-8").read().strip()
    assert txt == "在塔位搭好 3 层直塔，然后离开工作区", "提示原文没落盘：%r" % txt
    assert not os.path.exists(os.path.join(d, "ack-001")), "ack 用完没删"


@case
def test_file_mode_numbers_advance():
    d = _fresh()
    _set(mode="file", dir=d)
    for i in (1, 2):
        # 注意：不能事先放好 ack —— ask() 会把「开工前就躺在那里」的 ack
        # 当成上一轮的残留删掉（见 test_stale_ack_not_honoured）。
        th = threading.Thread(target=lambda i=i: (time.sleep(0.4),
                                                  _touch(os.path.join(
                                                      d, "ack-%03d" % i))))
        th.start()
        se.ask("第 %d 步" % i)
        th.join()
    assert os.path.exists(os.path.join(d, "ask-001.txt"))
    assert os.path.exists(os.path.join(d, "ask-002.txt"))
    assert se._ASK["seq"] == 2, "序号没自增：%r" % se._ASK["seq"]


@case
def test_stale_ack_not_honoured():
    """开工前就躺着的 ack-001 不许把第一步直接放过去。"""
    d = _fresh()
    _touch(os.path.join(d, "ack-001"))
    _set(mode="file", dir=d, timeout_s=0.35)
    try:
        se.ask("这一条必须等到真的 ack")
    except SystemExit as exc:
        assert exc.code == 3, "超时退出码应为 3，实际 %r" % exc.code
    else:
        raise AssertionError("残留的 ack 被当成放行了")


@case
def test_file_mode_timeout_exits_3():
    d = _fresh()
    _set(mode="file", dir=d, timeout_s=0.35)
    t0 = time.time()
    try:
        se.ask("没人理")
    except SystemExit as exc:
        assert exc.code == 3, "超时退出码应为 3，实际 %r" % exc.code
    else:
        raise AssertionError("超时没退出，一直在等")
    assert time.time() - t0 < 5.0, "超时用了太久"
    assert os.path.exists(os.path.join(d, "ask-001.txt")), "超时前没落提示"


@case
def test_file_mode_needs_dir():
    _set(mode="file", dir=None)
    try:
        se.ask("没有目录")
    except SystemExit as exc:
        assert "ask-dir" in str(exc.code), "缺目录时应报 ask-dir：%r" % (exc.code,)
    else:
        raise AssertionError("没给哨兵目录也继续了")


# ---------------------------------------------------------------- enter
@case
def test_enter_mode_eof_does_not_block():
    """管道/后台（stdin 已 EOF）：老行为不变 —— 只提示不阻塞。"""
    d = _fresh()
    script = (
        "import sys; sys.path.insert(0, %r)\n"
        "import stage3_entry as se\n"
        "se._ASK.update({'mode': 'enter', 'dir': %r})\n"
        "se.ask('回车那一步')\n"
        "print('REACHED')\n" % (HERE, d)
    )
    p = subprocess.run([sys.executable, "-c", script], stdin=subprocess.DEVNULL,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       timeout=60)
    out = p.stdout.decode("utf-8", "replace")
    assert p.returncode == 0, "退出码 %d：%s" % (p.returncode, out)
    assert "REACHED" in out, "没走到下一步：%s" % out
    assert ">>> 需要人动手：回车那一步" in out, "提示格式变了：%s" % out


@case
def test_enter_mode_waits_for_newline():
    """有 stdin 可读时 enter 必须等那一行。"""
    d = _fresh()
    script = (
        "import sys; sys.path.insert(0, %r)\n"
        "import stage3_entry as se\n"
        "se._ASK.update({'mode': 'enter', 'dir': %r})\n"
        "se.ask('等你按回车')\n"
        "print('REACHED')\n" % (HERE, d)
    )
    p = subprocess.run([sys.executable, "-c", script], input=b"\n",
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       timeout=60)
    out = p.stdout.decode("utf-8", "replace")
    assert p.returncode == 0 and "REACHED" in out, out


# ---------------------------------------------------------------- main 闸门
@case
def test_auto_refused_with_go():
    argv = ["--mode", "tolerance", "--run-dir", _fresh(),
            "--urdf", "/nonexistent.urdf", "--go", "--ask-mode", "auto"]
    try:
        se.main(argv)
    except SystemExit as exc:
        assert "auto" in str(exc.code) and "--go" in str(exc.code), \
            "拒绝理由不清楚：%r" % (exc.code,)
    else:
        raise AssertionError("--ask-mode auto 配 --go 没被拒")


@case
def test_ask_dir_defaults_under_run_dir():
    rd = _fresh()
    argv = ["--mode", "tolerance", "--run-dir", rd,
            "--urdf", "/nonexistent.urdf"]      # 不带 --go：只打协议
    rc = se.main(argv)
    assert rc == 0, "不带 --go 应返回 0，实际 %r" % rc
    assert se._ASK["dir"] == os.path.join(rd, "_ask"), \
        "哨兵目录默认值不对：%r" % se._ASK["dir"]


def main():
    bad = 0
    for fn in CASES:
        try:
            fn()
            print("ok   %s" % fn.__name__, flush=True)
        except Exception as exc:                                   # noqa: BLE001
            bad += 1
            print("FAIL %s：%s: %s" % (fn.__name__, type(exc).__name__, exc),
                  flush=True)
    shutil.rmtree(TMP, ignore_errors=True)
    print("\n%d/%d 通过" % (len(CASES) - bad, len(CASES)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
