#!/usr/bin/env python3
"""整轮编排器（``stage4_round_run.py``）的自检：**不连板子、不动臂、不做 IK**。

这一份测的是**编排与闸门**，不是几何：
· 几何（机位、抓放、动作表）由 ``stage4_clear_run`` 自己的测试把守；
· 这里把 ``plan_views`` / ``plan_sequence`` / ``actions_from_attempt`` 换成
  写假计划文件的桩，板子换成 ``FakeBoard``，于是「哪一阶段先跑、令牌不够在
  哪停、失败了会不会偷偷重试、b̂ 符号朝哪边」这些**只有编排器负责**的事，
  可以逐条钉死。

钉住的事：

A 纯函数：估时、R1/R2 边界、b̂ 种子三种来源、解垫文档校验、预检全绿/逐条红、
  facts 不伪造安全数字、关节排序；
B 布局表与 ``gen_scene`` 逐块一致（「布局即真值」的前提）；
C 阶段编排：
  1) 干跑只到阶段 4，不执行；
  2) 全绿一趟：八个阶段全落、run.sh 只跑**一次**、令牌守卫线在脚本里；
  3) R1 不足 ⇒ 停在 compute（rc 3），一条执行都没发；
  4) R1 够、R2 不足 ⇒ 同样不发执行（rc 3）；
  5) 预检红（夹爪未初始化）⇒ rc 2，连授权都不进入；
  6) 缺 b̂ 种子 ⇒ rc 2，不发执行；
  7) 执行失败 ⇒ rc 5，**不自动重试**（run.sh 只调一次）；
  8) 执行中令牌过期 ⇒ rc 3 且本轮 ``void=true``（aborted_token，不记抓取失败）；
  9) 判分不完整 ⇒ rc 6；
 10) 探针报清障 / 格位不符 ⇒ rc 4，不执行；
 11) ``--from/--to`` 只跑指定阶段。

跑法：``python3 scripts/test_stage4_round_run.py``；退出码 0 = 全通过。
"""

import contextlib
import io
import json
import os
import sys
import tempfile
import types

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_clear_model as cm                                 # noqa: E402
import stage4_layouts as lay                                    # noqa: E402
import stage4_round_run as r                                    # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []


def check(name, cond, detail=""):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print("  PASS  %s" % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print("  FAIL  %s%s" % (name, ("\n        " + detail) if detail else ""))


def quiet(fn, *a, **kw):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = fn(*a, **kw)
    return rc, buf.getvalue()


# =====================================================================
print("--- A. 纯函数 ---")

with tempfile.TemporaryDirectory() as td:
    good = os.path.join(td, "a.json")
    with open(good, "w") as f:
        json.dump({"waypoints": [{"duration": 10.0}, {"duration": 5.0}]}, f)
    view = os.path.join(td, "b.json")
    with open(view, "w") as f:
        json.dump({"waypoints": [{"duration": 2.0}]}, f)
    est = r.exec_estimate([
        {"path": good, "kind": "target"},
        {"path": view, "kind": "view", "view": "pick"}])
    # 15 + (5+1) + 2 + (5+1) + 1×6 = 35
    check("exec_estimate：Σduration + 每份开销 + burst 开销", abs(est - 35.0) < 1e-6,
          repr(est))
    miss = os.path.join(td, "nope.json")
    try:
        r.exec_estimate([{"path": miss, "kind": "target"}])
        check("exec_estimate：读不了计划 ⇒ 抛（不许当 0 用）", False)
    except OSError:
        check("exec_estimate：读不了计划 ⇒ 抛（不许当 0 用）", True)

check("r1_need = 计算 + 执行 + 余量", abs(r.r1_need(200, 300) - 560.0) < 1e-6)
ok, msg = r.r1_gate(560, 200, 300)
check("R1 恰好够 ⇒ 通过", ok, msg)
ok, msg = r.r1_gate(559.9, 200, 300)
check("R1 差 0.1 秒 ⇒ 不过（边界真的在判）", not ok, msg)
ok, msg = r.r1_gate(None, 200, 300)
check("R1 没有令牌 ⇒ 不过（fail-closed）", not ok and "STOP" in msg, msg)

check("R2 余量 = max(45, 15%)", r.r2_margin(100) == 45.0 and
      abs(r.r2_margin(400) - 60.0) < 1e-9)
ok, _ = r.r2_gate(160, 100)
check("R2 100 执行 + 45 余量 = 145，剩 160 ⇒ 通过", ok)
ok, _ = r.r2_gate(144, 100)
check("R2 剩 144 < 145 ⇒ 不过", not ok)
ok, _ = r.r2_gate(None, 100)
check("R2 没有令牌 ⇒ 不过", not ok)

b = r.bias_setup("fixed", "-2.791,-0.248", None)
check("b̂ fixed 显式种子：命令偏置 = 种子",
      b["mode"] == "fixed" and b["applied_mm"] == [-2.791, -0.248], repr(b))
b0 = r.bias_setup("none", "-2.791,-0.248", None)
check("b̂ none：显式给种子也不补（[0,0] + 标未补偿）",
      b0["mode"] == "none" and b0["applied_mm"] == [0.0, 0.0]
      and "none" in b0["note"], repr(b0))
try:
    r.bias_setup("fixed", None, "/no/such/estimate.json")
    check("b̂ 缺种子且非 none ⇒ 停轮（RoundError 2）", False)
except r.RoundError as e:
    check("b̂ 缺种子且非 none ⇒ 停轮（RoundError 2）", e.code == r.EXIT_MISSING)
try:
    r.bias_setup("adaptive", "1,1", None)
    check("b̂ 不收 adaptive（与整轮一次成脚本冲突）", False)
except ValueError:
    check("b̂ 不收 adaptive（与整轮一次成脚本冲突）", True)

with tempfile.TemporaryDirectory() as td:
    est_path = os.path.join(td, "bias_estimate.json")
    with open(est_path, "w") as f:
        json.dump({"version": 3, "model": "constant_pre_nudge",
                   "bias_mm": [-2.5, -0.1], "n": 40}, f)
    b2 = r.bias_setup("fixed", None, est_path)
    check("b̂ 缺 --bias-seed 时读 bias_estimate.json",
          b2["applied_mm"] == [-2.5, -0.1] and b2["source"] == est_path, repr(b2))
    with open(est_path, "w") as f:
        json.dump({"version": 1, "bias_mm": [-2.5, -0.1], "n": 40}, f)
    try:
        r.bias_setup("fixed", None, est_path)
        check("b̂ 版本不符的估计当没有 ⇒ 停轮", False)
    except r.RoundError:
        check("b̂ 版本不符的估计当没有 ⇒ 停轮", True)

pad_ok = {"kind": "stage2-grid", "n_frames_ok": 20, "n_frames_total": 20,
          "spread_xy_mm": {"max": 1.0}, "spread_yaw_deg": {"max": 0.2},
          "grid_center_base_mm": [424.2, -112.24],
          "grid_yaw_base_deg": -179.778}
check("解垫文档合格 ⇒ 无问题", r.validate_pad_doc(pad_ok) == [])
check("解垫：帧数不足 ⇒ 红",
      any("帧" in x for x in
          r.validate_pad_doc(dict(pad_ok, n_frames_ok=10))))
check("解垫：xy 极差超限 ⇒ 红",
      any("xy" in x for x in
          r.validate_pad_doc(dict(pad_ok, spread_xy_mm={"max": 5.0}))))
check("解垫：kind 不对 ⇒ 红",
      any("kind" in x for x in r.validate_pad_doc(dict(pad_ok, kind="x"))))

EXPECT = {"serial": "SN-1", "tree_sha256": "abc123", "tree_files": 7,
          "stack": {"ur_robot_driver": True}}
GOT = {"serial": "SN-1", "tree": {"sha256": "abc123", "files": 7},
       "gripper": {"ok": True, "init_state": 1},
       "realsense": [{"dev": "1-1", "speed": "5000"}],
       "stack": {"ur_robot_driver": True}}
check("预检全绿 ⇒ 无问题", r.evaluate_preflight(GOT, EXPECT) == [])
check("预检缺 --expect ⇒ 红（不知道是哪块板）",
      bool(r.evaluate_preflight(GOT, None)))
check("预检：序列号不符 ⇒ 红",
      bool(r.evaluate_preflight(dict(GOT, serial="SN-2"), EXPECT)))
check("预检：代码树哈希不符 ⇒ 红",
      bool(r.evaluate_preflight(
          dict(GOT, tree={"sha256": "ffff", "files": 7}), EXPECT)))
check("预检：夹爪 0x0200=0 ⇒ 红（未初始化，不得自动初始化）",
      bool(r.evaluate_preflight(
          dict(GOT, gripper={"ok": True, "init_state": 0}), EXPECT)))
check("预检：腕部相机在 USB2 ⇒ 红",
      bool(r.evaluate_preflight(
          dict(GOT, realsense=[{"dev": "1-1", "speed": "480"}]), EXPECT)))
check("预检：栈里没驱动 ⇒ 红",
      bool(r.evaluate_preflight(dict(GOT, stack={}), EXPECT)))
# 序列号真值不许出现在问题文本/日志里（AGENTS.md 红线 3）
_msg = "；".join(r.evaluate_preflight(dict(GOT, serial="SN-2"), EXPECT))
check("预检：序列号不符只打指纹（真值不进文本）",
      "SN-2" not in _msg and "SN-1" not in _msg and
      r.serial_fp("SN-2") in _msg, _msg)
check("预检：空树（路径对不上）⇒ 红，不靠期望值蒙过去",
      any("0 个文件" in x for x in r.evaluate_preflight(
          dict(GOT, tree={"sha256": "e3b0c442", "files": 0}),
          {"serial": "SN-1", "stack": {"ur_robot_driver": True}})))

facts = r.build_facts("S0-1-r1", "S0", 1, lay.layout_blocks("S0", 1),
                      {"mode": "none", "applied_mm": [0, 0]})
check("facts：人没给安全数字 ⇒ 不伪造（不写 safety 键）",
      "safety" not in facts and facts["auto_top1"] is None)
facts2 = r.build_facts("S0-1-r1", "S0", 1, lay.layout_blocks("S0", 1),
                       {"mode": "none", "applied_mm": [0, 0]},
                       {"safety": {"collisions": 0}, "interventions":
                        {"count": 1}, "human_review": {"required": False}})
check("facts：人给了就原样并入", facts2["safety"]["collisions"] == 0
      and facts2["interventions"]["count"] == 1)
check("facts：specs 按块 id 记颜色/边长",
      facts["specs"]["0"]["color"] == "yellowgreen" and
      abs(facts["specs"]["0"]["size_m"] - cm.BLOCK_M) < 1e-12)

names = ["b", "10", "a", "2"]
check("_sorted_ids：数字按数值、其余按串",
      r._sorted_ids(names) == ["2", "10", "a", "b"], repr(r._sorted_ids(names)))
cells = cm.parse_cells("8,6,4,2,0", 5)
amap = cm.assign_cells([4, 0, 3, 1, 2], cells)
check("assign_cells：id 升序 → 8,6,4,2,0",
      amap == {"0": 8, "1": 6, "2": 4, "3": 2, "4": 0}, repr(amap))

chain = types.SimpleNamespace(move_joints=["j2", "j1"])
doc = {"name": ["j1", "j2"], "position": [0.2, 0.1]}
check("_q_from_joints 按链的关节顺序重排",
      r._q_from_joints(doc, chain) == [0.1, 0.2])
try:
    r._q_from_joints({"name": ["j1"], "position": [0.2]}, chain)
    check("_q_from_joints 缺关节 ⇒ 停（不猜 0）", False)
except r.RoundError:
    check("_q_from_joints 缺关节 ⇒ 停（不猜 0）", True)

# =====================================================================
print("--- B. 布局表 = gen_scene（布局即真值的唯一出处） ---")
check("布局表与 gen_scene 逐块一致", lay.compare_with_gen_scene("S0") == [],
      repr(lay.compare_with_gen_scene("S0")))
errs = []
for k in sorted(lay.BATCHES["S0"]):
    errs += lay.validate_layout("S0", k)
check("三个布局自检（区内 + 间距 ≥60 mm）", errs == [], repr(errs))

# =====================================================================
print("--- C. 阶段编排（FakeBoard + 打桩规划） ---")

PLAN_CALLS = []
DEVIATION_CALLS = []


def _write_plan(path, duration=10.0):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump({"waypoints": [{"duration": duration},
                                 {"duration": duration}]}, f)


def fake_plan_views(chain_, q, views, out_dir, start=0):
    plans = []
    os.makedirs(out_dir, exist_ok=True)
    for i, v in enumerate(views, start=int(start) + 1):
        name = "%02d-view-%s.json" % (i, v["name"])
        path = os.path.join(out_dir, name)
        _write_plan(path)
        plans.append({"name": name, "path": path, "kind": "view",
                      "view": v["name"], "slot": [0.5, 0.3],
                      "n_waypoints": 2})
    return plans, list(q)


def fake_plan_sequence(chain_, q, actions, blocks, out_dir,
                       survey_tool0=None, place_clearance=None,
                       wrist_survey=True, views=None, place_yaw_equiv=False,
                       start=0, min_block_gap=0.0, gap_check="probe_verified"):
    PLAN_CALLS.append({"start": start, "wrist_survey": wrist_survey,
                       "gap_check": gap_check, "min_block_gap": min_block_gap,
                       "n_actions": len(actions)})
    plans = []
    os.makedirs(out_dir, exist_ok=True)
    for i, a in enumerate(actions):
        name = "%02d-%s-block-%s.json" % (int(start) + i, a["kind"], a["id"])
        path = os.path.join(out_dir, name)
        _write_plan(path)
        plans.append({"name": name, "path": path, "kind": a["kind"],
                      "slot": a.get("slot"), "n_waypoints": 2})
    new_blocks = [dict(b) for b in blocks]
    for a in actions:
        if a["kind"] == "target":
            for b in new_blocks:
                if str(b["id"]) == str(a["id"]):
                    b["x"], b["y"] = a.get("commanded_center_mm", [0, 0])
                    b["x"] /= 1000.0
                    b["y"] /= 1000.0
    return plans, new_blocks, list(q)


def fake_actions_from_attempt(truth, attempt, pad_thick, bias_mm=None):
    bias = bias_mm or [0.0, 0.0]
    cell = int(attempt["cell"])
    cx, cy = cm.cell_center(cell)
    desired = [cx * 1000.0, cy * 1000.0]
    cmd = [desired[0] - bias[0], desired[1] - bias[1]]
    act = {"kind": "target", "id": attempt["id"], "slot": [cmd[0] / 1000.0,
                                                           cmd[1] / 1000.0],
           "desired_center_mm": [round(v, 3) for v in desired],
           "commanded_center_mm": [round(v, 3) for v in cmd],
           "applied_bias_mm": [round(v, 3) for v in bias]}
    return [act], truth["blocks"]


class Rc:
    def __init__(self, code=0, out="", err=""):
        self.returncode = code
        self.stdout = out
        self.stderr = err


class FakeBoard:
    def __init__(self):
        self.calls = []
        self.pulls = []
        self.files = {}
        self.pushed = []
        self.preflight_doc = {
            "serial": "SN-1", "tree": {"sha256": "abc123", "files": 7},
            "gripper": {"ok": True, "init_state": 1},
            "realsense": [{"dev": "1-1", "speed": "5000"}],
            "stack": {"ur_robot_driver": True}}
        self.tokens = [(("t" * 32), 800.0, "")]
        self._last_token = (None, None, "没有令牌")
        self.pre_results = [(0, "PRE DONE")]
        self.run_results = [(0, "ALL DONE")]

    def _next(self, seq, default):
        return seq.pop(0) if seq else default

    def ssh(self, cmd, quiet=True, check=False):
        # 编排器只用 ssh 做 chmod 这类收尾动作；记录即可（真机路径由 RoundBoard 管）
        self.calls.append(("ssh", cmd))
        return Rc(0)

    def preflight(self, repo):
        self.calls.append(("preflight", repo))
        return self.preflight_doc

    def joints(self, dest):
        self.calls.append(("joints", dest))
        os.makedirs(dest, exist_ok=True)
        doc = {"name": ["j1", "j2"], "position": [0.1, 0.2]}
        with open(os.path.join(dest, "joint_states.json"), "w") as f:
            json.dump(doc, f)
        return doc

    def token_state(self):
        # 队列用完就**重复最后一条**（真实令牌在 600 s 内一直有效，不是一次性的）
        if self.tokens:
            self._last_token = self.tokens.pop(0)
        tok, rem, why = self._last_token
        self.calls.append(("token_state", rem))
        return tok, rem, why

    def wait_token(self, timeout_s, poll_s, now=None, sleep=None, out=None):
        self.calls.append(("wait_token", timeout_s))
        return ("t" * 32, 780.0)

    def push(self, local, remote, label=None):
        self.pushed.append(remote)

    def push_many(self, pairs):
        for local, remote in pairs:
            self.push(local, remote)

    def pull_file(self, remote, local):
        self.pulls.append(("file", remote))
        os.makedirs(os.path.dirname(os.path.abspath(local)) or ".",
                    exist_ok=True)
        val = self.files.get(remote)
        with open(local, "w", encoding="utf-8") as f:
            if isinstance(val, (dict, list)):
                json.dump(val, f)
            elif val is not None:
                f.write(str(val))
            else:
                f.write("")

    def pull_dir(self, remote, local):
        self.pulls.append(("dir", remote))
        os.makedirs(local, exist_ok=True)
        with open(os.path.join(local, "color_000.png"), "wb") as f:
            f.write(b"\x89PNG")

    def probe(self, repo, cells, group, truth_remote, grid_remote,
              report_remote):
        self.calls.append(("probe", cells))
        self.files[report_remote] = self.probe_report()
        return Rc(0)

    def probe_report(self):
        blocks = lay.layout_blocks("S0", 1)
        ids = r._sorted_ids([b["id"] for b in blocks])
        cmap = cm.assign_cells(ids, cm.parse_cells("8,6,4,2,0"))
        attempts = [{"id": int(i), "ok": True, "cell": cmap[i],
                     "clearing_steps": [], "reason": None,
                     "terminal": {"ok": True}}
                    for i in ids]
        return {"scenes": [{"attempts": attempts}],
                "scene_cleanup": {"ok": True, "world": [], "attached": []}}

    def run_script(self, remote, token):
        self.calls.append(("run_script", remote))
        if remote.endswith("pre.sh"):
            return self._next(self.pre_results, (0, "PRE DONE"))
        return self._next(self.run_results, (0, "ALL DONE"))


def make_args(td, **over):
    layout = over.pop("layout", 1)
    bias_seed = over.pop("bias_seed", "-2.791,-0.248")
    argv = ["--batch", "S0", "--layout", str(layout),
            "--run", "1",
            "--out-dir", os.path.join(td, "run", "s0-1-r1"),
            "--urdf", os.path.join(td, "robot.urdf"),
            "--board", "fake", "--board-repo", "/repo",
            "--board-dir", "/b",
            "--expect", os.path.join(td, "expect.json")]
    if bias_seed is not False:
        argv += ["--bias-seed", str(bias_seed)]
    for key, val in over.items():
        if val is False:
            continue
        flag = "--" + key.replace("_", "-")
        if val is True:
            argv.append(flag)
        else:
            argv += [flag, str(val)]
    ap = r.build_parser()
    args = r.prepare(ap, r.parse_cli(ap, argv))
    return args


def write_expect(td):
    with open(os.path.join(td, "expect.json"), "w") as f:
        json.dump(EXPECT, f)


# 板上镜像目录缺省值：Round 与 main() 必须同源（2026-11-11 不给 --board-dir
# 时 main() 曾把 None 交给 RoundBoard，崩在 None.rstrip）
_td_bb = tempfile.mkdtemp()
_out_bb = os.path.join(_td_bb, "s0-1-r0")
os.makedirs(_out_bb, exist_ok=True)
_dflt = r.default_board_dir(_out_bb)
check("default_board_dir = ~/precision/e9/<run>/<round>",
      _dflt == "~/precision/e9/%s/s0-1-r0" % os.path.basename(_td_bb), repr(_dflt))
write_expect(_td_bb)
_a_bb = make_args(_td_bb, run=0, out_dir=_out_bb)
_a_bb.board_dir = ""            # 真实情形：命令行没给 --board-dir
_rd_bb = r.Round(_a_bb, types.SimpleNamespace(alias="x", exp_dir=_td_bb))
check("Round 解析的 board_dir 与 default_board_dir 一致",
      _rd_bb.board_dir == _dflt, repr(_rd_bb.board_dir))
try:
    r.RoundBoard("x", "", _td_bb)
    check("RoundBoard 空目录 ⇒ 明确报错（不是 None.rstrip）", False)
except r.RoundError:
    check("RoundBoard 空目录 ⇒ 明确报错（不是 None.rstrip）", True)


def patched(deps):
    """装上规划桩与 deps；返回 (undo, deps)。"""
    saves = {}

    def setm(obj, name, val):
        saves.setdefault((id(obj), name), (obj, name, getattr(obj, name)))
        setattr(obj, name, val)

    setm(r.urkin, "Chain", lambda urdf: types.SimpleNamespace(
        move_joints=["j1", "j2"]))
    setm(r.cr, "plan_views", fake_plan_views)
    setm(r.cr, "plan_sequence", fake_plan_sequence)
    setm(r.cr, "actions_from_attempt", fake_actions_from_attempt)
    setm(r.cr, "compare_layout",
         lambda man, post, **kw: {"ok": True, "n_blocks": len(post)})

    def undo():
        for (_, _), (obj, name, val) in saves.items():
            setattr(obj, name, val)

    return undo


def fake_solve_pad_writer():
    def f(burst, out, min_frames=20, max_spread_mm=2.0, max_spread_deg=0.5):
        with open(out, "w") as fh:
            json.dump(pad_ok, fh)
        return 0
    return f


def fake_truth(burst, out):
    with open(out, "w") as f:
        json.dump({"blocks": []}, f)
    return 0


def fake_colour(offset=(1.0, 0.5), ok=True):
    def f(burst, xy_mm, top_z, expect_class, hue_deg, search_mm):
        return {"ok": ok, "xy_mm": [xy_mm[0] + offset[0], xy_mm[1] + offset[1]],
                "hue_deg": 30.0, "n_frames": 8, "n_missed": 0}
    return f


def fake_deviation(argv):
    DEVIATION_CALLS.append(list(argv))
    return 0


def base_deps(**over):
    d = {"solve_pad": fake_solve_pad_writer(),
         "truth_from_view": fake_truth,
         "measure_colour": fake_colour(),
         "round_report": lambda argv: 0,
         "placement_deviation": fake_deviation,
         "score": lambda argv: 0,
         "now": (lambda: 1000.0),
         "sleep": (lambda s: None)}
    d.update(over)
    return d


def run(td, board, deps=None, **over):
    write_expect(td)
    global PLAN_CALLS, DEVIATION_CALLS
    PLAN_CALLS = []
    DEVIATION_CALLS = []
    undo = patched(deps)
    try:
        args = make_args(td, **over)
        rd = r.Round(args, board, deps=base_deps(**(deps or {})))
        rc, out = quiet(rd.run)
        if rc != 0 and os.environ.get("ROUND_TEST_DEBUG"):
            print(out)
        return rc, out, rd
    finally:
        undo()
        cm.reset_pad()


def phases_of(rd):
    return [p["phase"] for p in rd.phases]


# ---- C1 干跑
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, dry_run=True)
    check("干跑：rc 0 且停在 compute", rc == 0 and
          phases_of(rd) == ["preflight", "views", "authorize", "observe",
                            "compute"], repr(phases_of(rd)))
    check("干跑：run.sh 已写出但不执行",
          os.path.exists(rd.p("run.sh")) and
          not any(c[0] == "run_script" and c[1].endswith("run.sh")
                  for c in b.calls), repr([c for c in b.calls
                                           if c[0] == "run_script"]))
    runsh = open(rd.p("run.sh")).read()
    check("干跑：run.sh 带令牌守卫线",
          "real_motion_authorize.py --check-token" in runsh and
          "--min-remaining" in runsh)
    presh = open(rd.p("pre.sh")).read()
    check("跑前只拍 pick/pad 两机位",
          "01-view-pick" in presh and "02-view-pad" in presh and
          "view-free" not in presh, presh[:400])
    check("跑后三机位只追加一次（06/07/08）",
          all(("%02d-view-%s" % (i, n)) in runsh
              for i, n in ((6, "pick"), (7, "free"), (8, "pad"))),
          runsh[-600:])

# ---- C2 全绿
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b)
    check("全绿：rc 0、八个阶段齐全",
          rc == 0 and phases_of(rd) == list(r.PHASES), repr(phases_of(rd)))
    runs = [c for c in b.calls if c[0] == "run_script"]
    check("全绿：pre.sh 与 run.sh 各一次、顺序对",
          [os.path.basename(c[1]) for c in runs] == ["pre.sh", "run.sh"],
          repr(runs))
    check("全绿：run.sh 输出含 ALL DONE 才认", "ALL DONE" in out)
    man = json.load(open(rd.p("plan", "manifest-0.json")))
    cmd = [a for a in man["actions"] if a["kind"] == "target"][0]
    check("manifest：命令 = 期望 − b̂，且期望仍是格心",
          abs(cmd["commanded_center_mm"][0]
              - (cmd["desired_center_mm"][0] + 2.791)) < 1e-6 and
          abs(cmd["commanded_center_mm"][1]
              - (cmd["desired_center_mm"][1] + 0.248)) < 1e-6,
          repr(cmd))
    check("manifest：applied_bias 与 --bias-seed 同值（整轮 fixed）",
          man["bias_mm"] == [-2.791, -0.248], repr(man.get("bias_mm")))
    check("manifest：gap_check 记 layout_gap，不是 probe_verified",
          man["gap_check"] == "layout_gap")
    check("plan_sequence 逐目标 wrist_survey=False",
          PLAN_CALLS and all(p["wrist_survey"] is False for p in PLAN_CALLS),
          repr(PLAN_CALLS))
    check("plan_sequence 的 start 逐段递增（计划名不撞）",
          [p["start"] for p in PLAN_CALLS] == [0, 1, 2, 3, 4], repr(PLAN_CALLS))
    bias = json.load(open(rd.p("bias.json")))
    # 样本 = 彩色质心 − 命令 = (期望 + 彩偏) − (期望 − 种子) = 彩偏 + 种子
    want = [1.0 + (-2.791), 0.5 + (-0.248)]
    check("判分：逐块 b̂ = 彩色质心 − 命令 = 彩偏 + 种子",
          bias["samples_mm"] and all(
              abs(s["bias_mm"][0] - want[0]) < 5e-3 and
              abs(s["bias_mm"][1] - want[1]) < 5e-3
              for s in bias["samples_mm"]), repr(bias.get("samples_mm")))
    est = os.path.join(td, "run", "bias_estimate.json")
    check("跑完写回 <run>/bias_estimate.json（下一轮种子）",
          os.path.exists(est) and
          abs(json.load(open(est))["bias_mm"][0] - want[0]) < 5e-3)
    timing = json.load(open(rd.p("timing.json")))
    check("timing：R1 / R2 / exec_est 都落盘",
          timing["r1"]["ok"] and timing["r2"]["ok"] and
          timing["exec_est_s"] > 0, repr(timing))
    check("摆位偏差：用跑前 pick burst 出 record-only 表（不缺数就不跳过）",
          len(DEVIATION_CALLS) == 1
          and "--truth" in DEVIATION_CALLS[0]
          and DEVIATION_CALLS[0][DEVIATION_CALLS[0].index("--out") + 1]
          .endswith("placement-deviation.json"), repr(DEVIATION_CALLS))

# ---- C3 R1 不足
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    b.tokens = [(("t" * 32), 100.0, "")]
    rc, out, rd = run(td, b)
    check("R1 不足 ⇒ rc 3、停在 compute", rc == r.EXIT_GATE and
          phases_of(rd)[-1] == "compute", repr((rc, phases_of(rd))))
    check("R1 不足 ⇒ 一条执行都没发",
          not any(c[0] == "run_script" and c[1].endswith("run.sh")
                  for c in b.calls))
    check("R1 不足 ⇒ 写 timing.json 供重标",
          json.load(open(rd.p("timing.json")))["r1"]["ok"] is False)

# ---- C4 R2 不足
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    b.tokens = [(("t" * 32), 750.0, ""), (("t" * 32), 100.0, "")]
    rc, out, rd = run(td, b)
    check("R2 不足 ⇒ rc 3、停在 compute",
          rc == r.EXIT_GATE and phases_of(rd)[-1] == "compute",
          repr((rc, phases_of(rd))))
    check("R2 不足 ⇒ 不发执行",
          not any(c[0] == "run_script" and c[1].endswith("run.sh")
                  for c in b.calls))
    t = json.load(open(rd.p("timing.json")))
    check("timing 记 R1 通过、R2 不过", t["r1"]["ok"] and
          t["r2"]["ok"] is False, repr(t))

# ---- C5 预检红
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    b.preflight_doc = dict(b.preflight_doc,
                           gripper={"ok": True, "init_state": 0})
    rc, out, rd = run(td, b)
    check("预检红（夹爪未初始化）⇒ rc 2、停在 preflight",
          rc == r.EXIT_MISSING and phases_of(rd) == ["preflight"],
          repr((rc, phases_of(rd))))
    check("预检红 ⇒ 不进入授权、不跑任何脚本",
          not any(c[0] in ("wait_token", "run_script") for c in b.calls))

# ---- C6 缺 b̂ 种子
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, bias_seed=False)
    check("缺 b̂ 种子且未显式 none ⇒ rc 2、停在 compute",
          rc == r.EXIT_MISSING and phases_of(rd)[-1] == "compute",
          repr((rc, phases_of(rd))))
    check("缺 b̂ 种子 ⇒ 不发执行",
          not any(c[0] == "run_script" and c[1].endswith("run.sh")
                  for c in b.calls))

# ---- C7 执行失败不重试
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    b.run_results = [(3, "boom")]
    rc, out, rd = run(td, b)
    runs = [c for c in b.calls if c[0] == "run_script"
            and c[1].endswith("run.sh")]
    check("执行失败 ⇒ rc 5、停在 execute", rc == r.EXIT_EXEC and
          phases_of(rd)[-1] == "execute", repr((rc, phases_of(rd))))
    check("执行失败 ⇒ run.sh 只调一次（不自动重试）", len(runs) == 1,
          repr(runs))
    check("执行失败 ⇒ 仍回拉证据（jsonl + 跑后 burst 都试过）",
          any(k == "dir" and "wrist-post" in p for k, p in b.pulls) and
          any(k == "file" and p.endswith(".jsonl") for k, p in b.pulls),
          repr(b.pulls))

# ---- C8 执行中令牌过期 ⇒ void
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    b.run_results = [(9, "!!!!! STOP: 令牌剩余不足 20 秒（rc=1）—— 先重做 STOP→PLAY；"
                         "本轮 void（aborted_token），不算抓取失败")]
    rc, out, rd = run(td, b)
    row = [p for p in rd.phases if p["phase"] == "execute"][0]
    check("执行中令牌过期 ⇒ rc 3（不是 5）", rc == r.EXIT_GATE, repr(rc))
    check("且本轮标 void=true（aborted_token，不记抓取失败）",
          row.get("void") is True, repr(row))
    check("aborted_token 的提示写明不算抓取失败", "不算抓取失败" in out)

# ---- C9 判分不完整
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, deps={"round_report": lambda argv: 2})
    check("判分不完整 ⇒ rc 6、停在 score", rc == r.EXIT_SCORE and
          phases_of(rd)[-1] == "score", repr((rc, phases_of(rd))))

# ---- C10 探针问题
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()

    def with_clearing():
        doc = b.probe_report()
        doc["scenes"][0]["attempts"][2]["clearing_steps"] = [{"block": 0}]
        return doc
    b.probe_report = with_clearing
    rc, out, rd = run(td, b)
    check("探针要求清障 ⇒ rc 4（累积清障未实现，不假装能跑）",
          rc == r.EXIT_PLAN and phases_of(rd)[-1] == "compute", repr(rc))
    check("探针报清障 ⇒ 不发执行",
          not any(c[0] == "run_script" and c[1].endswith("run.sh")
                  for c in b.calls))

with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()

    def bad_cell():
        doc = b.probe_report()
        doc["scenes"][0]["attempts"][0]["cell"] = 7
        return doc
    b.probe_report = bad_cell
    rc, out, rd = run(td, b)
    check("探针格与本轮格不符 ⇒ rc 4", rc == r.EXIT_PLAN, repr(rc))

with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()

    def dirty():
        doc = b.probe_report()
        doc["scene_cleanup"] = {"ok": False, "world": ["block-0"],
                                "attached": []}
        return doc
    b.probe_report = dirty
    rc, out, rd = run(td, b)
    check("探针留下幽灵块 ⇒ rc 4（先清场景）", rc == r.EXIT_PLAN, repr(rc))

# ---- C11 --from/--to
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, to="observe")
    check("--to observe：只跑前四段",
          [p["phase"] for p in rd.phases] == list(r.PHASES[:4]),
          repr([p["phase"] for p in rd.phases]))
    check("--to observe：compute 没跑 ⇒ 没有 run.sh",
          not os.path.exists(rd.p("run.sh")))

with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, dry_run=True)
    args = rd.args
    args.from_ = "archive"
    args.to = "archive"
    args.dry_run = False
    undo = patched(None)
    try:
        rd2 = r.Round(args, b, deps=base_deps())
        rc2, out2 = quiet(rd2.run)
    finally:
        undo()
        cm.reset_pad()
    check("--from/--to archive：只跑归档一段",
          [p["phase"] for p in rd2.phases] == ["archive"] and rc2 == 0,
          repr((rc2, [p["phase"] for p in rd2.phases])))

# ---- C12 |b̂|>15 mm 记失败、不当偏置
with tempfile.TemporaryDirectory() as td:
    b = FakeBoard()
    rc, out, rd = run(td, b, deps={"measure_colour": fake_colour(offset=(20.0, 0.0))})
    bias = json.load(open(rd.p("bias.json")))
    check("|b̂|>15 mm ⇒ 记 failures，不进 samples",
          bias["samples_mm"] == [] and len(bias["failures_mm"]) == 5,
          repr((bias["samples_mm"], bias["failures_mm"])))
    check("|b̂|>15 mm ⇒ 不写 bias_estimate",
          not os.path.exists(os.path.join(td, "run", "bias_estimate.json")))
    check("|b̂|>15 mm 仍能完成判分（失败分类交给 round_report）", rc == 0,
          repr(rc))

# =====================================================================
print()
print("通过 %d，失败 %d" % (N_PASS, N_FAIL))
if FAILS:
    print("失败项：")
    for f in FAILS:
        print("  - %s" % f)
sys.exit(0 if N_FAIL == 0 else 1)
