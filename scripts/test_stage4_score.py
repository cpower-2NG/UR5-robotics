#!/usr/bin/env python3
"""E13 判分器自检：尝试级 / 轮级 / 出口连续轮 / 变异。

为什么每节都要有变异：判分器最容易的死法不是算错，而是**口径悄悄放宽**——
void 混进分母、rc=0 当成功、真值偷偷来自控制链、人眼复核缺失默认绿。
每节最后都把源码改坏一次，确认对应的用例真的会红；改坏的模块不落盘。

跑法：
    python3 scripts/test_stage4_score.py
"""
import inspect
import os
import sys

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import stage4_score as sc                                      # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print('  PASS  %s' % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print('  FAIL  %s%s' % (name, '   ← %s' % detail if detail else ''))


def _merge(base, over):
    out = dict(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def att(**kw):
    """一次「全绿」的块级尝试，按需覆盖字段。"""
    base = {
        "target": {"block_id": "b0"},
        "spec": "red right",
        "selected": {"instance_id": 0, "auto_top1": True},
        "truth": {"instance_id": 0, "truth_source": "topcam-independent"},
        "grasp": {"exec_rc": 0, "lifted": True, "dropped": False,
                  "wrong_block": False,
                  "confirm": {"topcam": True, "wrist": True, "human": True}},
        "place": {"in_slot": True, "over_line": False, "out_of_bounds": False},
        "failure": {"class": None},
    }
    return _merge(base, kw)


def obs(atts, **kw):
    base = {
        "round": "r1", "batch": "S0", "layout": "L1", "repeat": 1,
        "n_targets": len(atts), "attempts": atts,
        "terminal": {"blocks": [{"id": 0, "zone": "slot",
                                 "in_bounds": True, "legal": True}],
                     "overlaps": 0, "off_table": 0, "out_of_bounds": 0,
                     "source": "topcam"},
        "safety": {"collisions": 0, "drops": 0, "unexpected_motion": 0,
                   "estop": 0},
        "interventions": {"count": 0},
        "human_review": {"approved": True},
    }
    if "n_targets" in kw:
        base["n_targets"] = kw.pop("n_targets")
    return _merge(base, kw)


def _mut(old, new):
    """把判分器源码改坏一处，返回改坏的模块命名空间（不落盘）。"""
    src = inspect.getsource(sc)
    assert old in src, '变异锚点不在源码里：%r' % old
    ns = {'__file__': sc.__file__, '__name__': 'mut'}
    exec(compile(src.replace(old, new, 1), '<mut>', 'exec'), ns)
    return ns


# ------------------------------------------------------------------ 常量
def section_const():
    print('\n[0] 判据快照：门槛与契约 §4.3 逐字一致')
    check('四条率门 = 90/80/90/90',
          sc.DEFAULT_THRESHOLDS == {"id_rate": 0.90, "grasp_rate": 0.80,
                                    "place_rate": 0.90, "terminal_rate": 0.90,
                                    "safety_events": 0, "interventions": 0},
          str(sc.DEFAULT_THRESHOLDS))
    check('失败分类覆盖契约七类',
          set(sc.FAIL_CLASSES) == {"perception", "candidate", "ik", "planning",
                                   "execution", "placement", "human"})
    check('独立真值白名单里没有控制链来源',
          not any('perception' in s or 'planner' in s or 'grasp' in s
                  for s in sc.TRUTH_SOURCES), str(sc.TRUTH_SOURCES))
    check('退出码映射 0/1/1/2',
          [sc.exit_code({"verdict": v})
           for v in ("ok", "fail", "person_review", "missing_data")]
          == [0, 1, 1, 2])


# ------------------------------------------------------------------ 尝试级
def section_attempt():
    print('\n[1] 尝试级：rc=0 不算成功、真值来源必查、失败必分类')
    r = sc.score_attempt(att())
    check('全绿尝试 → ok，三项都真', r["status"] == "ok" and r["id_ok"] is True
          and r["grasp_ok"] is True and r["place_ok"] is True, str(r))

    r = sc.score_attempt(att(selected={"instance_id": 1},
                             failure={"class": "candidate"}))
    check('选错块 + 已分类 → fail', r["status"] == "fail" and r["id_ok"] is False)
    check('选错块的原因写清楚', any('指定 ID' in x for x in r["reasons"]))

    r = sc.score_attempt(att(selected={"instance_id": 1}))
    check('选错块但没分类 → person_review（不许静默算失败）',
          r["status"] == "person_review"
          and any('未分类' in x for x in r["reasons"]))

    r = sc.score_attempt(att(truth={"truth_source": "perception_3d"}))
    check('真值来自控制链 → missing_data（E13 的死穴）',
          r["status"] == "missing_data"
          and any('白名单' in x for x in r["reasons"]), str(r["reasons"]))

    r = sc.score_attempt(att(truth={"truth_source": None}))
    check('真值没写来源 → missing_data', r["status"] == "missing_data")

    r = sc.score_attempt(att(grasp={"lifted": False,
                                    "failure": {"class": "execution"}}))
    check('没抬起来 → grasp False（rc=0 也救不了）', r["grasp_ok"] is False)

    r = sc.score_attempt(att(grasp={"exec_rc": 1,
                                    "failure": {"class": "execution"}}))
    check('抬起来了但 rc≠0 → 仍算失败', r["grasp_ok"] is False)

    r = sc.score_attempt(att(grasp={"wrong_block": True,
                                    "failure": {"class": "perception"}}))
    check('抓错块 → 失败', r["grasp_ok"] is False)

    r = sc.score_attempt(att(grasp={"dropped": True,
                                    "failure": {"class": "execution"}}))
    check('中途掉了 → 失败', r["grasp_ok"] is False)

    r = sc.score_attempt(att(grasp={"confirm": {"wrist": False}}))
    check('三路确认缺腕部 → person_review，不算成功也不算失败',
          r["status"] == "person_review" and r["grasp_ok"] is None,
          str(r))

    r = sc.score_attempt(att(place={"over_line": True},
                             failure={"class": "placement"}))
    check('压线 → 放置不算中', r["place_ok"] is False and r["status"] == "fail")

    r = sc.score_attempt(att(void={"reason": "camera_fault"}))
    check('void → 状态 void 且不进任何统计',
          r["status"] == "void" and r["id_ok"] is None
          and r["grasp_ok"] is None)


# ------------------------------------------------------------------ 轮级
def section_round():
    print('\n[2] 轮级：void 不进分母、四条率门、零容忍、人眼复核')

    res = sc.score_round(obs([att(), att()]))
    check('两块全绿 → ok / 100%', res["verdict"] == "ok"
          and res["rates"]["id_rate"] == 1.0, str(res["reason"]))

    bad = [att(selected={"instance_id": 1}, failure={"class": "candidate"})
           for _ in range(2)]
    res = sc.score_round(obs([att() for _ in range(8)] + bad))
    check('10 块错 2 块 → 指定 ID 80% 判红',
          res["verdict"] == "fail" and '指定 ID' in res["reason"],
          str(res["reason"]))

    res = sc.score_round(obs([att() for _ in range(9)] + bad[:1]))
    check('9/10 = 90% 恰好在门上 → 过', res["verdict"] == "ok",
          str(res["reason"]))

    # 抓取门 80%：10 块里 8 块抓成。这里**只让抓取轴变**，放置仍记 in_slot ——
    # 单测要隔离这条门，不能被放置率一起带红。
    grasped = [att() for _ in range(8)] + [
        att(grasp={"lifted": False}, failure={"class": "execution"})
        for _ in range(2)]
    res = sc.score_round(obs(grasped))
    check('抓取 8/10 = 80% 恰好过门', res["verdict"] == "ok", str(res["reason"]))
    res = sc.score_round(obs(grasped[1:]))       # 7/10 = 70%
    check('抓取 7/10 = 70% → 判红',
          res["verdict"] == "fail" and '抓取' in res["reason"], str(res["reason"]))

    # void 不进分母：8 绿 + 2 void 仍是 100%
    res = sc.score_round(obs([att() for _ in range(8)]
                             + [att(void={"reason": "occluded"})] * 2))
    check('void 不进分母（8 绿 + 2 void → 100%，不是 80%）',
          res["n_scored"] == 8 and res["n_void"] == 2
          and res["rates"]["id_rate"] == 1.0 and res["verdict"] == "ok",
          '%s %s' % (res["n_scored"], res["rates"]["id_rate"]))

    res = sc.score_round(obs([att(void={"reason": "x"})] * 2))
    check('全 void → missing_data（分母为 0，不是 100%）',
          res["verdict"] == "missing_data", str(res["reason"]))

    res = sc.score_round(obs([att() for _ in range(5)]
                             + [att(void={"reason": "x"})] * 3))
    check('连续 3 次 void → 停（fail）',
          res["verdict"] == "fail" and 'void' in res["reason"],
          str(res["reason"]))
    check('这一轮被标成 void 轮（给出口跨轮计数用）', res["void_round"] is True)

    res = sc.score_round(obs([att() for _ in range(8)]
                             + [att(void={"reason": "x"})] * 2))
    check('8 绿 + 2 void 不算 void 轮', res["void_round"] is False)

    res = sc.score_round(obs([att(void={"reason": "x"})] * 2))
    check('整轮全 void 也算 void 轮（2 块永远数不到 3 连）',
          res["void_round"] is True)

    res = sc.score_round(obs([att(), att()], safety={"estop": 1}))
    check('急停 1 次 → 判红', res["verdict"] == "fail"
          and '安全事件' in res["reason"], str(res["reason"]))

    res = sc.score_round(obs([att(), att()], interventions={"count": 1}))
    check('人工干预 1 次 → 判红', res["verdict"] == "fail"
          and '人工干预' in res["reason"], str(res["reason"]))

    res = sc.score_round(obs([att(), att()],
                             terminal={"off_table": 1}))
    check('有块掉桌 → 全桌终态率判红',
          res["verdict"] == "fail" and '终态' in res["reason"],
          str(res["reason"]))

    res = sc.score_round(obs([att(), att()],
                             terminal={"blocks": [{"id": 0, "zone": "floor",
                                                   "in_bounds": False}]}))
    check('终态有非法块 → 判红', res["verdict"] == "fail", str(res["reason"]))

    o = obs([att()])
    o["terminal"] = {}                       # 用「删掉」而不是 merge 空 dict
    res = sc.score_round(o)
    check('终态缺失 → missing_data（不是判失败）',
          res["verdict"] == "missing_data", str(res["reason"]))

    res = sc.score_round(obs([att(), att()], human_review={"approved": False}))
    check('人眼复核未批准 → person_review',
          res["verdict"] == "person_review", str(res["reason"]))

    o = obs([att(), att()])
    del o["human_review"]
    res = sc.score_round(o)
    check('人眼复核缺失 → person_review（不能默认绿）',
          res["verdict"] == "person_review", str(res["reason"]))

    res = sc.score_round(obs([att(selected={"auto_top1": False}) for _ in range(4)]))
    check('自主 top-1 全否 → 仍 ok，只报不设门',
          res["verdict"] == "ok" and res["rates"]["auto_top1_rate"] == 0.0,
          str(res["reason"]))

    res = sc.score_round(obs([att(), att()], n_targets=5))
    check('尝试数 ≠ 指定块数 → missing_data',
          res["verdict"] == "missing_data", str(res["reason"]))

    res = sc.score_round(obs([]))
    check('空轮 → missing_data', res["verdict"] == "missing_data")


# ------------------------------------------------------------------ 出口
def _rr(verdict="ok", th=None, ch="aaaa", rnd="1", void_round=False):
    return {"verdict": verdict,
            "thresholds": th or {"id_rate": 0.90},
            "code_hash": ch, "round": rnd, "batch": "S0",
            "layout": "L1", "reason": "", "void_round": void_round}


def section_exit():
    print('\n[3] 出口：连续 3 轮，中间改了阈值/代码就重来')

    res = sc.score_exit([_rr(rnd=str(i)) for i in (1, 2, 3)])
    check('连续 3 轮 ok → 达标', res["ok"] and res["streak"] == 3)

    res = sc.score_exit([_rr(rnd="1"), _rr(rnd="2"), _rr("fail", rnd="3")])
    check('第 3 轮红 → 连续清零', not res["ok"] and res["streak"] == 0
          and res["best"] == 2, str(res))

    res = sc.score_exit([_rr(rnd=str(i)) for i in (1, 2)])
    check('只有 2 轮 → 不达标', not res["ok"] and res["streak"] == 2)

    res = sc.score_exit([_rr(rnd=str(i)) for i in (1, 2, 3, 4)])
    check('连续 4 轮 → 仍达标（streak 4）', res["ok"] and res["streak"] == 4)

    th2 = {"id_rate": 0.85}
    res = sc.score_exit([_rr(rnd="1"), _rr(rnd="2"),
                         _rr(th=th2, rnd="3")])
    check('第 3 轮换了阈值 → 从该轮重起（streak 1，不达标）',
          not res["ok"] and res["streak"] == 1, str(res))

    res = sc.score_exit([_rr(rnd="1"), _rr(rnd="2"), _rr(ch="bbbb", rnd="3")])
    check('第 3 轮换了代码 → 同样重起',
          not res["ok"] and res["streak"] == 1, str(res))

    res = sc.score_exit([_rr("fail", rnd="1"), _rr(rnd="2"), _rr(rnd="3")])
    check('红轮之后连绿两轮 → 只有 2，不达标',
          not res["ok"] and res["streak"] == 2)

    # 契约 §4.1：同一判据连续 3 次 void ⇒ 停
    res = sc.score_exit([_rr("missing_data", rnd=str(i), void_round=True)
                         for i in (1, 2, 3)])
    check('连续 3 个 void 轮 → 停', res["stop"] and not res["ok"]
          and res["void_run"] == 3, str(res["stop_reason"]))
    res = sc.score_exit([_rr("missing_data", rnd="1", void_round=True),
                         _rr("missing_data", rnd="2", void_round=True),
                         _rr(rnd="3")])
    check('只有 2 个 void 轮 → 不停', not res["stop"] and res["void_run"] == 0)


# ------------------------------------------------------------------ 变异
def section_mutation():
    print('\n[4] 变异：把口径改坏，对应用例必须红')

    # A. void 混进分母
    a = [att() for _ in range(8)] + [att(void={"reason": "x"})] * 2
    real = sc.score_round(obs(a))
    mut = _mut('    scored = [r for r in rows if r["status"] not in '
               '(A_VOID, A_MISSING)]',
               '    scored = list(rows)')
    bad = mut["score_round"](obs(a))
    check('变异「void 进分母」：真实现 100%/ok，改坏后 80%/红',
          real["verdict"] == "ok" and real["rates"]["id_rate"] == 1.0
          and bad["rates"]["id_rate"] == 0.8 and bad["verdict"] == "fail",
          '%s → %s' % (real["rates"]["id_rate"], bad["rates"]["id_rate"]))

    # B. 真值来源闸门失效
    real = sc.score_round(obs([att(truth={"truth_source": "perception_3d"})]))
    mut = _mut('    if tsrc not in TRUTH_SOURCES:',
               '    if False:')
    bad = mut["score_round"](obs([att(truth={"truth_source": "perception_3d"})]))
    check('变异「关掉真值来源闸门」：真实判 missing_data，改坏后放行',
          real["verdict"] == "missing_data" and bad["verdict"] != "missing_data",
          '%s → %s' % (real["verdict"], bad["verdict"]))

    # C. 不看 exec_rc
    one = att(grasp={"exec_rc": 1, "failure": {"class": "execution"}})
    real = sc.score_attempt(one)
    mut = _mut('        and not bool(gr.get("wrong_block")) and '
               'int(gr.get("exec_rc", 1)) == 0',
               '        and not bool(gr.get("wrong_block"))')
    bad = mut["score_attempt"](one)
    check('变异「rc≠0 也算抬成功」：真实 grasp False，改坏后 True',
          real["grasp_ok"] is False and bad["grasp_ok"] is True,
          '%s → %s' % (real["grasp_ok"], bad["grasp_ok"]))

    # D. 人眼复核缺失默认绿
    o = obs([att(), att()])
    del o["human_review"]
    real = sc.score_round(o)
    mut = _mut('        if review.get("approved") is not True:',
               '        if False:')
    bad = mut["score_round"](o)
    check('变异「复核缺失默认绿」：真实 person_review，改坏后 ok',
          real["verdict"] == "person_review" and bad["verdict"] == "ok",
          '%s → %s' % (real["verdict"], bad["verdict"]))

    # E. 换阈值不再重起
    rounds = [_rr(rnd="1"), _rr(rnd="2"), _rr(th={"id_rate": 0.85}, rnd="3")]
    real = sc.score_exit(rounds)
    mut = _mut('        same = (start is None', '        same = (True')
    bad = mut["score_exit"](rounds)
    check('变异「换阈值不重起」：真实不达标，改坏后达标',
          not real["ok"] and bad["ok"], '%s → %s' % (real, bad))


def main():
    print('=' * 68)
    section_const()
    section_attempt()
    section_round()
    section_exit()
    section_mutation()
    print('\n%d PASS  %d FAIL' % (N_PASS, N_FAIL))
    if FAILS:
        print('未通过：' + '; '.join(FAILS))
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
