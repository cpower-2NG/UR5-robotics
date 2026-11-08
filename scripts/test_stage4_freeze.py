#!/usr/bin/env python3
"""E20 冻结工具的纯函数自检：**不需要 GPU、权重、第三方仓库**。

钉住三件事（每件都对应一次真实会发生的误判）：

1. 哈希对内容敏感 —— 改一个字节必须变；同名不同内容不能算「一致」；
2. `verify` 的漂移检测真的会红 —— 少一个权重、换一个版本、HEAD 变了都必须报；
3. `compare` 忽略耗时/显存但**不忽略数值** —— 把某个抓取分数改 1e-3 必须红，
   否则「同输入同输出」就成了一句口号。

跑法：``python3 scripts/test_stage4_freeze.py``；退出码 0 = 全通过。
"""
import json
import os
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import stage4_freeze as fz                                        # noqa: E402

N_PASS = N_FAIL = 0
FAILS = []


def check(name, cond, detail=''):
    global N_PASS, N_FAIL
    if cond:
        N_PASS += 1
        print("  ok   %s" % name)
    else:
        N_FAIL += 1
        FAILS.append(name)
        print("  FAIL %s %s" % (name, detail))


def test_hash():
    print("[哈希]")
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "w.bin")
        open(p, "wb").write(b"abc")
        h1 = fz._sha256(p)
        check("已知内容 sha256", h1 ==
              "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")
        open(p, "wb").write(b"abd")
        check("改一字节哈希必变", fz._sha256(p) != h1)


def test_diff_manifest():
    print("[清单漂移]")
    base = {"weights": {"yolo": {"path": "~/a.pt", "exists": True, "sha256": "x"}},
            "env": {"graspnet": {"torch": "2.1.2"}},
            "created": "t1"}
    same = json.loads(json.dumps(base))
    same["created"] = "t2"
    check("只有 created 不同 ⇒ 不算漂移", fz.diff_manifest(base, same) == [])

    gone = json.loads(json.dumps(base))
    gone["weights"]["yolo"]["exists"] = False
    check("权重消失能检出", len(fz.diff_manifest(base, gone)) > 0)

    newsha = json.loads(json.dumps(base))
    newsha["weights"]["yolo"]["sha256"] = "y"
    check("权重哈希变了能检出", len(fz.diff_manifest(base, newsha)) > 0)

    vup = json.loads(json.dumps(base))
    vup["env"]["graspnet"]["torch"] = "2.5.0"
    check("torch 版本变了能检出", len(fz.diff_manifest(base, vup)) > 0)


def test_compare():
    print("[同输入同输出]")
    a = {"blocks": [{"idx": 0, "score": 0.3141592,
                     "xyz": [0.4, -0.2, 0.03]}],
         "infer_ms": [101.2], "vram_peak_mb": 208.5, "frame": "f1"}
    b = json.loads(json.dumps(a))
    b["infer_ms"] = [97.8]
    b["vram_peak_mb"] = 210.0
    check("耗时/显存不同不算不一致", fz.compare_outputs(a, b) == [])

    c = json.loads(json.dumps(a))
    c["blocks"][0]["score"] = 0.3151592      # +1e-3
    diffs = fz.compare_outputs(a, c)
    check("抓取分数 +1e-3 必红", len(diffs) == 1)
    check("差值的路径指向 score",
          diffs and diffs[0].startswith("/blocks[0]/score"))

    d = json.loads(json.dumps(a))
    d["blocks"].append({"idx": 1})
    check("候选数变了必红", len(fz.compare_outputs(a, d)) > 0)

    e = json.loads(json.dumps(a))
    e["blocks"][0]["xyz"][2] = 0.0300001     # 1e-7，容差内
    check("容差内的浮点抖动放行", fz.compare_outputs(a, e) == [])


def test_manifest_shape():
    print("[清单结构]")
    m = {"weights": {k: {"path": p} for k, p in fz.WEIGHTS},
         "env": {k: {} for k, _ in fz.VENVS},
         "third_party": {k: {} for k, _ in fz.THIRD_PARTY}}
    check("三类产物都有条目",
          all(m[k] for k in ("weights", "env", "third_party")))
    check("权重路径都是 ~/ 开头（不写绝对路径）",
          all(v["path"].startswith("~/") for v in m["weights"].values()))


def main():
    test_hash()
    test_diff_manifest()
    test_compare()
    test_manifest_shape()
    print("\n%d 项通过，%d 项失败" % (N_PASS, N_FAIL))
    if FAILS:
        print("失败：" + ", ".join(FAILS))
    return 0 if N_FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
