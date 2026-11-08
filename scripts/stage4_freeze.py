#!/usr/bin/env python3
"""E20 模型/环境冻结 —— 入口判据的两个半边。

判据（契约 §3）：「权重/配置哈希记录；同输入同输出」。分三个子命令：

  emit    记录一批产物的身份（sha256 + 大小）、推理环境版本、第三方仓库 HEAD
  verify  重算并与清单逐项比对；任何一项对不上 ⇒ 退出 1（fail-closed）
  compare 比较同一输入两次推理的输出；忽略耗时/显存这类本来就该变的字段

为什么不是只写一份 Markdown：清单要能被机器复核。仓库里
`docs/archive/ATTACHMENT_MANIFEST.md` 是**教学附件**的静态清单，本文件管的是
**推理链**（权重 + 环境 + 第三方代码），每次换权重/升 torch 都必须重出。

用法：
  python3 scripts/stage4_freeze.py emit --out runs/<run>/e20/freeze-manifest.json
  python3 scripts/stage4_freeze.py verify --manifest runs/<run>/e20/freeze-manifest.json
  python3 scripts/stage4_freeze.py compare --a a.json --b b.json

退出码：0 一致 / 1 漂移 / 2 用法或文件问题。
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
from datetime import datetime

HOME = os.path.expanduser("~")
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 推理链上「变了就必须重出清单」的东西。路径写在本文件里，不散落。
WEIGHTS = [
    ("yolo/detector", "~/robotics/models/block_best_7cls.pt"),
    ("graspnet/baseline", "~/robotics/third_party/graspnet-baseline/logs/checkpoint-rs.tar"),
    ("rngnet/realsense", "~/robotics/third_party/rngnet/checkpoints/realsense"),
]
THIRD_PARTY = [
    ("graspness_unofficial", "~/robotics/third_party/graspness_unofficial"),
    ("graspnet-baseline", "~/robotics/third_party/graspnet-baseline"),
    ("rngnet", "~/robotics/third_party/rngnet"),
]
VENVS = [
    ("graspnet", "~/robotics/venvs/graspnet/bin/python"),
    ("rngnet", "~/robotics/venvs/rngnet/bin/python"),
]
# 推理链上属于**本仓**的代码：改一行就可能改数值，git 提交号盖不住未提交的改动
INFERENCE_CODE = [
    "graspnet_candidates.py",
    "e6_graspnet_crop.py",
    "rngnet_offline.py",
    "e6_compare.py",
]
# 环境版本只记这几个：换 torch/CUDA 会直接改数值，其余包偶尔才动
ENV_PKGS = ("torch", "torchvision", "numpy", "MinkowskiEngine", "pointnet2")
# 同输入两次输出里**允许变**的字段（耗时与资源统计）
VOLATILE_KEYS = {
    "infer_ms", "infer_sec", "vram_peak_mb", "rss_peak_mb", "model_load_s",
    "load_s", "elapsed_s", "created", "timestamp", "duration_ms",
}


def _sha256(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for blk in iter(lambda: f.read(chunk), b""):
            h.update(blk)
    return h.hexdigest()


def _expand(p):
    return p if os.path.isabs(p) else os.path.expanduser(p)


def _rel(p):
    """清单里的路径一律相对家目录，公开仓里不出现绝对路径。"""
    p = os.path.abspath(_expand(p))
    if p.startswith(HOME + os.sep):
        return "~/" + os.path.relpath(p, HOME)
    return p


def _hash_item(path):
    ap = _expand(path)
    if not os.path.exists(ap):
        return {"path": _rel(ap), "exists": False}
    rec = {"path": _rel(ap), "exists": True,
           "size": os.path.getsize(ap), "sha256": _sha256(ap)}
    if os.path.islink(ap):
        rec["symlink_to"] = _rel(os.path.realpath(ap))
    return rec


def _git_head(repo):
    """worktree 的 .git 是**文件**不是目录，所以判 exists 不判 isdir。"""
    ap = _expand(repo)
    if not os.path.exists(os.path.join(ap, ".git")):
        return None
    try:
        out = subprocess.run(["git", "-C", ap, "rev-parse", "HEAD"],
                             capture_output=True, text=True, timeout=10)
        return out.stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def _tree_hash(root, exts=(".py",)):
    """非 git 副本（如 rngnet 是拷过来的）用代码树哈希认身份。

    只哈希指定扩展名的文件；顺序按相对路径排，保证可复现。
    """
    ap = _expand(root)
    if not os.path.isdir(ap):
        return None
    h = hashlib.sha256()
    n = 0
    for dirpath, dirnames, filenames in os.walk(ap):
        dirnames[:] = sorted(d for d in dirnames if d != "__pycache__")
        for fn in sorted(filenames):
            if not fn.endswith(exts):
                continue
            fp = os.path.join(dirpath, fn)
            rel = os.path.relpath(fp, ap).replace(os.sep, "/")
            h.update(rel.encode("utf-8") + b"\0")
            h.update(_sha256(fp).encode("ascii") + b"\n")
            n += 1
    return {"n_files": n, "sha256": h.hexdigest()} if n else None


def probe_env(python):
    """在目标 venv 里问版本；失败不抛，记 error（verify 时同样会复现）。"""
    code = (
        "import json,sys\n"
        "d={'python':sys.version.split()[0]}\n"
        "for m in %r:\n"
        "    try:\n"
        "        mod=__import__(m)\n"
        "        d[m]=str(getattr(mod,'__version__','?'))\n"
        "    except Exception as e:\n"
        "        d[m]=None\n"
        "try:\n"
        "    import torch; d['cuda']=torch.version.cuda; d['cuda_avail']=torch.cuda.is_available()\n"
        "except Exception: pass\n"
        "print(json.dumps(d))\n" % (list(ENV_PKGS),)
    )
    try:
        r = subprocess.run([_expand(python), "-c", code],
                           capture_output=True, text=True, timeout=180)
        if r.returncode != 0:
            return {"error": (r.stderr or "").strip()[-300:]}
        return json.loads(r.stdout.strip().splitlines()[-1])
    except (OSError, subprocess.SubprocessError, ValueError) as e:
        return {"error": str(e)[:300]}


def build_manifest(in_repo_configs=True):
    m = {
        "kind": "stage4-freeze-manifest",
        "version": 1,
        "created": datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "repo_commit": _git_head(REPO_ROOT),
        "weights": {name: _hash_item(p) for name, p in WEIGHTS},
        "third_party": {},
        "env": {name: probe_env(py) for name, py in VENVS},
        "system_python": probe_env(sys.executable),
    }
    for name, p in THIRD_PARTY:
        rec = {"git_head": _git_head(p)}
        if rec["git_head"] is None:          # 不是 git 副本 ⇒ 退回代码树哈希
            rec["tree"] = _tree_hash(p)
        m["third_party"][name] = rec
    if in_repo_configs:
        m["inference_code"] = {
            fn: _hash_item(os.path.join(REPO_ROOT, "scripts", fn))
            for fn in INFERENCE_CODE}
        cfg = {}
        cdir = os.path.join(REPO_ROOT, "config")
        if os.path.isdir(cdir):
            for fn in sorted(os.listdir(cdir)):
                if fn.endswith((".yaml", ".yml")):
                    cfg[fn] = _hash_item(os.path.join(cdir, fn))
        m["configs"] = cfg
    return m


def diff_manifest(old, new):
    """只比「应该稳定」的字段；created 这种自然不算。"""
    out = []

    def walk(a, b, path):
        if isinstance(a, dict) and isinstance(b, dict):
            for k in sorted(set(a) | set(b)):
                if k in ("created",):
                    continue
                if k not in a:
                    out.append("%s: 新增 %r" % (path + "/" + k, b[k]))
                elif k not in b:
                    out.append("%s: 消失" % (path + "/" + k))
                else:
                    walk(a[k], b[k], path + "/" + k)
            return
        if a != b:
            out.append("%s: %r → %r" % (path, a, b))

    walk(old, new, "")
    return out


def compare_outputs(a, b, tol=1e-6, path="", out=None):
    """同输入两次推理的数值比对。容差默认 1e-6（fp32 累加顺序不同也不会超）。"""
    if out is None:
        out = []
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            if k in VOLATILE_KEYS:
                continue
            if k not in a or k not in b:
                out.append("%s/%s: 只在一侧出现" % (path, k))
            else:
                compare_outputs(a[k], b[k], tol, path + "/" + k, out)
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            out.append("%s: 长度 %d ≠ %d" % (path, len(a), len(b)))
            return out
        for i, (x, y) in enumerate(zip(a, b)):
            compare_outputs(x, y, tol, "%s[%d]" % (path, i), out)
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)) \
            and not isinstance(a, bool) and not isinstance(b, bool):
        if abs(float(a) - float(b)) > tol:
            out.append("%s: %r ≠ %r（差 %.3g）" % (path, a, b, abs(a - b)))
    elif a != b:
        out.append("%s: %r ≠ %r" % (path, a, b))
    return out


def cmd_emit(args):
    m = build_manifest()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(m, f, ensure_ascii=False, indent=1)
    missing = [k for k, v in m["weights"].items() if not v.get("exists")]
    print("清单已写：%s" % args.out)
    print("  权重 %d 项%s" % (len(m["weights"]),
                           ("（缺：%s）" % ", ".join(missing)) if missing else "，全在"))
    print("  环境 %s" % ", ".join(sorted(m["env"])))
    return 1 if missing else 0


def cmd_verify(args):
    with open(args.manifest, encoding="utf-8") as f:
        old = json.load(f)
    new = build_manifest()
    d = diff_manifest(old, new)
    if not d:
        print("清单一致：权重 / 环境 / 第三方 HEAD / 配置 全部未变。")
        return 0
    print("!! 清单漂移 %d 处：" % len(d))
    for line in d[:50]:
        print("  " + line)
    return 1


def cmd_compare(args):
    with open(args.a, encoding="utf-8") as f:
        a = json.load(f)
    with open(args.b, encoding="utf-8") as f:
        b = json.load(f)
    d = compare_outputs(a, b, tol=args.tol)
    if not d:
        print("同输入同输出：两份结果在 %d 项数值上一致（容差 %g）。" %
              (_count_numbers(a), args.tol))
        return 0
    print("!! 两次输出不同 %d 处：" % len(d))
    for line in d[:50]:
        print("  " + line)
    return 1


def _count_numbers(x):
    if isinstance(x, dict):
        return sum(_count_numbers(v) for v in x.values())
    if isinstance(x, list):
        return sum(_count_numbers(v) for v in x)
    if isinstance(x, (int, float)) and not isinstance(x, bool):
        return 1
    return 0


def main():
    ap = argparse.ArgumentParser(description="E20 模型/环境冻结")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("emit", help="生成冻结清单")
    p.add_argument("--out", required=True)
    p.set_defaults(func=cmd_emit)
    p = sub.add_parser("verify", help="按清单复核，漂移即退出 1")
    p.add_argument("--manifest", required=True)
    p.set_defaults(func=cmd_verify)
    p = sub.add_parser("compare", help="同输入两次推理输出比对")
    p.add_argument("--a", required=True)
    p.add_argument("--b", required=True)
    p.add_argument("--tol", type=float, default=1e-6)
    p.set_defaults(func=cmd_compare)
    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
