#!/usr/bin/env python3
"""板上一次性分析入口：**解垫 / 真值 / 彩色质心**，只回传 KB 级 JSON。

为什么要有它（2026-11-12 用户定调）
──────────────────────────────────
「本机不能承担运行时任务」。阶段 4 一轮观察段 193.6 s 里 ~125 s 花在把 44 MB
burst 拉回开发机再算（轮内只有 0.33–1.4 MB/s），而板子 6 核、cv2 4.5.4（含
aruco）/ numpy / PIL 齐全 —— 搬运是架构遗留，不是算力约束。所以分析段一律
在板上跑，轮内只回传 JSON；图像本体在整轮结束后的归档段再拉。

这个脚本不含任何运动 / ROS / 夹爪代码，纯函数封装：

    pad     burst → 垫位姿（``stage4_pad_from_burst``；需 ``--mat``）
    truth   burst → 块真值（``stage4_truth_from_frame --consensus``）
    colour  burst → 目标块彩色顶面质心（窗 25 mm，``measure_target_colour``）

板端跑法（``~/precision/run.sh`` 就是 ``exec python3 "$@"``）::

    ~/precision/run.sh scripts/stage4_board_analysis.py pad \
        --burst <burst> --mat <stage2-mat.json> --out <grid.json>

退出码：0 成功；2 参数错；其余透传被封装脚本的退出码。
``colour`` 只要产出 JSON 就返回 0（``ok=false`` 是**记录**，不是失败 ——
判分器按 ``ok`` 判，不按 rc）。
"""

import argparse
import json
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# 线程钉必须在 import numpy / cv2 **之前**（2026-11-12 板上实测：6 个 worker
# 各自开 BLAS 多线程 ⇒ 36 线程抢 6 核，同一批 6 帧从 3.6 s 恶化到 22.4 s）。
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import stage4_pad_from_burst as pfb                               # noqa: E402
import stage4_truth_from_frame as tf                              # noqa: E402

EXIT_BAD_ARGS = 2


def cmd_pad(a):
    argv = [a.burst, "--out", a.out, "--mat", a.mat,
            "--camera", a.camera,
            "--min-frames", str(int(a.min_frames)),
            "--max-spread-mm", str(float(a.max_spread_mm)),
            "--max-spread-deg", str(float(a.max_spread_deg))]
    if a.pad_spec:
        # C1 后置：grid 文档自洽（嵌足迹/厚度真值），下游新进程不再缺参
        argv += ["--pad-spec", a.pad_spec]
    return int(pfb.main(argv) or 0)


def cmd_truth(a):
    argv = ["--frame", a.burst, "--consensus",
            "--min-frames", str(int(a.min_frames)),
            "--camera", a.camera, "--seg", a.seg, "--out", a.out]
    if a.zone:
        argv += ["--zone", a.zone]
    if a.allow_empty:
        argv.append("--allow-empty")
    if a.jobs is not None:
        argv += ["--jobs", str(int(a.jobs))]
    return int(tf.main(argv) or 0)


def cmd_colour(a):
    # ``stage4_clear_run`` 拖进 cv2/aruco/gen_scene（还要求脚本位于整仓里），
    # 只有 colour 用得上 ⇒ 到这里才 import。2026-11-12 板上实测：放在模块级会让
    # truth/pad 在只拷了 scripts/ 的目录里直接 ModuleNotFoundError: gen_scene。
    import stage4_clear_run as cr
    if a.camera not in cr.COLOUR_CAMERAS:
        print("--camera 只认 %s" % (sorted(cr.COLOUR_CAMERAS),), file=sys.stderr)
        return EXIT_BAD_ARGS
    try:
        x_mm, y_mm = [float(v) for v in a.xy.split(",")]
    except ValueError:
        print("--xy 要写成 \"x,y\"（毫米）", file=sys.stderr)
        return EXIT_BAD_ARGS
    hue = None if a.hue is None else float(a.hue)
    doc = cr.measure_target_colour(
        a.burst, (x_mm / 1000.0, y_mm / 1000.0), float(a.top_z),
        search_mm=float(a.search_mm), camera=a.camera,
        expect_hue_deg=hue, expect_class=a.expect_class)
    doc["search_mm"] = float(a.search_mm)
    doc["top_z"] = float(a.top_z)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)) or ".", exist_ok=True)
    with open(a.out, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=1)
    print("colour %s ok=%s xy=%s 帧 %s/%s"
          % (a.burst, doc.get("ok"), doc.get("xy_mm"),
             doc.get("n_frames"), (doc.get("n_frames", 0)
                                  + doc.get("n_missed", 0))))
    return 0


def build_parser():
    ap = argparse.ArgumentParser(description="板上分析入口（解垫 / 真值 / 彩色）")
    sub = ap.add_subparsers(dest="kind", required=True)

    p = sub.add_parser("pad", help="burst → 垫位姿")
    p.add_argument("--burst", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--mat", required=True,
                   help="2D 平面标定 JSON（stage2-mat.json；板端路径由编排器推）")
    p.add_argument("--camera", choices=("wrist", "topcam"), default="wrist")
    p.add_argument("--min-frames", type=int, default=20)
    p.add_argument("--max-spread-mm", type=float, default=2.0)
    p.add_argument("--max-spread-deg", type=float, default=0.5)
    p.add_argument("--pad-spec", default=None,
                   help="pad-spec.json（足迹/厚度真值）：给了就嵌进 grid 文档，"
                        "使其自洽（探针/判分新进程 load_grid 即全量）")
    p.set_defaults(func=cmd_pad)

    t = sub.add_parser("truth", help="burst → 块真值（20 帧共识）")
    t.add_argument("--burst", required=True)
    t.add_argument("--out", required=True)
    t.add_argument("--camera", choices=sorted(tf.CAMERAS), default="wrist")
    t.add_argument("--seg", choices=("grid", "sam"), default="sam",
                   help="分割/身份感知链（第 345 轮 SAM 立项、347 轮回放"
                        "达标后缺省 sam；grid=网格链回退/对照）")
    t.add_argument("--zone", default=None, metavar="X0,X1,Y0,Y1",
                   help="真值保留区（米）；区外的「块」按幻影丢")
    t.add_argument("--min-frames", type=int, default=5)
    t.add_argument("--allow-empty", action="store_true",
                   help="干净的机位是合法结果（只给跑后三机位）")
    t.add_argument("--jobs", type=int, default=None,
                   help="逐帧量测进程数（缺省 min(CPU, 帧数)）")
    t.set_defaults(func=cmd_truth)

    c = sub.add_parser("colour", help="burst → 目标块彩色顶面质心")
    c.add_argument("--burst", required=True)
    c.add_argument("--out", required=True)
    c.add_argument("--xy", required=True, help="期望格心 \"x,y\"（毫米）")
    c.add_argument("--top-z", type=float, required=True,
                   help="顶面高于 base 的高度（米）")
    c.add_argument("--search-mm", type=float, default=25.0)
    # 相机名不在 argparse 里取 choices：那要在 build_parser 阶段 import
    # ``stage4_clear_run``（拖 cv2/aruco/gen_scene），而 truth/pad 用不上它。
    # 改在 cmd_colour 里对着 ``COLOUR_CAMERAS`` 校验，写错仍是参数错（rc 2）。
    c.add_argument("--camera", default="wrist",
                   help="相机名（wrist/topcam，以 stage4_clear_run.COLOUR_CAMERAS 为准）")
    c.add_argument("--hue", type=float, default=None,
                   help="期望色相（只交叉核对，不改 ok）")
    c.add_argument("--expect-class", default=None)
    c.set_defaults(func=cmd_colour)
    return ap


def main(argv=None):
    ap = build_parser()
    a = ap.parse_args(argv)
    return int(a.func(a) or 0)


if __name__ == "__main__":
    sys.exit(main())
