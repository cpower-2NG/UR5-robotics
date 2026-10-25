#!/usr/bin/env python3
"""`capture_baseline.py` 的自检：**只测纯函数**（不碰 ROS、不连相机）。

为什么单列
==========
基线轮的每一帧都是「事后才用得上」的证据：当时看着没问题、几周后复算发现
少了 TF 或者时间戳按错了字段 —— 那批数据就废了，而且**不会报错**。所以这里
把能离线钉住的东西全钉住：目录名的边界、manifest 追加不毁旧记录、消息里的
下标与字段名。

⚠️ 真正的「拍一帧」只能现场验（话题/相机/臂都在板上）。本文件**不**假装验过它。
"""

import json
import os
import shutil
import sys
import tempfile
from types import SimpleNamespace

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import capture_baseline as cb                               # noqa: E402

_failed = []


def check(name, cond, detail=''):
    print("  [%s] %s%s" % ("PASS" if cond else "FAIL", name,
                           ("  " + detail) if detail else ""))
    if not cond:
        _failed.append(name)


def section(t):
    print("\n" + t)


def _stamp(sec, nsec=0):
    return SimpleNamespace(sec=sec, nanosec=nsec)


def _header(frame="base_link", sec=100, nsec=5):
    return SimpleNamespace(frame_id=frame, stamp=_stamp(sec, nsec))


section("[1] 时间戳：只按 ROS 的 sec/nanosec 拼，不猜别的字段")
ns = cb.stamp_ns(_stamp(7, 500_000_000))
check("7 s + 0.5 s ⇒ 7.5e9", ns == 7_500_000_000, str(ns))
check("None ⇒ None（缺 stamp 不是 0 —— 0 会被当成「1970 年拍的那一帧」）",
      cb.stamp_ns(None) is None)

section("[2] 目录名：tag 不许跑出帧目录")
check("正常 tag 原样通过", cb.safe_tag("L1-P1") == "L1-P1")
for bad in ("", "a/b", "a\\b", "..", "."):
    try:
        cb.safe_tag(bad)
        ok = False
    except ValueError:
        ok = True
    check("拒绝 %r" % bad, ok)

section("[3] 消息 → JSON：字段名与下标")
ci = SimpleNamespace(width=640, height=480, distortion_model="plumb_bob",
                     d=[0.1, 0.2, 0.0, 0.0, 0.0],
                     k=[600.0, 0, 320.0, 0, 601.0, 240.0, 0, 0, 1],
                     r=[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0],
                     p=[600.0, 0, 320.0, 0, 0, 601.0, 240.0, 0, 0, 0, 1.0, 0],
                     header=_header("camera_color_optical_frame"))
d = cb.camera_info_dict(ci)
check("K 原样保留 9 个（不重排成 fx/fy/…）", d["K"] == [float(x) for x in ci.k])
check("cx 就在 K[2]（评估端按这个下标取）", d["K"][2] == 320.0)
check("D 5 项", len(d["D"]) == 5)
check("frame_id 带过来", d["frame_id"] == "camera_color_optical_frame")

img = SimpleNamespace(width=640, height=480, encoding="16UC1",
                      is_bigendian=0, step=1280, header=_header(
                          "camera_color_optical_frame"))
im = cb.image_meta(img)
check("深度编码原样记（16UC1 与 32FC1 的数值含义不同，不能糊成一样）",
      im["encoding"] == "16UC1")
check("step 也记（行对齐不总是 width×bpp）", im["step"] == 1280)

section("[4] 检测/投影：分数取最大的那条假设，类别从它取")
det = SimpleNamespace(
    bbox=SimpleNamespace(center=SimpleNamespace(
        position=SimpleNamespace(x=10.0, y=20.0), theta=0.1),
        size_x=30.0, size_y=31.0),
    results=[SimpleNamespace(score=0.4, id="1"),
             SimpleNamespace(score=0.9, id="3")])
d2 = cb.detections_dict(SimpleNamespace(header=_header(), detections=[det]))
check("两个框只出一条（这里给的是一个检测）", len(d2["detections"]) == 1)
best = d2["detections"][0]["best"]
check("取分数最大的假设 0.9 / 类别 3", best["score"] == 0.9
      and best["class_id"] == "3", str(best))
check("bbox 中心与尺寸都在", d2["detections"][0]["size_x"] == 30.0
      and d2["detections"][0]["cx"] == 10.0)

det3 = SimpleNamespace(
    bbox=SimpleNamespace(
        center=SimpleNamespace(
            position=SimpleNamespace(x=0.1, y=0.2, z=0.3),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)),
        size=SimpleNamespace(x=0.0285, y=0.0285, z=0.0285)),
    results=[SimpleNamespace(score=0.7, id="blk")])
d3 = cb.targets_dict(SimpleNamespace(header=_header(), detections=[det3]))
t = d3["targets"][0]
check("3D 位置/四元数/尺寸都在",
      (t["x"], t["y"], t["z"]) == (0.1, 0.2, 0.3)
      and t["qw"] == 1.0 and t["size_z"] == 0.0285)

section("[5] 预检：缺哪条话题要说得出是哪条")
ok, missing = cb.judge_topics({"/a": 1, "/b": 0}, ["/a", "/b", "/c"])
check("缺 /b 与 /c（/b 有键但 0 个发布者也算缺）",
      (not ok) and missing == ["/b", "/c"], str(missing))
ok, missing = cb.judge_topics({"/a": 2}, ["/a"])
check("都有 ⇒ 过", ok and missing == [])

section("[6] manifest：追加，不毁旧记录；坏文件拒绝写")
tmp = tempfile.mkdtemp(prefix="capbase_test_")
try:
    mp = os.path.join(tmp, "manifest.json")
    cb.write_manifest(mp, {"tag": "a"})
    cb.write_manifest(mp, {"tag": "b"})
    with open(mp, encoding="utf-8") as fh:
        rows = json.load(fh)
    check("两条都在、顺序保持", [r["tag"] for r in rows] == ["a", "b"])
    check("没有留 .tmp 残骸", not os.path.exists(mp + ".tmp"))

    with open(mp, "w", encoding="utf-8") as fh:
        fh.write('{"not": "a list"}')
    try:
        cb.write_manifest(mp, {"tag": "c"})
        raised = False
    except ValueError:
        raised = True
    check("manifest 不是数组 ⇒ 抛（不许把别人的记录覆盖掉）", raised)
    with open(mp, encoding="utf-8") as fh:
        check("抛了以后原文件**一个字节没动**",
              json.load(fh) == {"not": "a list"})

    section("[7] 站点外参：抄下来，读不到就如实记 None")
    good = os.path.join(tmp, "site.yaml")
    with open(good, "w", encoding="utf-8") as fh:
        fh.write("robot_ip: 0.0.0.0\ntopcam:\n  extrinsics:\n"
                 "    camera_x: 0.1\n    camera_qw: 1.0\n")
    got = cb.load_site_extrinsics(good)
    check("读到 topcam.extrinsics", got.get("topcam_extrinsics", {}).get(
        "camera_x") == 0.1, str(got))
    got = cb.load_site_extrinsics(os.path.join(tmp, "nope.yaml"))
    check("文件不在 ⇒ 记 error（**不**编一份默认外参）",
          "error" in got and "topcam_extrinsics" not in got, str(got))
finally:
    shutil.rmtree(tmp, ignore_errors=True)

section("[8] 树身份：取得到就带分支/SHA，取不到不抛")
ti = cb.tree_info()
check("有 path", ti.get("path", "").endswith("next-precision")
      or bool(ti.get("path")), str(ti))
check("在 git 树里时 sha 是 40 位十六进制",
      ti.get("sha") is None or (len(ti["sha"]) == 40), str(ti.get("sha")))

print("\n" + "=" * 66)
if _failed:
    print("❌ %d 项没过：" % len(_failed))
    for n in _failed:
        print("   - " + n)
    sys.exit(1)
print("✅ 全部通过（%s）" % os.path.basename(__file__))
sys.exit(0)
