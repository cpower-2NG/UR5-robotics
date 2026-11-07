#!/usr/bin/env python3
"""把仿真真值（scene_truth.json）写成 RNGNet/GraspNet 都能吃的 candidates.json。

为什么用真值做「指定实例」
------------------------
E6/O2 要比的是**候选生成**，不是实例分割（那是 E5/O1）。离线层按设计文档
§11 用「固定帧集 + 仿真真值」：指定块的身份、中心、yaw 都取 gz 真值，
两网拿到的输入因此完全一致；分割误差不混进这一层。

输出的 blocks 字段与 ``graspnet_candidates.py`` 的实测块表**同名字段**：
idx / class / cx / cy（mm，base 系）/ top（mm）/ yaw（deg）/ uv（全分辨率像素），
另加 ``truth`` 标记。per_block / chosen 置空（本工具只做指定，不做锚定）。

用法：
  python3 scripts/e6_truth_blocks.py FRAME_DIR
  （FRAME_DIR 里要有 truth.json、meta.json、tf.json）
"""
import json
import os
import sys

import numpy as np


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    frame = sys.argv[1]
    truth = json.load(open(os.path.join(frame, "truth.json")))
    meta = json.load(open(os.path.join(frame, "meta.json")))
    tf = json.load(open(os.path.join(frame, "tf.json")))
    K = np.array(meta["camera_info"]["K"], float).reshape(3, 3)

    t = tf["ok"]["base_link:camera_color_optical_frame"]
    r = t["rotation"]
    R_bc = quat_to_R([r["x"], r["y"], r["z"], r["w"]])
    t_bc = np.array([t["translation"][k] for k in "xyz"])

    blocks = []
    for b in truth["blocks"]:
        p = np.array([float(b["x"]), float(b["y"]), float(b["top"])])
        pc = R_bc.T @ (p - t_bc)
        uv = K @ pc
        u, v = (float(uv[0] / uv[2]), float(uv[1] / uv[2])) if uv[2] > 1e-6 \
            else (0.0, 0.0)
        blocks.append({
            "idx": int(b["id"]), "class": b["color"],
            "cx": round(float(b["x"]) * 1000, 3),
            "cy": round(float(b["y"]) * 1000, 3),
            "top": round(float(b["top"]) * 1000, 3),
            "yaw": round(float(b["yaw"]), 3),
            "uv": [round(u, 1), round(v, 1)],
            "truth": True,
        })
    out = {"frame": frame, "table_z_mm": 0.0, "blocks": blocks,
           "per_block": [], "chosen": [], "source": "scene_truth"}
    json.dump(out, open(os.path.join(frame, "candidates.json"), "w"),
              ensure_ascii=False, indent=1)
    print("写出 %s/candidates.json：%d 块（真值指定）" % (frame, len(blocks)))
    for b in blocks:
        print("  #%d %-11s uv=(%6.1f,%6.1f) base=(%7.1f,%7.1f) top=%5.1f yaw=%6.1f"
              % (b["idx"], b["class"], b["uv"][0], b["uv"][1],
                 b["cx"], b["cy"], b["top"], b["yaw"]))


if __name__ == "__main__":
    main()
