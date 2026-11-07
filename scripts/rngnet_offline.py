#!/usr/bin/env python3
"""RNGNet 离线对账 runner（只读帧，不发任何指令、不碰真机）。

对每一帧目录（须含 candidates.json / color.png / depth.png / meta.json / tf.json）：
  1. 读 meta.json 的 K，patch 进 rngnet 的 dataset.config（我们的 D435i 腕部内参）；
  2. 用 candidates.json 里量块出的块心 uv 作 patch 中心（每块 3×3 邻域，替代
     官方 demo 的鼠标点击——等价于论文 Table 11 的 foreground 定位消融）；
  3. 跑 RNGNet → 6-DoF 候选（相机系）→ 转 base 系 → 与 graspnet-baseline 的
     同帧候选同口径对比：锚定数（≤12 mm）、离块心距离、贴块边差角（mod 90°）。
输出：runs/2026-10-27-stage2/rngnet-offline/<帧名>.json + <帧名>.png 叠加图。

用法（rngnet venv；2026-11-07 从 ~/robotics/scratch 搬进仓内，原位留桩）：
  ~/robotics/venvs/rngnet/bin/python scripts/rngnet_offline.py FRAME_DIR [...] \
      --out-root RUN_ROOT/rngnet-out
"""
import argparse
import json
import os
import sys
import time
import types
from pathlib import Path

REPO = Path(os.path.expanduser("~/robotics/third_party/rngnet"))
SCRATCH = Path(os.path.expanduser("~/robotics/scratch"))  # 只有 pytorch3d 垫片还在这
SCRIPTS = Path(__file__).resolve().parent        # cam_pose 等同目录复用
OUT_ROOT = Path(os.path.expanduser("~/robotics/notes/ur5-vision-grasp/runs/"
                                   "2026-10-27-stage2/rngnet-offline"))
CKPT = REPO / "checkpoints" / "realsense"

ANCHOR_MM = 12.0          # 与 graspnet_candidates.ANCHOR_MM 同口径
EDGE_GATE_DEG = 15.0      # 与 stage2_round A 案同口径
MIN_SCORE = 0.1           # RNGNet 分数分布挤在 ~0.5–0.63（与 graspnet-baseline
                          # 的 0.20–0.31 不同刻度），对账看锚定/贴边质量，不设高门槛
CENTER_NUM = 48

sys.path.insert(0, str(REPO))
sys.path.insert(0, str(SCRATCH))    # pytorch3d 垫片
sys.path.insert(0, str(SCRIPTS))    # cam_pose 等复用

# ---- cupoch 桩：evaluation 顶层 import collision_detector，推理闭包不需要碰撞检测
_stub = types.ModuleType("dataset.collision_detector")


class _NoCollision:
    def __init__(self, *a, **k):
        raise RuntimeError("collision_detect 已在离线 runner 禁用（cupoch 不装）")


_stub.ModelFreeCollisionDetector = _NoCollision
sys.modules["dataset.collision_detector"] = _stub

import numpy as np  # noqa: E402
import torch  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

from dataset.config import get_camera_intrinsic as _unused_gci  # noqa: F401,E402
import dataset.config as dcfg  # noqa: E402
import dataset.evaluation as deval  # noqa: E402
import dataset.pc_dataset_tools as dpdt  # noqa: E402
import dataset.utils as dutils  # noqa: E402
from dataset.grasp import RectGraspGroup  # noqa: E402
from dataset.pc_dataset_tools import center2dtopc  # noqa: E402
from models.localgraspnet import PatchMultiGraspNet  # noqa: E402


def ang_diff_mod90(a, b):
    d = (float(a) - float(b)) % 90.0
    return min(d, 90.0 - d)


def load_K(frame_dir):
    m = json.loads((frame_dir / "meta.json").read_text())
    return np.array(m["camera_info"]["K"], float).reshape(3, 3)


def scene_points(rgb_t, depth_t, K):
    """demo.PointCloudHelper 的逐行转写，内参取自帧 meta.json。"""
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    ymap, xmap = np.meshgrid(np.arange(720), np.arange(1280))
    px = torch.from_numpy(((xmap - cx) / fx)).float().cuda()
    py = torch.from_numpy(((ymap - cy) / fy)).float().cuda()
    z = depth_t / 1000.0
    mask = depth_t > 0
    xs, ys = px * z, py * z
    pts = torch.stack([xs, ys, z], -1)[mask]
    cols = rgb_t[:, mask].T
    n = pts.shape[0]
    idx = torch.randperm(n, device=pts.device)[:25600]
    return torch.cat([pts[idx], cols[idx]], 1)


def find_candidates_json(frame_dir):
    """candidates.json 在帧目录或轮根；轮根的块 uv 只对 frame-pick 主帧成立。"""
    if (frame_dir / "candidates.json").exists():
        return frame_dir / "candidates.json"
    up = frame_dir.parent / "candidates.json"
    if up.exists() and frame_dir.name == "frame-pick":
        return up
    return None


def run_frame(frame_dir, localnet, anchors):
    K = load_K(frame_dir)
    # 四个模块都各自 from-import 了 get_camera_intrinsic（绑定各自的全局名），
    # 必须逐一替换——漏一个就会混进数据集的 kinect 内参（fx 631 vs 我们 908）。
    ours = (lambda camera=None, _K=K: _K.copy())
    for mod in (dcfg, deval, dpdt, dutils):
        mod.get_camera_intrinsic = ours

    cj_path = find_candidates_json(frame_dir)
    if cj_path is None:
        return None, None, None, "no candidates.json (或非 frame-pick 主帧，块 uv 不适用)"
    cj = json.loads(cj_path.read_text())
    blocks = cj["blocks"]
    table_z = cj.get("table_z_mm", 0.0)

    color = np.array(Image.open(frame_dir / "color.png").convert("RGB"))
    depth = np.array(Image.open(frame_dir / "depth.png")).astype(np.float32)
    depth = np.clip(depth, 0, 1000)

    ori_rgb = torch.from_numpy(color / 255.0).permute(2, 1, 0)[None]
    ori_rgb = ori_rgb.to(device="cuda", dtype=torch.float32)
    ori_depth = torch.from_numpy(depth).T[None]
    ori_depth = ori_depth.to(device="cuda", dtype=torch.float32)

    view_points = scene_points(ori_rgb.squeeze(), ori_depth[0], K)

    # patch 中心：每块 uv 的 3×3 邻域（640×360 坐标），int 数组（center2dtopc 回写截断约定）
    centers = []
    for b in blocks:
        u, v = b["uv"][0] / 2.0, b["uv"][1] / 2.0
        for dy in (-4, 0, 4):
            for dx in (-4, 0, 4):
                centers.append([int(round(u + dx)), int(round(v + dy))])
    centers = np.array(sorted(set(map(tuple, centers))), dtype=np.int64)

    rg_g = RectGraspGroup(centers=centers,
                          heights=np.full((len(centers),), 25.0),
                          depths=np.zeros((len(centers),)))
    t0 = time.time()
    valid_local_centers, _ = center2dtopc(
        [rg_g], CENTER_NUM, ori_depth, (640, 360),
        append_random_center=False, is_training=False)
    # center2dtopc 的零深度修补路径会把 float32 升成 float64，这里统一压回
    valid_local_centers = [t.float() for t in valid_local_centers]

    # ---- 以下为 demo.py 243–287 行的逐行转写（K 注入）
    rgbd = torch.cat([ori_rgb.squeeze(), _xyz_maps(ori_depth, K).squeeze()], 0)
    _, w, h = rgbd.shape
    xg = torch.linspace(0, 1, 64, device="cuda", dtype=torch.float32)
    gx, gy = torch.meshgrid(xg, xg)
    grid_idxs = torch.stack([gx, gy], -1) - 0.5
    ratio = w / 640
    centers_t = ratio * torch.from_numpy(rg_g.centers.astype(np.float32)).cuda()
    grid_idxs = grid_idxs[None].expand(len(centers_t), -1, -1, -1)
    fx = K[0, 0]
    radius = torch.full((len(centers_t),), 0.10, device="cuda")
    radius = radius * 2 * fx / valid_local_centers[0][:, 2]
    radius = radius * (60.0 / 60.0)
    grid_idxs = grid_idxs * radius[:, None, None, None]
    grid_idxs = grid_idxs + torch.flip(centers_t[:, None, None], [-1])
    grid_idxs = grid_idxs / torch.FloatTensor([(h - 1), (w - 1)]).cuda() * 2 - 1
    local_patches = torch.nn.functional.grid_sample(
        rgbd[None].expand(len(centers_t), -1, -1, -1), grid_idxs,
        mode="nearest", align_corners=False)
    local_patches = local_patches.permute(0, 3, 2, 1).contiguous()
    m = (local_patches[..., -1:] > 0)
    patch_centers = valid_local_centers[0][:, None, None].expand(
        -1, 64, 64, -1)
    local_patches[..., 3:] -= m * patch_centers
    local_patches[..., 3:] /= 60.0 / 1e3

    with torch.no_grad():
        _, pred, offset, theta_cls, theta_offset, width_reg = localnet(
            local_patches)
    theta_cls = theta_cls.sigmoid().clip(1e-6, 1 - 1e-6).cpu().numpy().squeeze()
    theta_offset = theta_offset.clip(-0.5, 0.5).cpu().numpy().squeeze()
    width_reg = width_reg.cpu().numpy().squeeze()
    thetas, widths_6d = deval.get_thetas_widths(
        theta_cls, theta_offset, width_reg, anchor_w=60.0, rotation_num=1)

    # 6 个 theta 类别全枚举（上界检查：按块边选类别能否恢复对齐——A 案思路）
    theta_all = (np.arange(6, dtype=float)[None, :] + 0.5) * 30.0 - 90.0  # 相机系，绕光轴
    pred_grasp, pred_6d_gg = deval.detect_6d_grasp_multi(
        thetas, widths_6d, pred, offset, valid_local_centers, anchors,
        alpha=0.02 * 60.0 / 60.0, k=10)
    dt = time.time() - t0

    # ---- 转 base 系 + 与 graspnet_candidates.candidates_base 同口径
    sys.path.insert(0, str(SCRIPTS))
    from graspnet_candidates import cam_pose  # noqa: E402
    R_bc, t_bc = cam_pose(str(frame_dir))
    R_b = np.einsum("ij,njk->nik", R_bc, pred_6d_gg.rotations)
    t_b = pred_6d_gg.translations @ R_bc.T + t_bc            # m，base 系
    approach = R_b[:, :, 0]
    tilt = np.degrees(np.arccos(np.clip(-approach[:, 2], -1, 1)))
    rows = []
    for i in range(len(pred_grasp)):
        angle = float(np.degrees(np.arctan2(R_b[i][1, 1], R_b[i][0, 1])))
        rows.append({
            "score": float(pred_6d_gg.scores[i]),
            "width_mm": float(pred_grasp[i][6] * 1000),
            "tilt_deg": float(tilt[i]),
            "theta_cam_deg": float(np.degrees(pred_grasp[i][3])),
            "xyz_base": [float(v) * 1000 for v in t_b[i]],
            "angle_deg": round(angle, 1),
            "u_px": float(K[0, 0] * pred_grasp[i][0] / pred_grasp[i][2] + K[0, 2]),
            "v_px": float(K[1, 1] * pred_grasp[i][1] / pred_grasp[i][2] + K[1, 2]),
        })
    keep = [r for r in rows
            if r["score"] >= MIN_SCORE
            and r["tilt_deg"] <= 35.0
            and table_z - 30.0 < r["xyz_base"][2] < table_z + 80.0]
    keep.sort(key=lambda r: -r["score"])

    # ---- 锚定 + 贴边差角（与 A 案同口径）
    vlc_np = valid_local_centers[0].cpu().numpy()
    centers_base = vlc_np @ R_bc.T * 1000 + t_bc * 1000  # mm, base 系
    per_block = []
    for b in blocks:
        near = []
        for r in keep:
            d = float(np.hypot(r["xyz_base"][0] - b["cx"],
                               r["xyz_base"][1] - b["cy"]))
            if d <= ANCHOR_MM:
                nr = dict(r, dist_mm=round(d, 2),
                          edge_off_deg=round(ang_diff_mod90(
                              r["angle_deg"], b["yaw"]), 1))
                near.append(nr)
        near.sort(key=lambda c: (round(c["dist_mm"]), -c["score"]))
        aligned = [c for c in near if c["edge_off_deg"] <= EDGE_GATE_DEG]
        aligned.sort(key=lambda c: (round(c["dist_mm"]), -c["score"]))
        # theta 全类别上界：近邻中心 × 6 个 theta 类别，与块边（相机系）的最小差
        bdir = R_bc.T @ np.array([np.cos(np.radians(b["yaw"])),
                                  np.sin(np.radians(b["yaw"])), 0.0])
        yaw_cam = np.degrees(np.arctan2(bdir[1], bdir[0]))
        nc = np.hypot(centers_base[:, 0] - b["cx"],
                      centers_base[:, 1] - b["cy"]) <= ANCHOR_MM
        if nc.any():
            dth = np.abs(((theta_all - yaw_cam + 45) % 90) - 45)
            dth = np.repeat(dth, nc.shape[0], axis=0)[nc]
            theta_any = float(dth.min())
        else:
            theta_any = None
        per_block.append({
            "block_idx": b["idx"], "class": b["class"],
            "block_yaw": round(b["yaw"], 1),
            "n_near": len(near), "n_aligned": len(aligned),
            "best_dist_mm": near[0]["dist_mm"] if near else None,
            "near": near[:20],
            "aplan_pick": aligned[0] if aligned else None,
            "theta_any_off_deg": round(theta_any, 1) if theta_any is not None else None,
        })

    # ---- baseline 对照：新轮 per_block 带 near 全集；旧轮只有 chosen（当时会抓的候选）
    chosen = cj.get("chosen", [])
    if isinstance(chosen, dict):
        chosen = [chosen]
    by_idx = {c["block_idx"]: c for c in chosen}
    baseline = []
    for pb in cj.get("per_block", []):
        b_yaw = next(bb["yaw"] for bb in blocks if bb["idx"] == pb["block_idx"])
        near = pb.get("near", [])
        if near:
            offs = [ang_diff_mod90(c["angle_deg"], b_yaw) for c in near]
            aligned = [o for o in offs if o <= EDGE_GATE_DEG]
            row = {"block_idx": pb["block_idx"], "class": pb.get("class"),
                   "n_near": pb.get("n_near"), "n_aligned": len(aligned),
                   "best_dist_mm": pb.get("dist_mm"),
                   "min_edge_off_deg": round(min(offs), 1) if offs else None,
                   "aplan_off_deg": round(min(aligned), 1) if aligned else None}
        else:
            ch = by_idx.get(pb["block_idx"])
            row = {"block_idx": pb["block_idx"], "class": pb.get("class"),
                   "n_near": pb.get("n_near"), "n_aligned": None,
                   "best_dist_mm": pb.get("dist_mm"),
                   "min_edge_off_deg": None,
                   "aplan_off_deg": (round(ang_diff_mod90(ch["angle_deg"],
                                                           b_yaw), 1)
                                     if ch and "angle_deg" in ch else None)}
        baseline.append(row)

    return {
        "frame": str(frame_dir), "infer_sec": round(dt, 2),
        "n_centers": len(centers), "n_raw": len(rows), "n_keep": len(keep),
        "table_z_mm": table_z,
        "keep_head": keep[:500],
        "per_block": per_block, "baseline": baseline,
    }, color, keep, None


def _xyz_maps(depth_t, K):
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    ymap, xmap = np.meshgrid(np.arange(720), np.arange(1280))
    px = torch.from_numpy((xmap - cx) / fx).float().cuda()
    py = torch.from_numpy((ymap - cy) / fy).float().cuda()
    z = depth_t / 1000.0
    xyz = torch.stack([px * z, py * z, z], -1).permute(0, 3, 1, 2)
    return xyz


def draw_overlay(color, keep, blocks, path, fx):
    img = Image.fromarray(color.copy())
    dr = ImageDraw.Draw(img)
    for b in blocks:
        u, v = b["uv"]
        dr.ellipse([u - 6, v - 6, u + 6, v + 6], outline=(255, 255, 0), width=2)
    for r in keep:
        u, v = r["u_px"], r["v_px"]
        ang = np.radians(r["theta_cam_deg"])
        half = max(4.0, r["width_mm"] * fx / 1000.0 / 2)
        dx, dy = np.cos(ang) * half, np.sin(ang) * half
        c = (0, 255, 0) if r["score"] > 0.7 else (0, 160, 255)
        dr.line([u - dx, v - dy, u + dx, v + dy], fill=c, width=2)
    img.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames", nargs="+")
    ap.add_argument("--ckpt", default=str(CKPT))
    ap.add_argument("--out-root", default=str(OUT_ROOT))
    args = ap.parse_args()

    torch.manual_seed(123)
    np.random.seed(123)
    localnet = PatchMultiGraspNet(49, theta_k_cls=6, feat_dim=256,
                                  anchor_w=60.0).cuda().eval()
    ck = torch.load(args.ckpt, map_location="cuda")
    # checkpoint 里混着 thop 的 total_ops/total_params 统计 buffer（官方 demo
    # 存盘前跑过 profile），滤掉后按 strict 加载，真实缺键/多键仍会报错。
    local_sd = {k: v for k, v in ck["local"].items()
                if "total_ops" not in k and "total_params" not in k}
    localnet.load_state_dict(local_sd, strict=True)
    anchors = {"gamma": ck["gamma"].cuda(), "beta": ck["beta"].cuda()}
    print(f"[ckpt] {args.ckpt} loaded; gamma {tuple(ck['gamma'].shape)}")

    out_root = Path(args.out_root)
    out_root.mkdir(parents=True, exist_ok=True)

    for fs in args.frames:
        fd = Path(fs)
        report, color, keep, note = run_frame(fd, localnet, anchors)
        if report is None:
            print(f"== {fd}: SKIP（{note}）")
            continue
        blocks = json.loads(
            (find_candidates_json(fd)).read_text())["blocks"]
        stem = f"{fd.parent.name}-{fd.name}"
        (out_root / f"{stem}.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=1))
        draw_overlay(color, keep, blocks, str(out_root / f"{stem}.png"),
                     fx=load_K(fd)[0, 0])
        print(f"\n== {stem}: raw {report['n_raw']} keep {report['n_keep']} "
              f"({report['infer_sec']}s)")
        for pb, bl in zip(report["per_block"], report["baseline"]):
            print(f"  block {pb['block_idx']} ({pb['class']:>10}) "
                  f"rngnet: near {pb['n_near']:>3} aligned {pb['n_aligned']:>2} "
                  f"best {str(pb['best_dist_mm']):>5} mm "
                  f"aplan_off {pb['aplan_pick']['edge_off_deg'] if pb['aplan_pick'] else None} "
                  f"theta_any {pb['theta_any_off_deg']} "
                  f"| baseline: near {bl['n_near']} aligned {bl['n_aligned']} "
                  f"min_off {bl['min_edge_off_deg']}")


if __name__ == "__main__":
    main()
