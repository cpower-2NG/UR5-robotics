#!/usr/bin/env python3
"""录制一次抓取过程，供 **离线复盘渲染**（`replay_render.py`）使用。

为什么不是录 rosbag
───────────────────
复盘要的是「**逐帧对齐的画面 + 当时的关节角 + 当时的检测**」，而且要在**另一台
机器**上反复看、逐帧比对。bag 能做，但每次看都要装一套读取环境；这里落成
「一帧两张图 + 一行 JSON」，用 `cv2.imread` 和 `json.loads` 就能读。

录什么
──────
    frames/000123.jpg      彩色（JPEG，质量 92）
    depth/000123.png       对齐深度（**16 位无损 PNG**，单位 mm）
    meta.jsonl             每帧一行：时间戳、关节角、检测结果、以及由关节角
                           算出的 tool0 位姿（`urdf_fk`）
    info.json              相机的内参与外参（渲染端要用同一份）

⚠️ 深度必须是 **16 位无损**：它是毫米整数，转成 8 位会丢掉约 25 mm 的分辨率，
而复盘看的正是十几毫米量级的偏差。JPEG 也不能用于深度（有损）。

⚠️ 录制会让 CPU/带宽变忙，可能改变被测系统的时间行为。要看「纯粹的运动」时
建议单独跑一次不录制的；要**分析精度**时用录制的这一份。

用法
────
    python3 scripts/replay_record.py /tmp/run1 --seconds 90 --fps 4
"""

import argparse
import json
import os
import sys
import time

sys.dont_write_bytecode = True

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def main():
    ap = argparse.ArgumentParser(description="录制一次抓取过程用于离线复盘")
    ap.add_argument("out", help="输出目录")
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--fps", type=float, default=4.0,
                    help="录制帧率。彩色约 100 KB/帧、深度约 400 KB/帧，"
                         "4 fps × 90 s ≈ 180 MB")
    ap.add_argument("--extrinsics", default=os.path.expanduser(
        '~/yolo_ros_original/docs/calib/extrinsics-2026-10-03-base_link.json'))
    ap.add_argument("--jpeg-quality", type=int, default=92)
    a = ap.parse_args()

    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from vision_msgs.msg import Detection3DArray

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from ur5_fk_check import urdf_fk

    os.makedirs(os.path.join(a.out, 'frames'), exist_ok=True)
    os.makedirs(os.path.join(a.out, 'depth'), exist_ok=True)

    rclpy.init()
    n = Node('replay_record')
    got = {}

    # ⚠️ 每条话题各存一个 **(时间戳, 消息)** 的缓冲，采样时**按时间戳配对**。
    #    旧版是「各自取最新的一条」—— 两条话题的到达时刻可以差几十毫秒，
    #    于是「这一帧画面」和「这一条检测」根本不是同一时刻的东西。
    #    症状不是画面糊，而是**叠加位置看着偏**、且偏得没规律 —— 我曾据此
    #    判「检测框偏心」，查了很久才发现是录制器自己造成的。
    #    回放里的叠加要能当证据，就必须成对取。
    def keep(store, m, stamp):
        store.append((stamp, m))
        while len(store) > 8:                     # 8 条足够配对，更久就是延迟
            store.pop(0)

    def stamp_of(msg):
        h = msg.header.stamp
        return float(h.sec) + float(h.nanosec) * 1e-9

    def pick(key, ref_stamp, tol, drop_stale=True):
        """取**时间戳最接近 ref_stamp** 的那一条；差超过 ``tol`` 就返回 None。

        宁可这一帧不录，也不要录一对「不是一个时刻」的东西 —— 那正是旧版的病。
        """
        best, bd = None, tol
        for st, m in bufs[key]:
            d = abs(st - ref_stamp)
            if d <= bd:
                best, bd = (st, m), d
        if drop_stale and best is not None:
            bufs[key] = [x for x in bufs[key] if x[0] >= best[0]]
        return best

    bufs = {'color': [], 'depth': [], 'det': [], 'js': []}
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    n.create_subscription(CameraInfo, '/camera/color/camera_info',
                          lambda m: got.setdefault('info', m), 5)
    n.create_subscription(Image, '/camera/color/image_raw',
                          lambda m: keep(bufs['color'], m, stamp_of(m)), 5)
    n.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw',
                          lambda m: keep(bufs['depth'], m, stamp_of(m)), qos)
    # ⚠️ QoS 用 BEST_EFFORT：对 RELIABLE 与 BEST_EFFORT 两种发布端都兼容
    #    （订阅端用 RELIABLE 而发布端是 BEST_EFFORT 时会**收不到任何消息**）。
    n.create_subscription(Detection3DArray, '/targets_3d',
                          lambda m: keep(bufs['det'], m, stamp_of(m)), qos)

    from sensor_msgs.msg import JointState
    n.create_subscription(JointState, '/joint_states',
                          lambda m: keep(bufs['js'], m, stamp_of(m)), 10)


    t0 = time.time()
    print(f'录制到 {a.out}，最多 {a.seconds:.0f}s @ {a.fps:g} fps，Ctrl-C 可提前停')
    nxt, idx, skipped = t0, 0, 0
    try:
        while rclpy.ok() and time.time() - t0 < a.seconds:
            rclpy.spin_once(n, timeout_sec=0.05)
            if time.time() < nxt:
                continue
            nxt = time.time() + 1.0 / a.fps
            if not bufs['color']:
                skipped += 1
                continue
            # 以**彩色帧的时间戳**为基准，其余按「最接近」配对 —— 见上面 keep() 的说明
            ref, color_msg = bufs['color'][-1]
            got_d = pick('depth', ref, a.pair_tol)
            got_j = pick('js', ref, a.pair_tol)
            got_t = pick('det', ref, a.pair_tol, drop_stale=False)
            if got_d is None or got_j is None:
                skipped += 1
                continue
            try:
                color = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(
                    color_msg.height, color_msg.width, -1)[:, :, :3]
            except Exception:                                     # noqa: BLE001
                skipped += 1
                continue
            depth = np.frombuffer(got_d[1].data, dtype=np.uint16).reshape(
                got_d[1].height, got_d[1].width)
            js_msg = got_j[1]
            det_msg = None if got_t is None else got_t[1]
            det_dt = None if got_t is None else round(abs(got_t[0] - ref), 4)
            stamp = time.time() - t0

            if idx == 0 and 'info' in got:
                ci = got['info']
                json.dump({'fx': ci.k[0], 'fy': ci.k[4], 'cx': ci.k[2],
                           'cy': ci.k[5], 'width': ci.width, 'height': ci.height,
                           'D': list(ci.d), 'extrinsics_file': a.extrinsics},
                          open(os.path.join(a.out, 'info.json'), 'w'), indent=1)

            cv2.imwrite(os.path.join(a.out, 'frames', f'{idx:06d}.jpg'),
                        color, [cv2.IMWRITE_JPEG_QUALITY, a.jpeg_quality])
            cv2.imwrite(os.path.join(a.out, 'depth', f'{idx:06d}.png'), depth)

            d = dict(zip(js_msg.name, js_msg.position))
            q = [float(d.get(k, float('nan'))) for k in ARM]
            rec = {'i': idx, 't': round(stamp, 3), 'stamp': round(ref, 4),
                   'det_dt': det_dt, 'joints': [round(v, 6) for v in q]}
            if all(v == v for v in q):        # NaN 检查：缺关节角就不画 tool0
                rec['tool0'] = [round(float(v), 5) for v in urdf_fk(q)[:3, 3]]
            if det_msg is not None:
                rec['dets'] = [
                    {'x': round(x.results[0].pose.pose.position.x, 5),
                     'y': round(x.results[0].pose.pose.position.y, 5),
                     'z': round(x.results[0].pose.pose.position.z, 5),
                     's': round(x.results[0].hypothesis.score, 3),
                     'cls': x.results[0].hypothesis.class_id}
                    for x in det_msg.detections]
            with open(os.path.join(a.out, 'meta.jsonl'), 'a', encoding='utf-8') as fh:
                fh.write(json.dumps(rec, ensure_ascii=False) + '\n')
            idx += 1
            if idx % 20 == 0:
                print(f'  已录 {idx} 帧（{stamp:.0f}s）'
                      f'{"，跳过 " + str(skipped) + " 次" if skipped else ""}')
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.destroy_node()
        except Exception:                                         # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
    sz = sum(os.path.getsize(os.path.join(r, f))
             for r, _, fs in os.walk(a.out) for f in fs)
    print(f'✅ 录了 {idx} 帧 → {a.out}（{sz / 1e6:.0f} MB，跳过 {skipped} 次）')
    return 0 if idx else 2


if __name__ == '__main__':
    sys.exit(main())
