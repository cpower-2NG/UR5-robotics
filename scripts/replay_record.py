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
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    n.create_subscription(CameraInfo, '/camera/color/camera_info',
                          lambda m: got.setdefault('info', m), 5)
    n.create_subscription(Image, '/camera/color/image_raw',
                          lambda m: got.__setitem__('color', m), 5)
    n.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw',
                          lambda m: got.__setitem__('depth', m), qos)
    # ⚠️ QoS 必须用 BEST_EFFORT：`/targets_3d` 由 projector_node 以 BEST_EFFORT 发布，
    #    订阅端用 RELIABLE 会**兼容失败**，症状是「一条消息都收不到」而发布端
    #    看起来一切正常（第一版就是这么录到 detections=0 的）。
    #    BEST_EFFORT 订阅端对 RELIABLE 与 BEST_EFFORT 两种发布端都兼容。
    n.create_subscription(Detection3DArray, '/targets_3d',
                          lambda m: got.__setitem__('det', m), qos)

    from sensor_msgs.msg import JointState
    n.create_subscription(JointState, '/joint_states',
                          lambda m: got.__setitem__('js', m), 10)

    t0 = time.time()
    print(f'录制到 {a.out}，最多 {a.seconds:.0f}s @ {a.fps:g} fps，Ctrl-C 可提前停')
    nxt, idx, skipped = t0, 0, 0
    try:
        while rclpy.ok() and time.time() - t0 < a.seconds:
            rclpy.spin_once(n, timeout_sec=0.05)
            if time.time() < nxt:
                continue
            nxt = time.time() + 1.0 / a.fps
            need = ('color', 'depth', 'js')
            if any(k not in got for k in need):
                skipped += 1
                continue
            try:
                color = np.frombuffer(got['color'].data, dtype=np.uint8).reshape(
                    got['color'].height, got['color'].width, -1)
                color = color[:, :, :3]
            except Exception:                                     # noqa: BLE001
                skipped += 1
                continue
            depth = np.frombuffer(got['depth'].data, dtype=np.uint16).reshape(
                got['depth'].height, got['depth'].width)
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

            d = dict(zip(got['js'].name, got['js'].position))
            q = [float(d.get(k, float('nan'))) for k in ARM]
            rec = {'i': idx, 't': round(stamp, 3), 'joints': [round(v, 6) for v in q]}
            if all(v == v for v in q):        # NaN 检查：缺关节角就不画 tool0
                rec['tool0'] = [round(float(v), 5) for v in urdf_fk(q)[:3, 3]]
            if 'det' in got:
                rec['dets'] = [
                    {'x': round(x.results[0].pose.pose.position.x, 5),
                     'y': round(x.results[0].pose.pose.position.y, 5),
                     'z': round(x.results[0].pose.pose.position.z, 5),
                     's': round(x.results[0].hypothesis.score, 3),
                     'cls': x.results[0].hypothesis.class_id}
                    for x in got['det'].detections]
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
