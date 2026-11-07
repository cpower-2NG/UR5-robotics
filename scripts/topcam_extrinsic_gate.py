#!/usr/bin/env python3
"""阶段 4 · E3b：俯视外参门（每轮都跑，只读，不动臂）。

一轮的固定次序（任何一步拿不到数据 ⇒ 退出 2，**不**拿旧值硬跑）：

1. **漂移探测器**：``topcam_drift_check.py --old <上次标定的图> --new <刚拍的图>``。
   返回 1（动了）或 2（证据不足）**都**继续进第 2 步 —— 它只负责报警，
   从不证明「外参还有效」。返回 0 不可能出现；真出现就按坏数据停车。
   第 1 轮没有「上次的图」⇒ 明确记 ``skipped: 无法 drift``，不假装做过。
2. **ChArUco PnP**：``--solve`` 的同一套（板位姿存盘 × 本次 cam←板）解出
   ``base←topcam_optical``；板没检出 / RMS 超限 ⇒ 退出 2。
3. **增量门**：与存盘外参比，平移 >1 mm 或转角 >0.2° ⇒ 必须**重解**（给
   ``--reimage`` 再拍一帧）；重解后仍超 ⇒ 退出 1，停车。
4. **端到端复核**：把新外参交给 ``stage4_joint_calib``（``--base-topcam-link``）
   跑两路垫 PnP，要求 ≤2 mm / 0.5°；不过 ⇒ 退出 1。
   通过后本门的输出就是外参的**事实源**（``site.yaml`` 只是启动参数副本）。

退出码：0 = 全过；1 = 判据不过（停车）；2 = 数据缺失/坏（停车）。
"""

import argparse
import datetime
import json
import os
import subprocess
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np                                                # noqa: E402

import stage4_joint_calib as jc                                   # noqa: E402
import topcam_marker_calib as tmc                                 # noqa: E402

DRIFT_SCRIPT = os.path.join(HERE, 'topcam_drift_check.py')


class GateData(Exception):
    """数据缺失/坏 ⇒ 退出 2（与「判据不过」的退出 1 分开）。"""


# ---------------------------------------------------------------- 小工具
def _T(R, t):
    return jc._T(R, t)


def _diff(Ta, Tb):
    """两份 base←optical 的差：平移范数（mm）与总转角（度）。"""
    rel = np.linalg.inv(np.asarray(Ta, dtype=np.float64)) @ np.asarray(
        Tb, dtype=np.float64)
    t_mm = float(np.linalg.norm(rel[:3, 3])) * 1000.0
    ang = float(np.degrees(np.arccos(float(np.clip(
        (np.trace(rel[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)))))
    return t_mm, ang


def optical_from_doc(doc, link_to_optical=None):
    """外参 JSON → ``base←topcam_optical`` 的 4×4。

    认两种：本工具写的（``R``/``t`` 已是光学系）、joint_calib 写的
    （``R_link``/``t_link``，再左乘 link←optical）。别的拒绝。
    """
    child = str(doc.get('frame_child') or doc.get('camera_frame') or '')
    if 'R' in doc and child in tmc.OPTICAL_FRAMES:
        return _T(doc['R'], doc['t'])
    if 'R_link' in doc and 't_link' in doc:
        return _T(doc['R_link'], doc['t_link']) @ _T(
            *((link_to_optical or (tmc.R_LINK_TO_OPTICAL, np.zeros(3)))))
    if 'R' in doc and child == tmc.TOPCAM_LINK:
        return _T(doc['R'], doc['t']) @ _T(
            *((link_to_optical or (tmc.R_LINK_TO_OPTICAL, np.zeros(3)))))
    raise GateData('认不出外参文件（frame_child=%r）：只认光学系 R/t 或 R_link/t_link'
                   % child)


def _link_to_optical(tf_doc):
    if tf_doc is None:
        return (tmc.R_LINK_TO_OPTICAL.copy(), np.zeros(3)), 'ideal'
    try:
        R, t = tmc.chain_from_tf(tf_doc, tmc.TOPCAM_LINK_TO_OPTICAL_KEYS)
    except SystemExit as exc:
        raise GateData('帧的 tf.json 缺 link→optical 链：%s' % exc)
    return (R, t), 'tf-json'


# ---------------------------------------------------------------- 1. drift
def run_drift(old_image, new_image, info_path=None, timeout=120):
    cmd = [sys.executable, DRIFT_SCRIPT, '--old', old_image, '--new', new_image]
    if info_path:
        cmd += ['--info', info_path]
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError) as exc:
        raise GateData('漂移探测器跑不起来：%s' % exc)
    verdict = ''
    for line in reversed((p.stdout or '').strip().splitlines()):
        if '判据' in line or '判定' in line:
            verdict = line.strip()
            break
    return {'ran': True, 'rc': int(p.returncode), 'verdict': verdict,
            'tail': (p.stdout or '').strip().splitlines()[-3:]}


# ---------------------------------------------------------------- 2. PnP
def solve_extrinsic(image, info_path, board_path, store_path, tf_doc,
                    max_rms_px=1.0):
    if not os.path.exists(store_path):
        raise GateData('没有板位姿存盘 %s：先跑一次 '
                       'topcam_marker_calib.py --init' % store_path)
    with open(board_path, encoding='utf-8') as fh:
        board_meta = json.load(fh)
    with open(store_path, encoding='utf-8') as fh:
        store = json.load(fh)
    K, dist, (w, h) = tmc.load_camera(info_path)
    det = tmc.detect_board(image, board_meta, K, dist)
    if det is None:
        raise GateData('ChArUco 板没检出足够的角点：%s' % image)
    (R_cb, t_cb), n_pts, rms = det
    if rms > max_rms_px:
        raise GateData('ChArUco 重投影 RMS %.3f px > %.2f px（纸不平/被缩放）'
                       % (rms, max_rms_px))
    R_bb = np.asarray(store['base_from_board_R'], dtype=np.float64)
    t_bb = np.asarray(store['base_from_board_t'], dtype=np.float64)
    R_bo, t_bo = tmc.compose((R_bb, t_bb), tmc.invert((R_cb, t_cb)))
    (R_lo, t_lo), lo_source = _link_to_optical(tf_doc)
    T_bo = _T(R_bo, t_bo)
    T_bl = T_bo @ np.linalg.inv(_T(R_lo, t_lo))
    return {
        'T_optical': T_bo, 'R': R_bo, 't': t_bo,
        'R_link': T_bl[:3, :3], 't_link': T_bl[:3, 3],
        'rms_px': float(rms), 'n_corners': int(n_pts),
        'link_to_optical_source': lo_source,
        'image': image, 'info': info_path,
    }


def write_extrinsic(path, sol):
    doc = {
        'version': 1, 'kind': 'camera-extrinsics',
        'frame_parent': 'base_link',
        'frame_child': 'topcam_color_optical_frame',
        'R': np.asarray(sol['R']).tolist(), 't': np.asarray(sol['t']).tolist(),
        'q': tmc.matrix_to_quat(np.asarray(sol['R'])).tolist(),
        'R_link': np.asarray(sol['R_link']).tolist(),
        't_link': np.asarray(sol['t_link']).tolist(),
        'link_to_optical_source': sol['link_to_optical_source'],
        'reproj_rms_px': sol['rms_px'], 'n_corners': sol['n_corners'],
        'image': sol['image'],
        'created': datetime.datetime.now().isoformat(timespec='seconds'),
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh, ensure_ascii=False, indent=1)
    return doc


# ---------------------------------------------------------------- 4. 端到端
def joint_check(extrinsic_path, wrist_frame, topcam_frame, mat,
                max_trans_mm=2.0, max_rot_deg=0.5, out_path=None):
    ns = argparse.Namespace(
        wrist_frame=wrist_frame, topcam_frame=topcam_frame, mat=mat,
        base_topcam_link=extrinsic_path,
        max_trans_mm=max_trans_mm, max_rot_deg=max_rot_deg,
        max_rms_px=1.5, min_markers=4, margin_px=20.0,
        pick_zone=list(jc.PICK_ZONE), table_z_mm=jc.TABLE_Z_MM,
        block_mm=29.5, hover_mm=80.0)
    res = jc.run(ns)
    if out_path:
        os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
        with open(out_path, 'w', encoding='utf-8') as fh:
            json.dump(res, fh, ensure_ascii=False, indent=1)
    return res


# ---------------------------------------------------------------- 主流程
def run(args):
    out_dir = args.out_dir or os.path.join(
        os.path.dirname(os.path.abspath(args.image)), 'extrinsic-gate')
    os.makedirs(out_dir, exist_ok=True)
    tag = 'round-%02d' % args.round if args.round else 'round'
    ext_path = os.path.join(out_dir, tag + '-extrinsics.json')
    joint_path = os.path.join(out_dir, tag + '-joint-calib.json')
    gate_path = os.path.join(out_dir, tag + '.json')

    report = {'version': 1, 'kind': 'stage4-extrinsic-gate',
              'round': args.round,
              'created': datetime.datetime.now().isoformat(timespec='seconds'),
              'reasons': [], 'ok': False}

    # ---- 1. drift（只报警，不放行）
    if args.prev_image and args.new_image:
        drift = run_drift(args.prev_image, args.new_image, args.info)
        report['drift'] = drift
        if drift['rc'] not in (1, 2):
            report['reasons'].append(
                '漂移探测器返回 %s（只允许 1/2）—— 按坏数据停车' % drift['rc'])
            report['ok'] = False
            _write(gate_path, report)
            return report, 1
        report['drift']['policy'] = '1/2 都进 PnP；本工具从不放行旧值'
    else:
        report['drift'] = {'ran': False,
                           'reason': '没有上次标定的图 / 没给 --new-image：'
                                     '第 1 轮无法 drift，只凭 PnP（记录在案）'}

    # ---- 2. PnP
    tf_doc = None
    if args.tf_json:
        with open(args.tf_json, encoding='utf-8') as fh:
            tf_doc = json.load(fh)
    sol = solve_extrinsic(args.image, args.info, args.board, args.store,
                          tf_doc, max_rms_px=args.max_rms_px)
    report['pnp'] = {'image': args.image, 'rms_px': sol['rms_px'],
                     'n_corners': sol['n_corners'],
                     'link_to_optical_source': sol['link_to_optical_source'],
                     'ok': True}
    doc = write_extrinsic(ext_path, sol)
    T_new = sol['T_optical']

    # ---- 3. 增量门（>1 mm / 0.2° 必须重解；重解仍超就停）
    cmp = {'prev': args.prev_extrinsics, 'ran': bool(args.prev_extrinsics)}
    if args.prev_extrinsics:
        with open(args.prev_extrinsics, encoding='utf-8') as fh:
            prev_doc = json.load(fh)
        link_pair, _ = _link_to_optical(tf_doc)
        T_prev = optical_from_doc(prev_doc, link_pair)
        t_mm, ang = _diff(T_prev, T_new)
        cmp.update(t_mm=round(t_mm, 3), angle_deg=round(ang, 4),
                   thresholds={'translation_mm': args.max_drift_mm,
                               'angle_deg': args.max_drift_deg})
        over = (t_mm > args.max_drift_mm) or (ang > args.max_drift_deg)
        cmp['within'] = not over
        if over:
            if not args.reimage:
                cmp['resolved'] = False
                report['reasons'].append(
                    '与存盘外参差 %.2f mm / %.3f°，超限且没给 --reimage 重拍'
                    % (t_mm, ang))
            else:
                sol2 = solve_extrinsic(args.reimage, args.reinfo or args.info,
                                       args.board, args.store, tf_doc,
                                       max_rms_px=args.max_rms_px)
                doc = write_extrinsic(ext_path, sol2)
                T_new = sol2['T_optical']
                t2, ang2 = _diff(T_prev, T_new)
                cmp.update(reimage=args.reimage, t_mm_resolved=round(t2, 3),
                           angle_deg_resolved=round(ang2, 4),
                           resolved=(t2 <= args.max_drift_mm
                                     and ang2 <= args.max_drift_deg))
                report['pnp']['reimage_rms_px'] = sol2['rms_px']
                if not cmp['resolved']:
                    report['reasons'].append(
                        '重解后仍差 %.2f mm / %.3f°（上限 %.2f mm / %.2f°）'
                        % (t2, ang2, args.max_drift_mm, args.max_drift_deg))
        else:
            cmp['resolved'] = None            # 没超限，不需要重解
    else:
        cmp['baseline'] = True
        cmp['reason'] = '第 1 轮：没有存盘外参，本轮即基准'
    report['compare'] = cmp
    report['extrinsic_file'] = ext_path

    # ---- 4. 端到端两路垫 PnP（用**新**外参）
    if report['reasons']:
        report['joint'] = {'ran': False, 'reason': '增量门已红，不再跑端到端'}
    elif not (args.wrist_frame and args.topcam_frame):
        report['joint'] = {'ran': False,
                           'reason': '没给 --wrist-frame/--topcam-frame：'
                                     '缺端到端复核 ⇒ 不算通过（fail-closed）'}
        report['reasons'].append('缺端到端两路复核')
    else:
        try:
            jr = joint_check(ext_path, args.wrist_frame, args.topcam_frame,
                             args.mat, max_trans_mm=args.max_trans_mm,
                             max_rot_deg=args.max_rot_deg,
                             out_path=joint_path)
            report['joint'] = {
                'ran': True, 'ok': bool(jr.get('ok')),
                'file': joint_path,
                'two_way': jr.get('two_way'),
                'extrinsic_source': jr.get('extrinsic', {}).get('source'),
                'reasons': jr.get('reasons', []),
            }
            if not jr.get('ok'):
                report['reasons'].append('端到端两路复核不过：%s'
                                         % '；'.join(jr.get('reasons') or ['未知']))
        except GateData:
            raise
        except (OSError, ValueError, KeyError) as exc:
            report['joint'] = {'ran': False, 'reason': '端到端复核数据缺失：%s'
                                                       % exc}
            report['reasons'].append('端到端复核数据缺失')

    report['ok'] = not report['reasons']
    _write(gate_path, report)
    return report, (0 if report['ok'] else 1)


def _write(path, report):
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(report, fh, ensure_ascii=False, indent=1, default=str)


def _add_thresholds(ap):
    ap.add_argument('--max-drift-mm', type=float, default=1.0)
    ap.add_argument('--max-drift-deg', type=float, default=0.2)
    ap.add_argument('--max-trans-mm', type=float, default=2.0)
    ap.add_argument('--max-rot-deg', type=float, default=0.5)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='阶段 4 E3b：俯视外参门（只读，不动臂）')
    ap.add_argument('--round', type=int, default=None)
    ap.add_argument('--image', required=True, help='本轮俯视帧（color.png）')
    ap.add_argument('--info', required=True, help='同一次采集的 info.json')
    ap.add_argument('--prev-image', default=None, help='上次标定用的图（drift 用）')
    ap.add_argument('--new-image', default=None, help='本轮刚拍的图（drift 用）')
    ap.add_argument('--board', default='docs/calib/charuco_A4_30mm_5x5.json')
    ap.add_argument('--store', default=os.path.expanduser('~/topcam_marker.json'))
    ap.add_argument('--tf-json', default=None,
                    help='本轮俯视帧的 tf.json（用实测 link→optical 链）')
    ap.add_argument('--prev-extrinsics', default=None,
                    help='存盘外参 JSON（第 1 轮不给 ⇒ 本轮即基准）')
    ap.add_argument('--reimage', default=None,
                    help='增量超限时用来重解的帧')
    ap.add_argument('--reinfo', default=None, help='重解帧的内参（默认同 --info）')
    ap.add_argument('--wrist-frame', default=None)
    ap.add_argument('--topcam-frame', default=None)
    ap.add_argument('--mat', default=jc.DEFAULT_MAT)
    ap.add_argument('--max-rms-px', type=float, default=1.0)
    ap.add_argument('--out-dir', default=None)
    _add_thresholds(ap)
    args = ap.parse_args(argv)

    try:
        report, code = run(args)
    except GateData as exc:
        print('!! 数据缺失/坏：%s' % exc)
        return 2
    except (OSError, ValueError, KeyError) as exc:
        print('!! 数据缺失/坏：%s' % exc)
        return 2

    print('drift：%s' % (report['drift'].get('verdict')
                        or report['drift'].get('reason', '—')))
    print('PnP：RMS %.3f px（%d 角点，link→optical：%s）'
          % (report['pnp']['rms_px'], report['pnp']['n_corners'],
             report['pnp']['link_to_optical_source']))
    c = report['compare']
    if c.get('baseline'):
        print('增量门：本轮即基准（没有存盘外参）')
    elif c.get('ran'):
        print('增量门：%.2f mm / %.3f°（上限 %.2f mm / %.2f°）%s'
              % (c['t_mm'], c['angle_deg'],
                 c['thresholds']['translation_mm'], c['thresholds']['angle_deg'],
                 '' if c['within'] else '  ← 超限'))
        if c.get('reimage'):
            print('  重解：%.2f mm / %.3f° ⇒ %s'
                  % (c['t_mm_resolved'], c['angle_deg_resolved'],
                     '过' if c.get('resolved') else '仍超'))
    j = report.get('joint', {})
    print('端到端两路：%s' % ('OK' if j.get('ok') else j.get('reason', 'FAIL')))
    gate_dir = args.out_dir or os.path.join(
        os.path.dirname(os.path.abspath(args.image)), 'extrinsic-gate')
    print('写入 %s' % os.path.join(
        gate_dir, ('round-%02d.json' % args.round) if args.round
        else 'round.json'))
    if not report['ok']:
        for r in report['reasons']:
            print('判红：%s' % r)
    return code


if __name__ == '__main__':
    sys.exit(main())
