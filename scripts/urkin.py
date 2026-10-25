#!/usr/bin/env python3
"""从板上取回的 robot_description 建 base_link→tool0 的运动学链：FK + 数值 IK。

用部署的那份 URDF（不是标称 DH），所以与驱动/move_group 是同一套几何。
"""
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot


def _rpy_to_mat(r, p, y):
    return Rot.from_euler('xyz', [r, p, y]).as_matrix()


def _origin(el):
    T = np.eye(4)
    if el is None:
        return T
    xyz = [float(v) for v in (el.get('xyz') or '0 0 0').split()]
    rpy = [float(v) for v in (el.get('rpy') or '0 0 0').split()]
    T[:3, :3] = _rpy_to_mat(*rpy)
    T[:3, 3] = xyz
    return T


def _axis(el):
    a = np.array([float(v) for v in (el.get('xyz') or '1 0 0').split()])
    return a / np.linalg.norm(a)


class Chain:
    def __init__(self, urdf_path, root='base_link', tip='tool0'):
        tree = ET.parse(urdf_path).getroot()
        self.joints = {}
        child_of = {}
        self.limits = {}
        for j in tree.findall('joint'):
            name = j.get('name')
            self.joints[name] = {
                'type': j.get('type'),
                'parent': j.find('parent').get('link'),
                'child': j.find('child').get('link'),
                'origin': _origin(j.find('origin')),
                'axis': _axis(j.find('axis')) if j.find('axis') is not None else None,
            }
            lim = j.find('limit')
            if lim is not None and j.get('type') == 'revolute':
                # 用 **soft** 限位：控制器真正拦的是 safety_controller 的
                # soft_lower/soft_upper（±6.133 / ±2.992），不是 URDF 的硬限位
                # （±2π / ±π）。拿硬限位当判据会放过控制器会拒的关节角。
                sc = j.find('safety_controller')
                lo = float(sc.get('soft_lower_limit')) if sc is not None and sc.get('soft_lower_limit') is not None else float(lim.get('lower'))
                hi = float(sc.get('soft_upper_limit')) if sc is not None and sc.get('soft_upper_limit') is not None else float(lim.get('upper'))
                self.limits[name] = (lo, hi, float(lim.get('velocity') or 0.0))
            child_of[self.joints[name]['child']] = name
        # root → tip 的链
        link, chain = tip, []
        while link != root:
            jn = child_of[link]
            chain.append(jn)
            link = self.joints[jn]['parent']
        chain.reverse()
        self.chain = chain
        self.move_joints = [n for n in chain if self.joints[n]['type'] == 'revolute']

    def fk(self, q, upto=None):
        """q: {joint_name: angle} 或按 self.move_joints 顺序的序列。返回 4x4。"""
        if not isinstance(q, dict):
            q = dict(zip(self.move_joints, q))
        T = np.eye(4)
        for name in (self.chain if upto is None else self.chain[:upto + 1]):
            j = self.joints[name]
            T = T @ j['origin']
            if j['type'] == 'revolute':
                R = np.eye(4)
                R[:3, :3] = Rot.from_rotvec(j['axis'] * q[name]).as_matrix()
                T = T @ R
        return T

    def fk_link(self, link, q):
        """任意链上 link 的位姿（link 必须在 base_link 链上）。"""
        n = self.chain.index([n for n in self.chain if self.joints[n]['child'] == link][0])
        if isinstance(q, dict):
            q = [q[n] for n in self.move_joints]
        T = np.eye(4)
        for name in self.chain[:n + 1]:
            j = self.joints[name]
            T = T @ j['origin']
            if j['type'] == 'revolute':
                R = np.eye(4)
                R[:3, :3] = Rot.from_rotvec(j['axis'] * q[self.move_joints.index(name)]).as_matrix()
                T = T @ R
        return T

    def ik(self, T_target, seed, max_iter=200):
        """数值 IK。返回 (q, pos_err_mm, rot_err_deg, 迭代数)。"""
        def resid(q):
            T = self.fk(q)
            e_p = T[:3, 3] - T_target[:3, 3]
            e_r = Rot.from_matrix(T_target[:3, :3].T @ T[:3, :3]).as_rotvec()
            return np.concatenate([e_p, e_r * 0.25])   # 0.25 m/rad 加权 ≈ 25 cm 对 1 rad

        sol = least_squares(resid, np.array(seed, float), method='lm',
                            xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=max_iter)
        T = self.fk(sol.x)
        pe = np.linalg.norm(T[:3, 3] - T_target[:3, 3]) * 1000
        re = np.degrees(np.linalg.norm(Rot.from_matrix(T_target[:3, :3].T @ T[:3, :3]).as_rotvec()))
        return sol.x, pe, re, sol.nfev

    def link_names(self):
        """base_link→tip 这条链上的所有 link（含起止），按从根到梢排序。"""
        return [self.joints[n]['child'] for n in self.chain]

    def pose(self, q):
        T = self.fk(q)
        rpy = Rot.from_matrix(T[:3, :3]).as_euler('xyz')
        return T[:3, 3], np.degrees(rpy)
