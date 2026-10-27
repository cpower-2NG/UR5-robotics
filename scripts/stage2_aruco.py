#!/usr/bin/env python3
"""阶段 2 的 ArUco 兼容层 —— 同一份代码在 OpenCV 4.5.4 与 4.10 上都能跑。

为什么需要这一层
────────────────
本项目的两台机器上装着两个**互不兼容**的 OpenCV ArUco API：

  · 系统 ``/usr/bin/python3`` + OpenCV 4.5.4：
    模块级 ``detectMarkers`` / ``DetectorParameters_create`` / ``drawMarker``；
    没有 ``ArucoDetector`` / ``DetectorParameters`` / ``generateImageMarker``。
  · ``~/robotics/venvs/yolo-grasp`` + OpenCV 4.10：
    ``ArucoDetector`` / ``DetectorParameters`` / ``generateImageMarker``；
    没有模块级 ``detectMarkers`` / ``DetectorParameters_create`` / ``drawMarker``。

生成器在开发机、检测端可能在另一份解释器里跑；把版本判断散落在各脚本里，
迟早会出现「生成器能画、检测端 AttributeError」这种只在现场才炸的事。
这里集中收口，脚本只调用本模块。

坐标约定
────────
本模块只做「字典/检测/PnP」这类纯几何动作，**不定义垫系**。
垫系（原点、y 轴朝哪）由 ``stage2_mat.py`` 的 sidecar 与 ``stage2_pad.py`` 决定。
检测返回的角点顺序是 ArUco 的规范顺序：标记自身的左上、右上、右下、左下
（从标记图案的正面看）。sidecar 里每个标记的角点必须按同一顺序存。
"""

import sys

import cv2
import numpy as np

sys.dont_write_bytecode = True

try:                                                              # OpenCV 4.10+
    _ARUCO = cv2.aruco
except AttributeError:                                            # pragma: no cover
    raise SystemExit('这个 OpenCV 没编 aruco 模块（cv2.aruco 不存在）。')


# ------------------------------------------------------------------ 字典
def get_dictionary(name):
    """按名字取字典，兼容新旧 API。``name`` 例如 ``DICT_4X4_50``。"""
    if not hasattr(_ARUCO, name):
        raise ValueError('OpenCV 没有这个 ArUco 字典：%s' % name)
    ident = getattr(_ARUCO, name)
    fn = getattr(_ARUCO, 'getPredefinedDictionary', None)
    if fn is not None:
        return fn(ident)
    fn = getattr(_ARUCO, 'Dictionary_get', None)                  # 很老的 4.x
    if fn is not None:
        return fn(ident)
    raise RuntimeError('这个 OpenCV 既没有 getPredefinedDictionary 也没有 '
                       'Dictionary_get，无法取字典。')


# ------------------------------------------------------------------ 生成
def draw_marker(dict_name, marker_id, side_pixels, border_bits=1):
    """生成一张 ``side_pixels × side_pixels`` 的标记位图（uint8，0/255）。

    4.10+ 用 ``generateImageMarker``；4.5.4 用 ``drawMarker``。
    两者的语义都是「整张图就是标记本身」——4.5.4 实测四角像素为黑，
    即没有额外的白边；白静区由页面留白提供（本项目的标记间隙 6 mm）。
    """
    d = get_dictionary(dict_name)
    side_pixels = int(side_pixels)
    if side_pixels <= 0:
        raise ValueError('side_pixels 必须为正')
    try:
        return _ARUCO.generateImageMarker(d, int(marker_id), side_pixels,
                                          borderBits=border_bits)
    except (AttributeError, TypeError):
        return _ARUCO.drawMarker(d, int(marker_id), side_pixels,
                                 borderBits=border_bits)


# ------------------------------------------------------------------ 检测
def detector_parameters():
    """返回当前 OpenCV 能用的 DetectorParameters 对象。"""
    cls = getattr(_ARUCO, 'DetectorParameters', None)
    if cls is not None:
        return cls()
    fn = getattr(_ARUCO, 'DetectorParameters_create', None)
    if fn is not None:
        return fn()
    raise RuntimeError('这个 OpenCV 没有 DetectorParameters / '
                       'DetectorParameters_create。')


def detect_markers(gray, dict_name, parameters=None):
    """检测一帧灰度图里的标记，返回 ``(corners, ids)``。

    ``corners`` 是 list[ndarray(4,2)]（每个标记的四个角，规范顺序），
    ``ids`` 是 ndarray(N,) 或 None。没有检出时返回 ``([], None)``。
    """
    d = get_dictionary(dict_name)
    params = parameters or detector_parameters()
    cls = getattr(_ARUCO, 'ArucoDetector', None)
    if cls is not None:
        detector = cls(d, params)
        corners, ids, _rejected = detector.detectMarkers(gray)
    else:
        fn = getattr(_ARUCO, 'detectMarkers', None)
        if fn is None:
            raise RuntimeError('这个 OpenCV 既没有 ArucoDetector 也没有 '
                               'detectMarkers。')
        corners, ids, _rejected = fn(gray, d, parameters=params)
    if ids is None:
        return [], None
    return [np.asarray(c, dtype=np.float64).reshape(4, 2) for c in corners], \
        np.asarray(ids).reshape(-1)


# ------------------------------------------------------------------ PnP
def solve_plane_pnp(object_points, image_points, camera_matrix, dist_coeffs=None):
    """平面物点 + 图像点 → 位姿；返回 ``(rvec, tvec, reproj_rmse_px)``。

    object_points 与 image_points 必须一一对应且至少 4 点。
    先 ``SOLVEPNP_ITERATIVE``，再用 ``solvePnPRefineLM``（若可用）精修。
    重投影 RMS 用**精修后**的姿态算——它是对「这组点是否被同一个位姿解释」
    的直接度量，比 PnP 自己报的误差更可信。
    """
    obj = np.asarray(object_points, dtype=np.float64)
    if obj.ndim == 2 and obj.shape[1] == 2:          # 平面点：自动补 z=0
        obj = np.column_stack([obj, np.zeros(obj.shape[0])])
    obj = obj.reshape(-1, 3)
    img = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    if obj.shape[0] != img.shape[0] or obj.shape[0] < 4:
        raise ValueError('PnP 点对数量不对：obj=%d img=%d' %
                         (obj.shape[0], img.shape[0]))
    K = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    dist = np.zeros((5, 1), dtype=np.float64) if dist_coeffs is None else \
        np.asarray(dist_coeffs, dtype=np.float64).reshape(-1, 1)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError('solvePnP 失败')
    refine = getattr(cv2, 'solvePnPRefineLM', None)
    if refine is not None:
        rvec, tvec = refine(obj, img, K, dist, rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    err = np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)
    return rvec.reshape(3), tvec.reshape(3), float(np.sqrt(np.mean(err ** 2)))


def project_points(object_points, rvec, tvec, camera_matrix, dist_coeffs=None):
    """把物点投到像素；返回 (N,2)。"""
    obj = np.asarray(object_points, dtype=np.float64)
    if obj.ndim == 2 and obj.shape[1] == 2:          # 平面点：自动补 z=0
        obj = np.column_stack([obj, np.zeros(obj.shape[0])])
    obj = obj.reshape(-1, 3)
    K = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    dist = np.zeros((5, 1), dtype=np.float64) if dist_coeffs is None else \
        np.asarray(dist_coeffs, dtype=np.float64).reshape(-1, 1)
    proj, _ = cv2.projectPoints(obj, np.asarray(rvec).reshape(3, 1),
                                np.asarray(tvec).reshape(3, 1), K, dist)
    return proj.reshape(-1, 2)


def rodrigues_to_matrix(rvec, tvec):
    """(rvec, tvec) → 4×4 ``cam ← object`` 齐次变换。"""
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T
