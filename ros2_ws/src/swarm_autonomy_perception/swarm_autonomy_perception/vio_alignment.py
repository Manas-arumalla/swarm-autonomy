"""ROS-free VIO->NED frame conversion and trajectory-shape (Umeyama) alignment.

The pure core behind the VIO -> PX4 EKF2 external-vision bridges. OpenVINS reports odometry in
its own gravity-aligned world frame whose yaw and origin are arbitrary (fixed at VIO init) and do
not match PX4's local NED, and stereo scale can carry a small residual error. Feeding that to
EKF2 unaligned makes it silently reject (or worse, fuse) inconsistent measurements.

Two pieces, both unit-tested:

* :func:`odom_to_ned` — the ENU/FLU -> NED/FRD conversion chain for a ROS odometry pose
  (``v_ned = M_W @ v_enu``, ``R_ned_frd = M_W @ R_enu_flu @ M_B``).
* :class:`TrajectoryAligner` — collects (vio, reference) position pairs while the vehicle flies,
  waits until the track spans a real ARC (motion in both axes, so rotation and scale are
  observable — a straight line cannot pin down yaw), then fits the 2-D similarity transform
  (scale + yaw + translation, Umeyama) mapping the VIO track onto the reference NED track. Using
  the trajectory *shape* rather than a single instantaneous attitude makes the yaw estimate
  robust to initialisation wobble and absorbs residual stereo-scale error in one shot.

Validated live: this alignment held the fused external-vision error to 0.26 m mean / 0.70 m max
on a 5 m survey circle, after which the vehicle flew GPS-denied on vision alone.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation as Rot

M_W = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)   # world ENU -> NED
M_B = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)  # body FLU -> FRD


def odom_to_ned(px: float, py: float, pz: float, qx: float, qy: float, qz: float, qw: float):
    """ROS ENU/FLU odometry pose -> (position NED, yaw in NED/FRD)."""
    ned = M_W @ np.array([px, py, pz])
    r_ned_frd = M_W @ Rot.from_quat([qx, qy, qz, qw]).as_matrix() @ M_B
    yaw = Rot.from_matrix(r_ned_frd).as_euler("ZYX")[0]
    return ned, float(yaw)


class TrajectoryAligner:
    """Accumulates (vio_ned, reference_ned) pairs, fits the 2-D similarity when ready.

    ``min_pairs`` / ``min_span_m`` / ``min_major_span_m`` define "the track spans a real arc";
    the fitted scale is clamped to ``scale_limits`` to guard against a degenerate fit.
    """

    def __init__(self, min_pairs: int = 40, min_span_m: float = 2.5,
                 min_major_span_m: float = 4.0, scale_limits=(0.6, 1.5)):
        self.min_pairs = int(min_pairs)
        self.min_span = float(min_span_m)
        self.min_major_span = float(min_major_span_m)
        self.scale_limits = scale_limits
        self.buf_vio: list[np.ndarray] = []
        self.buf_ref: list[np.ndarray] = []
        self.aligned = False
        self.R2 = np.eye(2)          # includes scale: aligned_xy = R2 @ vio_xy + t2
        self.t2 = np.zeros(2)
        self.dyaw = 0.0
        self.z_off = 0.0
        self.scale = 1.0

    def add_pair(self, vio_ned, ref_ned) -> bool:
        """Add one (vio, reference) NED position pair; fit when the arc is ready.
        Returns True the moment alignment is (newly) fitted."""
        if self.aligned:
            return False
        self.buf_vio.append(np.asarray(vio_ned, float).copy())
        self.buf_ref.append(np.asarray(ref_ned, float).copy())
        if self.ready():
            self._fit()
            return True
        return False

    def ready(self) -> bool:
        if len(self.buf_vio) <= self.min_pairs:
            return False
        v = np.array(self.buf_vio)[:, :2]
        span = v.max(0) - v.min(0)
        return bool(span.min() > self.min_span and span.max() > self.min_major_span)

    def _fit(self) -> None:
        """2-D similarity (scale s, yaw R, translation t) mapping the VIO track onto the
        reference NED track (Umeyama)."""
        V = np.array(self.buf_vio)
        P = np.array(self.buf_ref)
        v, p = V[:, :2], P[:, :2]
        vc, pc = v.mean(0), p.mean(0)
        vv, pp = v - vc, p - pc
        H = (vv.T @ pp) / len(v)
        U, S, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        R = Vt.T @ np.diag([1.0, d]) @ U.T               # 2x2 rotation vio -> reference
        var_v = (vv ** 2).sum() / len(v)
        s = float((S * np.array([1.0, d])).sum() / var_v)
        s = min(self.scale_limits[1], max(self.scale_limits[0], s))
        self.scale = s
        self.R2 = s * R
        self.t2 = pc - self.R2 @ vc
        self.dyaw = math.atan2(R[1, 0], R[0, 0])
        self.z_off = float((P[:, 2] - V[:, 2]).mean())
        self.aligned = True

    def apply(self, vio_ned, vio_yaw: float = 0.0):
        """Map a raw VIO NED position (+yaw) through the fitted alignment."""
        vio_ned = np.asarray(vio_ned, float)
        xy = self.R2 @ vio_ned[:2] + self.t2
        pos = np.array([xy[0], xy[1], vio_ned[2] + self.z_off])
        yaw = math.atan2(math.sin(vio_yaw + self.dyaw), math.cos(vio_yaw + self.dyaw))
        return pos, yaw
