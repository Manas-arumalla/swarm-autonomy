"""Unit tests for the VIO frame conversion + trajectory (Umeyama) alignment core."""
import math

import numpy as np

from swarm_autonomy_perception.vio_alignment import TrajectoryAligner, odom_to_ned


def _track(n=80, step=0.12):
    """An L-shaped reference track in NED (motion in both axes -> rotation observable)."""
    pts = []
    for k in range(n // 2):
        pts.append(np.array([step * k, 0.0, -5.0]))
    base = pts[-1]
    for k in range(n - n // 2):
        pts.append(base + np.array([0.0, step * k, 0.0]))
    return pts


def _vio_view(ref_pts, dyaw, scale, t, z_off):
    """Generate the VIO's view of the track: ref = s*R(dyaw) @ vio + t (so vio = inverse)."""
    R = np.array([[math.cos(dyaw), -math.sin(dyaw)], [math.sin(dyaw), math.cos(dyaw)]])
    out = []
    for p in ref_pts:
        vio_xy = np.linalg.inv(scale * R) @ (p[:2] - t)
        out.append(np.array([vio_xy[0], vio_xy[1], p[2] - z_off]))
    return out


def test_recovers_yaw_scale_translation():
    ref = _track()
    vio = _vio_view(ref, dyaw=math.radians(37.0), scale=1.3, t=np.array([5.0, -2.0]), z_off=1.4)
    al = TrajectoryAligner()
    for v, p in zip(vio, ref):
        al.add_pair(v, p)
    assert al.aligned
    assert abs(math.degrees(al.dyaw) - 37.0) < 0.5
    assert abs(al.scale - 1.3) < 0.01
    assert abs(al.z_off - 1.4) < 1e-6
    # applying the fit maps every VIO point back onto the reference
    for v, p in zip(vio, ref):
        pos, _ = al.apply(v)
        assert np.linalg.norm(pos - p) < 1e-6


def test_scale_clamp_guards_degenerate_fit():
    # Long track so the (shrunk-by-3) VIO view still spans a fittable arc.
    ref = _track(n=120, step=0.4)
    vio = _vio_view(ref, dyaw=0.0, scale=3.0, t=np.zeros(2), z_off=0.0)  # absurd scale
    al = TrajectoryAligner()
    for v, p in zip(vio, ref):
        al.add_pair(v, p)
    assert al.aligned
    assert al.scale <= 1.5 + 1e-9                       # clamped, not trusted


def test_straight_line_is_not_ready():
    # Motion along one axis only: yaw/scale unobservable -> the aligner must keep waiting.
    al = TrajectoryAligner()
    for k in range(120):
        p = np.array([0.1 * k, 0.0, -5.0])
        al.add_pair(p, p)
    assert not al.aligned


def test_odom_to_ned_frame_chain():
    # Identity attitude at ENU (1, 2, 3): NED position swaps x/y and flips z.
    ned, yaw = odom_to_ned(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    assert np.allclose(ned, [2.0, 1.0, -3.0])
    # ENU identity (facing East) is NED yaw +90 deg (FRD x toward East).
    assert abs(math.degrees(yaw) - 90.0) < 1e-6
    # Facing North in ENU (yaw +90 about ENU z) -> NED yaw 0.
    q = [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
    _, yaw_n = odom_to_ned(0.0, 0.0, 0.0, *q)
    assert abs(yaw_n) < 1e-6
