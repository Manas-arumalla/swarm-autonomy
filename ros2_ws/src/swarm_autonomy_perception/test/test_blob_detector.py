"""Unit tests for the pure blob-detection + ground back-projection core."""
import math

import numpy as np

from swarm_autonomy_perception.blob_detector import back_project, detect_blob


def _frame_with_red_square(w=640, h=480, cx=420, cy=130, half=12):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, :] = (90, 90, 90)                                   # grey ground
    img[cy - half:cy + half, cx - half:cx + half] = (0, 0, 255)  # BGR red square
    return img


def test_detects_red_square_centroid():
    found, cx, cy, area = detect_blob(_frame_with_red_square())
    assert found
    assert abs(cx - 420) < 2 and abs(cy - 130) < 2             # centroid on the square
    assert area > 200


def test_ignores_blank_and_small_blobs():
    blank = np.full((480, 640, 3), 90, dtype=np.uint8)
    assert detect_blob(blank)[0] is False
    tiny = _frame_with_red_square(cx=320, cy=240, half=2)       # 4x4 px < min_area
    assert detect_blob(tiny, min_area=40)[0] is False


def test_nadir_pixel_back_projects_to_directly_below():
    # Image centre, level flight -> the ground point straight under the drone.
    ok, e, n, edge = back_project(640, 480, 1280, 960, hfov=1.74,
                                  drone_east=5.0, drone_north=-3.0, alt=20.0)
    assert ok
    assert math.hypot(e - 5.0, n - (-3.0)) < 1e-6
    assert edge < 1e-9


def test_pitch_shifts_ground_point_by_h_tan_pitch():
    # A 10 deg pitch at 20 m must displace the nadir-pixel ground fix by h*tan(10deg) ~ 3.53 m
    # (the identity verified against the live Gazebo calibration).
    ok, e, n, _ = back_project(640, 480, 1280, 960, hfov=1.74,
                               drone_east=0.0, drone_north=0.0, alt=20.0,
                               pitch=math.radians(10.0))
    assert ok
    assert abs(math.hypot(e, n) - 20.0 * math.tan(math.radians(10.0))) < 0.05


def test_upward_ray_is_rejected():
    # A pixel at the very top of the image with the drone pitched far up -> ray leaves the
    # ground plane; the fix must be rejected, not extrapolated to a huge bogus position.
    ok, _, _, _ = back_project(640, 0, 1280, 960, hfov=1.74,
                               drone_east=0.0, drone_north=0.0, alt=20.0,
                               pitch=math.radians(60.0))
    assert ok is False
