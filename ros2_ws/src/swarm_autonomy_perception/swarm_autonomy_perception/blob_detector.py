"""ROS-free colour-blob target detection + monocular ground back-projection.

The pure core behind both the ROS 2 ``target_detector`` node and the standalone Gazebo
camera script (``sim/scripts/vision_detect.py``): OpenCV HSV blob detection on a BGR frame,
and the tilt-correct pixel -> world ground-position back-projection (altitude prior: the
target is on the ground, z = 0). Pure/unit-tested like the other cores.

Back-projection: form the camera-optical ray for the detected pixel, rotate it through the
drone's full attitude (NED/FRD), the camera mount, and into world ENU, then intersect the
ground plane. The returned ``edge`` (normalised distance from the image centre) tells the
caller how ill-conditioned the fix is — off-nadir rays amplify attitude error into metres on
the ground, so consumers down-weight edge detections.
"""

from __future__ import annotations

import math

import numpy as np

try:
    import cv2
    _CV_OK = True
except Exception as _e:                      # pragma: no cover - import guard
    _CV_OK = False
    _CV_ERR = _e

from scipy.spatial.transform import Rotation as _Rot

# Camera-optical -> body mount rotation, FITTED from 130 in-flight samples to make the
# back-projection accurate (0.18 m RMS) at ANY tilt: a clean 90 deg about the optical Z axis.
# World transforms: MAVLink attitude is NED/FRD; v_enu = T @ v_ned.
R_BODY_CAM = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
T_NED_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])

# HSV range for a saturated orange/red target (the red hue wraps, hence two bands).
HSV_LO1, HSV_HI1 = (0, 120, 100), (15, 255, 255)
HSV_LO2, HSV_HI2 = (160, 120, 100), (180, 255, 255)


def detect_blob(bgr: np.ndarray, min_area: float = 40.0):
    """Find the largest saturated red/orange blob in a BGR frame.

    Returns ``(found, cx, cy, area)`` with the blob centroid in pixels."""
    if not _CV_OK:                            # pragma: no cover
        raise RuntimeError(f"OpenCV unavailable: {_CV_ERR}")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LO1, HSV_HI1) | cv2.inRange(hsv, HSV_LO2, HSV_HI2)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return (False, 0.0, 0.0, 0.0)
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area:
        return (False, 0.0, 0.0, 0.0)
    m = cv2.moments(c)
    return (True, m["m10"] / m["m00"], m["m01"] / m["m00"], float(area))


def back_project(cx: float, cy: float, width: int, height: int, hfov: float,
                 drone_east: float, drone_north: float, alt: float,
                 yaw: float = 0.0, roll: float = 0.0, pitch: float = 0.0):
    """Back-project an image pixel to a world ground position (tilt-correct, z=0 prior).

    Returns ``(ok, East, North, edge)``; ``ok`` is False when the pixel's ray does not point
    down toward the ground (e.g. extreme tilt). ``edge`` in [0, ~1.4] is the normalised
    off-centre distance — the fix's conditioning metric."""
    vfov = 2 * math.atan(math.tan(hfov / 2) * height / width)
    nx = (cx - width / 2) / (width / 2)
    ny = (cy - height / 2) / (height / 2)
    edge = math.hypot(nx, ny)
    d_cam = np.array([nx * math.tan(hfov / 2), ny * math.tan(vfov / 2), 1.0])
    d_cam /= np.linalg.norm(d_cam)
    r_ned_body = _Rot.from_euler("ZYX", [yaw, pitch, roll]).as_matrix()
    d_enu = T_NED_ENU @ (r_ned_body @ (R_BODY_CAM @ d_cam))
    if d_enu[2] >= -1e-3:                     # ray must point down to hit the ground
        return (False, 0.0, 0.0, edge)
    s = alt / (-d_enu[2])
    return (True, drone_east + s * d_enu[0], drone_north + s * d_enu[1], edge)
