"""Image-based target detection for the Swarm Autonomy swarm.

Subscribes to a Gazebo downward camera over gz-transport, runs OpenCV colour-blob
detection on the actual rendered frames, and back-projects the detected pixel to a
world ground position. Pursuers locate the target from their own cameras rather
than ground-truth state: a pursuer only "sees" the target when it is genuinely in
its camera's field of view and the detector fires on the image.

Monocular back-projection uses the altitude prior (the target is on the ground,
z=0): for a downward camera on a roughly-level drone at height h, a pixel offset
from the image centre maps to a ground offset of h*tan(angle). Yaw is applied so
the image axes rotate into the world frame.
"""

from __future__ import annotations

import math
import os
import sys
import threading

import numpy as np

# The detection/back-projection math is the shared, unit-tested pure core in the perception
# package; this file owns only the gz-transport camera shell around it.
sys.path.insert(0, os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "ros2_ws", "src",
    "swarm_autonomy_perception")))

try:
    import cv2
    from gz.transport13 import Node as GzNode
    from gz.msgs10.image_pb2 import Image as GzImage
    from swarm_autonomy_perception.blob_detector import back_project, detect_blob
    _OK = True
except Exception as _e:        # pragma: no cover - import guard for tooling checks
    _OK = False
    _IMPORT_ERR = _e


# gz Image pixel_format_type values we handle (gz.msgs.PixelFormatType).
_RGB_INT8 = 3
_BGR_INT8 = 6


def _to_bgr(msg) -> np.ndarray | None:
    """Decode a gz.msgs.Image into an OpenCV BGR ndarray."""
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if buf.size < h * w * 3:
        return None
    img = buf[: h * w * 3].reshape(h, w, 3)
    if msg.pixel_format_type == _BGR_INT8:
        return img.copy()
    # default / RGB_INT8 -> convert to BGR for OpenCV
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


class CameraDetector:
    """Subscribes to one camera topic and detects an orange/red target blob."""

    MIN_AREA = 40        # px; reject noise

    def __init__(self, topic: str, hfov: float = 1.74, width: int = 1280, height: int = 960):
        if not _OK:
            raise RuntimeError(f"vision deps unavailable: {_IMPORT_ERR}")
        self.topic = topic
        self.hfov = hfov
        self.W, self.H = width, height
        self.vfov = 2 * math.atan(math.tan(hfov / 2) * height / width)
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._node = GzNode()
        self._node.subscribe(GzImage, topic, self._on_image)

    def _on_image(self, msg) -> None:
        img = _to_bgr(msg)
        if img is not None:
            with self._lock:
                self.W, self.H = msg.width, msg.height
                self._frame = img

    def latest(self) -> np.ndarray | None:
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def detect_pixel(self):
        """Return (found, cx, cy, area) of the largest target blob, or (False,...)."""
        frame = self.latest()
        if frame is None:
            return (False, 0, 0, 0)
        return detect_blob(frame, min_area=self.MIN_AREA)

    def detect_world(self, drone_E: float, drone_N: float, alt: float,
                     yaw: float = 0.0, roll: float = 0.0, pitch: float = 0.0):
        """Detect the target and back-project to a world ground position.

        Returns (found, East, North, edge); see
        :func:`swarm_autonomy_perception.blob_detector.back_project` for the geometry and the
        meaning of ``edge`` (the fix's conditioning metric — callers downweight edge fixes).
        """
        found, cx, cy, _area = self.detect_pixel()
        if not found:
            return (False, 0.0, 0.0, 1.0)
        return back_project(cx, cy, self.W, self.H, self.hfov,
                            drone_E, drone_N, alt, yaw=yaw, roll=roll, pitch=pitch)
