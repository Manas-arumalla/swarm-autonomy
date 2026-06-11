"""Onboard target detector -> shared target observation.

Detects the fleeing target in the drone's camera stream (HSV blob backend) and emits a
:class:`swarm_autonomy_msgs.msg.TargetObservation` — ground position in the map frame, a
conditioning-aware confidence, and range — which the comms layer shares for cooperative fusion.
The detection and tilt-correct back-projection math is the pure, unit-tested
:mod:`swarm_autonomy_perception.blob_detector` core, shared verbatim with the standalone Gazebo
demo (``sim/scripts/vision_detect.py``). The drone's own pose comes from its outbound
``comms/out/pose`` traffic; ``fiducial``/``learned`` backends are planned.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from swarm_autonomy_msgs.msg import NeighborPose, TargetObservation

from .blob_detector import back_project, detect_blob


def _quat_to_euler_zyx(x: float, y: float, z: float, w: float):
    """Quaternion -> (yaw, pitch, roll), ZYX convention."""
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    sp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sp)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return yaw, pitch, roll


class TargetDetector(Node):
    """Blob backend: HSV blob detection + tilt-correct ground back-projection (the same pure
    core as the Gazebo demo, :mod:`swarm_autonomy_perception.blob_detector`). Needs the drone's
    own pose, taken from its outbound ``comms/out/pose`` traffic."""

    def __init__(self) -> None:
        super().__init__("target_detector")
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("camera_topic", "camera/image_raw")
        self.declare_parameter("detector", "blob")  # blob | fiducial | learned
        self.declare_parameter("hfov_rad", 1.74)
        self.declare_parameter("min_blob_area_px", 40.0)

        self.drone_id = int(self.get_parameter("drone_id").value)
        self.hfov = float(self.get_parameter("hfov_rad").value)
        self.min_area = float(self.get_parameter("min_blob_area_px").value)
        self._pose = None                      # (E, N, alt, yaw, pitch, roll)
        self.create_subscription(
            Image, self.get_parameter("camera_topic").value, self._on_image, 5)
        self.create_subscription(NeighborPose, "comms/out/pose", self._on_pose, 10)
        self._pub = self.create_publisher(TargetObservation, "comms/out/target", 10)
        self.get_logger().info(
            f"target_detector up (drone {self.drone_id}, "
            f"backend={self.get_parameter('detector').value})")

    def _on_pose(self, msg: NeighborPose) -> None:
        p, q = msg.pose.position, msg.pose.orientation
        yaw, pitch, roll = _quat_to_euler_zyx(q.x, q.y, q.z, q.w)
        self._pose = (p.x, p.y, p.z, yaw, pitch, roll)

    @staticmethod
    def _to_bgr(msg: Image) -> np.ndarray | None:
        if msg.encoding not in ("rgb8", "bgr8"):
            return None
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        row = np.reshape(buf[: msg.height * msg.step], (msg.height, msg.step))
        img = row[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if msg.encoding == "rgb8" else img.copy()

    def _on_image(self, msg: Image) -> None:
        obs = TargetObservation()
        obs.header = msg.header
        obs.drone_id = self.drone_id
        obs.detected = False
        obs.confidence = 0.0
        frame = self._to_bgr(msg)
        if frame is not None and self._pose is not None:
            found, cx, cy, area = detect_blob(frame, min_area=self.min_area)
            if found:
                e0, n0, alt, yaw, pitch, roll = self._pose
                ok, te, tn, edge = back_project(
                    cx, cy, msg.width, msg.height, self.hfov,
                    e0, n0, alt, yaw=yaw, roll=roll, pitch=pitch)
                if ok:
                    obs.detected = True
                    obs.position.x, obs.position.y, obs.position.z = float(te), float(tn), 0.0
                    # near-nadir fixes are well-conditioned; edge fixes are not
                    obs.confidence = float(max(0.0, 1.0 - 0.6 * edge))
                    obs.range = float(math.hypot(te - e0, tn - n0, alt))
        self._pub.publish(obs)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
