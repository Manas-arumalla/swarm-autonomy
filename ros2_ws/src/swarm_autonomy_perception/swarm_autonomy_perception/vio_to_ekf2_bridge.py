"""VIO -> PX4 EKF2 external-vision bridge.

OpenVINS publishes odometry in its own gravity-aligned frame; PX4 EKF2 consumes external vision
(with ``EKF2_GPS_CTRL=0``) to fly GPS-denied. This node applies the three hard-won lessons from
the validated live handover (0.26 m mean / 0.70 m max — see ``docs/engineering-notes.md`` and the
standalone ``sim/scripts/vio_to_px4.py``):

1. **Frame**: VIO yaw/origin are arbitrary, so after a short warm-up the bridge collects
   (vio, EKF2) position pairs while the vehicle flies an arc and fits the 2-D similarity
   (yaw + scale + translation) mapping VIO onto PX4's local NED — the pure, unit-tested
   :class:`~swarm_autonomy_perception.vio_alignment.TrajectoryAligner`.
2. **Time**: messages are stamped with the odometry header's capture time (PX4 SITL and the
   camera share the lockstep sim clock). Stamping with "now" makes EKF2 treat ~150 ms-old vision
   as current (unstable control), and wall-clock stamps look decades in the future (every
   measurement dropped).
3. **Position-only fusion**: orientation/velocity are marked invalid (NaN) — fusing EV yaw
   rotated the frame into a runaway in the live debugging.

The PX4 message import is the only dependency requiring the ``px4_msgs`` overlay (installed by
``scripts/setup.sh``); without it the node idles in dry mode so the launch graph still comes up.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from .vio_alignment import TrajectoryAligner, odom_to_ned

try:
    from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry  # px4_msgs overlay
    _HAVE_PX4 = True
except ImportError:  # allow build/import without the PX4 overlay present
    VehicleLocalPosition = None
    VehicleOdometry = None
    _HAVE_PX4 = False


class VioToEkf2Bridge(Node):
    def __init__(self) -> None:
        super().__init__("vio_to_ekf2_bridge")
        self.declare_parameter("vio_odom_topic", "odometry")    # OpenVINS output
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("warmup_msgs", 30)               # let VIO settle before pairing
        self.declare_parameter("position_std_m", 0.3)           # EV position std after alignment

        self._aligner = TrajectoryAligner()
        self._warm = 0
        self._ref_ned = None                                     # latest EKF2 local position
        self._min_period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self._last_pub = -1.0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            Odometry, self.get_parameter("vio_odom_topic").value, self._on_odom, qos)
        if _HAVE_PX4:
            self.create_subscription(
                VehicleLocalPosition, "fmu/out/vehicle_local_position", self._on_local, qos)
            self._pub = self.create_publisher(
                VehicleOdometry, "fmu/in/vehicle_visual_odometry", 10)
            self.get_logger().info("vio_to_ekf2_bridge up; collecting an arc for alignment...")
        else:
            self._pub = None
            self.get_logger().warn(
                "px4_msgs not found — bridge runs in dry mode. "
                "Install the overlay via scripts/setup.sh.")

    def _on_local(self, msg) -> None:
        self._ref_ned = (float(msg.x), float(msg.y), float(msg.z))

    def _on_odom(self, msg: Odometry) -> None:
        if self._pub is None:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        vio_ned, vio_yaw = odom_to_ned(p.x, p.y, p.z, q.x, q.y, q.z, q.w)

        if not self._aligner.aligned:
            self._warm += 1
            if self._warm > int(self.get_parameter("warmup_msgs").value) \
                    and self._ref_ned is not None:
                if self._aligner.add_pair(vio_ned, self._ref_ned):
                    a = self._aligner
                    self.get_logger().info(
                        f"ALIGNED (trajectory fit, N={len(a.buf_vio)}): "
                        f"dyaw={math.degrees(a.dyaw):+.1f}deg scale={a.scale:.3f} "
                        f"t=({a.t2[0]:+.2f},{a.t2[1]:+.2f}) z_off={a.z_off:+.2f}")
            return

        # Throttle on the message's own capture time (sim clock), not the wall clock.
        t_cap = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t_cap - self._last_pub < self._min_period:
            return
        self._last_pub = t_cap

        pos, _yaw = self._aligner.apply(vio_ned, vio_yaw)
        std = float(self.get_parameter("position_std_m").value)
        nan = float("nan")
        out = VehicleOdometry()
        usec = int(msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000)
        out.timestamp = usec
        out.timestamp_sample = usec
        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out.position = [float(pos[0]), float(pos[1]), float(pos[2])]
        out.q = [nan, nan, nan, nan]                 # orientation NOT fused (EV-yaw runaway)
        out.velocity = [nan, nan, nan]               # position-only EV
        out.position_variance = [std * std] * 3
        out.orientation_variance = [nan, nan, nan]
        out.velocity_variance = [nan, nan, nan]
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VioToEkf2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
