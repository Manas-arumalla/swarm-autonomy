"""VIO -> PX4 EKF2 external-vision bridge (MAVLink VISION_POSITION_ESTIMATE), with
one-shot frame ALIGNMENT to PX4's local NED and PX4-clock timestamping.

OpenVINS publishes /odomimu in its own gravity-aligned frame whose yaw/origin are
arbitrary (set at VIO init) and do NOT match PX4's NED. Two things must be fixed or
EKF2 silently ignores the data:
  1. FRAME: after a short warmup we capture the rigid offset (yaw + translation)
     between raw-VIO-NED and PX4's own NED (read from /tmp/px4_pose.txt, written by the
     flight node) and apply it, so aligned-vision tracks PX4 exactly.
  2. TIME: PX4 SITL runs on lockstep sim time; we stamp each message with PX4's clock
     using the wall->PX4 offset shared in /tmp/px4_pose.txt. (Wall-clock stamps look
     decades in the future to EKF2, which then drops every measurement.)

Sends to PX4's offboard-local port 14580 (PX4 receives there); the offboard controller
owns 14540, so there is no socket collision.
"""
import math, os, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as Rot
from pymavlink import mavutil

M_W = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)   # world ENU->NED
M_B = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)   # body FLU->FRD


def read_px4_pose():
    try:
        with open("/tmp/px4_pose.txt") as f:
            n, e, d, yaw, toff = (float(x) for x in f.read().split())
        return np.array([n, e, d]), yaw, toff
    except (OSError, ValueError):
        return None, None, 0.0


class VioToPx4(Node):
    def __init__(self):
        super().__init__("vio_to_px4")
        self.mav = mavutil.mavlink_connection("udpout:127.0.0.1:14580", source_system=1, source_component=197)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, "/odomimu", self._on_odom, qos)
        self.warm = 0
        self.aligned = False
        self.R2 = np.eye(2); self.dyaw = 0.0; self.z_off = 0.0
        self.t2 = np.zeros(2)
        self.buf_vio = []; self.buf_px = []     # trajectory pairs for the rigid fit
        self.n = 0; self.last_send = 0.0
        self.get_logger().info("VIO->PX4 bridge up; collecting an arc for trajectory alignment...")

    def _fit_alignment(self):
        """2D similarity (scale s, yaw R, translation t) that best maps the VIO track onto
        PX4's NED track, via Umeyama over the collected arc. Using the trajectory SHAPE (not
        a single instantaneous attitude) makes dyaw robust to the yaw wobble and absorbs any
        residual stereo-scale error in one shot."""
        V = np.array(self.buf_vio); P = np.array(self.buf_px)
        v = V[:, :2]; p = P[:, :2]
        vc = v.mean(0); pc = p.mean(0)
        vv = v - vc; pp = p - pc
        H = (vv.T @ pp) / len(v)
        U, S, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        R = Vt.T @ np.diag([1.0, d]) @ U.T              # 2x2 rotation vio->px
        var_v = (vv ** 2).sum() / len(v)
        s = float((S * np.array([1.0, d])).sum() / var_v)
        s = min(1.5, max(0.6, s))                       # guard against a degenerate fit
        self.R2 = s * R
        self.t2 = pc - self.R2 @ vc
        self.dyaw = math.atan2(R[1, 0], R[0, 0])
        self.z_off = float((P[:, 2] - V[:, 2]).mean())
        self.aligned = True
        self.get_logger().info(
            f"ALIGNED (trajectory fit, N={len(v)}): dyaw={math.degrees(self.dyaw):+.1f}deg "
            f"scale={s:.3f}  t=({self.t2[0]:+.2f},{self.t2[1]:+.2f})  z_off={self.z_off:+.2f}")

    def _raw_ned(self, msg):
        p = msg.pose.pose.position; q = msg.pose.pose.orientation
        ned = M_W @ np.array([p.x, p.y, p.z])
        R_ned_frd = M_W @ Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix() @ M_B
        yaw, pitch, roll = Rot.from_matrix(R_ned_frd).as_euler("ZYX")
        return ned, yaw

    def _on_odom(self, msg):
        ned, yaw = self._raw_ned(msg)

        if not self.aligned:
            px_pos, px_yaw, _ = read_px4_pose()
            self.warm += 1
            # let VIO settle, then collect (vio,px) pairs until they span a real ARC (motion
            # in both axes) so the rotation+scale are well-observed, not fit to a straight line.
            if self.warm > 30 and px_pos is not None:
                self.buf_vio.append(ned.copy()); self.buf_px.append(px_pos.copy())
                V = np.array(self.buf_vio)[:, :2]
                span = V.max(0) - V.min(0)
                if len(self.buf_vio) > 40 and span.min() > 2.5 and span.max() > 4.0:
                    self._fit_alignment()
            return

        xy = self.R2 @ ned[:2] + self.t2
        a_pos = np.array([xy[0], xy[1], ned[2] + self.z_off])
        a_yaw = math.atan2(math.sin(yaw + self.dyaw), math.cos(yaw + self.dyaw))

        now = time.time()
        if now - self.last_send < 0.033:       # throttle to ~30 Hz
            return
        self.last_send = now
        # Stamp with the odometry's ACTUAL capture time, not "now". gz and PX4 share the same
        # lockstep sim clock, and OpenVINS carries the camera image time in the header, so this
        # timestamp is already in PX4's clock. Using "now" made EKF2 treat ~150ms-old vision as
        # current -> unstable control -> runaway when GPS was removed.
        st_ = msg.header.stamp
        usec = int(st_.sec * 1_000_000 + st_.nanosec // 1000)
        self.mav.mav.vision_position_estimate_send(
            usec, float(a_pos[0]), float(a_pos[1]), float(a_pos[2]), 0.0, 0.0, float(a_yaw))
        self.n += 1
        if self.n % 30 == 0:
            px_pos, _, _ = read_px4_pose()
            err = np.linalg.norm(a_pos[:2] - px_pos[:2]) if px_pos is not None else float("nan")
            self.get_logger().info(
                f"vis #{self.n} alignedNED=({a_pos[0]:+.2f},{a_pos[1]:+.2f},{a_pos[2]:+.2f}) "
                f"yaw={math.degrees(a_yaw):+.0f}  |err_vs_PX4|={err:.2f}m")


def main():
    rclpy.init()
    node = VioToPx4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
