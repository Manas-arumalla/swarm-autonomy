"""PX4 offboard controller: takeoff/hover bringup + PID waypoint following.

Streams ``OffboardControlMode`` + ``TrajectorySetpoint`` at >2 Hz and arms (retrying through the
EKF warm-up). With no trajectory it holds a position setpoint at the takeoff altitude (PX4's own
controller closes that loop). When the planner publishes a route (``nav_msgs/Path`` on
``trajectory``, world ENU — see ``planner_node``), the node walks the waypoints: per-axis
:class:`~swarm_autonomy_control.pid.PID` on world-frame position error produces a velocity
command, converted to the NED setpoint fields by the unit-tested
:mod:`~swarm_autonomy_control.px4_frames` helpers. ``spawn_east``/``spawn_north`` give the
drone's world spawn (PX4's local-NED origin), mapping local NED into the shared map frame.

NOTE: the loop logic is built from tested cores (PID, px4_frames), but this node has not yet been
re-validated in live flight end-to-end; the proven flight path remains ``sim/scripts/*``.

Namespacing: instantiated once per drone under ``/drone_N`` so PX4 topics resolve
to that vehicle's uXRCE-DDS client (``fmu/in/*``, ``fmu/out/*``). Requires the
``px4_msgs`` overlay; in its absence the node logs and idles so the launch graph
still comes up.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from .pid import PID
from .px4_frames import ned_to_world_pos, world_to_ned_vel

try:
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleLocalPosition,
        VehicleStatus,
    )
    _HAVE_PX4 = True
except ImportError:
    OffboardControlMode = TrajectorySetpoint = VehicleCommand = None
    VehicleLocalPosition = VehicleStatus = None
    _HAVE_PX4 = False


class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__("offboard_control")
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("takeoff_alt_m", 2.5)
        self.declare_parameter("kp_xy", 1.2)
        self.declare_parameter("kp_z", 1.5)
        self.declare_parameter("spawn_east", 0.0)    # world position of PX4's local-NED origin
        self.declare_parameter("spawn_north", 0.0)
        self.declare_parameter("waypoint_tol_m", 1.0)

        self.alt = float(self.get_parameter("takeoff_alt_m").value)
        self.spawn = (float(self.get_parameter("spawn_east").value),
                      float(self.get_parameter("spawn_north").value))
        self.wp_tol = float(self.get_parameter("waypoint_tol_m").value)
        kxy = float(self.get_parameter("kp_xy").value)
        kz = float(self.get_parameter("kp_z").value)
        self.pid = {
            "x": PID(kxy, out_min=-5, out_max=5),    # world East
            "y": PID(kxy, out_min=-5, out_max=5),    # world North
            "z": PID(kz, out_min=-3, out_max=3),     # world Up
        }
        self._route: list[tuple[float, float, float]] = []   # world ENU waypoints
        self._wp_i = 0
        self._pos = None                                       # world (E, N, Up)

        if not _HAVE_PX4:
            self.get_logger().warn(
                "px4_msgs not found — offboard_control idling. "
                "Install the overlay via scripts/setup.sh.")
            return

        self._mode_pub = self.create_publisher(
            OffboardControlMode, "fmu/in/offboard_control_mode", 10)
        self._sp_pub = self.create_publisher(
            TrajectorySetpoint, "fmu/in/trajectory_setpoint", 10)
        self._cmd_pub = self.create_publisher(
            VehicleCommand, "fmu/in/vehicle_command", 10)

        # PX4 publishes /fmu/out/* as BEST_EFFORT — a reliable subscriber won't
        # match it, so mirror PX4's sensor-data QoS here.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)
        self._status_sub = self.create_subscription(
            VehicleStatus, "fmu/out/vehicle_status_v4", self._on_status, px4_qos)
        self.create_subscription(
            VehicleLocalPosition, "fmu/out/vehicle_local_position", self._on_local, px4_qos)
        self.create_subscription(Path, "trajectory", self._on_route, 10)
        self._arming_state = 0
        self._nav_state = 0

        self._counter = 0
        self.create_timer(0.1, self._tick)  # 10 Hz offboard heartbeat
        self.get_logger().info("offboard_control streaming; will arm once EKF is ready")

    def _on_status(self, msg) -> None:
        self._arming_state = msg.arming_state
        self._nav_state = msg.nav_state

    def _on_local(self, msg) -> None:
        e, n = ned_to_world_pos(msg.x, msg.y, spawn_east=self.spawn[0],
                                spawn_north=self.spawn[1])
        self._pos = (e, n, -float(msg.z))                       # world (E, N, Up)

    def _on_route(self, msg: Path) -> None:
        """Accept a planner route (world ENU); restart waypoint walking from its head."""
        self._route = [(p.pose.position.x, p.pose.position.y,
                        p.pose.position.z if p.pose.position.z > 0.1 else self.alt)
                       for p in msg.poses]
        self._wp_i = 0

    def _current_waypoint(self):
        """First not-yet-reached waypoint of the active route, advancing as we arrive."""
        while self._wp_i < len(self._route) and self._pos is not None:
            wx, wy, _ = self._route[self._wp_i]
            if math.hypot(wx - self._pos[0], wy - self._pos[1]) > self.wp_tol:
                return self._route[self._wp_i]
            self._wp_i += 1
        return None

    def _tick(self) -> None:
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        wp = self._current_waypoint()
        if wp is not None and self._pos is not None:
            # Waypoint following: world-frame PID -> velocity setpoint (NED via px4_frames).
            v_e = self.pid["x"].step(wp[0] - self._pos[0], 0.1)
            v_n = self.pid["y"].step(wp[1] - self._pos[1], 0.1)
            v_up = self.pid["z"].step(wp[2] - self._pos[2], 0.1)
            nan = float("nan")
            sp.position = [nan, nan, nan]
            sp.velocity = list(world_to_ned_vel(v_e, v_n, v_up))
            self._publish_mode(velocity=True)
        else:
            # Bringup / route finished: hold a position setpoint at the takeoff altitude.
            sp.position = [0.0, 0.0, -self.alt]  # NED: negative z is up
            self._publish_mode(velocity=False)
        self._sp_pub.publish(sp)

        # Retry offboard + arm once a second until PX4 reports ARMED. This rides
        # out the EKF/preflight warm-up instead of giving up after one attempt.
        armed = self._arming_state == VehicleStatus.ARMING_STATE_ARMED
        if not armed and self._counter % 10 == 0 and self._counter >= 10:
            self._set_offboard_and_arm()
        self._counter += 1

    def _publish_mode(self, velocity: bool = False) -> None:
        m = OffboardControlMode()
        m.position = not velocity
        m.velocity = velocity
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._mode_pub.publish(m)

    def _set_offboard_and_arm(self) -> None:
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # offboard
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def _send_cmd(self, command: int, p1: float = 0.0, p2: float = 0.0) -> None:
        c = VehicleCommand()
        c.command = command
        c.param1 = p1
        c.param2 = p2
        c.target_system = 1
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        c.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._cmd_pub.publish(c)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
