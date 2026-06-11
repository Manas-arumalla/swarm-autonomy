"""Planner bringup shim.

Swarm Autonomy plans by integrating proven systems:

* **ego-planner** for single-drone nav to goal through unknown clutter;
  **FUEL** for single-drone exploration.
* **ego-planner-swarm** for decentralized inter-agent avoidance, exchanging
  trajectories through the comms middleware.
* **RACER** for cooperative multi-UAV exploration + map sharing.

Those packages are vendored under ``ros2_ws/src/third_party`` by
``scripts/setup.sh`` and launched with Swarm Autonomy namespacing/params. This node is
the thin Swarm Autonomy-side shim that:

* accepts a goal / role-conditioned goal (from coordination),
* selects the active planner profile (nav / explore / coop-explore),
* relays the chosen planner's trajectory to the controller.

The shim keeps the launch graph complete regardless of which planner backend is
active; the heavy planners drop in behind it without touching the rest of the stack.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# CPU planning fallback (ego-planner role without the GPU/ROS1 build). Guarded like the
# px4_msgs imports so the launch graph still comes up if numpy/scipy are absent.
try:
    from swarm_autonomy_mapping.esdf import GridMap
    from swarm_autonomy_planning.planner import plan
    _HAVE_CPU_PLANNER = True
except ImportError:  # pragma: no cover
    _HAVE_CPU_PLANNER = False


def _parse_buildings(spec: str):
    out = []
    for tok in spec.split(";"):
        tok = tok.strip()
        if tok:
            x, y = tok.split(",")
            out.append((float(x), float(y)))
    return out


class PlannerNode(Node):
    PROFILES = ("nav", "explore", "coop_explore")

    def __init__(self) -> None:
        super().__init__("planner_node")
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("profile", "nav")
        # CPU-fallback map params (mirror map_merge_node) so 'nav' can route headless.
        self.declare_parameter("local_buildings", "")
        self.declare_parameter("map_half_extent_m", 16.0)
        self.declare_parameter("building_half_m", 3.5)
        self.declare_parameter("voxel_size_m", 0.4)
        self.declare_parameter("clearance_m", 1.3)

        profile = self.get_parameter("profile").value
        if profile not in self.PROFILES:
            self.get_logger().warn(f"unknown profile '{profile}', defaulting to 'nav'")
            profile = "nav"
        self.profile = profile
        self.d_safe = float(self.get_parameter("clearance_m").value)
        self._pos = (0.0, 0.0)         # last-known position (front of the plan)

        self._grid = None
        if _HAVE_CPU_PLANNER:
            ext = float(self.get_parameter("map_half_extent_m").value)
            self._grid = GridMap((-ext, -ext), (ext, ext),
                                 res=float(self.get_parameter("voxel_size_m").value))
            self._grid.add_buildings(
                _parse_buildings(self.get_parameter("local_buildings").value),
                float(self.get_parameter("building_half_m").value))

        self.create_subscription(PoseStamped, "goal", self._on_goal, 10)
        self.create_subscription(PoseStamped, "pose", self._on_pose, 10)
        self._traj = self.create_publisher(Path, "trajectory", 10)
        self.get_logger().info(
            f"planner_node up (drone {self.get_parameter('drone_id').value}, "
            f"profile={self.profile}) — CPU planner={'on' if self._grid else 'off'}")

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pos = (msg.pose.position.x, msg.pose.position.y)

    def _on_goal(self, msg: PoseStamped) -> None:
        goal = (msg.pose.position.x, msg.pose.position.y)
        # 'explore'/'coop_explore' map to FUEL/RACER (vendored under third_party); 'nav' uses
        # the CPU ego-planner fallback to route around buildings on the local ESDF.
        if self.profile != "nav" or self._grid is None:
            self.get_logger().debug(f"goal received: {goal} (profile={self.profile})")
            return
        path = plan(self._grid, self._pos, goal, d_safe=self.d_safe)
        if path is None:
            self.get_logger().warn(f"no collision-free route {self._pos} -> {goal}")
            return
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        for x, y in path:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(msg.pose.position.z)
            out.poses.append(ps)
        self._traj.publish(out)
        self.get_logger().info(
            f"routed -> {goal}: {len(out.poses)} waypoints, "
            f"len~{sum(((path[i][0]-path[i-1][0])**2+(path[i][1]-path[i-1][1])**2)**0.5 for i in range(1, len(path))):.1f} m")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
