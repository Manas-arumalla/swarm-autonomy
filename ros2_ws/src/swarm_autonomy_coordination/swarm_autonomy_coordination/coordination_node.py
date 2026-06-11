"""ROS 2 node wrapping CBBA role allocation + pursuit goal generation.

Subscribes to the fused target estimate and neighbour poses (delivered through the
comms middleware), runs :mod:`cbba` to settle on scout/blocker/interceptor roles,
then turns the assigned role into a goal using :mod:`pursuit`, published for the
planner.

NOTE: the consensus message exchange over :class:`swarm_autonomy_msgs.msg.TaskBid` is
not yet wired to PX4/planner topics. The algorithmic cores it calls
(``cbba.run_cbba``, ``pursuit.*``) are fully implemented and unit-tested.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from swarm_autonomy_msgs.msg import DroneRole, TargetObservation

from . import pursuit
from .cbba import run_cbba


class CoordinationNode(Node):
    def __init__(self) -> None:
        super().__init__("coordination_node")
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("num_drones", 3)
        self.declare_parameter("containment_radius_m", 8.0)
        self.declare_parameter("pursuer_speed_mps", 3.0)
        self.declare_parameter("predict_horizon_s", 1.5)

        self.drone_id = int(self.get_parameter("drone_id").value)
        self.num_drones = int(self.get_parameter("num_drones").value)

        self.create_subscription(
            TargetObservation, "comms/in/target", self._on_target, 10)
        self.role_pub = self.create_publisher(DroneRole, "role", 10)
        self._latest_target: TargetObservation | None = None
        self.create_timer(0.5, self._tick)
        self.get_logger().info(f"coordination_node up for drone {self.drone_id}")

    def _on_target(self, msg: TargetObservation) -> None:
        if msg.detected:
            self._latest_target = msg

    def _tick(self) -> None:
        if self._latest_target is None:
            self._publish_role(DroneRole.ROLE_SCOUT, 0)
            return

        # One task per containment slot; score = -distance to that slot (DMG-safe
        # for the single-assignment case used here). Real node feeds live poses.
        tp = self._latest_target.position
        slots = pursuit.containment_ring(
            (tp.x, tp.y, tp.z),
            float(self.get_parameter("containment_radius_m").value),
            self.num_drones)

        def score_fn(agent, path, task):  # simple distance proxy
            return 1.0 / (1.0 + task + agent)

        assignment, _ = run_cbba(self.num_drones, len(slots), score_fn, max_bundle=1)
        my_tasks = assignment.get(self.drone_id, [])
        role = DroneRole.ROLE_INTERCEPTOR if my_tasks else DroneRole.ROLE_BLOCKER
        self._publish_role(role, my_tasks[0] if my_tasks else 0)

    def _publish_role(self, role: int, task_id: int) -> None:
        m = DroneRole()
        m.header.stamp = self.get_clock().now().to_msg()
        m.drone_id = self.drone_id
        m.role = role
        m.assigned_task_id = task_id
        self.role_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoordinationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
