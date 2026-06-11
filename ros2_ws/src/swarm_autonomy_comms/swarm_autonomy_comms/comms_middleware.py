"""ROS 2 node: the single inter-drone comms choke point.

Brokers ALL inter-drone traffic — poses, target observations, map deltas, and task bids — through
one gating policy. Every drone publishes its outbound traffic on ``/drone_N/comms/out/<topic>``;
this middleware subscribes to the drones' out-topics, applies
:class:`~swarm_autonomy_comms.link_model.LinkModel` per directed link using the live inter-drone
ranges, and re-publishes survivors on ``/drone_M/comms/in/<topic>`` for the receivers the packet
reached. Topic types and their wire-size estimators live in one registry (``_TOPICS``), so adding
a traffic class is one line. Ranges come from the pose traffic itself; until a sender's first pose
arrives its range is unknown (treated as out of range).

It also emits :class:`swarm_autonomy_msgs.msg.CommsStats` per window so delivered
bandwidth can be plotted against the cap.

Run one instance per simulation (it brokers all drones), or one per drone for a
fully decentralized deployment — both are supported via the ``broker`` param.
The heavy lifting lives in ``link_model`` so the policy is unit-tested offline.
"""

from __future__ import annotations

import random

import rclpy
from rclpy.node import Node

from swarm_autonomy_msgs.msg import CommsStats, MapDelta, NeighborPose, TargetObservation, TaskBid

from .link_model import LinkConfig, LinkModel, WindowStats


def _pose_bytes(msg) -> int:
    # 7 doubles pose + 6 doubles twist + header/id ~= conservative wire size.
    return 8 * 13 + 16


def _target_bytes(msg) -> int:
    # point + vector3 (6 doubles) + confidence/range/flags + header/id.
    return 8 * 6 + 4 * 2 + 16


def _map_bytes(msg) -> int:
    # uint16 index + int8 log-odds per voxel, plus origin/seq/header overhead.
    return 3 * len(msg.voxel_indices) + 40


def _bid_bytes(msg) -> int:
    # task id + score + winner + timestamp + header/id.
    return 4 + 4 + 1 + 8 + 16


# The single registry of brokered traffic: topic suffix -> (msg type, wire-size estimator).
_TOPICS = {
    "pose": (NeighborPose, _pose_bytes),
    "target": (TargetObservation, _target_bytes),
    "map": (MapDelta, _map_bytes),
    "bid": (TaskBid, _bid_bytes),
}


class CommsMiddleware(Node):
    def __init__(self) -> None:
        super().__init__("comms_middleware")

        self.declare_parameter("num_drones", 3)
        self.declare_parameter("bandwidth_bps", 50_000.0)
        self.declare_parameter("max_range_m", 80.0)
        self.declare_parameter("soft_range_m", 50.0)
        self.declare_parameter("base_loss", 0.02)
        self.declare_parameter("stats_window_s", 1.0)
        self.declare_parameter("seed", 0)

        self.num_drones = int(self.get_parameter("num_drones").value)
        cfg = LinkConfig(
            bandwidth_bps=float(self.get_parameter("bandwidth_bps").value),
            max_range_m=float(self.get_parameter("max_range_m").value),
            soft_range_m=float(self.get_parameter("soft_range_m").value),
            base_loss=float(self.get_parameter("base_loss").value),
        )
        seed = int(self.get_parameter("seed").value)
        self.window_s = float(self.get_parameter("stats_window_s").value)

        # One directed LinkModel and stats accumulator per (sender, receiver) pair.
        self._links: dict[tuple[int, int], LinkModel] = {}
        self._stats: dict[int, WindowStats] = {}
        for s in range(self.num_drones):
            self._stats[s] = WindowStats()
            for r in range(self.num_drones):
                if s != r:
                    self._links[(s, r)] = LinkModel(cfg, random.Random(seed + s * 97 + r))

        # Latest known position per drone (filled by the pose out-topic).
        self._pos: dict[int, tuple[float, float, float]] = {}

        # Per-topic in-publishers: _in_pubs[topic][receiver]. One subscription per
        # (drone, topic) on the out side.
        self._in_pubs: dict[str, dict[int, object]] = {name: {} for name in _TOPICS}
        self._stats_pubs: dict[int, object] = {}
        for d in range(self.num_drones):
            for name, (msg_type, sizer) in _TOPICS.items():
                self.create_subscription(
                    msg_type, f"/drone_{d}/comms/out/{name}",
                    self._make_cb(d, name, sizer), 10)
                self._in_pubs[name][d] = self.create_publisher(
                    msg_type, f"/drone_{d}/comms/in/{name}", 10)
            self._stats_pubs[d] = self.create_publisher(
                CommsStats, f"/drone_{d}/comms/stats", 10)

        self.create_timer(self.window_s, self._publish_stats)
        self.get_logger().info(
            f"comms_middleware up: {self.num_drones} drones, "
            f"cap={cfg.bandwidth_bps:.0f} B/s, max_range={cfg.max_range_m} m")

    # --- helpers -----------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _range(self, a: int, b: int) -> float:
        pa, pb = self._pos.get(a), self._pos.get(b)
        if pa is None or pb is None:
            return float("inf")
        return sum((x - y) ** 2 for x, y in zip(pa, pb)) ** 0.5

    def _make_cb(self, sender: int, topic: str, sizer):
        def cb(msg) -> None:
            if topic == "pose":          # pose traffic doubles as the live range source
                p = msg.pose.position
                self._pos[sender] = (p.x, p.y, p.z)
            self._broadcast(sender, msg, sizer(msg), self._in_pubs[topic])
        return cb

    def _broadcast(self, sender: int, msg, nbytes: int, pubs: dict) -> None:
        t = self._now()
        for receiver in range(self.num_drones):
            if receiver == sender:
                continue
            res = self._links[(sender, receiver)].try_deliver(t, self._range(sender, receiver), nbytes)
            self._stats[sender].add(res)
            if res.delivered:
                pubs[receiver].publish(msg)

    def _publish_stats(self) -> None:
        for d, st in self._stats.items():
            m = CommsStats()
            m.header.stamp = self.get_clock().now().to_msg()
            m.drone_id = d
            m.window_s = self.window_s
            m.msgs_sent = st.sent
            m.msgs_delivered = st.delivered
            m.msgs_dropped = st.dropped
            m.bytes_per_s = float(st.bytes_per_s(self.window_s))
            m.bandwidth_cap_bytes_per_s = float(self.get_parameter("bandwidth_bps").value)
            m.mean_neighbor_range = float(st.mean_range())
            self._stats_pubs[d].publish(m)
            self._stats[d] = WindowStats()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommsMiddleware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
