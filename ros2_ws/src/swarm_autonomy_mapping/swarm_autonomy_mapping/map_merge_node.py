"""Shared-map merge node.

Each drone builds a local ESDF/occupancy map (nvblox on GPU, or an ego-planner
grid-map fallback — see the GPU note in the README). This node:

* emits the drone's local map as compact :class:`swarm_autonomy_msgs.msg.MapDelta`
  blocks on ``comms/out/map`` (so the comms middleware can meter bandwidth), and
* merges neighbour deltas arriving on ``comms/in/map`` into the shared map by
  log-odds fusion.

The delta-merge bookkeeping and sequence/dedup handling are owned here.
``merge_log_odds`` is pure and unit-tested.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from swarm_autonomy_msgs.msg import MapDelta

from .esdf import GridMap
from .grid_io import apply_delta_to_grid, occupancy_to_delta
from .merge import merge_log_odds


def _parse_buildings(spec: str):
    """Parse a "cx,cy;cx,cy;..." param into [(cx, cy), ...]. Empty → []."""
    out = []
    for tok in spec.split(";"):
        tok = tok.strip()
        if not tok:
            continue
        x, y = tok.split(",")
        out.append((float(x), float(y)))
    return out


class MapMergeNode(Node):
    def __init__(self) -> None:
        super().__init__("map_merge_node")
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("voxel_size_m", 0.4)
        # The local map this drone has built. In the full stack this is filled from
        # nvblox / the depth sensor; the CPU fallback seeds it from a known-building list so
        # the occupancy -> delta -> comms -> merge path runs headless without a GPU.
        self.declare_parameter("local_buildings", "")
        self.declare_parameter("map_half_extent_m", 16.0)
        self.declare_parameter("building_half_m", 3.5)

        self.drone_id = int(self.get_parameter("drone_id").value)
        self._last_seq: dict[int, int] = {}  # per-drone dedup of incoming deltas
        self._merged: dict[int, int] = {}     # flattened voxel index -> log-odds
        self._seq = 0

        res = float(self.get_parameter("voxel_size_m").value)
        ext = float(self.get_parameter("map_half_extent_m").value)
        self._local = GridMap((-ext, -ext), (ext, ext), res=res)
        self._local.add_buildings(
            _parse_buildings(self.get_parameter("local_buildings").value),
            float(self.get_parameter("building_half_m").value))

        self.create_subscription(MapDelta, "comms/in/map", self._on_delta, 20)
        self._out = self.create_publisher(MapDelta, "comms/out/map", 20)
        self.create_timer(0.5, self._emit_local_delta)
        self.get_logger().info(
            f"map_merge_node up (drone {self.drone_id}, "
            f"{int(self._local.occ.sum())} occupied cells in local map)")

    def _on_delta(self, msg: MapDelta) -> None:
        if msg.drone_id == self.drone_id:
            return
        if self._last_seq.get(msg.drone_id, -1) >= msg.seq:
            return  # stale / duplicate
        self._last_seq[msg.drone_id] = msg.seq
        for idx, lo in zip(msg.voxel_indices, msg.voxel_log_odds):
            self._merged[idx] = merge_log_odds(self._merged.get(idx, 0), lo)
        # Write the fused evidence back into the LOCAL grid (cells whose fused log-odds clear the
        # occupancy threshold), so the planner-facing ESDF actually gains the neighbours'
        # obstacles — fusion that never reaches the grid is bookkeeping, not a shared map.
        # Indices assume both drones run the same grid geometry (shared params; see grid_io).
        apply_delta_to_grid(self._local, self._local.ny,
                            list(self._merged.keys()), threshold=0,
                            log_odds=list(self._merged.values()))

    def _emit_local_delta(self) -> None:
        """Serialise this drone's local occupancy as a MapDelta on comms/out/map so the
        comms layer meters it and neighbours fuse it. (CPU fallback for nvblox's delta.)"""
        if not self._local.occ.any():
            return  # nothing mapped yet → no heartbeat traffic
        (ox, oy), vsize, ny, indices, log_odds = occupancy_to_delta(self._local)
        msg = MapDelta()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drone_id = self.drone_id
        msg.seq = self._seq
        msg.voxel_size = vsize
        msg.origin = Point(x=ox, y=oy, z=0.0)
        msg.voxel_indices = [int(k) for k in indices]
        msg.voxel_log_odds = [int(v) for v in log_odds]
        msg.uncompressed_bytes = len(indices) * 3      # uint16 idx + int8 log-odds
        self._out.publish(msg)
        self._seq += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapMergeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
