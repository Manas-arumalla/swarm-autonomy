"""ROS-free occupancy-grid fusion helpers (kept separate so they unit-test
without rclpy, matching link_model/cbba/pid)."""

from __future__ import annotations


def merge_log_odds(current: int, incoming: int, clamp: int = 127) -> int:
    """Fuse two int8 log-odds occupancy values with saturation."""
    return max(-clamp, min(clamp, int(current) + int(incoming)))
