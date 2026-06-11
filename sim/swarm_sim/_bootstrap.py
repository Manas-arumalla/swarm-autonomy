"""Put the Swarm Autonomy ROS package source dirs on sys.path so the simulator can
import the real algorithm modules (link_model, cbba, pursuit, pid) without a
built ROS 2 workspace. None of those modules import rclpy, so this is enough."""

from __future__ import annotations

import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.normpath(os.path.join(_THIS, "..", "..", "ros2_ws", "src"))

for _pkg in ("swarm_autonomy_comms", "swarm_autonomy_coordination", "swarm_autonomy_control",
             "swarm_autonomy_mapping"):
    _p = os.path.join(_SRC, _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)
