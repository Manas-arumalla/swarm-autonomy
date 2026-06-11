"""Swarm Autonomy headless swarm simulator.

A ROS-free, physics-lite multi-drone simulation that drives the Swarm Autonomy
algorithm modules — the comms middleware link model, CBBA role allocation,
pursuit/interception geometry and the PID controller — through the full
scenario: cooperative exploration of an unknown city followed by decentralized
role allocation and interception of a fleeing target.

Runs without the full PX4 + Gazebo + OpenVINS + RACER stack; the same
coordination code runs here and in the ROS nodes.
"""

from . import _bootstrap  # noqa: F401  (sets sys.path to find the algorithm pkgs)
