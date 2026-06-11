#!/usr/bin/env bash
# Bring up the simulation substrate for Swarm Autonomy.
#   1. Micro-XRCE-DDS-Agent  (PX4 <-> ROS 2 transport, UDP :8888)
#   2. N PX4 SITL instances on Gazebo Harmonic, namespaced per vehicle
#
# The ROS 2 application graph is launched separately:
#   ros2 launch swarm_autonomy_bringup multi_drone.launch.py num_drones:=N
#
# Env:
#   PX4_DIR   path to PX4-Autopilot (default ~/PX4-Autopilot)
#   N         number of vehicles (default 3)
#   WORLD     gz world name (default swarm_autonomy_city)
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
N="${N:-3}"
WORLD="${WORLD:-swarm_autonomy_city}"

if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
  echo "MicroXRCEAgent not found — run scripts/setup.sh first." >&2
  exit 1
fi
if [ ! -d "$PX4_DIR" ]; then
  echo "PX4-Autopilot not found at $PX4_DIR — run scripts/setup.sh first." >&2
  exit 1
fi

echo "[sim] starting Micro-XRCE-DDS-Agent on udp4 :8888"
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
trap 'kill $AGENT_PID 2>/dev/null || true' EXIT

echo "[sim] spawning $N PX4 SITL vehicles in world '$WORLD'"
# PX4's multi-vehicle SITL helper assigns instance ids / namespaces.
# Headless by default for CI; unset HEADLESS=0 for the GUI.
export PX4_GZ_WORLD="$WORLD"
HEADLESS="${HEADLESS:-1}" "$PX4_DIR/Tools/simulation/gz/sitl_multiple_run.sh" -n "$N" -m gz_x500_depth

wait $AGENT_PID
