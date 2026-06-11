#!/usr/bin/env bash
# Swarm Autonomy dependency bootstrap (Ubuntu 24.04 + ROS 2 Jazzy).
#
# Installs / vendors everything Swarm Autonomy requires that isn't in the repo:
#   * px4_msgs + px4_ros_com  (uXRCE-DDS bridge interfaces)        -> ros2_ws/src/third_party
#   * Micro-XRCE-DDS-Agent     (PX4 <-> ROS 2 transport)
#   * PX4-Autopilot            (SITL firmware + gz Harmonic airframes)
#   * ego-planner-swarm, RACER, FUEL  (planners/exploration)      -> ros2_ws/src/third_party
#   * OpenVINS                 (VIO)                               -> ros2_ws/src/third_party
#
# Idempotent: re-running skips clones that already exist. Heavy — run once.
set -euo pipefail

WS_SRC="$(cd "$(dirname "$0")/.." && pwd)/ros2_ws/src"
TP="$WS_SRC/third_party"
mkdir -p "$TP"

clone() {  # clone <url> <dir> [branch]
  local url="$1" dir="$2" branch="${3:-}"
  if [ -d "$TP/$dir/.git" ]; then echo "  [skip] $dir present"; return; fi
  echo "  [clone] $dir"
  git clone --depth 1 ${branch:+-b "$branch"} "$url" "$TP/$dir"
}

echo "== apt deps =="
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-ros-gz ros-jazzy-rmw-cyclonedds-cpp \
  build-essential cmake git

echo "== ROS 2 interface + bridge sources =="
clone https://github.com/PX4/px4_msgs.git          px4_msgs        release/1.15
clone https://github.com/PX4/px4_ros_com.git       px4_ros_com     release/1.15

echo "== planners / exploration / VIO =="
clone https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git ego-planner-swarm
clone https://github.com/SYSU-STAR/RACER.git               RACER
clone https://github.com/HKUST-Aerial-Robotics/FUEL.git    FUEL
clone https://github.com/rpng/open_vins.git                open_vins

echo "== Micro-XRCE-DDS-Agent =="
if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
  tmp="$(mktemp -d)"
  git clone --depth 1 -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$tmp"
  cmake -S "$tmp" -B "$tmp/build" -DUAGENT_USE_SYSTEM_FASTDDS=OFF
  cmake --build "$tmp/build" -j"$(nproc)"
  sudo cmake --install "$tmp/build"
  sudo ldconfig
else
  echo "  [skip] MicroXRCEAgent present"
fi

echo "== PX4-Autopilot (SITL + gz Harmonic) =="
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
if [ ! -d "$PX4_DIR/.git" ]; then
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive "$PX4_DIR"
  bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx
else
  echo "  [skip] PX4-Autopilot present at $PX4_DIR"
fi

echo
echo "Done. Next:"
echo "  rosdep install --from-paths $WS_SRC --ignore-src -r -y"
echo "  cd $(dirname "$WS_SRC") && colcon build --symlink-install"
