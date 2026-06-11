#!/usr/bin/env bash
# Configurable multi-drone PX4 SITL + Gazebo Harmonic swarm.
#
#   run_px4_swarm.sh [WORLD] [N]
#     WORLD : gz world name (default: swarm_autonomy_city — the project's city). Any world synced
#             into PX4's Tools/simulation/gz/worlds also works (e.g. swarm_autonomy_vio, default,
#             walls, baylands, forest).
#     N     : number of drones (default: 3)
#
#   Env:
#     GUI=0           run headless (no window), default GUI=1
#     NOTAKEOFF=1     just spawn the drones, don't auto take off
#     PX4_DIR         default ~/PX4-Autopilot
#     AGENT           default ~/swarm_autonomy_tools/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent
#
# Renders on the NVIDIA RTX via PRIME offload. Keep N modest on GPUs with limited VRAM
# (2-4 drones is smooth; many drones is heavy). VISION=1 needs the GUI (GUI=0 renders black
# cameras) and the camera-pursuit detects nothing.
set -euo pipefail

WORLD="${1:-swarm_autonomy_city}"
N="${2:-3}"
GUI="${GUI:-1}"
# Vehicle airframe. VISION mode uses the downward-camera x500 so pursuers can SEE.
if [ "${VISION:-0}" = "1" ]; then
  SIM_MODEL="${SIM_MODEL:-gz_x500_mono_cam_down}"; SYS_AUTOSTART="${SYS_AUTOSTART:-4014}"
else
  SIM_MODEL="${SIM_MODEL:-gz_x500}"; SYS_AUTOSTART="${SYS_AUTOSTART:-4001}"
fi
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
AGENT="${AGENT:-$HOME/swarm_autonomy_tools/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent}"
HERE="$(cd "$(dirname "$0")" && pwd)"
BUILD="$PX4_DIR/build/px4_sitl_default"

# Render Gazebo on the dedicated NVIDIA GPU (desktop is on the AMD iGPU).
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
# Stop PX4 from locking the GUI camera onto a single drone (px4-rc.gzsim checks this).
export PX4_GZ_NO_FOLLOW=1
# Run our Gazebo in an ISOLATED transport partition so it never collides with any
# other gz instance on this machine (e.g. another project's sim). All our gz
# clients (gz topic queries, the vision detector, the pursuit bridge) must export
# the SAME GZ_PARTITION to see this sim.
export GZ_PARTITION="${GZ_PARTITION:-swarm_autonomy}"

command -v "$AGENT" >/dev/null 2>&1 || [ -x "$AGENT" ] || {
  echo "MicroXRCEAgent not found at $AGENT — set AGENT=... or build it." >&2; exit 1; }
[ -x "$BUILD/bin/px4" ] || { echo "PX4 not built at $BUILD" >&2; exit 1; }

# Make the Swarm Autonomy custom worlds available to PX4 (sync from the repo).
REPO_WORLDS="$(cd "$HERE/../worlds" && pwd)"
PX4_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
for w in "$REPO_WORLDS"/*.sdf; do
  [ -e "$w" ] && cp -u "$w" "$PX4_WORLDS_DIR/" 2>/dev/null || true
done

# Friendly: list selectable worlds and validate the requested one exists.
WORLDS_AVAIL="$(ls "$PX4_WORLDS_DIR"/*.sdf 2>/dev/null | xargs -n1 basename | sed 's/.sdf//' | tr '\n' ' ')"
if [ ! -f "$PX4_WORLDS_DIR/$WORLD.sdf" ]; then
  echo "Unknown world '$WORLD'. Available: $WORLDS_AVAIL" >&2
  exit 1
fi
echo "  worlds available: $WORLDS_AVAIL"

echo "== Swarm Autonomy swarm: $N drones in '$WORLD' (GUI=$GUI, GPU=RTX via PRIME) =="
"$AGENT" udp4 -p 8888 >/tmp/sw_agent.log 2>&1 &
sleep 1

cd "$BUILD"
for i in $(seq 0 $((N - 1))); do
  # Spread along X on the y=-6 street — a clear east-west lane in swarm_autonomy_city (no building
  # rows at y=-6), so drones never spawn INSIDE a building (y=0 runs through the b_*_0 column).
  POSE="$((i * 4 - 6)),-6,0.3"
  if [ "$i" -eq 0 ]; then
    echo "  drone 0 -> starts gz + world '$WORLD'"
    if [ "$GUI" = "1" ]; then HEADLESS_ENV="env -u HEADLESS"; else HEADLESS_ENV="env HEADLESS=1"; fi
    $HEADLESS_ENV PX4_GZ_WORLD="$WORLD" PX4_SYS_AUTOSTART="$SYS_AUTOSTART" \
      PX4_GZ_MODEL_POSE="$POSE" PX4_SIM_MODEL="$SIM_MODEL" \
      ./bin/px4 -i 0 -d >"/tmp/sw_px4_0.log" 2>&1 &
    # CRITICAL: wait for drone 0's world to actually be up (its /world/<W>/clock advertised)
    # before launching the rest. px4-rc.gzsim joins an existing world only if that clock topic
    # exists; otherwise a slow first render makes each later instance spawn its OWN default world
    # (separate worlds -> cameras see nothing, pursuit desyncs). A fixed sleep is not enough.
    echo "  waiting for world '$WORLD' (clock) ..."
    for _ in $(seq 1 90); do
      gz topic -l 2>/dev/null | grep -q "^/world/$WORLD/clock" && break
      sleep 1
    done
  else
    echo "  drone $i -> joins world at $POSE"
    PX4_SYS_AUTOSTART="$SYS_AUTOSTART" PX4_GZ_MODEL_POSE="$POSE" PX4_SIM_MODEL="$SIM_MODEL" \
      ./bin/px4 -i "$i" -d >"/tmp/sw_px4_$i.log" 2>&1 &
    sleep 3
  fi
done

echo "  all $N vehicles spawned."

# PX4 auto-locks the GUI camera onto x500_0; release it so the whole swarm is
# visible and you can orbit the camera freely (NONE = no follow/track). PX4 may set the
# follow a little after startup, so retry a few times over ~15 s in the background.
if [ "$GUI" = "1" ]; then
  ( for _ in $(seq 1 8); do
      gz topic -t /gui/track -m gz.msgs.CameraTrack -p 'track_mode: 0' >/dev/null 2>&1 || true
      sleep 2
    done ) &
fi

if [ "${VISION:-0}" = "1" ]; then
  echo "== Swarm Autonomy VISION pursuit: $N camera drones detect + corner a ground target =="
  # Spawn the bright target the pursuers will detect by camera.
  TGT_SDF='<sdf version="1.9"><model name="evader"><static>true</static><link name="l"><visual name="v"><geometry><box><size>2.5 2.5 0.3</size></box></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse><emissive>0.9 0 0 1</emissive></material></visual></link></model></sdf>'
  gz service -s "/world/$WORLD/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
    --timeout 3000 --req "sdf: '$TGT_SDF', pose: {position: {x: 6, y: 6, z: 0.15}}" >/dev/null 2>&1 || true
  python3 -u "$HERE/gazebo_pursuit_vision.py" "$N" "$WORLD" >/tmp/sw_takeoff.log 2>&1 &
elif [ "${PURSUIT:-0}" = "1" ]; then
  echo "== Swarm Autonomy pursuit: $((N-1)) pursuers corner 1 fleeing target (CBBA + comms) =="
  python3 -u "$HERE/gazebo_pursuit.py" "$((N-1))" "3" >/tmp/sw_takeoff.log 2>&1 &
elif [ "${MISSION:-0}" = "1" ]; then
  echo "== arming all $N drones + coordinated city patrol (offboard formation) =="
  python3 -u "$HERE/swarm_mission.py" "$N" >/tmp/sw_takeoff.log 2>&1 &
elif [ "${NOTAKEOFF:-0}" != "1" ]; then
  echo "== arming + taking off all $N drones (hover) =="
  python3 -u "$HERE/swarm_takeoff.py" "$N" >/tmp/sw_takeoff.log 2>&1 &
fi

cat <<EOF

Running. Watch them:
  gz topic -e -t /world/$WORLD/dynamic_pose/info -n 1 | grep -A4 x500
  for i in \$(seq 0 $((N-1))); do echo -n "drone \$i: "; tail -1 /tmp/sw_px4_\$i.log; done
Stop everything:
  pkill -f bin/px4; pkill -f "gz sim"; pkill -f MicroXRCEAgent; pkill -f swarm_takeoff
EOF
wait
