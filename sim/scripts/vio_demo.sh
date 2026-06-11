#!/usr/bin/env bash
# Full clean teardown + relaunch of the Swarm Autonomy VIO demo:
# Gazebo (visible drone) + IMU/cam bridges + fixed-yaw flight + OpenVINS + RViz.
# Run as a script file so pkill patterns never self-match an inline shell.
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
PX4="${PX4_DIR:-$HOME/PX4-Autopilot}/build/px4_sitl_default"
SC="$REPO/sim/scripts"
OV="$REPO/ros2_ws/src/third_party/open_vins/config/swarm_autonomy/estimator_config.yaml"
RVIZ="$REPO/sim/rviz/swarm_autonomy_vio.rviz"
W=swarm_autonomy_vio
export GZ_PARTITION="${GZ_PARTITION:-swarm_autonomy}"
export DISPLAY="${DISPLAY:-:0}"
# Render gz on the NVIDIA RTX via PRIME offload. Without these, EGL falls back to a
# null driver -> camera sensor renders black (OpenVINS starves) and the drone is invisible.
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
source /opt/ros/jazzy/setup.bash
source "$REPO/ros2_ws/install/setup.bash"

echo "==[1/6] KILL EVERYTHING (loop until zero so no leftover GUI re-pauses the next boot)=="
for p in vio_to_px4 run_subscribe_msckf rviz2 vio_fly.py min_arm parameter_bridge mavlink_shell; do
  pkill -9 -f "$p" 2>/dev/null
done
# gz server + GUI both run as 'ruby'; PX4 as 'px4'. Loop until BOTH are truly zero.
for try in $(seq 1 10); do
  pkill -9 -x px4 2>/dev/null; pkill -9 -x ruby 2>/dev/null
  sleep 2
  [ "$(pgrep -xc ruby)" = "0" ] && [ "$(pgrep -xc px4)" = "0" ] && break
done
rm -f /tmp/go_gps_denied /tmp/px4_pose.txt
echo "  cleaned: px4=$(pgrep -xc px4) ruby/gz=$(pgrep -xc ruby)"

echo "==[2/6] LAUNCH PX4 + GAZEBO SERVER (HEADLESS: no GUI window)=="
# HEADLESS: run the gz SERVER only (no GUI). The camera sensor still renders offscreen for
# VIO at full rate (~15Hz); with the GUI on, the degraded GPU starves the camera to ~7Hz
# (OpenVINS drifts) and fails to render the drone. No GUI also means the sim can't be paused.
# Visualization is via RViz (camera feed + feature tracks + VIO trajectory + pose).
cd "$PX4" || exit 1
HEADLESS=1 PX4_GZ_WORLD=$W PX4_SYS_AUTOSTART=4010 PX4_GZ_MODEL_POSE="0,0,0.3" \
    PX4_SIM_MODEL=gz_x500_mono_cam nohup ./bin/px4 -i 0 -d > /tmp/vio_drone_f.log 2>&1 &
# Keep the sim UNPAUSED throughout PX4 boot. The commander initializes its sensors in the
# first seconds; if a stray GUI pauses the sim during that window the commander stays stuck
# in UNINIT forever (never reaches STANDBY) and the drone can never arm. Unpausing once after
# boot is too late, so hammer unpause for the whole boot window.
( for k in $(seq 1 50); do
    gz service -s /world/$W/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
      --timeout 500 --req 'pause: false' >/dev/null 2>&1
    sleep 0.5
  done ) &
UNPAUSER=$!
for i in $(seq 1 40); do
  grep -q "Startup script returned" /tmp/vio_drone_f.log 2>/dev/null && break
  sleep 1
done
pgrep -xc px4 | grep -q 1 && echo "  PX4 up" || { echo "  PX4 FAILED"; tail -5 /tmp/vio_drone_f.log; exit 1; }
sleep 3
# CRITICAL: ensure the sim is RUNNING, not paused. A leftover GUI or the GUI's default
# can leave gz paused -> sim frozen -> no sensor updates -> EKF2 never converges -> the
# drone can't arm ("system health failures"). Explicitly unpause and verify it steps.
gz service -s /world/$W/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pause: false' >/dev/null 2>&1
T0=$(gz topic -e -t /world/$W/stats -n 1 2>/dev/null | grep -A2 sim_time | grep -oE 'sec: [0-9]+' | head -1)
sleep 2
T1=$(gz topic -e -t /world/$W/stats -n 1 2>/dev/null | grep -A2 sim_time | grep -oE 'sec: [0-9]+' | head -1)
echo "  sim stepping: $T0 -> $T1 $([ "$T0" != "$T1" ] && echo '(RUNNING)' || echo '(STILL FROZEN!)')"

echo "==[3/6] BRIDGES (IMU + STEREO cameras: cam0 left, cam1 right)=="
CAM="/world/$W/model/x500_mono_cam_0/link/camera_link/sensor/camera/image"
CAMR="/world/$W/model/x500_mono_cam_0/link/camera_link_right/sensor/camera_right/image"
IMU="/world/$W/model/x500_mono_cam_0/link/base_link/sensor/imu_sensor/imu"
nohup ros2 run ros_gz_bridge parameter_bridge "${IMU}@sensor_msgs/msg/Imu[gz.msgs.IMU" \
  --ros-args -r "${IMU}:=/imu0" > /tmp/bri.log 2>&1 &
nohup ros2 run ros_gz_bridge parameter_bridge "${CAM}@sensor_msgs/msg/Image[gz.msgs.Image" \
  --ros-args -r "${CAM}:=/cam0/image_raw" > /tmp/brc.log 2>&1 &
nohup ros2 run ros_gz_bridge parameter_bridge "${CAMR}@sensor_msgs/msg/Image[gz.msgs.Image" \
  --ros-args -r "${CAMR}:=/cam1/image_raw" > /tmp/brcr.log 2>&1 &
sleep 6
CR=$(timeout 6 ros2 topic hz /cam0/image_raw 2>/dev/null | grep -m1 -oE "average rate: [0-9.]+")
CR1=$(timeout 6 ros2 topic hz /cam1/image_raw 2>/dev/null | grep -m1 -oE "average rate: [0-9.]+")
IR=$(timeout 6 ros2 topic hz /imu0 2>/dev/null | grep -m1 -oE "average rate: [0-9.]+")
echo "  cam0 $CR | cam1 $CR1 | imu $IR"

echo "==[4/6] FLIGHT (fixed-yaw gentle circle, EV fusion armed)=="
nohup python3 -u "$SC/vio_fly.py" > /tmp/flyf.log 2>&1 &
for i in $(seq 1 30); do grep -q "airborne" /tmp/flyf.log 2>/dev/null && break; sleep 1; done
echo "  settling into steady circle (18s)..."; sleep 18

echo "==[5/6] OPENVINS=="
nohup ros2 run ov_msckf run_subscribe_msckf "$OV" > /tmp/ovf.log 2>&1 &
sleep 2

if [ "${NORVIZ:-0}" = "1" ]; then
  echo "==[6/6] RVIZ skipped (NORVIZ=1, saving memory) =="
else
  echo "==[6/6] RVIZ (software GL so it does not fight Gazebo for the GPU)=="
  LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe nohup rviz2 -d "$RVIZ" > /tmp/rvizf.log 2>&1 &
  sleep 6
fi
echo "== watching VIO for 25s (dist grows = healthy odometry) =="
for i in 1 2 3 4 5; do sleep 5; D=$(grep -oE "dist = [0-9.]+" /tmp/ovf.log | tail -1); echo "  t=$((i*5))s ${D:-(no init yet)}"; done
echo "DONE. px4=$(pgrep -xc px4) gz_gui=$(pgrep -fc 'gz sim -g') rviz=$(pgrep -xc rviz2) ov=$(pgrep -fc run_subscribe_msckf)"
echo "Next: launch vision bridge -> python3 $SC/vio_to_px4.py ; then 'touch /tmp/go_gps_denied' to go GPS-denied."
