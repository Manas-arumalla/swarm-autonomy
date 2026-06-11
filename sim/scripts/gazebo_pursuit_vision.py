#!/usr/bin/env python3
"""Swarm Autonomy VISION pursuit: a swarm corners a fleeing target it detects by CAMERA.

Usage: gazebo_pursuit_vision.py N [world] [alt]

N pursuer drones (downward cameras) fly over the city; each runs OpenCV colour-blob
detection on its *own rendered camera frames* (vision_detect.CameraDetector) and
back-projects the target's pixel to a world position. Pursuers locate the target
from their own cameras or from a neighbour's detection that survives the comms link
model — fully decentralized perception. The target is a bright ground marker
executing a scripted evasion.

Coordination uses the Swarm Autonomy algorithm modules: cbba (containment-slot
allocation), pursuit (interception geometry), swarm_control (reciprocal collision
avoidance), link_model (range-gated detection sharing). Drones are flown in
offboard velocity control with altitude hold.
"""

from __future__ import annotations

import struct
import math
import os
import random
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.normpath(os.path.join(_HERE, "..", "..", "ros2_ws", "src"))
for _p in ("swarm_autonomy_coordination", "swarm_autonomy_comms", "swarm_autonomy_control",
           "swarm_autonomy_mapping"):
    sys.path.insert(0, os.path.join(_SRC, _p))
sys.path.insert(0, _HERE)

from pymavlink import mavutil
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_v_pb2 import Pose_V

from swarm_autonomy_coordination import pursuit
from swarm_autonomy_coordination.cbba import run_cbba
from swarm_autonomy_coordination.swarm_control import control_velocity, avoidance_velocity
from swarm_autonomy_coordination.mpc_pursuit import mpc_velocity
from swarm_autonomy_coordination.target_tracker import TargetTracker
from swarm_autonomy_comms.link_model import LinkConfig, LinkModel
from swarm_autonomy_control.px4_frames import (VELOCITY_SETPOINT_MASK, ned_to_world_pos,
                                               ned_to_world_vel, world_to_ned_vel)
from swarm_autonomy_mapping.esdf import GridMap
from vision_detect import CameraDetector

# CONTROL=mpc (default): smooth optimal MPC guidance + ORCA avoidance.
# CONTROL=reactive: the lead-pursuit + adaptive-smoothing reactive law (kept for A/B).
CONTROL = os.environ.get("CONTROL", "mpc")
# CALIB=1: log labelled (pixel, drone pose, true target) calibration samples to /tmp/calib.csv
# for offline re-fitting of the camera back-projection (experiments/fit_camera_extrinsic.py).
CALIB = os.environ.get("CALIB", "0") == "1"
# DIAG=1: log each drone's estimated position against its gz ground-truth pose to /tmp/diag.csv,
# to characterize the target-estimate error against ground truth.
DIAG = os.environ.get("DIAG", "0") == "1"

N = int(sys.argv[1]) if len(sys.argv) > 1 else 3
WORLD = sys.argv[2] if len(sys.argv) > 2 else "swarm_autonomy_city"
# TRACKING altitude: low enough for tight, well-conditioned fixes on the target.
ALT = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
# SEARCH altitude: high enough that one camera footprint spans most of the city
# (half-width = alt*tan(hfov/2) ~ 24 m at 20 m), so a lost target is reacquired in seconds.
# Classic search-high / track-low profile; the 50 Hz loop tracks whichever is active.
SEARCH_ALT = max(20.0, ALT)

SPACING = 4.0
CITY = 16.0
CAPTURE_R = 3.0
CONTAIN_R = 3.5
PURSUER_SPEED = 5.0      # outrun the evader to close in
TARGET_SPEED = 1.5       # fast enough to range across the city, slow enough to stay trackable
SAFETY_R = 2.2
ALT_STEP = 2.5           # per-drone cruise-altitude offset: vertical separation makes mid-air
                         # collisions geometrically impossible (standard altitude deconfliction)
KP_Z = 1.2
VEL_MASK = VELOCITY_SETPOINT_MASK   # named-bit, unit-tested (see swarm_autonomy_control.px4_frames)
FRAME = mavutil.mavlink.MAV_FRAME_LOCAL_NED

# Must MATCH the gz spawn poses in run_px4_swarm.sh (East=x, North=y): the clear y=-6 street,
# so world_pos() = spawn + local-NED is correct and drones never start inside building b_0_0.
spawn = [(SPACING * i - 6.0, -6.0) for i in range(N)]
conns = [mavutil.mavlink_connection(f"udpin:0.0.0.0:{14540 + i}",
                                    source_system=255, source_component=190)
         for i in range(N)]
cams = [CameraDetector(
    f"/world/{WORLD}/model/x500_mono_cam_down_{i}/link/camera_link/sensor/camera/image")
    for i in range(N)]
for _c in cams:
    # A target HALF OUT OF FRAME at the image corner shows only a sliver of pixels; the default
    # blob-area floor rejected exactly the corner glimpses that should trigger a chase. The sim
    # target is emissive/saturated so a low floor is safe; quality is handled by the per-fix
    # noise weighting downstream, not by discarding the sighting.
    _c.MIN_AREA = 12

pos_ned = [(0.0, 0.0)] * N
vel_ned = [(0.0, 0.0)] * N
yaw = [0.0] * N
roll = [0.0] * N
pitch = [0.0] * N
alt_up = [0.0] * N
LEVEL_TILT = 0.10        # rad (~6 deg): only trust detections when near-level
goal = [[spawn[i][0], spawn[i][1]] for i in range(N)]
# Per-drone goal-velocity feedforward: ONLY the interceptor chases a moving goal (the evader);
# the blockers hold STATIC containment slots — feeding them the evader velocity drags their
# reference point along and makes them orbit/overshoot the ring.
goal_vels = [[0.0, 0.0] for _ in range(N)]
goal_t = [0.0]           # wall-time the goals were last set (for 50 Hz extrapolation)
cur_alt = [ALT]          # active altitude setpoint: ALT while tracking, SEARCH_ALT while lost
state_lock = threading.Lock()
running = True

belief = [None] * N
_cfg = LinkConfig(bandwidth_bps=1e9, max_range_m=20.0, soft_range_m=14.0, base_loss=0.03)
links = {(a, b): LinkModel(_cfg, random.Random(a * 131 + b))
         for a in range(N) for b in range(N) if a != b}

# Target (ground marker) — we move it; pursuers must SEE it to know where it is.
target = [6.0, 6.0]              # clear street intersection (NOT a building cell)
gz = GzNode()

# Drone SELF-position from gz ground truth (position only; attitude still comes from MAVLink).
# The PX4 EKF's local-position belief drifts and occasionally RESETS under GPS noise; measured
# live, the belief sat 35-45 m off the vehicle's true pose, which (a) walked the swarm out of
# the city while its goals were clamped inside it, and (b) injected a matching systematic bias
# into every camera back-projection (the fix inherits the drone's self-position error 1:1).
# Self-localization is not what this demo evaluates — the TARGET is still found by camera only —
# so the coordination runs on true poses, with the MAVLink belief as a fallback.
true_pose = {}                  # model name -> (x_east, y_north, z_up, yaw)
true_vel = {}                   # model name -> (vE, vN)  (finite-difference, EMA-smoothed)
_prev_pose = {}                 # model name -> (x, y, t_wall)
_dbg = {"printed": False}


def _on_dyn(msg):
    if not _dbg["printed"]:
        names = [p.name for p in msg.pose if "x500" in p.name]
        print(f"[gz] pose topic models with x500: {names}", flush=True)
        _dbg["printed"] = True
    now = time.time()
    for p in msg.pose:
        if "x500" in p.name:
            q = p.orientation
            yw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                            1 - 2 * (q.y * q.y + q.z * q.z))
            true_pose[p.name] = (p.position.x, p.position.y, p.position.z, yw)
            prev = _prev_pose.get(p.name)
            if prev is not None and now - prev[2] > 0.04:
                dt = now - prev[2]
                vE = (p.position.x - prev[0]) / dt
                vN = (p.position.y - prev[1]) / dt
                oE, oN = true_vel.get(p.name, (vE, vN))
                true_vel[p.name] = (0.6 * oE + 0.4 * vE, 0.6 * oN + 0.4 * vN)
                _prev_pose[p.name] = (p.position.x, p.position.y, now)
            elif prev is None:
                _prev_pose[p.name] = (p.position.x, p.position.y, now)


for _tp in (f"/world/{WORLD}/dynamic_pose/info", f"/world/{WORLD}/pose/info"):
    gz.subscribe(Pose_V, _tp, _on_dyn)


def world_pos(i):
    tp = true_pose.get(f"x500_mono_cam_down_{i}")
    if tp is not None:
        return (tp[0], tp[1])
    n, e = pos_ned[i]                       # fallback: MAVLink belief until gz pose arrives
    sE, sN = spawn[i]
    return ned_to_world_pos(n, e, spawn_east=sE, spawn_north=sN)


def world_alt(i):
    tp = true_pose.get(f"x500_mono_cam_down_{i}")
    return tp[2] if tp is not None else alt_up[i]


def world_vel(i):
    tv = true_vel.get(f"x500_mono_cam_down_{i}")
    if tv is not None:
        return tv
    vn, ve = vel_ned[i]                     # fallback: EKF velocity until gz pose flows
    return ned_to_world_vel(vn, ve)


def telemetry():
    while running:
        for i, c in enumerate(conns):
            m = c.recv_match(blocking=False)
            if not m:
                continue
            t = m.get_type()
            if t == "LOCAL_POSITION_NED":
                pos_ned[i] = (m.x, m.y)
                vel_ned[i] = (m.vx, m.vy)
                alt_up[i] = -m.z
            elif t == "ATTITUDE":
                yaw[i] = m.yaw
                roll[i] = m.roll
                pitch[i] = m.pitch
        time.sleep(0.005)


CMD_ACCEL = 6.0      # m/s^2 slew limit on the FINAL command (MPC + ORCA + ESDF additions can
                     # individually be smooth yet sum to a step; the limiter guarantees the
                     # commanded velocity is continuous, like PX4's own velocity tracking)


def stream():
    cmd = [[0.0, 0.0] for _ in range(N)]    # smoothed velocity command per drone
    while running:
        with state_lock:
            g = [list(x) for x in goal]
            gvs = [tuple(v) for v in goal_vels]  # per-drone goal velocity (feedforward)
            gt = goal_t[0]
        # The control runs at 50 Hz but the goals refresh at the (detection-bound) main-loop
        # rate. Extrapolate each goal by its FULL age along the drone's goal velocity — chasing
        # a stale point made the interceptor aim where the target WAS, cross it, and lose it
        # behind a building before reacquisition. Bounded at 1 s to keep divergence in check.
        age = max(0.0, min(1.0, time.time() - gt))
        positions = [world_pos(i) for i in range(N)]
        velocities = [world_vel(i) for i in range(N)]
        for i, c in enumerate(conns):
            gv = gvs[i]                                           # this drone's goal velocity
            lg = (g[i][0] + gv[0] * age, g[i][1] + gv[1] * age)   # lightly-extrapolated goal
            neighbors = [(positions[j], velocities[j]) for j in range(N) if j != i]
            if CONTROL == "mpc":
                # MPC plans a smooth optimal interception trajectory (target predicted at the
                # drone's goal velocity) and returns the first velocity; ORCA adjusts only for
                # inter-drone collisions.
                pref = mpc_velocity(positions[i], velocities[i], lg, gv,
                                    vmax=PURSUER_SPEED, amax=6.0)
                vE, vN = avoidance_velocity(positions[i], velocities[i], pref, neighbors,
                                            PURSUER_SPEED, safety_radius=SAFETY_R)
            else:
                # reactive lead-pursuit + adaptive smoothing (responsive far, smoothed near)
                vE, vN = control_velocity(positions[i], velocities[i], lg,
                                          neighbors, PURSUER_SPEED, safety_radius=SAFETY_R)
                dg = math.hypot(lg[0] - positions[i][0], lg[1] - positions[i][1])
                a = 0.45 if dg < 4.0 else 0.95
                vE = cmd[i][0] + a * (vE - cmd[i][0])
                vN = cmd[i][1] + a * (vN - cmd[i][1])
            # Deflect around buildings (ESDF repulsion) when below roof height — neither the
            # MPC nor ORCA knows the city geometry; above the roofs this is a no-op.
            vE, vN = esdf_avoid(vE, vN, positions[i][0], positions[i][1],
                                world_alt(i), PURSUER_SPEED)
            # Slew-rate limit the FINAL command: guidance + ORCA + ESDF terms are each smooth,
            # but their sum (and the ~1.5 Hz goal refresh) steps; bounding the per-tick change
            # to CMD_ACCEL*dt makes the commanded velocity continuous -> visibly stable flight.
            # Geofence has the LAST word before rate limiting: no guidance or estimation fault
            # may carry a vehicle outside the operational volume.
            vE, vN = geofence(vE, vN, positions[i][0], positions[i][1], PURSUER_SPEED)
            dvE, dvN = vE - cmd[i][0], vN - cmd[i][1]
            dv = math.hypot(dvE, dvN)
            dv_max = CMD_ACCEL * 0.02
            if dv > dv_max:
                dvE, dvN = dvE / dv * dv_max, dvN / dv * dv_max
            cmd[i][0] += dvE
            cmd[i][1] += dvN
            # Per-drone cruise altitude: base (phase-scheduled) + i*ALT_STEP. The fixed vertical
            # stagger makes mid-air collisions geometrically impossible for ANY drone count —
            # ORCA only has to solve the horizontal problem.
            vUp = max(-2.5, min(2.5, KP_Z * (cur_alt[0] + ALT_STEP * i - world_alt(i))))
            vx, vy, vz = world_to_ned_vel(cmd[i][0], cmd[i][1], vUp)   # (vE,vN,up) -> NED
            c.mav.set_position_target_local_ned_send(
                0, i + 1, 1, FRAME, VEL_MASK, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)
        time.sleep(0.02)    # 50 Hz control


def heartbeats():
    while running:
        for c in conns:
            c.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.3)


def move_target(E, Nn):
    req = Pose()
    req.name = "evader"
    req.position.x = E
    req.position.y = Nn
    req.position.z = 0.1
    req.orientation.w = 1.0
    try:
        gz.request(f"/world/{WORLD}/set_pose", req, Pose, Boolean, 200)
    except Exception:
        pass


def clamp(E, Nn):
    return (max(-CITY, min(CITY, E)), max(-CITY, min(CITY, Nn)))


# ===================== ENVIRONMENT CONTRACT (swap these for other worlds) ====================
# Everything below — ESDF avoidance, geofence, LOS tests, search strips, the evader's routing —
# derives from THREE things: the square operational bound (CITY, set above), this obstacle list,
# and the max obstacle height. For a different environment, supply its obstacle footprints and
# height here (or build the GridMap from a sensed/known map) and the rest follows unchanged.
BGRID = list(range(-24, 25, 12))
B_HALF = 3.5             # obstacle footprint half-width (7x7 m buildings, gen_city.py)
OBSTACLES = [(cx, cy) for cx in BGRID for cy in BGRID]
MAX_BLDG_H = 12.0        # tallest obstacle: LOS occluder height + avoidance altitude gate
# =============================================================================================

# Environment ESDF for DRONE obstacle avoidance (the same swarm_autonomy_mapping core the
# planner uses): when a drone is BELOW roof height the 50 Hz control loop adds a signed-distance
# repulsion that deflects it around any obstacle it would clip. Above the tallest roof the air
# is open and the repulsion is OFF — a 2-D push up there only fights the pursuit (the streets
# are just ~5 m wide between building faces, so an always-on margin turns every street into a
# force field).
AVOID_MARGIN = 1.8       # m: start deflecting below this clearance (streets are ~5 m wide)
AVOID_GAIN = 2.5         # m/s of push per metre of margin violation
ROOF_CLEAR_ALT = MAX_BLDG_H + 1.0   # above this, no obstacle avoidance needed
_city_map = GridMap((-2.0 * CITY, -2.0 * CITY), (2.0 * CITY, 2.0 * CITY), res=0.5)
_city_map.add_buildings(OBSTACLES, B_HALF)
_city_map.esdf()         # precompute once; lookups in the control loop are O(1)


CANYON_SPEED = 3.0       # m/s cap below roof height: at 5 m/s the 6 m/s^2 slew limiter needs
                         # ~2.1 m to redirect, more than the street margin — slower flight gives
                         # the avoidance real authority (standard urban-canyon practice)
GEOFENCE = CITY + 5.0    # hard operational boundary; outside it the command is overridden home


def esdf_avoid(vE, vN, x, y, alt, vmax):
    """Blend an ESDF repulsion into a velocity command so a below-roof drone deflects around
    buildings, and cap below-roof speed so the deflection has authority. No-op above
    ROOF_CLEAR_ALT (open air)."""
    if alt >= ROOF_CLEAR_ALT:
        return vE, vN
    d = _city_map.distance(x, y)
    if d < AVOID_MARGIN:
        gx, gy = _city_map.gradient(x, y)
        push = AVOID_GAIN * (AVOID_MARGIN - d)
        vE += push * gx
        vN += push * gy
    sp = math.hypot(vE, vN)
    cap = min(vmax, CANYON_SPEED)
    if sp > cap:
        vE, vN = vE / sp * cap, vN / sp * cap
    return vE, vN


def geofence(vE, vN, x, y, vmax):
    """Hard operational boundary: outside the fence, override the command toward the city.
    Standard flight-stack behaviour (PX4 geofence) implemented at the command layer; with it,
    no guidance/estimation fault can walk a vehicle out of the test volume."""
    if abs(x) <= GEOFENCE and abs(y) <= GEOFENCE:
        return vE, vN
    d = math.hypot(x, y) or 1.0
    return (-x / d * vmax, -y / d * vmax)


def los_blocked(x0, y0, z0, x1, y1, z1=0.0, max_h=MAX_BLDG_H, step=1.0):
    """Conservative 3-D line-of-sight test against the city: sample the segment and report
    blocked if any sample below the (max) roof height falls inside a building footprint."""
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.hypot(dx, dy) or 1e-6
    n = max(2, int(length / step))
    for k in range(1, n):
        f = k / n
        z = z0 + dz * f
        if z >= max_h:
            continue
        if _city_map.distance(x0 + dx * f, y0 + dy * f) < 0.0:
            return True
    return False


def in_building(E, Nn, margin=0.5):
    h = B_HALF + margin
    for cx, cy in OBSTACLES:
        if abs(E - cx) < h and abs(Nn - cy) < h:
            return True
    return False


def building_push(E, Nn):
    """Repulsion vector pushing a point away from nearby building centres."""
    rx = ry = 0.0
    for cx, cy in OBSTACLES:
        dx, dy = E - cx, Nn - cy
        d = math.hypot(dx, dy)
        if 1e-3 < d < 8.0:
            w = (8.0 - d) / 8.0
            rx += dx / d * w
            ry += dy / d * w
    return rx, ry


def main():
    for t in (telemetry, stream, heartbeats):
        threading.Thread(target=t, daemon=True).start()
    print(f"vision pursuit: {N} camera drones vs a ground target in '{WORLD}'", flush=True)
    time.sleep(1.5)

    I32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
    for i, c in enumerate(conns):
        for name, val in [("NAV_DLL_ACT", 0), ("CBRK_SUPPLY_CHK", 894281),
                          ("NAV_RCL_ACT", 0), ("COM_RCL_EXCEPT", 4)]:
            c.mav.param_set_send(i + 1, 1, name.encode(), struct.unpack("<f", struct.pack("<i", int(val)))[0], I32)
            time.sleep(0.05)
    time.sleep(1)
    for attempt in range(6):
        for i, c in enumerate(conns):
            c.mav.command_long_send(i + 1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
            c.mav.command_long_send(i + 1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        if attempt == 0:
            print("OFFBOARD + arm; climbing to search altitude...", flush=True)
        time.sleep(3)
    print("hunt on: pursuers SEARCH, then DETECT by camera + corner the target", flush=True)

    t0 = time.time()
    t_prev = t0
    t = 0.0
    captured = False
    detect_count = 0
    smooth = None          # fused target estimate (East, North) from the Kalman track
    last_seen_t = -100.0   # time of the most recent fresh detection
    search_wp = [0] * N    # per-drone serpentine waypoint index (persists across mode switches)
    evader_rng = random.Random(11)        # deterministic evader roaming
    evader_wp = [None, None, 0.0]         # current roaming waypoint (x, y, expiry time)
    est_vel = [0.0, 0.0]   # filtered evader velocity, for predictive cut-off interception
    ring_r = [4.5]         # encirclement radius; shrinks to capture when well-encircled
    prev_est = None
    prev_est_t = 0.0
    # Kalman target tracker: fuses every pursuer's noisy back-projection into ONE stable track,
    # gates out the multi-metre outlier jumps, and coasts through detection dropouts so the swarm
    # stays locked on (and leads) the target instead of chasing a teleporting phantom.
    tracker = TargetTracker(q_accel=0.6, r_meas=4.0, gate=3.0, vmax=1.4 * TARGET_SPEED + 1.0)
    import csv as _csv
    _rec = open("/tmp/pursuit_rec.csv", "w", newline="")
    _rw = _csv.writer(_rec)
    _rw.writerow(["t", "tgtE", "tgtN", "estE", "estN", "captured"]
                 + [f"d{i}{ax}" for i in range(N) for ax in "EN"])
    _crec = _cw = None
    if CALIB:
        _crec = open("/tmp/calib.csv", "w", newline="")
        _cw = _csv.writer(_crec)
        _cw.writerow(["cx", "cy", "W", "H", "dE", "dN", "alt", "roll", "pitch", "yaw",
                      "tgtE", "tgtN"])
    _drec = _dw = None
    if DIAG:
        _drec = open("/tmp/diag.csv", "w", newline="")
        _dw = _csv.writer(_drec)
        _dw.writerow(["t", "i", "wpE", "wpN", "trueE", "trueN", "wyaw", "tyaw"])
    while running:
        now = time.time()
        dt = min(0.2, now - t_prev)   # real loop period (clamped for first iter / stalls)
        t_prev = now
        t = now - t0
        positions = [world_pos(i) for i in range(N)]

        if DIAG and t > 5.0:           # compare belief vs gz truth (after takeoff settles)
            for i in range(N):
                tp = next((v for k, v in true_pose.items() if k.endswith(f"_{i}")), None)
                if tp is not None:
                    _dw.writerow([f"{t:.2f}", i, f"{positions[i][0]:.3f}", f"{positions[i][1]:.3f}",
                                  f"{tp[0]:.3f}", f"{tp[1]:.3f}", f"{yaw[i]:.4f}", f"{tp[2]:.4f}"])
            _drec.flush()

        # --- scripted evader: COMMITTED waypoint travel; threat changes the DESTINATION ------
        # The evader always walks toward its current waypoint (full commitment -> real traverses
        # across the map, no orbiting). Threat does not bend the instantaneous heading — blending
        # "to waypoint" with "away from pursuer" cancels near buildings and traps it in corner
        # dances. Instead, when a pursuer is close and the destination lies toward it, the
        # DESTINATION is redrawn far into the escape half-plane (constraints relaxed in stages
        # so a draw always exists). Deterministic RNG.
        nearest = min(positions, key=lambda p: math.hypot(p[0] - target[0], p[1] - target[1]))
        d_threat = math.hypot(nearest[0] - target[0], nearest[1] - target[1])
        away = math.atan2(target[1] - nearest[1], target[0] - nearest[0])
        threatened = d_threat < 9.0
        wp_toward_threat = False
        if evader_wp[0] is not None:
            wvx, wvy = evader_wp[0] - target[0], evader_wp[1] - target[1]
            wp_toward_threat = (wvx * math.cos(away) + wvy * math.sin(away)) < 0.0
        if (evader_wp[0] is None or t > evader_wp[2]
                or math.hypot(evader_wp[0] - target[0], evader_wp[1] - target[1]) < 1.5
                or (threatened and wp_toward_threat)):
            picked = False
            for min_d, want_escape in ((8.0, threatened), (5.0, threatened), (5.0, False)):
                for _ in range(40):
                    wx = evader_rng.uniform(-(CITY - 2.0), CITY - 2.0)
                    wy = evader_rng.uniform(-(CITY - 2.0), CITY - 2.0)
                    if in_building(wx, wy, margin=1.0):
                        continue
                    if math.hypot(wx - target[0], wy - target[1]) < min_d:
                        continue
                    if want_escape and ((wx - target[0]) * math.cos(away)
                                        + (wy - target[1]) * math.sin(away)) < 0.0:
                        continue
                    evader_wp[0], evader_wp[1], evader_wp[2] = wx, wy, t + 25.0
                    picked = True
                    break
                if picked:
                    break
            if not picked:               # pathological corner: flee straight away, clamped
                evader_wp[0], evader_wp[1] = clamp(target[0] + 10.0 * math.cos(away),
                                                   target[1] + 10.0 * math.sin(away))
                evader_wp[2] = t + 6.0
        to_wp = math.atan2(evader_wp[1] - target[1], evader_wp[0] - target[0])
        px, py = building_push(target[0], target[1])      # avoid buildings
        hx = math.cos(to_wp) + 1.6 * px
        hy = math.sin(to_wp) + 1.6 * py
        hh = math.hypot(hx, hy) or 1.0
        step = TARGET_SPEED * dt    # time-based: evader speed is m/s, not per-iteration
        cand = clamp(target[0] + hx / hh * step, target[1] + hy / hh * step)
        # if the heading still drives into a building, rotate to slide around its edge
        if in_building(*cand):
            base = math.atan2(hy, hx)
            for off in (0.6, -0.6, 1.2, -1.2, 1.8, -1.8, 2.5, -2.5, math.pi):
                c2 = clamp(target[0] + math.cos(base + off) * step,
                           target[1] + math.sin(base + off) * step)
                if not in_building(*c2):
                    cand = c2
                    break
            else:
                # boxed in: edge toward the open city centre instead of freezing
                tc = math.atan2(-target[1], -target[0])
                c2 = clamp(target[0] + math.cos(tc) * step, target[1] + math.sin(tc) * step)
                cand = c2 if not in_building(*c2) else (target[0], target[1])
        if not in_building(*cand):                        # hard guard: never enter a building
            target[0], target[1] = cand
        move_target(target[0], target[1])

        # --- REAL perception: each pursuer detects the target from its camera ---
        # Tilt-correct back-projection (uses the drone's own roll/pitch/yaw). Banked and
        # image-edge fixes are the least accurate, so the tracker takes near-level views and
        # downweights edge ones (see the per-detection noise below).
        any_detect = False
        dets = []                      # this tick's camera detections fed to the tracker
        det_noise = []                 # per-detection noise scale (edge/off-nadir -> distrust)
        for i in range(N):
            found, e, n, edge = cams[i].detect_world(positions[i][0], positions[i][1],
                                                     world_alt(i), yaw[i], roll[i], pitch[i])
            if CALIB and found:        # ground-truth pixel<->world pairs for camera re-fit
                fp, _cx, _cy, _ar = cams[i].detect_pixel()
                if fp:
                    _cw.writerow([f"{_cx:.2f}", f"{_cy:.2f}", cams[i].W, cams[i].H,
                                  f"{positions[i][0]:.3f}", f"{positions[i][1]:.3f}",
                                  f"{world_alt(i):.3f}", f"{roll[i]:.4f}", f"{pitch[i]:.4f}",
                                  f"{yaw[i]:.4f}", f"{target[0]:.3f}", f"{target[1]:.3f}"])
            if found:
                belief[i] = ((e, n), t)
                any_detect = True
                # Weight every detection by its CONDITIONING instead of discarding tilted ones:
                # a near-nadir, near-level fix is accurate; image-edge and banked fixes are
                # ill-conditioned (the off-nadir ray amplifies attitude error into metres), so
                # they carry a large noise scale. A drone BANKING TOWARD the target therefore
                # still anchors the track (it used to be dropped outright — "detects but doesn't
                # follow" during aggressive manoeuvres); an overhead level drone still dominates.
                tilt = max(abs(roll[i]), abs(pitch[i]))
                reacquiring = (t - last_seen_t) > 3.0
                if tilt < 3.5 * LEVEL_TILT or reacquiring:
                    # While REACQUIRING, any sighting beats none — even a banked, image-corner
                    # glimpse is accepted (with a big noise scale) so the swarm turns toward it
                    # immediately instead of finishing its search leg. While tracking, extreme
                    # banking (>~20 deg) is still rejected to protect the established track.
                    tilt_w = (tilt / LEVEL_TILT) ** 2  # 0 when level, ~12 at the reject limit
                    dets.append((e, n))
                    det_noise.append(1.0 + 6.0 * edge * edge + 4.0 * tilt_w)
        if any_detect:
            detect_count += 1

        # --- comms-gated sharing of camera detections between pursuers ---
        for s in range(N):
            if belief[s] is None or t - belief[s][1] > 0.4:
                continue
            for d in range(N):
                if s == d:
                    continue
                rsd = math.hypot(positions[s][0] - positions[d][0], positions[s][1] - positions[d][1])
                if links[(s, d)].try_deliver(t, rsd, 64).delivered:
                    if belief[d] is None or belief[s][1] > belief[d][1]:
                        belief[d] = belief[s]

        # Fuse this tick's detections into the shared Kalman track. Gating rejects the
        # several-metre back-projection outliers (the jumps that made the swarm peel off);
        # on a dropout the filter COASTS on its velocity estimate so the drones keep pursuing
        # and leading the target rather than reverting to search or freezing.
        if dets:
            # TRACK RE-INITIALIZATION — the camera outranks the filter. The Mahalanobis gate
            # protects an established track from single-frame outliers, but its failure mode is
            # rejecting GENUINE sightings whenever the (coasted or badly-seeded) estimate is
            # wrong: the swarm would "see" the target yet keep chasing a phantom. Whenever the
            # filter materially disagrees with what the cameras are actually reporting (mean
            # sighting > 8 m away, or the track is stale by > 3 s), restart the track at the
            # sightings. Standard track management; works for any drone count.
            mE = sum(d[0] for d in dets) / len(dets)
            mN = sum(d[1] for d in dets) / len(dets)
            stale = (t - last_seen_t) > 3.0
            disagree = False
            if tracker.initialized():
                pe, pn = tracker.position()
                disagree = math.hypot(pe - mE, pn - mN) > 8.0
            if stale or disagree:
                tracker = TargetTracker(q_accel=0.6, r_meas=4.0, gate=3.0,
                                        vmax=1.4 * TARGET_SPEED + 1.0)
            tracker.update(dets, t, noise_scales=det_noise)
            last_seen_t = t
        elif tracker.initialized():
            tracker.predict_to(t)
        if tracker.initialized():
            smooth = list(tracker.position())
            # The target is a GROUND vehicle: it cannot be inside a building. If the coasted/
            # fused estimate drifts into a footprint (occluded coasting does this), project it
            # back to the nearest street with the ESDF escape gradient — occlusion-aware
            # constraint of the motion model to free space.
            for _ in range(8):
                if _city_map.distance(smooth[0], smooth[1]) > 0.3:
                    break
                gx, gy = _city_map.gradient(smooth[0], smooth[1])
                smooth[0] += 0.5 * gx
                smooth[1] += 0.5 * gy
            est_vel[0], est_vel[1] = tracker.velocity()
            # deadband: a near-stationary evader -> zero feedforward so the lead point and the
            # drones hover stably over it; real flight still triggers full lead pursuit.
            if math.hypot(est_vel[0], est_vel[1]) < 0.5:
                est_vel[0] = est_vel[1] = 0.0

        with state_lock:
            # Pursue as long as we've seen the target within the last 5 s; only fall back to
            # search after a longer loss. The coasted estimate is CLAMPED to the city: the
            # target physically cannot leave it, so a drifting extrapolation must never drag
            # the swarm off the map (observed live: a phantom led both drones ~80 m out).
            if smooth is not None and t - last_seen_t < 5.0:
                cur_alt[0] = ALT                    # track low: tight, well-conditioned fixes
                tE, tN = clamp(*smooth)
                tvel = (est_vel[0], est_vel[1], 0.0)
                # One interceptor dives at the target while the others hold a tight containment
                # ring to cut off escape. MPC predicts internally so it gets the CURRENT target;
                # the reactive law gets the explicit lead. (A shrinking-encirclement variant was
                # tried but the perception/control lag kept the drones orbiting outside the ring.)
                lead = pursuit.predict_target((tE, tN, 0.0), tvel, 0.7)
                ctr = (tE, tN) if CONTROL == "mpc" else (lead[0], lead[1])
                slots = pursuit.containment_ring((ctr[0], ctr[1], 0.0), CONTAIN_R, N)
                dist = [[math.hypot(positions[a][0] - s[0], positions[a][1] - s[1]) for s in slots]
                        for a in range(N)]
                # Visibility-aware slot value: a slot whose camera sightline to the target is
                # cut by a building is worth far less — the blocker should hold a vantage that
                # KEEPS EYES ON, not just a geometric ring position (occlusion-aware tasking).
                slot_vis = [0.25 if los_blocked(s[0], s[1], ALT, ctr[0], ctr[1], 0.0) else 1.0
                            for s in slots]

                def score(agent, path, task):
                    return slot_vis[task] / (1.0 + dist[agent][task])

                assignment, _ = run_cbba(N, len(slots), score, max_bundle=1)
                interceptor = min(range(N),
                                  key=lambda a: math.hypot(positions[a][0] - tE, positions[a][1] - tN))
                for a in range(N):
                    if a == interceptor:
                        # Only the interceptor's goal moves with the evader -> it alone gets
                        # the velocity feedforward.
                        goal_vels[a][0], goal_vels[a][1] = est_vel[0], est_vel[1]
                        if CONTROL == "mpc":
                            goal[a] = [tE, tN]
                        else:
                            ip = pursuit.intercept_point((positions[a][0], positions[a][1], 0.0),
                                                         (tE, tN, 0.0), tvel, PURSUER_SPEED)
                            goal[a] = [ip[0], ip[1]]
                    else:
                        # Static containment slot: NO feedforward (the slot is recomputed when
                        # the ring centre moves; dragging it at evader velocity makes blockers
                        # orbit/overshoot the ring).
                        goal_vels[a][0] = goal_vels[a][1] = 0.0
                        slot = assignment[a][0] if assignment[a] else a
                        goal[a] = [slots[slot][0], slots[slot][1]]
            else:
                cur_alt[0] = SEARCH_ALT             # search high: footprint spans the area
                for a in range(N):                  # no feedforward while searching
                    goal_vels[a][0] = goal_vels[a][1] = 0.0
                # Search: SENTINEL + STRIP SEARCHERS, derived from the operational area for ANY
                # drone count and ANY (rectangular) environment:
                #   * the LAST drone is the SENTINEL — it holds station over the area centre at
                #     the (highest) search altitude, where its footprint covers most of the map:
                #     a persistent reacquisition anchor that never stops watching;
                #   * drones 0..N-2 are SEARCHERS — the area is split into that many gap-
                #     separated vertical strips (no shared seams -> no converging goals), each
                #     mowed by a serpentine whose corners advance on arrival and persist across
                #     mode switches.
                # With N == 1 the single drone is a searcher over the whole area.
                span = CITY - 2.0
                n_search = max(1, N - 1)
                strip_w = 2.0 * span / n_search
                for a in range(N):
                    if N > 1 and a == N - 1:
                        goal[a] = [0.0, 0.0]        # sentinel: station over the area centre
                        continue
                    x0 = -span + a * strip_w + 1.0  # 2 m gap between neighbouring strips
                    x1 = -span + (a + 1) * strip_w - 1.0
                    n_legs = 5
                    wps = []
                    for k in range(n_legs):         # serpentine corners over this strip
                        yy = -span + 2.0 * span * k / (n_legs - 1)
                        wps.append(((x0, yy) if k % 2 == 0 else (x1, yy)))
                        wps.append(((x1, yy) if k % 2 == 0 else (x0, yy)))
                    wp = wps[search_wp[a] % len(wps)]
                    if math.hypot(positions[a][0] - wp[0], positions[a][1] - wp[1]) < 3.0:
                        search_wp[a] += 1
                        wp = wps[search_wp[a] % len(wps)]
                    goal[a] = [wp[0], wp[1]]
            goal_t[0] = time.time()    # 50 Hz control extrapolates the goal from this instant

        dmin = min(math.hypot(p[0] - target[0], p[1] - target[1]) for p in positions)
        if dmin < CAPTURE_R and not captured:
            captured = True
            print(f"*** TARGET INTERCEPTED (camera-guided) at t={t:.1f}s, closest {dmin:.1f} m ***", flush=True)
        if int(t) % 3 == 0:
            seen = "TRACKING" if (t - last_seen_t < 1.0) else (
                "pursuing(memory)" if (smooth is not None and t - last_seen_t < 8.0) else "searching")
            print(f"  t={t:4.1f}s  {seen}  closest {dmin:4.1f} m  detections={detect_count}", flush=True)
        # record positions for the demo GIF (real telemetry + scripted target + swarm estimate)
        eE, eN = (smooth if smooth is not None else ("", ""))
        _rw.writerow([f"{t:.2f}", f"{target[0]:.2f}", f"{target[1]:.2f}",
                      (f"{eE:.2f}" if eE != "" else ""), (f"{eN:.2f}" if eN != "" else ""),
                      int(captured)]
                     + [f"{positions[i][ax]:.2f}" for i in range(N) for ax in (0, 1)])
        _rec.flush()
        time.sleep(0.02)            # small yield; loop runs ~as fast as detection allows (t/dt set at top)


if __name__ == "__main__":
    main()
