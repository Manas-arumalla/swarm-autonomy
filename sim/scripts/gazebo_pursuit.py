#!/usr/bin/env python3
"""Swarm Autonomy pursuit in Gazebo: a decentralized swarm corners a fleeing target.

Usage: gazebo_pursuit.py N_PURSUERS [spacing] [alt]

Runs the pursuit scenario on the real PX4/Gazebo swarm, driven by the Swarm Autonomy
algorithm modules:
  * swarm_autonomy_coordination.cbba    — decentralized role / containment-slot allocation
  * swarm_autonomy_coordination.pursuit — interception geometry + containment ring
  * swarm_autonomy_comms.link_model     — range-gated target-observation sharing

Setup: launch N_PURSUERS+1 PX4 SITL vehicles. The LAST one is the evader (flies a
scripted evasion away from the nearest pursuer); the rest are pursuers. Every
vehicle is flown in OFFBOARD mode via streamed local-NED position setpoints.

Frames: gz world is ENU (x=East, y=North, z=Up); PX4 local NED has origin at each
vehicle's spawn. Vehicle i spawns at (spacing*i East, 0 North). World ENU pos of a
vehicle = (spawn_E + local.e, spawn_N + local.n); a world goal (gE,gN,alt) maps to
that vehicle's NED setpoint (n=gN-spawn_N, e=gE-spawn_E, d=-alt).

Each pursuer only knows the target from its own sensing or from a neighbour message
that survives the comms link model.
"""

from __future__ import annotations

import struct
import math
import os
import sys
import threading
import time

# Make the Swarm Autonomy algorithm packages importable (no ROS needed; none import rclpy).
_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.normpath(os.path.join(_HERE, "..", "..", "ros2_ws", "src"))
for _p in ("swarm_autonomy_coordination", "swarm_autonomy_comms"):
    sys.path.insert(0, os.path.join(_SRC, _p))

from pymavlink import mavutil
from swarm_autonomy_coordination import pursuit
from swarm_autonomy_coordination.cbba import run_cbba
from swarm_autonomy_coordination.swarm_control import control_velocity
from swarm_autonomy_comms.link_model import LinkConfig, LinkModel

# ---- config ---------------------------------------------------------------
NP = int(sys.argv[1]) if len(sys.argv) > 1 else 3          # number of pursuers
SPACING = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0  # spawn spacing (m, East)
ALT = float(sys.argv[3]) if len(sys.argv) > 3 else 16.0    # flight altitude (m) — above rooftops
N = NP + 1                                                  # +1 evader (last index)
EVADER = NP

CAPTURE_R = 3.0
CONTAIN_R = 6.0
SENSOR_R = 18.0
PURSUER_SPEED = 5.0
EVADER_SPEED = 3.6                                          # slower than pursuers (catchable)
CITY = 20.0                                                 # patrol/escape half-extent
PREDICT_S = 1.0                                             # interception lead time
SAFETY_R = 2.5                                              # inter-drone safety radius (m)
KP_Z = 1.2                                                  # altitude-hold gain
# velocity-control setpoint: ignore position, accel, force, yaw, yaw_rate (use vx,vy,vz)
VEL_MASK = 0b0000_1111_1100_0111      # = 4039
FRAME = mavutil.mavlink.MAV_FRAME_LOCAL_NED

spawn = [(SPACING * i, 0.0) for i in range(N)]              # (East, North) per vehicle
# Connect on each instance's onboard API port (14540+i): this BOTH receives the
# vehicle's telemetry (LOCAL_POSITION_NED) and sends it commands/setpoints — the
# standard PX4 offboard link. (A send-only socket gets no telemetry, which makes
# altitude-hold blind and the swarm climb forever.)
conns = [mavutil.mavlink_connection(f"udpin:0.0.0.0:{14540 + i}",
                                    source_system=255, source_component=190)
         for i in range(N)]

# Shared state updated by the control loop, consumed by the setpoint streamer.
goal = [[spawn[i][0], spawn[i][1]] for i in range(N)]       # world ENU goal per vehicle
state_lock = threading.Lock()
running = True

# Per-pursuer belief of the evader's world position, and the comms links.
belief = [None] * NP
links = {}
_cfg = LinkConfig(bandwidth_bps=1e9, max_range_m=18.0, soft_range_m=12.0, base_loss=0.03)
for a in range(NP):
    for b in range(NP):
        if a != b:
            links[(a, b)] = LinkModel(_cfg, __import__("random").Random(a * 131 + b))


def world_to_ned(i, gE, gN):
    """World ENU goal -> vehicle i local-NED setpoint (n, e)."""
    sE, sN = spawn[i]
    return (gN - sN, gE - sE)


def stream():
    """Closed-loop velocity control: each vehicle flies toward its goal with
    decentralized reciprocal collision avoidance against all the others."""
    while running:
        with state_lock:
            g = [list(x) for x in goal]
        positions = [world_pos(i) for i in range(N)]
        velocities = [world_vel(i) for i in range(N)]
        for i, c in enumerate(conns):
            vmax = EVADER_SPEED if i == EVADER else PURSUER_SPEED
            neighbors = [(positions[j], velocities[j]) for j in range(N) if j != i]
            vE, vN = control_velocity(positions[i], velocities[i], (g[i][0], g[i][1]),
                                      neighbors, vmax, safety_radius=SAFETY_R)
            vUp = max(-2.5, min(2.5, KP_Z * (ALT - alt_up[i])))   # altitude hold
            # world ENU velocity -> PX4 NED: vn=North, ve=East, vd=-Up
            c.mav.set_position_target_local_ned_send(
                0, i + 1, 1, FRAME, VEL_MASK, 0, 0, 0,
                vN, vE, -vUp, 0, 0, 0, 0, 0)
        time.sleep(0.05)


def heartbeats():
    while running:
        for c in conns:
            c.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.3)


# Track each vehicle's local-NED state from telemetry, -> world ENU.
pos_ned = [(0.0, 0.0)] * N      # (north, east)
vel_ned = [(0.0, 0.0)] * N      # (vnorth, veast)
alt_up = [0.0] * N              # height above spawn (m)


def telemetry():
    while running:
        for i, c in enumerate(conns):
            m = c.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            if m:
                pos_ned[i] = (m.x, m.y)               # (north, east)
                vel_ned[i] = (m.vx, m.vy)             # (vnorth, veast)
                alt_up[i] = -m.z                       # up
        time.sleep(0.02)


def world_pos(i):
    n, e = pos_ned[i]
    sE, sN = spawn[i]
    return (sE + e, sN + n)                            # (East, North)


def world_vel(i):
    vn, ve = vel_ned[i]
    return (ve, vn)                                    # (vEast, vNorth)


def clamp_city(E, N_):
    return (max(-CITY, min(CITY, E)), max(-CITY, min(CITY, N_)))


def main():
    threading.Thread(target=stream, daemon=True).start()
    threading.Thread(target=heartbeats, daemon=True).start()
    threading.Thread(target=telemetry, daemon=True).start()
    print(f"pursuit: {NP} pursuers + 1 evader streaming offboard setpoints", flush=True)
    time.sleep(1.5)

    I32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
    for i, c in enumerate(conns):
        for name, val in [("NAV_DLL_ACT", 0), ("CBRK_SUPPLY_CHK", 894281),
                          ("NAV_RCL_ACT", 0), ("COM_RCL_EXCEPT", 4)]:
            c.mav.param_set_send(i + 1, 1, name.encode(), struct.unpack("<f", struct.pack("<i", int(val)))[0], I32)
            time.sleep(0.05)
    time.sleep(1)

    for attempt in range(6):                           # OFFBOARD + arm, ride out EKF warm-up
        for i, c in enumerate(conns):
            c.mav.command_long_send(i + 1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
            c.mav.command_long_send(i + 1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        if attempt == 0:
            print("OFFBOARD + arm; climbing to altitude...", flush=True)
        time.sleep(3)

    # Give the evader a head start to a city corner.
    with state_lock:
        goal[EVADER] = list(clamp_city(CITY - 4, CITY - 4))
    print("hunt on: swarm vs fleeing target", flush=True)
    time.sleep(4)

    t = 0.0
    captured = False
    prev_ev, prev_t = world_pos(EVADER), 0.0
    ev_vel = (0.0, 0.0)
    while running:
        ev = world_pos(EVADER)
        pursuers = [world_pos(i) for i in range(NP)]

        # Estimate the target's velocity (for interception lead), lightly smoothed.
        dt = max(1e-3, t - prev_t)
        if t - prev_t >= 0.3:
            vx = (ev[0] - prev_ev[0]) / dt
            vy = (ev[1] - prev_ev[1]) / dt
            ev_vel = (0.6 * ev_vel[0] + 0.4 * vx, 0.6 * ev_vel[1] + 0.4 * vy)
            prev_ev, prev_t = ev, t

        # --- evader: flee from the nearest pursuer, stay in the city ---
        nearest = min(pursuers, key=lambda p: math.hypot(p[0] - ev[0], p[1] - ev[1]))
        away = math.atan2(ev[1] - nearest[1], ev[0] - nearest[0])
        best, bestscore = None, -1e9
        for dh in [a * 0.4 for a in range(-3, 4)]:
            h = away + dh
            cand = clamp_city(ev[0] + math.cos(h) * 8, ev[1] + math.sin(h) * 8)
            sc = min(math.hypot(p[0] - cand[0], p[1] - cand[1]) for p in pursuers)
            if sc > bestscore:
                best, bestscore = cand, sc
        with state_lock:
            goal[EVADER] = list(best)

        # --- decentralized target belief: own sensing + comms-gated sharing ---
        for i in range(NP):
            if math.hypot(pursuers[i][0] - ev[0], pursuers[i][1] - ev[1]) <= SENSOR_R:
                belief[i] = (ev, t)
        for s in range(NP):
            if belief[s] is None or t - belief[s][1] > 0.4:
                continue
            for d in range(NP):
                if s == d:
                    continue
                rng_sd = math.hypot(pursuers[s][0] - pursuers[d][0], pursuers[s][1] - pursuers[d][1])
                if links[(s, d)].try_deliver(t, rng_sd, 64).delivered:
                    if belief[d] is None or belief[s][1] > belief[d][1]:
                        belief[d] = belief[s]

        # --- fused estimate that has propagated; CBBA containment + pursuit ---
        fused = None
        for b in belief:
            if b is not None and (fused is None or b[1] > fused[1]):
                fused = b

        if fused is not None and t - fused[1] < 3.0:
            tgt = fused[0]
            # Lead the target: build the containment ring around its predicted position.
            pred = pursuit.predict_target((tgt[0], tgt[1], 0.0),
                                          (ev_vel[0], ev_vel[1], 0.0), PREDICT_S)
            slots = pursuit.containment_ring(pred, CONTAIN_R, NP)
            dist = [[math.hypot(pursuers[a][0] - s[0], pursuers[a][1] - s[1]) for s in slots]
                    for a in range(NP)]

            def score(agent, path, task):
                return 1.0 / (1.0 + dist[agent][task])

            assignment, _ = run_cbba(NP, len(slots), score, max_bundle=1)
            interceptor = min(range(NP),
                              key=lambda a: math.hypot(pursuers[a][0] - tgt[0], pursuers[a][1] - tgt[1]))
            ip = pursuit.intercept_point(
                (pursuers[interceptor][0], pursuers[interceptor][1], 0.0),
                (tgt[0], tgt[1], 0.0), (ev_vel[0], ev_vel[1], 0.0), PURSUER_SPEED)
            with state_lock:
                for a in range(NP):
                    if a == interceptor:
                        goal[a] = list(clamp_city(ip[0], ip[1]))
                    else:
                        slot = assignment[a][0] if assignment[a] else a
                        goal[a] = list(clamp_city(slots[slot][0], slots[slot][1]))
        else:
            # No fresh estimate: fan out and search the city.
            with state_lock:
                for a in range(NP):
                    ang = 2 * math.pi * a / NP + t * 0.2
                    goal[a] = list(clamp_city(math.cos(ang) * 14, math.sin(ang) * 14))

        dmin = min(math.hypot(p[0] - ev[0], p[1] - ev[1]) for p in pursuers)
        if dmin < CAPTURE_R and not captured:
            captured = True
            print(f"*** TARGET INTERCEPTED at t={t:.1f}s (closest {dmin:.1f} m) ***", flush=True)
        if int(t) % 3 == 0:
            role = "tracking" if fused else "searching"
            print(f"  t={t:4.1f}s  {role}  closest pursuer-target {dmin:4.1f} m", flush=True)
        time.sleep(0.2)
        t += 0.2


if __name__ == "__main__":
    main()
