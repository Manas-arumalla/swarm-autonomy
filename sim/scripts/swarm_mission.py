#!/usr/bin/env python3
"""Coordinated offboard formation patrol for a PX4 multi-vehicle Gazebo swarm.

Usage: swarm_mission.py N [alt]

Each PX4 SITL instance i (MAV sys id i+1, MAVLink port 18570+i) is driven in
OFFBOARD mode with streamed local-NED position setpoints. Because each drone's
EKF local origin is its own spawn point, sending the SAME (north, east) target to
all of them makes the whole formation translate while keeping its spacing — so
the swarm flies a shared patrol loop over the city, in formation.

Sequence: stream hover setpoints -> set OFFBOARD + arm (retry through EKF warm-up)
-> climb -> walk a rectangular patrol path, looping. Heartbeats keep PX4's GCS
arming check satisfied throughout.
"""

from __future__ import annotations

import struct
import sys
import threading
import time

from pymavlink import mavutil


def _i32_bits(v: int) -> float:
    """PX4 reads the MAVLink param_value float as the raw bits of the typed value, so an INT32
    must be sent as the float whose bit pattern equals the integer (not float(v))."""
    return struct.unpack("<f", struct.pack("<i", int(v)))[0]

N = int(sys.argv[1]) if len(sys.argv) > 1 else 3
ALT = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

# position-only setpoint: ignore vel, accel, force, yaw, yaw_rate (use x,y,z)
TYPE_MASK = 0b0000_1111_1111_1000
FRAME = mavutil.mavlink.MAV_FRAME_LOCAL_NED

conns = [mavutil.mavlink_connection(f"udpout:127.0.0.1:{18570 + i}",
                                    source_system=255, source_component=190)
         for i in range(N)]

# Shared formation target (north, east), relative to each drone's own origin.
target = {"n": 0.0, "e": 0.0}
running = True


def stream_setpoints():
    while running:
        for i, c in enumerate(conns):
            c.mav.set_position_target_local_ned_send(
                0, i + 1, 1, FRAME, TYPE_MASK,
                target["n"], target["e"], -ALT,
                0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(0.05)            # 20 Hz


def heartbeats():
    while running:
        for c in conns:
            c.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.3)


threading.Thread(target=stream_setpoints, daemon=True).start()
threading.Thread(target=heartbeats, daemon=True).start()
print(f"streaming offboard setpoints to {N} vehicles", flush=True)
time.sleep(1.5)

I32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
for i, c in enumerate(conns):
    for name, val in [("NAV_DLL_ACT", 0), ("CBRK_SUPPLY_CHK", 894281),
                      ("NAV_RCL_ACT", 0), ("COM_RCL_EXCEPT", 4)]:
        c.mav.param_set_send(i + 1, 1, name.encode(), _i32_bits(val), I32)
        time.sleep(0.05)
time.sleep(1)

# OFFBOARD (custom main mode 6) + ARM, retried through EKF warm-up.
for attempt in range(5):
    for i, c in enumerate(conns):
        c.mav.command_long_send(i + 1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                1, 6, 0, 0, 0, 0, 0)
        c.mav.command_long_send(i + 1, 1,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                1, 0, 0, 0, 0, 0, 0)
    if attempt == 0:
        print("OFFBOARD + arm commanded; climbing to altitude", flush=True)
    time.sleep(3)

time.sleep(4)
PATROL = [(0.0, 0.0), (18.0, 0.0), (18.0, 18.0), (0.0, 18.0)]   # N, E corners
print("patrolling the city in formation (rectangular loop)", flush=True)
wp = 0
while True:
    target["n"], target["e"] = PATROL[wp]
    print(f"  formation -> waypoint {wp}: N={target['n']:.0f} E={target['e']:.0f}", flush=True)
    time.sleep(10)
    wp = (wp + 1) % len(PATROL)
