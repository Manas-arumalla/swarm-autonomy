#!/usr/bin/env python3
"""Arm + take off every PX4 SITL instance in a multi-vehicle Gazebo sim.

Usage: swarm_takeoff.py N [takeoff_alt]

PX4 multi-vehicle SITL: instance i binds local MAVLink udp port 18570+i and has
MAV system id i+1. We send a GCS heartbeat + the SITL arming params + an
AUTO.TAKEOFF/ARM to each instance's port, retrying for a while to ride out EKF
warm-up. Heartbeats keep flowing so PX4's "no GCS connection" arming check stays
satisfied.
"""

from __future__ import annotations

import struct
import sys
import time
import threading

from pymavlink import mavutil

N = int(sys.argv[1]) if len(sys.argv) > 1 else 2
ALT = float(sys.argv[2]) if len(sys.argv) > 2 else 2.5
I32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32

conns = [mavutil.mavlink_connection(f"udpout:127.0.0.1:{18570 + i}",
                                    source_system=255, source_component=190)
         for i in range(N)]


def heartbeats():
    while True:
        for c in conns:
            c.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.3)


threading.Thread(target=heartbeats, daemon=True).start()
print(f"GCS heartbeats to {N} vehicles (ports 18570..{18570 + N - 1})", flush=True)
time.sleep(2)

PARAMS = [("NAV_DLL_ACT", 0), ("CBRK_SUPPLY_CHK", 894281),
          ("NAV_RCL_ACT", 0), ("COM_RCL_EXCEPT", 4)]
for i, c in enumerate(conns):
    for name, val in PARAMS:
        c.mav.param_set_send(i + 1, 1, name.encode(), struct.unpack("<f", struct.pack("<i", int(val)))[0], I32)
        time.sleep(0.1)
print("SITL arming params set", flush=True)

# Arm + AUTO.TAKEOFF. Retry only briefly (to ride out EKF/heading warm-up); once
# commanded, STOP re-sending so PX4 takes off to MIS_TAKEOFF_ALT and holds there
# (re-sending takeoff on a loop made them keep climbing).
for attempt in range(4):  # ~15 s of retries
    for i, c in enumerate(conns):
        sysid = i + 1
        c.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                1, 4, 2, 0, 0, 0, 0)  # AUTO.TAKEOFF
        c.mav.command_long_send(sysid, 1,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                1, 0, 0, 0, 0, 0, 0)  # ARM
    if attempt == 0:
        print("arm + takeoff commanded; will stop commanding so they hover at takeoff alt", flush=True)
    time.sleep(4)

print("takeoff done; holding GCS heartbeats only (drones loiter at takeoff alt). Ctrl-C to stop.", flush=True)
while True:
    time.sleep(1)
