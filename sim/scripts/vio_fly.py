"""Fly the cam+IMU drone gently for clean VIO and (optionally) hand PX4 over to
GPS-denied external-vision flight.

Motion: fixed-yaw lateral circle with a small +/-20deg yaw WOBBLE -- the wobble gives
OpenVINS dynamic init the >10deg gyro excitation it needs without sweeping the forward
camera up to the featureless sky (which would diverge the filter).

EKF2 wiring: at startup we enable external-vision fusion (EKF2_EV_CTRL) but keep GPS on
so takeoff + VIO init work. We also publish PX4's own NED pose + the wall->PX4(lockstep)
time offset to /tmp/px4_pose.txt so the vision bridge can (a) align the VIO frame and
(b) stamp VISION_POSITION_ESTIMATE in PX4's clock. Touch /tmp/go_gps_denied to drop GPS
(EKF2_GPS_CTRL=0) -> the drone then navigates on OpenVINS alone.
"""
import math, threading, time, os, struct
from pymavlink import mavutil

c = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=255, source_component=190)
ALT = 3.0; R = 5.0; YAW = 0.0     # fixed yaw 0 -> forward cam faces +N (pillars)
RATE = 0.20                        # circle speed (translation -> parallax)
WOBBLE_A = 0.22                    # CONTINUOUS gentle yaw wobble (~13deg): keeps VIO scale/bias
WOBBLE_W = 0.5                     # observable (under-excited smooth flight diverges) w/o
                                   # sweeping features too hard (that adds drift)
POS_MASK = 0b0000_1011_1111_1000   # position + yaw
st = {"n": 0, "e": 0, "alt": 0, "d": 0, "yaw": 0, "toff": 0}
tgt = {"n": 0, "e": 0, "yaw": YAW}; run = True


def tel():
    last = 0
    while run:
        m = c.recv_match(blocking=False)
        if m:
            t = m.get_type()
            if t == "LOCAL_POSITION_NED":
                st["n"], st["e"], st["alt"], st["d"] = m.x, m.y, -m.z, m.z
            elif t == "ATTITUDE":
                st["yaw"] = m.yaw
            elif t == "STATUSTEXT":   # surface PX4 preflight/arming reasons in the flight log
                print(f"[PX4] {m.text}", flush=True)
            elif t == "COMMAND_ACK" and getattr(m, "command", 0) == 400:
                print(f"[ACK] arm result={m.result} (0=OK,1=TEMP_REJECT,2=DENIED,4=FAILED)", flush=True)
            elif t == "PARAM_VALUE" and m.param_id == "NAV_DLL_ACT":
                print(f"[PARAM] NAV_DLL_ACT={m.param_value}", flush=True)
            elif t == "GPS_RAW_INT" and not st.get("_gps"):
                st["_gps"] = 1
                print(f"[GPS] fix_type={m.fix_type} sats={m.satellites_visible} "
                      f"lat={m.lat/1e7:.6f} lon={m.lon/1e7:.6f} eph={m.eph/100.0:.1f}m", flush=True)
            elif t == "HOME_POSITION" and not st.get("_home"):
                st["_home"] = 1
                print(f"[HOME] set lat={m.latitude/1e7:.5f} lon={m.longitude/1e7:.5f}", flush=True)
            elif t == "HEARTBEAT" and m.get_srcSystem() == 1:
                if st.get("_sys") != m.system_status:
                    st["_sys"] = m.system_status
                    print(f"[HB] sys_status={m.system_status} (0=UNINIT,3=STANDBY,4=ARMED) custom_mode={m.custom_mode}", flush=True)
            elif t == "ESTIMATOR_STATUS" and not st.get("_est"):
                F = m.flags
                st["_est"] = 1
                print(f"[EST] pos_h_ratio={m.pos_horiz_ratio:.2f} pos_v_ratio={m.pos_vert_ratio:.2f} "
                      f"POS_HORIZ_ABS={bool(F & (1<<5))} PRED_POS_ABS={bool(F & (1<<10))} GPS_GLITCH={bool(F & (1<<12))} ACCEL_ERR={bool(F & (1<<13))}", flush=True)
            tb = getattr(m, "time_boot_ms", None)
            if tb:  # wall -> PX4 lockstep-sim time offset (us)
                st["toff"] = tb * 1000.0 - time.time() * 1e6
        else:
            time.sleep(0.003)
        now = time.time()
        if now - last > 0.05:
            last = now
            try:
                with open("/tmp/px4_pose.txt", "w") as f:
                    f.write(f"{st['n']} {st['e']} {st['d']} {st['yaw']} {st['toff']}")
            except OSError:
                pass


def stream():
    while run:
        c.mav.set_position_target_local_ned_send(
            0, 1, 1, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            POS_MASK, tgt["n"], tgt["e"], -ALT, 0, 0, 0, 0, 0, 0, tgt["yaw"], 0)
        time.sleep(0.05)


def beat():
    while run:
        c.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.3)


for fn in (tel, stream, beat):
    threading.Thread(target=fn, daemon=True).start()
time.sleep(1.5)

I32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
R32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
def pset(nm, v, t):  # PARAM_SET is fire-and-forget over UDP -> send several times
    # PX4 reads the param_value float field as RAW BITS of the typed value. For INT32
    # params we must reinterpret the int's bytes as a float, else PX4 stores garbage
    # (e.g. int 7 sent as float 7.0 -> stored as 1088421888, corrupting EKF2_GPS_CTRL).
    send = struct.unpack("<f", struct.pack("<i", int(v)))[0] if t == I32 else float(v)
    for _ in range(4):
        c.mav.param_set_send(1, 1, nm.encode(), send, t); time.sleep(0.06)

for nm, v in [("NAV_DLL_ACT", 0), ("CBRK_SUPPLY_CHK", 894281), ("NAV_RCL_ACT", 0), ("COM_RCL_EXCEPT", 4)]:
    pset(nm, v, I32)
# Clean GPS-only config for takeoff. EV stays OFF (enabling it before vision data exists
# fails the EKF2 health check). GPS_CTRL may have been left at 0 by a prior GPS-denied run,
# so force it back to 7 and give EKF2 time to re-converge before arming -- otherwise the
# estimator is mid-reset and arming is denied ("system health failures").
# EKF2_GPS_CHECK=0: accept the sim GPS without the strict quality gates so EKF2 sets its
# local origin + home (otherwise home never sets, sys stays UNINIT, offboard arm is rejected).
for nm, v in [("EKF2_EV_CTRL", 0), ("COM_ARM_WO_GPS", 1), ("EKF2_GPS_CTRL", 7), ("EKF2_GPS_CHECK", 0)]:
    pset(nm, v, I32)
c.mav.param_request_read_send(1, 1, b"NAV_DLL_ACT", -1)   # confirm it actually took
print("waiting 12s for EKF2 to converge on GPS...", flush=True)
time.sleep(12)

for _ in range(120):
    c.mav.param_set_send(1, 1, b"NAV_DLL_ACT", 0.0, I32)   # keep the GCS-required check disabled
    c.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
    # param2=21196 = force-arm magic: bypass remaining preflight checks (SITL demo).
    c.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    if st["alt"] > ALT - 1:
        break
    time.sleep(0.5)
print(f"airborne {st['alt']:.1f}m — fixed-yaw gentle circle for VIO", flush=True)

t0 = time.time(); gps_off = False
while run:
    t = time.time() - t0
    ang = t * RATE
    tgt["e"] = R * math.cos(ang); tgt["n"] = R * math.sin(ang)
    # CONTINUOUS gentle yaw wobble: keeps VIO scale/biases observable (a smooth constant-speed
    # circle is under-excited and the filter diverges -> ba blows up). Gentle (~13deg) so it
    # doesn't sweep features hard, which the richer world + more features further stabilize.
    tgt["yaw"] = YAW + WOBBLE_A * math.sin(t * WOBBLE_W)
    if not gps_off and os.path.exists("/tmp/go_gps_denied"):
        # Proper hand-off: configure + enable EV fusion first, let EKF2 lock onto vision,
        # THEN drop GPS.
        pset("EKF2_EV_NOISE_MD", 1, I32)
        # EVP_NOISE loose enough that the ~1-2m VIO-vs-GPS error (drift + latency) stays inside
        # the EV innovation gate, so EKF2 keeps fusing vision after GPS is dropped.
        pset("EKF2_EVP_NOISE", 1.0, R32); pset("EKF2_EVA_NOISE", 0.3, R32); pset("EKF2_EV_DELAY", 60.0, R32)
        # 1 = horizontal POSITION ONLY (no EV yaw -> mag keeps heading, baro keeps height).
        # Fusing EV yaw made the frame rotate on any orientation-transform error -> runaway.
        pset("EKF2_EV_CTRL", 1, I32)
        print(f"EV fusion enabled at t={t:.0f}s — letting EKF2 lock vision (5s)...", flush=True)
        time.sleep(5.0)
        pset("EKF2_GPS_CTRL", 0, I32)
        print(f"*** GPS DISABLED at t={t:.0f}s — now flying on VIO only ***", flush=True)
        gps_off = True
    time.sleep(0.1)
