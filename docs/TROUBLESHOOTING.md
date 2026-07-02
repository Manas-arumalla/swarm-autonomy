# Troubleshooting: PX4 SITL multi-vehicle + Gazebo Harmonic

Field notes from getting multiple PX4 SITL vehicles, Gazebo Harmonic, and a vision pipeline to
run reliably on one machine. Every entry here is a problem I actually hit while building this
project, written as *symptom → cause → fix* so it can be pattern-matched quickly. Most of these
fail silently or with misleading errors, which is what makes multi-vehicle SITL painful.

---

## Launch and world management

### Drones 1..N spawn their own empty worlds instead of joining drone 0's city

**Symptom.** The first PX4 instance starts the city world fine; later instances each open a
second Gazebo server running `default.sdf`. Vehicles exist but in *different* simulations —
cameras see nothing, "closest distance" telemetry freezes, everything desynchronizes.

**Cause.** `px4-rc.gzsim` decides whether to *start* or *join* Gazebo by looking for an existing
`/world/*/clock` topic at the moment the instance boots. If instance *i* launches before the
world's clock topic is discoverable (slow first render, cold shader cache, transport hiccup), it
concludes no simulation exists and starts its own.

**Fix.** Gate each launch on the world actually being up, not on a fixed `sleep`:

```bash
until gz topic -l | grep -q "^/world/$WORLD/clock"; do sleep 1; done   # before px4 -i 1
```

Also wait for the previous instance's *model* to exist (`gz model --list`) before launching the
next. Do **not** reach for `PX4_GZ_STANDALONE=1` to "fix" this — standalone mode makes the
instance wait for an already-spawned model and never spawns one itself, which is the opposite of
what a joining vehicle needs.

### Starting the Gazebo server yourself: models spawn "successfully" but have no sensors

**Symptom.** You start `gz sim -s <world>` manually (more reliable than letting PX4 do it), then
launch PX4. PX4 logs `Spawning Gazebo model` and `Startup script returned successfully` — but the
model never appears, or it appears and EKF2 reports every sensor missing (`Accel Sensor 0
missing`, `ekf2 missing data`).

**Cause.** PX4's models and, critically, its **sensor system plugins** resolve through
`GZ_SIM_RESOURCE_PATH`, `GZ_SIM_SYSTEM_PLUGIN_PATH`, and `GZ_SIM_SERVER_CONFIG_PATH`. When PX4
starts Gazebo itself it sources `gz_env.sh` first; a manually started server knows none of those
paths, so the spawn request references resources the server cannot load — and it fails without a
useful error.

**Fix.** Source the environment before starting the server:

```bash
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
gz sim -r -s "$PX4_GZ_WORLDS/my_world.sdf" &
```

### PX4 times out waiting for a slow-loading world

**Symptom.** `ERROR [init] Timed out waiting for Gazebo world`, then
`Startup script returned with return value: 256`. Heavier custom worlds trip this on first load.

**Cause.** `px4-rc.gzsim` polls the world's `scene/info` service ~30 times at 1 s intervals and
gives up.

**Fix.** Either pre-start the server and let PX4 join it (see above — the world is then already
up), or raise `ATTEMPTS` in `build/px4_sitl_default/etc/init.d-posix/px4-rc.gzsim` for the local
build.

---

## Gazebo transport

### The GUI freezes, `gz topic -l` shows nothing, PX4 waits forever — and a VPN is running

**Symptom.** The server process is alive and consuming CPU, but its topics are invisible to every
other process; the GUI opens and immediately shows "Gazebo GUI is not responding"; PX4 loops on
`Waiting for Gazebo world...`.

**Cause.** gz-transport discovers peers via UDP multicast on a network interface. A VPN tunnel
interface (WireGuard/`nordlynx` in my case) does not carry multicast, and gz can bind discovery
to it — every gz process then runs in its own invisible bubble. This one cost me an afternoon
because nothing prints an error.

**Fix.** Force transport onto loopback for every gz *and* PX4 process:

```bash
export GZ_IP=127.0.0.1
```

Also use a dedicated `GZ_PARTITION` per project so unrelated simulations on the machine can never
cross-talk.

### Cameras publish black frames when running headless

**Symptom.** With no GUI (`HEADLESS=1` and no display), camera topics publish frames but they are
entirely black; a colour detector simply never fires, with no error anywhere.

**Cause.** Camera sensors need a working GPU rendering context. Without a display/EGL setup the
render pipeline silently produces empty images.

**Fix.** Run the GUI (or configure offscreen EGL rendering) whenever camera output matters, and
add a watchdog to vision code: if frames arrive but stay at ~zero mean intensity for a few
seconds, say so loudly instead of "searching" forever.

---

## MAVLink and offboard control

### Which UDP port actually talks to instance *i*

Two link styles, easy to conflate:

- `udpin:0.0.0.0:14540+i` — bind locally and let PX4's *onboard* link (which initiates to
  `14540+i`) find you. Works, but silently depends on every instance's onboard link being up and
  on the single-world invariant holding.
- `udpout:127.0.0.1:18570+i` — connect out to the port PX4 listens on per instance. The more
  robust choice for scripted multi-vehicle control.

Whichever you use, filter received messages by `msg.get_srcSystem() == i + 1`; multi-vehicle
sockets see stray traffic.

### `param_set` corrupts INT32 parameters

**Symptom.** You set an INT32 parameter (e.g. `EKF2_GPS_CTRL`, `CBRK_SUPPLY_CHK`) over MAVLink
and PX4 stores garbage, or arming checks behave as if the parameter never changed.

**Cause.** The MAVLink `param_value` field is a float, but for integer parameters PX4 reads its
**raw bit pattern** as the integer. Sending `float(894281)` stores the bits of the float — a
completely different number.

**Fix.**

```python
import struct
param_value = struct.unpack("<f", struct.pack("<i", int(value)))[0]   # bit-reinterpret
conn.mav.param_set_send(sysid, 1, name, param_value, MAV_PARAM_TYPE_INT32)
```

### OFFBOARD engages sometimes, arms never, or both intermittently

**Symptom.** Blind `DO_SET_MODE` + `ARM_DISARM` bursts work on lucky runs and silently fail on
others; vehicles sit on the ground while the mission logic runs.

**Cause.** PX4 refuses OFFBOARD until setpoints have been streaming (>2 Hz) for a warm-up period,
and refuses arming until EKF preflight checks pass — both race a script that fires commands
immediately after boot.

**Fix.** Stream setpoints first, wait for EKF readiness (or a fixed ~20 s after model spawn in
SITL), then retry mode+arm in a loop until the heartbeat actually reports
`custom_mode == OFFBOARD` and `MAV_MODE_FLAG_SAFETY_ARMED`. Assume nothing succeeded until the
vehicle says so.

---

## State estimation

### Lockstep time: external-vision data is silently ignored — or destabilizes the filter

**Symptom.** A VIO→EKF2 bridge publishes visually perfect odometry and EKF2 fuses none of it; or
it fuses and control becomes twitchy, then diverges when GPS is removed.

**Cause.** PX4 SITL runs on lockstep *simulation* time. Stamping vision messages with wall-clock
time makes them look decades in the future — EKF2 drops every sample without complaint. Stamping
with "now" instead of the *capture* time makes ~150 ms-old measurements look current, which skews
the filter enough to matter exactly when GPS is gone.

**Fix.** Stamp with the odometry/image header time (the camera and PX4 share the lockstep clock)
and fuse position-only external vision unless you have a reason to trust EV yaw — fusing EV yaw
rotated the whole frame into a runaway in my testing.

### The EKF's local-position belief walks away on long flights

**Symptom.** On long aggressive runs with GPS noise, vehicles slowly leave the operational area
even though every commanded goal is inside it; downstream, any camera back-projection is offset
by the same amount as the self-position error — I measured a flat ~5–6 m bias in target fixes
that had nothing to do with the camera.

**Cause.** EKF2's local origin can drift and occasionally reset under sustained GPS noise; the
whole world-frame belief shifts, and everything derived from "where am I" inherits the error 1:1.

**Fix.** In simulation experiments where self-localization is not the thing under test, take
self-pose from simulator ground truth (declared in the code) — and treat flying the full loop on
the EKF belief as its own workstream. Independently, add a command-layer **geofence**: outside
the operational bound, override the command toward home. No upstream estimation fault should be
able to take a vehicle out of the test volume.

### Monocular VIO under-scales everything by ~5×

**Symptom.** GPS-denied handover diverges; the VIO's reconstructed trajectory is a perfect copy
of the real one at one-fifth the size.

**Cause.** Monocular-inertial scale is observable only through linear acceleration, and a gentle
survey trajectory has almost none — the scale ambiguity leaks into the accelerometer bias.

**Fix.** Stereo (scale from the baseline, not from motion), plus a trajectory-shape (Umeyama)
alignment between the VIO and EKF2 frames instead of a single-yaw-sample alignment. The full
diagnosis is in [engineering-notes.md](engineering-notes.md).

---

## Process hygiene

- **Kill scripts self-matching:** `pkill -f "px4|gz sim"` typed inline matches *its own shell's*
  command line and kills the very shell it was typed in. Put cleanup in a script file and run that.
- **Launcher scripts that respawn:** killing the stack but not the launcher that started it means
  the stack comes back mid-cleanup and races the next run. Kill launchers first, then processes.
- **One diagnostic at a time:** most of the failures above produce *no error message*. The only
  reliable approach I found is instrumenting one invariant per run (is the world's clock topic
  visible? does the model exist? is the heartbeat's mode OFFBOARD? does belief match truth?) and
  letting the failing invariant name the layer.
