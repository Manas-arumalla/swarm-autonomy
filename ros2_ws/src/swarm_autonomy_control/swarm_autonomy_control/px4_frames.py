"""PX4 MAVLink frame + setpoint-mask helpers (pure, unit-tested).

PX4's LOCAL_NED frame is (x = North, y = East, z = Down); the swarm code plans in world
(East, North, Up). A single wrong bit in the SET_POSITION_TARGET_LOCAL_NED ``type_mask`` (a SET
bit means *ignore* that field) or one swapped/sign-flipped axis silently re-routes the whole
swarm — PX4 falls back to treating the zero position fields as a fly-to-origin command, which
shows up as drones charging past their goals and sagging in altitude. These helpers centralise
the conversions and the mask so the conventions are locked by unit tests instead of comments.
"""

from __future__ import annotations

# --- POSITION_TARGET_TYPEMASK ignore bits (MAVLink common.xml) -----------------------------
IGNORE_PX = 1 << 0
IGNORE_PY = 1 << 1
IGNORE_PZ = 1 << 2
IGNORE_VX = 1 << 3
IGNORE_VY = 1 << 4
IGNORE_VZ = 1 << 5
IGNORE_AX = 1 << 6
IGNORE_AY = 1 << 7
IGNORE_AZ = 1 << 8
FORCE_SET = 1 << 9
IGNORE_YAW = 1 << 10
IGNORE_YAW_RATE = 1 << 11

#: Velocity-only offboard setpoint: position/accel/force/yaw ignored, velocity USED (bits clear).
VELOCITY_SETPOINT_MASK = (
    IGNORE_PX | IGNORE_PY | IGNORE_PZ
    | IGNORE_AX | IGNORE_AY | IGNORE_AZ
    | FORCE_SET | IGNORE_YAW | IGNORE_YAW_RATE
)
assert VELOCITY_SETPOINT_MASK & (IGNORE_VX | IGNORE_VY | IGNORE_VZ) == 0, \
    "velocity bits must be CLEAR (clear = field is used)"

#: Position-only offboard setpoint: velocity/accel/force/yaw ignored, position USED.
POSITION_SETPOINT_MASK = (
    IGNORE_VX | IGNORE_VY | IGNORE_VZ
    | IGNORE_AX | IGNORE_AY | IGNORE_AZ
    | FORCE_SET | IGNORE_YAW | IGNORE_YAW_RATE
)


# --- frame conversions ----------------------------------------------------------------------
def ned_to_world_pos(north: float, east: float,
                     spawn_east: float = 0.0, spawn_north: float = 0.0):
    """PX4 LOCAL_NED position (origin = the vehicle's spawn) -> world (East, North)."""
    return (spawn_east + east, spawn_north + north)


def ned_to_world_vel(v_north: float, v_east: float):
    """PX4 LOCAL_NED velocity -> world (vE, vN)."""
    return (v_east, v_north)


def world_to_ned_vel(v_east: float, v_north: float, v_up: float):
    """World (vE, vN, vUp) -> the (vx, vy, vz) fields of SET_POSITION_TARGET_LOCAL_NED.

    NED: vx = North, vy = East, vz = Down (so climbing is NEGATIVE vz).
    """
    return (v_north, v_east, -v_up)
