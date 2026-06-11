"""Lock the PX4 NED<->world conventions and the offboard setpoint mask with tests.

A single wrong ignore-bit or swapped axis silently re-routes the swarm (PX4 would fall back to
flying to the zero position fields), so the conventions are asserted here rather than trusted.
"""
from swarm_autonomy_control.px4_frames import (
    IGNORE_VX,
    IGNORE_VY,
    IGNORE_VZ,
    POSITION_SETPOINT_MASK,
    VELOCITY_SETPOINT_MASK,
    ned_to_world_pos,
    ned_to_world_vel,
    world_to_ned_vel,
)


def test_velocity_mask_uses_velocity_and_ignores_the_rest():
    # velocity bits CLEAR (used); position/accel/force/yaw bits SET (ignored)
    assert VELOCITY_SETPOINT_MASK & (IGNORE_VX | IGNORE_VY | IGNORE_VZ) == 0
    assert VELOCITY_SETPOINT_MASK == 0b0000_1111_1100_0111   # the long-serving literal, now named


def test_position_mask_uses_position():
    assert POSITION_SETPOINT_MASK & 0b111 == 0               # position bits clear (used)
    assert POSITION_SETPOINT_MASK & (IGNORE_VX | IGNORE_VY | IGNORE_VZ) \
        == (IGNORE_VX | IGNORE_VY | IGNORE_VZ)               # velocity ignored


def test_ned_world_round_trip():
    # A drone spawned at world (East=2, North=-6) that PX4 reports at local NED
    # (north=3, east=1) sits at world (3, -3).
    assert ned_to_world_pos(north=3.0, east=1.0, spawn_east=2.0, spawn_north=-6.0) == (3.0, -3.0)
    # NED velocity (vn=1, ve=2) is world (vE=2, vN=1) ...
    assert ned_to_world_vel(v_north=1.0, v_east=2.0) == (2.0, 1.0)
    # ... and sending world (vE=2, vN=1, climb 0.5) must emit NED (vx=1, vy=2, vz=-0.5).
    assert world_to_ned_vel(v_east=2.0, v_north=1.0, v_up=0.5) == (1.0, 2.0, -0.5)


def test_climb_is_negative_vz():
    # NED z is DOWN: a climb command must map to negative vz (the classic sag-into-the-ground
    # sign error this module exists to prevent).
    _, _, vz = world_to_ned_vel(0.0, 0.0, v_up=2.0)
    assert vz == -2.0
