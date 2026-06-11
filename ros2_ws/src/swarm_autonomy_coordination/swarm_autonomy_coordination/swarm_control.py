"""Decentralized reciprocal collision avoidance + velocity guidance.

A velocity-obstacle / reciprocal-avoidance control law (in the ORCA / RVO family,
Berg et al. 2008/2011) for a multi-agent swarm. Each agent independently computes
a collision-free velocity from only its own goal and its *neighbours' shared
state* (positions + velocities, which a real drone broadcasts over the comms
layer) — there is no central planner and no ground-truth-only shortcut for the
avoidance: it uses exactly the information a real vehicle would have about its
peers.

Outputs a velocity command sent to PX4 in offboard velocity-control mode.

ROS-free and unit-tested so the same law runs in the Gazebo bridge and the
headless sim. 2-D in the horizontal plane (altitude is held separately); the
swarm operates above building height so this handles inter-agent avoidance.
Static-obstacle avoidance is handled by the ego-planner/ESDF layer.
"""

from __future__ import annotations

import math

Vec2 = tuple[float, float]


def _sub(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] - b[0], a[1] - b[1])


def _add(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] + b[0], a[1] + b[1])


def _scale(a: Vec2, s: float) -> Vec2:
    return (a[0] * s, a[1] * s)


def _norm(a: Vec2) -> float:
    return math.hypot(a[0], a[1])


def _clamp_speed(v: Vec2, vmax: float) -> Vec2:
    s = _norm(v)
    return _scale(v, vmax / s) if s > vmax and s > 1e-9 else v


def preferred_velocity(
    pos: Vec2,
    goal: Vec2,
    vmax: float,
    slow_radius: float = 2.0,
    goal_vel: Vec2 = (0.0, 0.0),
    cur_vel: Vec2 = (0.0, 0.0),
    kd: float = 0.0,
) -> Vec2:
    """Goal-seeking velocity that slows near the goal, with optional target-velocity
    FEEDFORWARD and velocity DAMPING for stable tracking of a *moving* goal.

    v = goal_vel (feedforward)               -- move WITH the target, don't just chase
      + vmax * dir * min(1, d / slow_radius) -- proportional approach, decelerating
      - kd * (cur_vel - goal_vel)            -- damp velocity error -> no overshoot/limit cycle

    With goal_vel=0, kd=0 this reduces to the original P-with-deceleration law (so the
    existing unit tests are unchanged).
    """
    to_goal = _sub(goal, pos)
    d = _norm(to_goal)
    vp = (0.0, 0.0) if d < 1e-6 else _scale(to_goal, vmax * min(1.0, d / slow_radius) / d)
    v = (goal_vel[0] + vp[0] - kd * (cur_vel[0] - goal_vel[0]),
         goal_vel[1] + vp[1] - kd * (cur_vel[1] - goal_vel[1]))
    return _clamp_speed(v, vmax)


def avoidance_velocity(
    pos: Vec2,
    vel: Vec2,
    pref: Vec2,
    neighbors: list[tuple[Vec2, Vec2]],
    vmax: float,
    safety_radius: float = 2.5,
    time_horizon: float = 2.5,
    detect_radius: float = 8.0,
) -> Vec2:
    """Adjust the preferred velocity to avoid neighbours (reciprocal).

    Two terms per neighbour:
      * a hard *separation* push if already inside ``safety_radius`` (recover from
        encroachment), and
      * a predictive *velocity-obstacle* push if the current relative motion leads
        to a closest approach inside ``safety_radius`` within ``time_horizon``.
    The avoidance is halved (reciprocal): both agents share the manoeuvre, which
    keeps the motion smooth instead of each taking full responsibility.
    """
    v = list(pref)
    for npos, nvel in neighbors:
        rel = _sub(npos, pos)
        dist = _norm(rel)
        if dist < 1e-6 or dist > detect_radius:
            continue
        rdir = _scale(rel, 1.0 / dist)            # unit vector toward neighbour
        perp = (rdir[1], -rdir[0])                # right-hand tangent (consistent side)

        # Hard separation when inside the safety bubble: push grows as dist -> 0,
        # plus a tangential component so agents go *around* rather than deadlock.
        if dist < safety_radius:
            mag = min(3.0, safety_radius / dist - 1.0) * vmax
            v[0] += -rdir[0] * mag * 0.8 + perp[0] * mag * 0.6
            v[1] += -rdir[1] * mag * 0.8 + perp[1] * mag * 0.6
            continue

        # Predictive (velocity-obstacle): time to closest approach if we keep going.
        relv = _sub(vel, nvel)                     # our velocity relative to them
        closing = relv[0] * rdir[0] + relv[1] * rdir[1]   # >0 => approaching
        if closing <= 1e-3:
            continue
        ttc = (dist - safety_radius) / closing
        if ttc < time_horizon:
            strength = (1.0 - ttc / time_horizon) * vmax * 0.5   # reciprocal: half
            # Mostly veer sideways (go around); small radial brake.
            v[0] += perp[0] * strength - rdir[0] * strength * 0.4
            v[1] += perp[1] * strength - rdir[1] * strength * 0.4
    return _clamp_speed((v[0], v[1]), vmax)


def control_velocity(
    pos: Vec2,
    vel: Vec2,
    goal: Vec2,
    neighbors: list[tuple[Vec2, Vec2]],
    vmax: float,
    goal_vel: Vec2 = (0.0, 0.0),
    kd: float = 0.0,
    slow_radius: float = 2.0,
    **kw,
) -> Vec2:
    """Full guidance law: damped feedforward goal-tracking made collision-free vs neighbours.

    Pass ``goal_vel`` (the target's velocity) and ``kd`` (damping gain ~0.6-0.8) when the
    goal is MOVING, to track it without the lag-and-overshoot oscillation a plain
    proportional law shows. ``slow_radius`` controls the deceleration zone size.
    """
    pref = preferred_velocity(pos, goal, vmax, slow_radius=slow_radius,
                              goal_vel=goal_vel, cur_vel=vel, kd=kd)
    return avoidance_velocity(pos, vel, pref, neighbors, vmax, **kw)
