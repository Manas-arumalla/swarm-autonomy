"""Closed-loop unit tests for the MPC pursuit guidance."""
import math

from swarm_autonomy_coordination.mpc_pursuit import mpc_velocity

DT = 0.12
VMAX = 4.5
AMAX = 3.0


def _rollout(pos, goal_fn, steps, goal_vel=(0.0, 0.0)):
    """Simulate the closed loop: MPC -> double-integrator drone -> repeat."""
    p = list(pos)
    v = [0.0, 0.0]
    cmds = []
    for k in range(steps):
        goal = goal_fn(k)
        vx, vy = mpc_velocity((p[0], p[1]), (v[0], v[1]), goal, goal_vel,
                              vmax=VMAX, amax=AMAX, dt=DT)
        cmds.append((vx, vy))
        # the drone tracks the velocity command (one-step lag, like PX4's inner loop)
        v = [vx, vy]
        p = [p[0] + v[0] * DT, p[1] + v[1] * DT]
    return p, v, cmds


def test_mpc_converges_to_static_goal():
    goal = (10.0, -6.0)
    p, v, _ = _rollout((0.0, 0.0), lambda k: goal, steps=120)
    assert math.hypot(p[0] - goal[0], p[1] - goal[1]) < 0.5      # reached the goal
    assert math.hypot(v[0], v[1]) < 0.5                          # and stopped (no overshoot loop)


def test_mpc_tracks_moving_goal():
    gv = (1.5, 0.0)                                              # goal slides in +x
    goal_fn = lambda k: (3.0 + gv[0] * k * DT, 0.0)
    p, v, _ = _rollout((0.0, 0.0), goal_fn, steps=160, goal_vel=gv)
    gx = 3.0 + gv[0] * 159 * DT
    assert abs(p[0] - gx) < 1.5                                  # keeps station on the mover
    assert math.hypot(v[0] - gv[0], v[1] - gv[1]) < 0.6          # matches its velocity (feedforward)


def test_mpc_respects_vmax():
    _, _, cmds = _rollout((0.0, 0.0), lambda k: (50.0, 50.0), steps=20)
    assert all(math.hypot(vx, vy) <= VMAX + 1e-6 for vx, vy in cmds)


def test_mpc_is_smooth():
    # far-away goal: the command must not jump more than the accel limit allows per step
    _, _, cmds = _rollout((0.0, 0.0), lambda k: (30.0, 0.0), steps=30)
    for (a, _), (b, _) in zip(cmds, cmds[1:]):
        assert abs(b - a) <= AMAX * DT + 0.05                   # bounded step change = smooth
