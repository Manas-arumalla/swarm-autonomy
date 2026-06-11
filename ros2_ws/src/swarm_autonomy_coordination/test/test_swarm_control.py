"""Unit tests for the decentralized reciprocal collision-avoidance law."""

import math

from swarm_autonomy_coordination.swarm_control import (
    avoidance_velocity,
    control_velocity,
    preferred_velocity,
)


def _n(v):
    return math.hypot(v[0], v[1])


def test_preferred_velocity_points_at_goal():
    v = preferred_velocity((0, 0), (10, 0), vmax=5.0)
    assert v[0] > 0 and abs(v[1]) < 1e-9
    assert abs(_n(v) - 5.0) < 1e-9          # full speed when far


def test_preferred_velocity_slows_near_goal():
    v = preferred_velocity((0, 0), (0.5, 0), vmax=5.0, slow_radius=2.0)
    assert _n(v) < 5.0                       # ramps down approaching the goal


def test_no_neighbors_returns_goal_seeking():
    v = control_velocity((0, 0), (0, 0), (10, 0), [], vmax=4.0)
    assert v[0] > 0 and abs(v[1]) < 1e-9


def test_close_neighbor_pushes_away():
    # Neighbour just ahead and inside the safety radius -> velocity must point away.
    v = control_velocity((0, 0), (0, 0), (10, 0), [((1.0, 0.0), (0, 0))],
                         vmax=4.0, safety_radius=2.5)
    assert v[0] < 0                          # pushed back, away from neighbour


def test_speed_is_capped():
    v = control_velocity((0, 0), (0, 0), (100, 0),
                         [((1.0, 0.0), (0, 0)), ((1.0, 0.5), (0, 0))], vmax=3.0)
    assert _n(v) <= 3.0 + 1e-9


def test_head_on_pair_avoids_collision_in_sim():
    # Two agents swapping positions on a line should NOT pass through each other.
    a, b = (-8.0, 0.0), (8.0, 0.0)
    ga, gb = (8.0, 0.0), (-8.0, 0.0)
    va = vb = (0.0, 0.0)
    dt, vmax = 0.1, 3.0
    min_sep = 1e9
    for _ in range(400):
        va = control_velocity(a, va, ga, [(b, vb)], vmax)
        vb = control_velocity(b, vb, gb, [(a, va)], vmax)
        a = (a[0] + va[0] * dt, a[1] + va[1] * dt)
        b = (b[0] + vb[0] * dt, b[1] + vb[1] * dt)
        min_sep = min(min_sep, math.hypot(a[0] - b[0], a[1] - b[1]))
    assert min_sep > 1.0                     # never collided (radius-scale clearance)
    # ...and both still make progress past each other.
    assert a[0] > 0 and b[0] < 0


def test_predictive_term_engages_before_contact():
    # Neighbour outside safety radius but on a fast collision course -> still avoids.
    v = avoidance_velocity((0, 0), (4, 0), (4, 0), [((6.0, 0.0), (-4.0, 0.0))],
                           vmax=4.0, safety_radius=2.0, time_horizon=3.0)
    assert v[1] != 0.0 or v[0] < 4.0         # deflected from a straight closing path
