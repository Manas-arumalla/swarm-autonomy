"""Unit tests for pursuit/interception geometry."""

import math

from swarm_autonomy_coordination import pursuit


def test_predict_target_constant_velocity():
    p = pursuit.predict_target((0, 0, 0), (1, 0, 0), 2.0)
    assert p == (2.0, 0.0, 0.0)


def test_intercept_stationary_target_is_the_target():
    ip = pursuit.intercept_point((0, 0, 0), (5, 0, 0), (0, 0, 0), pursuer_speed=2.0)
    assert ip == (5, 0, 0)


def test_intercept_leads_a_crossing_target():
    # Target crosses ahead moving +y; intercept point must be ahead in y.
    ip = pursuit.intercept_point((0, 0, 0), (10, 0, 0), (0, 2.0, 0), pursuer_speed=5.0)
    assert ip[1] > 0.0  # we aim ahead of the target, not at it


def test_intercept_unreachable_falls_back_to_target_pos():
    # Target faster than pursuer and opening range -> no solution.
    tgt = (5, 0, 0)
    ip = pursuit.intercept_point((0, 0, 0), tgt, (10, 0, 0), pursuer_speed=1.0)
    assert ip == tgt


def test_containment_ring_is_evenly_spaced_at_radius():
    ring = pursuit.containment_ring((0, 0, 2), radius=4.0, n=4)
    assert len(ring) == 4
    for pt in ring:
        r = math.hypot(pt[0], pt[1])
        assert abs(r - 4.0) < 1e-9
        assert pt[2] == 2  # keeps the ring height


def test_assign_ring_slots_is_a_permutation():
    drones = [(0, 0, 0), (10, 0, 0), (0, 10, 0)]
    slots = pursuit.containment_ring((5, 5, 0), radius=3.0, n=3)
    assign = pursuit.assign_ring_slots(drones, slots)
    assert sorted(assign) == [0, 1, 2]  # each slot used exactly once
