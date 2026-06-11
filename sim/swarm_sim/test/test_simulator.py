"""Smoke + behaviour tests for the swarm simulator.

These run headless (record=False) and are fast. They verify the simulator wires
the real algorithm modules together correctly and produces the expected
qualitative behaviour, deterministically.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..")))

from swarm_sim.simulator import Simulator, SimConfig, ROLE_INTERCEPTOR, ROLE_BLOCKER
from swarm_sim.world import World, Rect, default_city


def _run(num_drones=4, seed=1, max_time=80.0):
    cfg = SimConfig(num_drones=num_drones, seed=seed, max_time_s=max_time, record=False)
    return Simulator(default_city(seed), cfg).run()


def test_is_deterministic():
    a = _run(seed=3)
    b = _run(seed=3)
    assert a.captured == b.captured
    assert a.capture_time == b.capture_time
    assert abs(a.final_coverage - b.final_coverage) < 1e-9


def test_swarm_covers_more_than_solo():
    solo = _run(num_drones=1, max_time=40.0)
    swarm = _run(num_drones=6, max_time=40.0)
    assert swarm.final_coverage > solo.final_coverage


def test_enough_drones_intercept_the_target():
    res = _run(num_drones=5, seed=1, max_time=90.0)
    assert res.captured
    assert res.min_distance < SimConfig().capture_radius


def test_roles_are_assigned_during_pursuit():
    cfg = SimConfig(num_drones=4, seed=1, max_time_s=90.0, record=True)
    sim = Simulator(default_city(1), cfg)
    sim.run()
    pursue_frames = [f for f in sim.frames if f.phase == "pursue"]
    assert pursue_frames, "pursuit phase never entered"
    roles_seen = {role for f in pursue_frames for (_, _, role) in f.drones}
    assert ROLE_INTERCEPTOR in roles_seen
    assert ROLE_BLOCKER in roles_seen


def test_comms_gating_blocks_distant_observations():
    # Two drones far apart with a tiny comms range: an observation by one must
    # NOT reach the other (decentralization is real).
    from swarm_autonomy_comms.link_model import LinkConfig
    cfg = SimConfig(num_drones=2, seed=1, max_time_s=1.0,
                    comms=LinkConfig(max_range_m=1.0, bandwidth_bps=1e9, base_loss=0.0))
    sim = Simulator(default_city(1), cfg)
    # Force a belief on drone 0 and place drones far apart.
    sim.drones[0].pos = np.array([2.0, 2.0])
    sim.drones[1].pos = np.array([38.0, 38.0])
    from swarm_sim.simulator import Belief
    sim.beliefs[0] = Belief(np.array([2.0, 2.0]), np.zeros(2), sim.t, 1.0)
    sim._share_target()
    assert sim.beliefs[1] is None  # out of range -> never delivered
