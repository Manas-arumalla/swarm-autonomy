"""Smoke + behaviour tests for the swarm simulator.

These run headless (record=False) and are fast. They verify the simulator wires
the real algorithm modules together correctly and produces the expected
qualitative behaviour, deterministically.
"""

import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..")))

from swarm_sim.simulator import Simulator, SimConfig, ROLE_INTERCEPTOR, ROLE_BLOCKER
from swarm_sim.drone import Drone
from swarm_sim.exploration import (DARTSARStage2Strategy, DARTSARStrategy,
                                   ExplorationGoalStrategy, NearestFrontierStrategy)
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


def test_simulator_defaults_to_nearest_frontier_strategy():
    sim = Simulator(World(width=5.0, height=5.0), SimConfig(num_drones=1, record=False))
    assert isinstance(sim.exploration_strategy, NearestFrontierStrategy)


def test_nearest_frontier_strategy_preserves_goals_and_assigns_nearest_unclaimed():
    world = World(width=5.0, height=5.0)
    drones = [
        Drone(0, np.array([0.0, 1.5])),
        Drone(1, np.array([3.5, 1.5])),
    ]
    for drone in drones:
        drone.initialize_local_map(world)
    drones[0].local_known[1, 1] = 1
    drones[1].local_known[3, 1] = 1
    drones[0].set_goal((1.5, 1.5))
    claimed = {(0, 0)}

    strategy: ExplorationGoalStrategy = NearestFrontierStrategy()
    strategy.assign_goals(world, drones, claimed)

    assert np.array_equal(drones[0].goal, np.array([1.5, 1.5]))
    assert np.array_equal(drones[1].goal, np.array([3.5, 1.5]))
    assert claimed == {(1, 1), (3, 1)}


def _dartsar_drone(world: World, pos=(1.5, 1.5), sensor_radius=2.1) -> Drone:
    drone = Drone(0, np.array(pos), sensor_radius=sensor_radius)
    drone.initialize_local_map(world)
    return drone


def test_dartsar_uses_local_known_to_generate_frontiers():
    world = World(width=6.0, height=6.0)
    drone = _dartsar_drone(world)
    world.known[:, :] = 1

    DARTSARStrategy().assign_goals(world, [drone], set())

    assert drone.goal is None
    drone.local_known[1, 1] = 1
    DARTSARStrategy().assign_goals(world, [drone], set())
    assert np.array_equal(drone.goal, np.array(world.cell_center(1, 1)))


def test_dartsar_information_gain_favors_more_visible_unknown_free_cells():
    world = World(width=10.0, height=10.0)
    drone = _dartsar_drone(world)
    drone.local_known[:, :] = 1
    low_gain, high_gain = (1, 1), (6, 6)
    drone.local_known[2, 1] = 0
    for i in range(5, 8):
        for j in range(5, 8):
            if (i, j) != high_gain:
                drone.local_known[i, j] = 0

    scores = DARTSARStrategy().score_frontiers(world, drone, [low_gain, high_gain])

    assert scores[high_gain].information_gain > scores[low_gain].information_gain
    assert scores[high_gain].information_gain == 1.0


def test_dartsar_distance_cost_affects_utility():
    world = World(width=10.0, height=10.0)
    drone = _dartsar_drone(world, sensor_radius=1.1)
    drone.local_known[:, :] = 1
    near, far = (1, 1), (6, 1)
    drone.local_known[2, 1] = 0
    drone.local_known[7, 1] = 0

    scores = DARTSARStrategy().score_frontiers(world, drone, [near, far])

    assert scores[near].information_gain == scores[far].information_gain
    assert scores[near].distance_cost < scores[far].distance_cost
    assert scores[near].utility > scores[far].utility


def test_dartsar_selects_the_highest_utility_local_frontier():
    world = World(width=10.0, height=10.0)
    drone = _dartsar_drone(world)
    drone.local_known[:, :] = 1
    drone.local_known[2, 1] = 0
    for i in range(5, 8):
        for j in range(5, 8):
            if (i, j) != (6, 6):
                drone.local_known[i, j] = 0
    strategy = DARTSARStrategy()
    scores = strategy.score_frontiers(world, drone)
    expected = max(scores, key=lambda frontier: scores[frontier].utility)

    strategy.assign_goals(world, [drone], set())

    assert np.array_equal(drone.goal, np.array(world.cell_center(*expected)))


def test_dartsar_decision_is_unchanged_by_global_world_known():
    world = World(width=6.0, height=6.0)
    drone = _dartsar_drone(world)
    drone.local_known[1, 1] = 1
    strategy = DARTSARStrategy()

    world.known[:, :] = 1
    strategy.assign_goals(world, [drone], set())
    first_goal = drone.goal.copy()
    drone.set_goal(None)
    drone.role = 0
    world.known[:, :] = 0
    strategy.assign_goals(world, [drone], set())

    assert np.array_equal(drone.goal, first_goal)


def test_dartsar_stage2_marginal_contribution_decreases_with_teammate_knowledge():
    world = World(width=6.0, height=6.0)
    drone = _dartsar_drone(world)
    drone.local_known[1, 1] = 1
    strategy = DARTSARStage2Strategy()
    candidate = (1, 1)
    without_teammate_info = strategy.score_frontiers(world, drone, [candidate])[candidate]
    drone.teammate_known[:, :] = 1
    with_teammate_info = strategy.score_frontiers(world, drone, [candidate])[candidate]

    assert with_teammate_info.marginal_contribution < without_teammate_info.marginal_contribution


def test_dartsar_stage2_no_teammate_information_matches_information_gain():
    world = World(width=6.0, height=6.0)
    drone = _dartsar_drone(world)
    drone.local_known[1, 1] = 1

    score = DARTSARStage2Strategy().score_frontiers(world, drone, [(1, 1)])[(1, 1)]

    assert score.marginal_contribution == score.information_gain
    assert score.redundancy_ratio == 0.0


def test_dartsar_stage2_redundancy_ratio_is_clamped():
    world = World(width=6.0, height=6.0)
    drone = _dartsar_drone(world)
    drone.local_known[1, 1] = 1
    drone.teammate_known[:, :] = 1

    score = DARTSARStage2Strategy().score_frontiers(world, drone, [(1, 1)])[(1, 1)]

    assert 0.0 <= score.redundancy_ratio <= 1.0


def test_cli_selects_dartsar_stage2_independently():
    result = subprocess.run(
        [sys.executable, os.path.join(os.path.dirname(__file__), "..", "..", "run_sim.py"),
         "--time", "0", "--no-gif", "--exploration-strategy", "dartsar-stage2"],
        capture_output=True, text=True, check=False)

    assert result.returncode == 0, result.stderr


def test_drones_have_independent_local_maps_and_sensing_does_not_leak():
    sim = Simulator(World(width=20.0, height=20.0), SimConfig(num_drones=2, record=False))
    first, second = sim.drones
    assert first.local_known is not second.local_known
    assert not first.local_known.any()
    assert not second.local_known.any()

    first.sense(sim.world)
    assert first.local_known.any()
    assert not second.local_known.any()
    assert not sim.world.known.any()


def test_exploration_information_propagates_only_through_link_model():
    from swarm_autonomy_comms.link_model import LinkConfig
    cfg = SimConfig(num_drones=2, record=False,
                    comms=LinkConfig(max_range_m=50.0, soft_range_m=50.0,
                                     bandwidth_bps=1e9, base_loss=0.0))
    sim = Simulator(World(width=20.0, height=20.0), cfg)
    first, second = sim.drones
    first.local_known[1, 1] = 1
    sim._queue_exploration_updates(first, {(1, 1)})

    sim._share_exploration()
    assert second.local_known[1, 1] == 1


def test_exploration_information_is_blocked_by_range_or_loss():
    from swarm_autonomy_comms.link_model import LinkConfig
    for comms in (
        LinkConfig(max_range_m=1.0, soft_range_m=1.0, bandwidth_bps=1e9, base_loss=0.0),
        LinkConfig(max_range_m=50.0, soft_range_m=50.0, bandwidth_bps=1e9, base_loss=1.0),
    ):
        sim = Simulator(World(width=20.0, height=20.0), SimConfig(num_drones=2, record=False,
                                                                    comms=comms))
        first, second = sim.drones
        second.pos = np.array([18.0, 18.0])
        first.local_known[1, 1] = 1
        sim._queue_exploration_updates(first, {(1, 1)})
        sim._share_exploration()
        assert second.local_known[1, 1] == 0


def _relay_sim(positions: list[float]) -> Simulator:
    from swarm_autonomy_comms.link_model import LinkConfig
    sim = Simulator(
        World(width=20.0, height=20.0),
        SimConfig(num_drones=3, record=False,
                  comms=LinkConfig(max_range_m=4.0, soft_range_m=4.0,
                                   bandwidth_bps=1e9, base_loss=0.0)),
    )
    for drone, x in zip(sim.drones, positions):
        drone.pos = np.array([x, 2.0])
    return sim


def test_exploration_information_relays_from_a_to_b_to_c():
    sim = _relay_sim([1.0, 4.0, 7.0])  # A-C is out of range; both hops are in range.
    first, middle, last = sim.drones
    first.local_known[1, 1] = 1
    sim._queue_exploration_updates(first, {(1, 1)})

    delivered = sim._share_exploration()

    assert (0, 1) in delivered
    assert (1, 2) in delivered
    assert middle.local_known[1, 1] == 1
    assert last.local_known[1, 1] == 1


def test_exploration_relay_stops_at_a_blocked_or_lost_intermediate_link():
    from swarm_autonomy_comms.link_model import LinkConfig, LinkModel

    for positions, lose_middle_hop in (([1.0, 4.0, 10.0], False),
                                       ([1.0, 4.0, 7.0], True)):
        sim = _relay_sim(positions)  # A-C is out of range in both arrangements.
        if lose_middle_hop:
            sim.links[(1, 2)] = LinkModel(LinkConfig(
                max_range_m=4.0, soft_range_m=4.0, bandwidth_bps=1e9, base_loss=1.0))
        first, middle, last = sim.drones
        first.local_known[1, 1] = 1
        sim._queue_exploration_updates(first, {(1, 1)})

        sim._share_exploration()

        assert middle.local_known[1, 1] == 1
        assert last.local_known[1, 1] == 0


def test_relay_does_not_rebroadcast_cells_already_known_locally():
    sim = _relay_sim([1.0, 4.0, 7.0])
    first, middle, last = sim.drones
    first.local_known[1, 1] = 1
    sim._queue_exploration_updates(first, {(1, 1)})

    sim._share_exploration()
    relay_sends = sim.stats[middle.drone_id].sent
    sim._share_exploration()

    assert last.local_known[1, 1] == 1
    assert relay_sends == 1
    assert sim.stats[middle.drone_id].sent == relay_sends


def test_nearest_frontier_strategy_uses_local_not_evaluation_knowledge():
    world = World(width=5.0, height=5.0)
    world.known[3, 3] = 1
    drone = Drone(0, np.array([1.5, 1.5]))
    drone.initialize_local_map(world)
    drone.local_known[1, 1] = 1

    NearestFrontierStrategy().assign_goals(world, [drone], set())
    assert np.array_equal(drone.goal, np.array([1.5, 1.5]))


def test_global_coverage_remains_an_evaluation_metric():
    world = World(width=5.0, height=5.0)
    assert world.coverage() == 0.0
    world.sense(0.5, 0.5, 1.0)
    assert world.coverage() > 0.0
