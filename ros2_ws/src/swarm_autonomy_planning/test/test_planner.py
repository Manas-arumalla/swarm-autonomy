"""Unit tests for the CPU A* + ESDF-optimizing planner (ROS-free)."""
import numpy as np

from swarm_autonomy_mapping.esdf import GridMap
from swarm_autonomy_planning.planner import astar, plan, optimize, path_metrics


def _city():
    g = GridMap((-16, -16), (16, 16), res=0.5)
    g.add_buildings([(cx, cy) for cx in (-12, 0, 12) for cy in (-12, 0, 12)], half=3.5)
    return g


def test_straight_line_when_clear():
    g = GridMap((-10, -10), (10, 10), res=0.5)        # no obstacles
    path = plan(g, (-8, 0), (8, 0))
    assert path is not None
    # with nothing to avoid, the optimized band stays ~straight (y ~ 0)
    assert max(abs(y) for _, y in path) < 0.5
    assert path_metrics(g, path)["length"] < 17.5     # ~16 m straight


def test_routes_around_a_building():
    g = _city()
    # (-6,0)->(6,0) sits in clear gaps but a straight line clips the centre building;
    # the planner must detour around it.
    path = plan(g, (-6.0, 0.0), (6.0, 0.0), d_safe=1.2)
    assert path is not None
    m = path_metrics(g, path)
    assert m["min_clearance"] > 0.4                    # never enters a building
    assert all(g.distance(x, y) > 0.0 for x, y in path)
    assert max(abs(y) for _, y in path) > 2.0          # actually detoured off the straight line


def test_no_path_when_goal_blocked():
    g = _city()
    # goal sits inside the centre building → front-end returns None
    assert astar(g, (-6.0, 0.0), (0.0, 0.0), clearance=0.5) is None
    assert plan(g, (-6.0, 0.0), (0.0, 0.0)) is None


def test_optimize_improves_clearance():
    g = _city()
    coarse = astar(g, (-6.0, 1.0), (6.0, -1.0), clearance=0.0)
    assert coarse is not None
    before = min(g.distance(x, y) for x, y in coarse)   # A* hugs walls → ~0
    smooth = optimize(coarse, g, d_safe=1.5, iters=120)
    after = min(g.distance(x, y) for x, y in smooth)
    assert after > before                               # the band pushed off the walls
    assert after > 0.3


def test_optimize_guarantees_no_waypoint_inside_obstacle():
    # A straight seed band driven THROUGH a building (interior points start inside the wall);
    # the safety projection must leave every waypoint outside the obstacle.
    g = GridMap((-10, -10), (10, 10), res=0.5)
    g.add_box(0.0, 0.0, 3.0)                               # building spans |x|,|y| <= 3
    seed = [(-8.0, 0.0)] + [(float(x), 0.0) for x in np.linspace(-6, 6, 13)] + [(8.0, 0.0)]
    out = optimize(seed, g, d_safe=1.5, iters=200)
    assert all(g.distance(x, y) > 0.0 for x, y in out)    # nothing left inside the wall


def test_endpoints_pinned():
    g = _city()
    path = plan(g, (-8.0, 2.0), (8.0, -2.0))
    assert np.allclose(path[0], (-8.0, 2.0))
    assert np.allclose(path[-1], (8.0, -2.0))
