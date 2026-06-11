"""Unit tests for the CPU ESDF (ROS-free)."""
import math

from swarm_autonomy_mapping.esdf import GridMap


def _city():
    g = GridMap((-16, -16), (16, 16), res=0.5)
    g.add_buildings([(cx, cy) for cx in (-12, 0, 12) for cy in (-12, 0, 12)], half=3.5)
    return g


def test_sign_inside_vs_outside():
    g = GridMap((-10, -10), (10, 10), res=0.5)
    g.add_box(0.0, 0.0, 3.0)
    assert g.distance(0.0, 0.0) < 0          # deep inside the box → negative
    assert g.distance(8.0, 8.0) > 0          # far outside → positive


def test_distance_magnitude_outside():
    g = GridMap((-10, -10), (10, 10), res=0.25)
    g.add_box(0.0, 0.0, 2.0)                  # building spans |x|,|y| <= 2
    # a point 3 m to the +x side: nearest wall is at x=2, so ~1 m clearance
    assert math.isclose(g.distance(3.0, 0.0), 1.0, abs_tol=0.3)


def test_gradient_points_away_from_obstacle():
    g = GridMap((-10, -10), (10, 10), res=0.25)
    g.add_box(0.0, 0.0, 2.0)
    gx, gy = g.gradient(3.0, 0.0)            # to the +x side, flee direction is +x
    assert gx > 0.8 and abs(gy) < 0.3


def test_clearance_query_and_buildings():
    g = _city()
    assert not g.is_free(0.0, 0.0, clearance=1.0)     # building at origin
    assert g.is_free(6.0, 6.0, clearance=1.0)         # the clear street diagonal
    assert g.distance(6.0, 6.0) > 1.0
