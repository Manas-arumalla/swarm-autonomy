"""Roundtrip test: occupancy -> MapDelta arrays -> merge -> reconstructed occupancy."""
from swarm_autonomy_mapping.esdf import GridMap
from swarm_autonomy_mapping.grid_io import (occupancy_to_delta, apply_delta_to_grid,
                                      delta_indices_to_cells)
from swarm_autonomy_mapping.merge import merge_log_odds


def test_occupancy_delta_roundtrip():
    src = GridMap((-8, -8), (8, 8), res=0.5)
    src.add_box(0.0, 0.0, 2.0)
    origin, vsize, ny, indices, log_odds = occupancy_to_delta(src)

    assert len(indices) == int(src.occ.sum())          # one entry per occupied cell
    assert ny == src.ny and vsize == src.res

    # a neighbour with the SAME geometry reconstructs the obstacle from the delta alone
    dst = GridMap((-8, -8), (8, 8), res=0.5)
    apply_delta_to_grid(dst, ny, indices)
    assert (dst.occ == src.occ).all()
    assert dst.distance(0.0, 0.0) < 0                  # rebuilt building reads as occupied


def test_indices_invert():
    g = GridMap((0, 0), (4, 4), res=1.0)               # 4x4
    cells = delta_indices_to_cells(g.ny, [0, 1, g.ny, g.ny + 3])
    assert cells == [(0, 0), (0, 1), (1, 0), (1, 3)]


def test_merge_saturates_repeated_evidence():
    # fusing the same occupied cell many times saturates at the clamp, not unbounded
    v = 0
    for _ in range(10):
        v = merge_log_odds(v, 100)
    assert v == 127
