"""Unit tests for frontier detection (ROS-free)."""
import numpy as np

from swarm_autonomy_mapping.frontier import (frontier_mask, cluster_frontiers,
                                       coverage_fraction, UNKNOWN, FREE, OCC)


def test_frontier_is_free_next_to_unknown():
    s = np.full((5, 5), UNKNOWN, dtype=int)
    s[2, 1:3] = FREE                     # a small known-free strip in a sea of unknown
    m = frontier_mask(s)
    assert m[2, 1] and m[2, 2]           # both free cells border unknown → frontier
    assert not m[0, 0]                   # unknown cells are never frontier


def test_fully_known_region_has_no_frontier():
    s = np.full((6, 6), FREE, dtype=int)
    s[0, :] = OCC                        # walls all around, nothing unknown
    s[-1, :] = OCC; s[:, 0] = OCC; s[:, -1] = OCC
    assert not frontier_mask(s).any()
    assert cluster_frontiers(s) == []


def test_clusters_split_and_centroid():
    s = np.full((9, 9), UNKNOWN, dtype=int)
    s[1, 1] = FREE                       # isolated free cell (frontier, size 1)
    s[5:8, 5] = FREE                     # a separate free column
    clusters = cluster_frontiers(s, min_size=2)
    assert len(clusters) == 1            # only the column meets min_size
    ci, cj, size = clusters[0]
    assert size == 3 and abs(cj - 5) < 1e-6


def test_coverage_fraction():
    truth = np.array([[1, 1, 0], [1, 1, 0]], dtype=bool)      # 4 free cells
    s = np.full((2, 3), UNKNOWN, dtype=int)
    s[0, 0] = FREE; s[0, 1] = FREE                            # 2 of 4 seen
    assert abs(coverage_fraction(s, truth) - 0.5) < 1e-9
