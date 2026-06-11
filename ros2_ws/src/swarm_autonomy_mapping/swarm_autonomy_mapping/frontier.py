"""ROS-free frontier detection for cooperative exploration (RACER role, CPU fallback).

A *frontier* is the boundary between known-free and not-yet-seen space — the set of places
worth flying to next. Each drone detects frontier cells on its (shared) occupancy belief and
the exploration coordinator hands them out so the swarm divides the city instead of re-covering
each other's ground. This is the CPU fallback for RACER's frontier machinery; pure so it
unit-tests alongside esdf/planner.

The belief grid uses three states: UNKNOWN (not yet sensed), FREE, OCC (occupied). A frontier
cell is a FREE cell 4-adjacent to at least one UNKNOWN cell. Adjacent frontier cells are
clustered (8-connected); each cluster's centroid + size is what the coordinator assigns.
"""

from __future__ import annotations

import numpy as np
from scipy import ndimage

UNKNOWN = 0
FREE = 1
OCC = 2

_CROSS = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool)
_BLOCK = np.ones((3, 3), dtype=bool)


def frontier_mask(state: np.ndarray) -> np.ndarray:
    """Boolean mask of frontier cells: FREE cells 4-adjacent to UNKNOWN."""
    free = state == FREE
    unknown = state == UNKNOWN
    touches_unknown = ndimage.binary_dilation(unknown, structure=_CROSS)
    return free & touches_unknown


def cluster_frontiers(state: np.ndarray, min_size: int = 2):
    """Cluster the frontier cells (8-connected). Returns a list of
    (centroid_i, centroid_j, size) sorted by descending size — the candidate goals."""
    mask = frontier_mask(state)
    labels, n = ndimage.label(mask, structure=_BLOCK)
    out = []
    for k in range(1, n + 1):
        cells = np.argwhere(labels == k)
        if len(cells) >= min_size:
            ci, cj = cells.mean(axis=0)
            out.append((float(ci), float(cj), int(len(cells))))
    out.sort(key=lambda c: -c[2])
    return out


def coverage_fraction(state: np.ndarray, free_truth: np.ndarray) -> float:
    """Fraction of the truly-free space that has been sensed FREE (the exploration metric)."""
    total = int(free_truth.sum())
    if total == 0:
        return 1.0
    seen = int(((state == FREE) & free_truth).sum())
    return seen / total
