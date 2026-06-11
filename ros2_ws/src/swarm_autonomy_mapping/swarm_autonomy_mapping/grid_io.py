"""ROS-free serialisation between an occupancy :class:`GridMap` and the flat index /
log-odds arrays carried by ``swarm_autonomy_msgs/MapDelta``.

Lets a drone publish its locally-mapped occupancy as a compact delta (occupied cells only)
that the comms layer meters and neighbours fuse with ``merge_log_odds``. Kept pure so the
occupancy -> delta -> merge roundtrip unit-tests without rclpy.

Flattened index convention: ``k = i * ny + j`` for cell (i, j); ``ny`` (rows in the y
direction) travels with the delta so the receiver can invert it.
"""

from __future__ import annotations

import numpy as np

OCC_LOG_ODDS = 100      # int8 occupied evidence per observation (saturates at ±127 on merge)


def occupancy_to_delta(grid):
    """Serialise a GridMap's occupied cells. Returns
    (origin_xy, voxel_size, ny, indices, log_odds) — plain data the node packs into MapDelta."""
    ii, jj = np.nonzero(grid.occ)
    indices = (ii * grid.ny + jj).astype(np.int64).tolist()
    log_odds = [OCC_LOG_ODDS] * len(indices)
    return (float(grid.lo[0]), float(grid.lo[1])), float(grid.res), int(grid.ny), indices, log_odds


def delta_indices_to_cells(ny: int, indices):
    """Invert the flat index into (i, j) cell pairs."""
    return [(int(k) // ny, int(k) % ny) for k in indices]


def apply_delta_to_grid(grid, ny: int, indices, threshold: int = 0, log_odds=None):
    """Mark the delta's cells occupied on `grid` (receiver side of a merge), for the cells whose
    log-odds clear `threshold` (occupied evidence only — cells at or below the threshold are left
    untouched, so negative/free evidence never *sets* occupancy here). Cells outside `grid` are
    skipped. With ``log_odds=None`` every cell is treated as occupied evidence (the
    occupied-cells-only delta produced by :func:`occupancy_to_delta`)."""
    values = log_odds if log_odds is not None else [OCC_LOG_ODDS] * len(indices)
    for (i, j), lo in zip(delta_indices_to_cells(ny, indices), values):
        if lo > threshold and 0 <= i < grid.nx and 0 <= j < grid.ny:
            grid.occ[i, j] = True
    grid._esdf = None
    return grid
