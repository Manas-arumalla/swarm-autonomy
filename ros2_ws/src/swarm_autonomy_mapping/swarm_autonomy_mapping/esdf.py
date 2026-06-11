"""ROS-free 2D occupancy grid + Euclidean Signed Distance Field (ESDF).

This is the Swarm Autonomy CPU mapping fallback (design-decisions D5): nvblox builds the
ESDF on the GPU, but on GPU-constrained hosts — or for high vehicle counts — the same
signed-distance field is built on the CPU with a distance transform. The ESDF is what
the local planner queries: a smooth field giving, at any point, the distance to the
nearest obstacle (+ outside, − inside) and its gradient (the direction to flee).

Kept pure/importable (no rclpy) so it unit-tests alongside link_model/cbba/pid/merge,
and the same field feeds both the headless planning experiment and the ROS map node.
"""

from __future__ import annotations

import math

import numpy as np
from scipy import ndimage


class GridMap:
    """A 2D occupancy grid over a world rectangle, with an ESDF.

    Cell (i, j) covers world x in [lo_x + i*res, ...], y likewise; `occ[i, j]` True
    means occupied. Indices are clamped at the borders so queries never throw.
    """

    def __init__(self, lo, hi, res: float = 0.5):
        self.lo = np.asarray(lo, dtype=float)
        self.hi = np.asarray(hi, dtype=float)
        self.res = float(res)
        self.nx = max(1, int(math.ceil((self.hi[0] - self.lo[0]) / self.res)))
        self.ny = max(1, int(math.ceil((self.hi[1] - self.lo[1]) / self.res)))
        self.occ = np.zeros((self.nx, self.ny), dtype=bool)
        self._esdf = None  # lazily (re)built
        self._free_idx = None  # nearest free-cell indices (feature transform), for in-obstacle escape

    # --- coordinate transforms -------------------------------------------------
    def world_to_cell(self, x: float, y: float):
        i = int((x - self.lo[0]) / self.res)
        j = int((y - self.lo[1]) / self.res)
        return (max(0, min(self.nx - 1, i)), max(0, min(self.ny - 1, j)))

    def cell_to_world(self, i: int, j: int):
        return (self.lo[0] + (i + 0.5) * self.res, self.lo[1] + (j + 0.5) * self.res)

    def in_bounds_world(self, x: float, y: float) -> bool:
        return self.lo[0] <= x <= self.hi[0] and self.lo[1] <= y <= self.hi[1]

    # --- building the map ------------------------------------------------------
    def add_box(self, cx: float, cy: float, half: float) -> None:
        """Mark the axis-aligned square [cx±half, cy±half] occupied."""
        i0, j0 = self.world_to_cell(cx - half, cy - half)
        i1, j1 = self.world_to_cell(cx + half, cy + half)
        self.occ[i0:i1 + 1, j0:j1 + 1] = True
        self._esdf = None

    def add_buildings(self, centers, half: float) -> None:
        for cx, cy in centers:
            self.add_box(cx, cy, half)

    # --- the ESDF ---------------------------------------------------------------
    def esdf(self):
        """Signed distance field (metres): +distance-to-obstacle outside, −depth inside.

        Built with two Euclidean distance transforms (free→obstacle and obstacle→free),
        cached until the occupancy changes.
        """
        if self._esdf is None:
            occ = self.occ
            if occ.any():
                dist_out = ndimage.distance_transform_edt(~occ, sampling=self.res)
                # return_indices gives, for every cell, the nearest FREE cell — an exact escape
                # direction inside obstacles, where the central-difference gradient is undefined.
                dist_in, self._free_idx = ndimage.distance_transform_edt(
                    occ, sampling=self.res, return_indices=True)
            else:                                   # empty map → everywhere is "far"
                big = max(self.nx, self.ny) * self.res
                dist_out = np.full(occ.shape, big, dtype=float)
                dist_in = np.zeros(occ.shape, dtype=float)
                self._free_idx = None
            self._esdf = dist_out - dist_in
        return self._esdf

    def distance(self, x: float, y: float) -> float:
        """Bilinearly-interpolated signed distance at a world point."""
        e = self.esdf()
        fi = (x - self.lo[0]) / self.res - 0.5
        fj = (y - self.lo[1]) / self.res - 0.5
        i0 = max(0, min(self.nx - 2, int(math.floor(fi))))
        j0 = max(0, min(self.ny - 2, int(math.floor(fj))))
        ti = max(0.0, min(1.0, fi - i0))
        tj = max(0.0, min(1.0, fj - j0))
        return float(
            e[i0, j0] * (1 - ti) * (1 - tj) + e[i0 + 1, j0] * ti * (1 - tj)
            + e[i0, j0 + 1] * (1 - ti) * tj + e[i0 + 1, j0 + 1] * ti * tj
        )

    def gradient(self, x: float, y: float):
        """Unit ∇(signed distance) at a world point — points AWAY from obstacles.

        Outside obstacles this is the central-difference gradient. Inside an obstacle the central
        difference is ill-defined (it points along the medial axis, not out), so use the exact
        direction to the nearest free cell from the feature transform — a clean escape everywhere.
        """
        self.esdf()
        if self._free_idx is not None and self.distance(x, y) <= 0.0:
            i, j = self.world_to_cell(x, y)
            wx, wy = self.cell_to_world(int(self._free_idx[0, i, j]), int(self._free_idx[1, i, j]))
            dx, dy = wx - x, wy - y
            n = math.hypot(dx, dy)
            if n > 1e-9:
                return (dx / n, dy / n)
        h = self.res
        gx = (self.distance(x + h, y) - self.distance(x - h, y)) / (2 * h)
        gy = (self.distance(x, y + h) - self.distance(x, y - h)) / (2 * h)
        n = math.hypot(gx, gy)
        return (gx / n, gy / n) if n > 1e-9 else (0.0, 0.0)

    def is_free(self, x: float, y: float, clearance: float = 0.0) -> bool:
        return self.distance(x, y) > clearance
