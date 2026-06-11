"""2D grid city: axis-aligned rectangular buildings, occupancy/known grids,
line-of-sight sensing, and coverage bookkeeping.

The world is flat (all drones at one altitude) — enough to exercise exploration
and pursuit while keeping the sim cheap and deterministic. Buildings block both
motion (repulsion in the controller) and line of sight (so sensing must route
around corners, which is what makes cooperative exploration pay off).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np


@dataclass
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (self.x0 - margin <= x <= self.x1 + margin
                and self.y0 - margin <= y <= self.y1 + margin)

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2)


@dataclass
class World:
    width: float
    height: float
    cell: float = 1.0
    buildings: list[Rect] = field(default_factory=list)

    occ: np.ndarray = field(init=False)      # 1 = obstacle cell, 0 = free
    known: np.ndarray = field(init=False)    # 1 = some drone has sensed this cell
    nx: int = field(init=False)
    ny: int = field(init=False)

    def __post_init__(self) -> None:
        self.nx = int(round(self.width / self.cell))
        self.ny = int(round(self.height / self.cell))
        self.occ = np.zeros((self.nx, self.ny), dtype=np.int8)
        self.known = np.zeros((self.nx, self.ny), dtype=np.int8)
        for b in self.buildings:
            i0, j0 = self._to_cell(b.x0, b.y0)
            i1, j1 = self._to_cell(b.x1, b.y1)
            self.occ[max(0, i0):i1 + 1, max(0, j0):j1 + 1] = 1

    # --- geometry ----------------------------------------------------------
    def _to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (min(self.nx - 1, max(0, int(x / self.cell))),
                min(self.ny - 1, max(0, int(y / self.cell))))

    def cell_center(self, i: int, j: int) -> tuple[float, float]:
        return ((i + 0.5) * self.cell, (j + 0.5) * self.cell)

    def in_bounds(self, x: float, y: float) -> bool:
        return 0 <= x <= self.width and 0 <= y <= self.height

    def is_free(self, x: float, y: float, margin: float = 0.0) -> bool:
        if not self.in_bounds(x, y):
            return False
        return not any(b.contains(x, y, margin) for b in self.buildings)

    def blocked(self, ax: float, ay: float, bx: float, by: float) -> bool:
        """True if the segment a->b passes through any building (LOS test)."""
        steps = int(math.hypot(bx - ax, by - ay) / (self.cell * 0.5)) + 1
        for s in range(steps + 1):
            t = s / steps
            x, y = ax + (bx - ax) * t, ay + (by - ay) * t
            if any(b.contains(x, y) for b in self.buildings):
                return True
        return False

    # --- sensing / coverage ------------------------------------------------
    def sense(self, x: float, y: float, radius: float) -> int:
        """Mark cells within ``radius`` and line of sight as known; return the
        number of newly discovered cells."""
        i0, j0 = self._to_cell(x - radius, y - radius)
        i1, j1 = self._to_cell(x + radius, y + radius)
        newly = 0
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                if self.known[i, j]:
                    continue
                cx, cy = self.cell_center(i, j)
                if math.hypot(cx - x, cy - y) <= radius and not self.blocked(x, y, cx, cy):
                    self.known[i, j] = 1
                    newly += 1
        return newly

    def coverage(self) -> float:
        """Fraction of the *free* space that has been sensed."""
        free = self.occ == 0
        n_free = int(free.sum())
        if n_free == 0:
            return 1.0
        return float((self.known[free] == 1).sum()) / n_free

    def frontier_cells(self) -> list[tuple[int, int]]:
        """Known-free cells adjacent to unknown free cells — exploration goals."""
        out: list[tuple[int, int]] = []
        for i in range(self.nx):
            for j in range(self.ny):
                if self.known[i, j] != 1 or self.occ[i, j] != 0:
                    continue
                for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ni, nj = i + di, j + dj
                    if 0 <= ni < self.nx and 0 <= nj < self.ny \
                            and self.known[ni, nj] == 0 and self.occ[ni, nj] == 0:
                        out.append((i, j))
                        break
        return out


def default_city(seed: int = 0) -> World:
    """A 40x40 m block grid with a regular-ish set of buildings and open streets."""
    rng = np.random.default_rng(seed)
    buildings: list[Rect] = []
    for bx in range(4, 36, 8):
        for by in range(4, 36, 8):
            jitter = rng.uniform(-0.5, 0.5, size=2)
            w = rng.uniform(3.0, 4.5)
            h = rng.uniform(3.0, 4.5)
            buildings.append(Rect(bx + jitter[0], by + jitter[1],
                                  bx + w + jitter[0], by + h + jitter[1]))
    return World(width=40.0, height=40.0, cell=1.0, buildings=buildings)
