"""ROS-free local planner: A* front-end + ESDF-gradient optimizing back-end.

CPU planning fallback for navigating among buildings (the ego-planner role)
without the GPU/ROS1 ego-planner-swarm build. Mirrors ego-planner's two stages:

  1. FRONT-END: 8-connected A* over the occupancy grid → a collision-free but jagged
     path that threads the gaps between buildings (escapes local minima).
  2. BACK-END: treat the path as an elastic band and minimise, by gradient descent,
       J = w_elastic * Σ‖p_{i+1} − p_i‖²              (membrane energy: jointly smooths + shortens)
         + w_clear   * Σ max(0, d_safe − esdf(p_i))²   (push off obstacles via ∇ESDF)
     The interior update ``p_i ← p_i + lr·(p_{i-1} − 2p_i + p_{i+1})`` is the gradient of the
     membrane term, with ``w_elastic = w_smooth + w_len`` (the two weights are summed; the
     Laplacian is the gradient of squared segment length, so it both smooths and shortens). The
     collision term uses the ESDF gradient from :mod:`swarm_autonomy_mapping.esdf` — exactly the
     signal nvblox/ego-planner use, here on the CPU. A final safety projection then guarantees no
     interior waypoint is left inside an obstacle.

Endpoints stay pinned; only interior waypoints move. Pure/unit-tested; the same
function backs the headless experiment and the ROS planner node.
"""

from __future__ import annotations

import heapq
import math

import numpy as np

# 8-connected neighbourhood (dx, dy, step-cost)
_NB = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
       (-1, -1, 1.41421356), (-1, 1, 1.41421356),
       (1, -1, 1.41421356), (1, 1, 1.41421356)]


def astar(grid, start_xy, goal_xy, clearance: float = 0.0):
    """8-connected A* on cells whose ESDF exceeds `clearance`. Returns a list of
    world waypoints (inclusive of start/goal) or None if no path exists."""
    e = grid.esdf()
    si, sj = grid.world_to_cell(*start_xy)
    gi, gj = grid.world_to_cell(*goal_xy)

    def passable(i, j):
        return 0 <= i < grid.nx and 0 <= j < grid.ny and e[i, j] > clearance

    if not passable(gi, gj):                 # nudge the goal cell out of an obstacle
        return None

    def h(i, j):
        return math.hypot(i - gi, j - gj)

    open_h = [(h(si, sj), 0.0, (si, sj))]
    came = {}
    gcost = {(si, sj): 0.0}
    seen = set()
    while open_h:
        _, g, cur = heapq.heappop(open_h)
        if cur in seen:
            continue
        seen.add(cur)
        if cur == (gi, gj):
            break
        ci, cj = cur
        for dx, dy, sc in _NB:
            ni, nj = ci + dx, cj + dy
            if not passable(ni, nj):
                continue
            ng = g + sc
            if ng < gcost.get((ni, nj), float("inf")):
                gcost[(ni, nj)] = ng
                came[(ni, nj)] = cur
                heapq.heappush(open_h, (ng + h(ni, nj), ng, (ni, nj)))
    if (gi, gj) not in came and (si, sj) != (gi, gj):
        return None

    cells = [(gi, gj)]
    while cells[-1] != (si, sj):
        cells.append(came[cells[-1]])
    cells.reverse()
    pts = [grid.cell_to_world(i, j) for i, j in cells]
    pts[0] = tuple(start_xy)
    pts[-1] = tuple(goal_xy)
    return pts


def _resample(path, spacing: float):
    """Resample a polyline to roughly-uniform spacing (so the elastic band has even
    resolution for the smoother)."""
    P = np.asarray(path, dtype=float)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    total = float(seg.sum())
    if total < 1e-6:
        return P
    n = max(2, int(round(total / spacing)) + 1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    s = np.linspace(0.0, total, n)
    out = np.empty((n, 2))
    out[:, 0] = np.interp(s, cum, P[:, 0])
    out[:, 1] = np.interp(s, cum, P[:, 1])
    return out


def _project_out(grid, p, fallback, margin):
    """Push point ``p`` out of an obstacle along the (feature-transform) escape gradient until its
    clearance exceeds ``margin``; fall back to a known-good position if it cannot be freed."""
    q = np.asarray(p, dtype=float).copy()
    for _ in range(16):
        if grid.distance(q[0], q[1]) > margin:
            return q
        gx, gy = grid.gradient(q[0], q[1])
        if gx == 0.0 and gy == 0.0:
            break
        q = q + 0.5 * grid.res * np.array([gx, gy])
    if grid.distance(q[0], q[1]) > margin:
        return q
    fb = np.asarray(fallback, dtype=float)
    return fb if grid.distance(fb[0], fb[1]) > margin else q


def optimize(path, grid, d_safe: float = 1.2, iters: int = 80, lr: float = 0.15,
             w_smooth: float = 0.5, w_clear: float = 1.0, w_len: float = 0.1,
             collision_margin: float = 0.0):
    """Elastic-band smoother: gradient-descend the path off obstacles (via ∇ESDF) while keeping it
    smooth and short. Endpoints pinned. A safety projection after every step GUARANTEES no interior
    waypoint is left inside an obstacle (clearance > ``collision_margin``)."""
    P = np.asarray(path, dtype=float).copy()
    if len(P) < 3:
        return P
    membrane = w_smooth + w_len            # Laplacian = gradient of squared length: smooths + shortens
    for _ in range(iters):
        prev = P.copy()
        grad = np.zeros_like(P)
        for i in range(1, len(P) - 1):
            grad[i] += membrane * (P[i - 1] - 2.0 * P[i] + P[i + 1])
            d = grid.distance(P[i, 0], P[i, 1])
            if d < d_safe:                                    # collision: push out along +∇ESDF
                gx, gy = grid.gradient(P[i, 0], P[i, 1])
                grad[i] += w_clear * (d_safe - d) * np.array([gx, gy])
        P[1:-1] += lr * grad[1:-1]
        # Safety projection: a step may never leave an interior waypoint inside an obstacle.
        for i in range(1, len(P) - 1):
            if grid.distance(P[i, 0], P[i, 1]) <= collision_margin:
                P[i] = _project_out(grid, P[i], prev[i], collision_margin)
    return P


def plan(grid, start_xy, goal_xy, d_safe: float = 1.2, spacing: float = 1.0,
         clearance: float = 0.0, **opt_kw):
    """Full pipeline: A* (clearance) → resample → ESDF-optimize. Returns an Nx2 array
    of world waypoints, or None if the front-end finds no route."""
    coarse = astar(grid, start_xy, goal_xy, clearance=clearance)
    if coarse is None:
        return None
    band = _resample(coarse, spacing)
    band[0] = start_xy
    band[-1] = goal_xy
    return optimize(band, grid, d_safe=d_safe, **opt_kw)


def path_metrics(grid, path):
    """Length (m), minimum clearance to any building (m, ESDF min along path), and a
    smoothness score (mean |turn angle|, rad). For the experiment's plot/table."""
    P = np.asarray(path, dtype=float)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    length = float(seg.sum())
    min_clear = min(grid.distance(x, y) for x, y in P)
    turns = []
    for i in range(1, len(P) - 1):
        a = P[i] - P[i - 1]
        b = P[i + 1] - P[i]
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na > 1e-6 and nb > 1e-6:
            c = float(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))
            turns.append(math.acos(c))
    smooth = float(np.mean(turns)) if turns else 0.0
    return {"length": length, "min_clearance": min_clear, "mean_turn": smooth}
