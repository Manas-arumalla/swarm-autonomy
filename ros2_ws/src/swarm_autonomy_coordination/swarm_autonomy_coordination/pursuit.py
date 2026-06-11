"""Pursuit / interception geometry.

ROS-free, deterministic helpers for the headline pursuit demo:

* :func:`predict_target` — constant-velocity lookahead of the fused target estimate.
* :func:`intercept_point` — proportional-navigation-style rendezvous point so an
  interceptor aims where the target *will be*, not where it is.
* :func:`containment_ring` — evenly spaced standoff goals around the target so
  blockers cut off escape corridors (the "corner the target" behaviour).
* :func:`assign_ring_slots` — Hungarian-free greedy nearest-slot assignment of
  drones to ring positions (cheap; the *who-does-what* role split is CBBA's job).

Vectors are plain ``(x, y, z)`` tuples to stay dependency-light and testable.
"""

from __future__ import annotations

import math
from typing import Sequence

Vec3 = tuple[float, float, float]


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _norm(a: Vec3) -> float:
    return math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)


def predict_target(pos: Vec3, vel: Vec3, horizon_s: float) -> Vec3:
    """Constant-velocity prediction of the target after ``horizon_s`` seconds."""
    return _add(pos, _scale(vel, horizon_s))


def intercept_point(
    pursuer: Vec3,
    target_pos: Vec3,
    target_vel: Vec3,
    pursuer_speed: float,
) -> Vec3:
    """Closed-form constant-velocity intercept point.

    Solves for the earliest time ``t`` at which a pursuer moving at
    ``pursuer_speed`` can reach the target, then returns the target's position at
    that time. Falls back to the current target position if no solution exists
    (e.g. target faster than pursuer and opening range).
    """
    r = _sub(target_pos, pursuer)
    # |r + v t| = pursuer_speed * t  ->  quadratic a t^2 + b t + c = 0
    a = target_vel[0] ** 2 + target_vel[1] ** 2 + target_vel[2] ** 2 - pursuer_speed ** 2
    b = 2.0 * (r[0] * target_vel[0] + r[1] * target_vel[1] + r[2] * target_vel[2])
    c = r[0] ** 2 + r[1] ** 2 + r[2] ** 2

    t = _smallest_positive_root(a, b, c)
    if t is None:
        return target_pos
    return predict_target(target_pos, target_vel, t)


def _smallest_positive_root(a: float, b: float, c: float) -> float | None:
    if abs(a) < 1e-9:
        if abs(b) < 1e-9:
            return None
        t = -c / b
        return t if t > 0 else None
    disc = b * b - 4 * a * c
    if disc < 0:
        return None
    sq = math.sqrt(disc)
    roots = sorted(t for t in ((-b - sq) / (2 * a), (-b + sq) / (2 * a)) if t > 1e-9)
    return roots[0] if roots else None


def containment_ring(center: Vec3, radius: float, n: int, z: float | None = None) -> list[Vec3]:
    """``n`` evenly spaced standoff goals on a horizontal ring around ``center``."""
    if n <= 0:
        return []
    zc = center[2] if z is None else z
    out: list[Vec3] = []
    for i in range(n):
        theta = 2.0 * math.pi * i / n
        out.append((center[0] + radius * math.cos(theta),
                    center[1] + radius * math.sin(theta),
                    zc))
    return out


def assign_ring_slots(drones: Sequence[Vec3], slots: Sequence[Vec3]) -> list[int]:
    """Greedy nearest-slot assignment; returns slot index per drone (-1 if none).

    Greedy is intentional: the high-level role split is CBBA's responsibility,
    this only places already-selected blockers onto the closest open arc.
    """
    assignment = [-1] * len(drones)
    taken = [False] * len(slots)
    order = sorted(
        ((_norm(_sub(d, s)), di, si) for di, d in enumerate(drones) for si, s in enumerate(slots))
    )
    for _, di, si in order:
        if assignment[di] == -1 and not taken[si]:
            assignment[di] = si
            taken[si] = True
    return assignment
