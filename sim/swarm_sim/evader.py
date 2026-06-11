"""Scripted fleeing target.

Greedy, obstacle-aware evasion: at each step the evader samples heading
candidates and picks the free direction that maximises distance to the nearest
pursuer while staying in bounds and away from walls. Deterministic given a seed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from .world import World


@dataclass
class Evader:
    pos: np.ndarray
    max_speed: float = 3.0
    vel: np.ndarray = field(default_factory=lambda: np.zeros(2))

    def __post_init__(self) -> None:
        self.pos = np.asarray(self.pos, dtype=float)

    def step(self, world: World, pursuers: list[np.ndarray], dt: float) -> None:
        if not pursuers:
            # Patrol when unobserved: keep the current heading, gently wandering. (The evader is
            # always stepped — it does not wait politely at its spawn until first detected.)
            heading = math.atan2(self.vel[1], self.vel[0]) if np.any(self.vel) else 0.0
            candidates = [heading + d for d in np.linspace(-0.6, 0.6, 5)]
        else:
            nearest = min(pursuers, key=lambda p: np.linalg.norm(p - self.pos))
            away = self.pos - nearest
            base = math.atan2(away[1], away[0])
            # Full-circle escape cone: when the away-direction is blocked (cornered against a
            # wall), the evader can still pick a flanking or doubling-back heading instead of
            # freezing in place.
            candidates = [base + d for d in np.linspace(-math.pi, math.pi, 17)]

        best, best_score = None, -1e18
        for h in candidates:
            step_vec = np.array([math.cos(h), math.sin(h)]) * self.max_speed * dt
            nxt = self.pos + step_vec
            if not (world.is_free(nxt[0], nxt[1], margin=0.3) and world.in_bounds(*nxt)):
                continue
            # Maximise clearance to the closest pursuer...
            score = min((np.linalg.norm(p - nxt) for p in pursuers), default=0.0)
            if pursuers:
                # ...preferring headings near "directly away" when free (escape > flanking).
                ddiff = abs((h - base + math.pi) % (2 * math.pi) - math.pi)
                score -= 0.4 * ddiff
            # Mild centring bias so it doesn't pin itself in a corner.
            score -= 0.05 * (abs(nxt[0] - world.width / 2) + abs(nxt[1] - world.height / 2))
            if score > best_score:
                best, best_score = nxt, score

        if best is not None:
            self.vel = (best - self.pos) / dt
            self.pos = best
        else:
            self.vel *= 0.0
