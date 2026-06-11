"""The Swarm Autonomy swarm simulator: cooperative exploration -> decentralized
role allocation (CBBA) -> interception of a fleeing target, all gated by the
real bandwidth-limited comms link model.

Each drone holds its *own* belief of the target and only learns the target's
position from (a) its own line-of-sight detection or (b) a neighbour observation
that survives the comms gating. Drones that never receive the target keep
exploring — the decentralization is real, not cosmetic.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import random

import numpy as np

# Real Swarm Autonomy algorithm modules (no ROS):
from swarm_autonomy_comms.link_model import LinkConfig, LinkModel, WindowStats
from swarm_autonomy_coordination.cbba import run_cbba
from swarm_autonomy_coordination import pursuit

from .drone import Drone
from .evader import Evader
from .world import World, default_city

# Role constants mirror swarm_autonomy_msgs/msg/DroneRole.msg
ROLE_IDLE, ROLE_SCOUT, ROLE_BLOCKER, ROLE_INTERCEPTOR = 0, 1, 2, 3


@dataclass
class Belief:
    pos: np.ndarray
    vel: np.ndarray
    t: float
    conf: float


@dataclass
class Frame:
    t: float
    drones: list[tuple[float, float, int]]      # (x, y, role)
    goals: list[tuple[float, float] | None]
    evader: tuple[float, float]
    known: np.ndarray
    coverage: float
    links: list[tuple[int, int]]                # delivered drone->drone this frame
    phase: str
    captured: bool


@dataclass
class SimConfig:
    num_drones: int = 4
    dt: float = 0.1
    max_time_s: float = 120.0
    capture_radius: float = 1.8
    containment_radius: float = 6.0
    predict_horizon_s: float = 1.2
    comms: LinkConfig = field(default_factory=lambda: LinkConfig(
        bandwidth_bps=40_000.0, max_range_m=22.0, soft_range_m=14.0, base_loss=0.02))
    seed: int = 0
    belief_timeout_s: float = 4.0
    evader_speed: float = 2.6
    record: bool = True
    explore_only: bool = False   # pure-exploration benchmark: no pursuit phase, no capture break


@dataclass
class SimResult:
    captured: bool
    capture_time: float | None
    final_coverage: float
    coverage_curve: list[tuple[float, float]]
    bandwidth_curve: list[tuple[float, float, float, float]]  # (t, total B/s, busiest-link B/s, cap)
    min_distance: float
    frames: list[Frame]


class Simulator:
    def __init__(self, world: World | None = None, cfg: SimConfig | None = None):
        self.cfg = cfg or SimConfig()
        self.world = world or default_city(self.cfg.seed)
        self.rng = random.Random(self.cfg.seed)
        c = self.cfg

        # Spawn drones along the bottom edge in free space; evader top area.
        self.drones: list[Drone] = []
        x = 2.0
        while len(self.drones) < c.num_drones:
            if self.world.is_free(x, 2.0, margin=0.5):
                self.drones.append(Drone(len(self.drones), np.array([x, 2.0])))
            x += 3.0
            if x > self.world.width - 2:
                x = 2.0 + self.rng.random()
        self.evader = Evader(np.array([self.world.width - 4.0, self.world.height - 4.0]),
                             max_speed=c.evader_speed)

        # Per-directed-link comms model + per-sender bandwidth accounting.
        self.links: dict[tuple[int, int], LinkModel] = {}
        self.stats: dict[int, WindowStats] = {d.drone_id: WindowStats() for d in self.drones}
        for a in self.drones:
            for b in self.drones:
                if a.drone_id != b.drone_id:
                    self.links[(a.drone_id, b.drone_id)] = LinkModel(
                        c.comms, random.Random(c.seed + a.drone_id * 131 + b.drone_id))

        self.beliefs: dict[int, Belief | None] = {d.drone_id: None for d in self.drones}
        self.t = 0.0
        self.phase = "explore"
        self.captured = False
        self.capture_time: float | None = None
        self.min_distance = float("inf")

        self._claimed_frontiers: set[tuple[int, int]] = set()
        self._win_stats = WindowStats()
        self._win_pair: dict[tuple[int, int], int] = {}   # delivered bytes per directed link/window
        self._win_start = 0.0
        self.coverage_curve: list[tuple[float, float]] = []
        # (t, total bytes/s all links, busiest single directed link bytes/s, per-link cap)
        self.bandwidth_curve: list[tuple[float, float, float, float]] = []
        self.frames: list[Frame] = []

    # ------------------------------------------------------------------ comms
    def _range(self, a: int, b: int) -> float:
        return float(np.linalg.norm(self.drones[a].pos - self.drones[b].pos))

    def _share_target(self) -> list[tuple[int, int]]:
        """Each detector broadcasts its observation; survivors update neighbour
        beliefs. Returns the delivered links (for rendering) and meters bytes."""
        delivered: list[tuple[int, int]] = []
        OBS_BYTES = 64
        for src in self.drones:
            b = self.beliefs[src.drone_id]
            if b is None or (self.t - b.t) > 0.5:      # only fresh self-detections
                continue
            for dst in self.drones:
                if dst.drone_id == src.drone_id:
                    continue
                res = self.links[(src.drone_id, dst.drone_id)].try_deliver(
                    self.t, self._range(src.drone_id, dst.drone_id), OBS_BYTES)
                self.stats[src.drone_id].add(res)
                self._win_stats.add(res)
                if res.delivered:
                    key = (src.drone_id, dst.drone_id)
                    self._win_pair[key] = self._win_pair.get(key, 0) + res.bytes
                    delivered.append((src.drone_id, dst.drone_id))
                    cur = self.beliefs[dst.drone_id]
                    if cur is None or b.t > cur.t:
                        self.beliefs[dst.drone_id] = Belief(b.pos.copy(), b.vel.copy(), b.t, b.conf)
        return delivered

    def _fused_belief(self) -> Belief | None:
        """Freshest belief any drone currently holds (the propagated estimate)."""
        best = None
        for b in self.beliefs.values():
            if b is not None and (best is None or b.t > best.t):
                best = b
        return best

    # -------------------------------------------------------------- detection
    def _detect(self) -> None:
        ev = self.evader.pos
        for d in self.drones:
            if d.sees(self.world, ev):
                self.beliefs[d.drone_id] = Belief(ev.copy(), self.evader.vel.copy(),
                                                  self.t, 1.0)

    # ------------------------------------------------------------ exploration
    def _assign_exploration_goals(self) -> None:
        frontiers = self.world.frontier_cells()
        if not frontiers:
            return
        self._claimed_frontiers.clear()
        for d in self.drones:
            if d.role in (ROLE_BLOCKER, ROLE_INTERCEPTOR):
                continue
            need = d.goal is None or d.at_goal(1.2)
            if not need and d.goal is not None:
                gi = self.world._to_cell(*d.goal)
                if gi in frontiers and gi not in self._claimed_frontiers:
                    self._claimed_frontiers.add(gi)
                    continue
            best, bestd = None, 1e18
            for (i, j) in frontiers:
                if (i, j) in self._claimed_frontiers:
                    continue
                cx, cy = self.world.cell_center(i, j)
                dist = np.linalg.norm(d.pos - np.array([cx, cy]))
                if dist < bestd:
                    best, bestd = (i, j), dist
            if best is not None:
                self._claimed_frontiers.add(best)
                d.set_goal(self.world.cell_center(*best))
                d.role = ROLE_SCOUT

    # ---------------------------------------------------------------- pursuit
    def _assign_pursuit_goals(self) -> None:
        fused = self._fused_belief()
        if fused is None:
            self.phase = "explore"
            for d in self.drones:
                d.role = ROLE_SCOUT
            return

        predicted = pursuit.predict_target(
            (fused.pos[0], fused.pos[1], 0.0),
            (fused.vel[0], fused.vel[1], 0.0), self.cfg.predict_horizon_s)
        slots = pursuit.containment_ring(predicted, self.cfg.containment_radius,
                                         self.cfg.num_drones)

        # CBBA: assign drones to containment slots (positive proximity score).
        dist = [[float(np.linalg.norm(d.pos - np.array([s[0], s[1]])))
                 for s in slots] for d in self.drones]

        def score_fn(agent, path, task):
            return 1.0 / (1.0 + dist[agent][task])

        assignment, _z = run_cbba(self.cfg.num_drones, len(slots), score_fn, max_bundle=1)

        # The drone closest to the target becomes the interceptor (PN-style lead).
        interceptor = min(range(self.cfg.num_drones),
                          key=lambda a: np.linalg.norm(self.drones[a].pos - fused.pos))
        ip = pursuit.intercept_point(
            (self.drones[interceptor].pos[0], self.drones[interceptor].pos[1], 0.0),
            (fused.pos[0], fused.pos[1], 0.0),
            (fused.vel[0], fused.vel[1], 0.0),
            self.drones[interceptor].max_speed)

        for a, d in enumerate(self.drones):
            if a == interceptor:
                d.role = ROLE_INTERCEPTOR
                d.set_goal((ip[0], ip[1]))
            else:
                d.role = ROLE_BLOCKER
                slot = assignment[a][0] if assignment[a] else a
                d.set_goal((slots[slot][0], slots[slot][1]))

    # ------------------------------------------------------------------- step
    def step(self) -> None:
        c = self.cfg
        self._detect()
        delivered = self._share_target()

        fused = self._fused_belief()
        if (not c.explore_only) and fused is not None and (self.t - fused.t) < c.belief_timeout_s:
            self.phase = "pursue"
            self._assign_pursuit_goals()
        else:
            self.phase = "explore"
            self._assign_exploration_goals()

        for d in self.drones:
            d.step(self.world, c.dt)
            self.world.sense(d.pos[0], d.pos[1], d.sensor_radius)

        # The evader is stepped EVERY frame (it does not wait at its spawn until first detected):
        # it patrols while unthreatened and flees from pursuers it can plausibly see (within its
        # own sensing range, line of sight).
        threats = [d.pos for d in self.drones
                   if np.linalg.norm(d.pos - self.evader.pos) <= 12.0
                   and not self.world.blocked(self.evader.pos[0], self.evader.pos[1],
                                              d.pos[0], d.pos[1])]
        self.evader.step(self.world, threats, c.dt)

        # Capture check.
        dmin = min(float(np.linalg.norm(d.pos - self.evader.pos)) for d in self.drones)
        self.min_distance = min(self.min_distance, dmin)
        if dmin < c.capture_radius and not self.captured:
            self.captured = True
            self.capture_time = self.t

        # Metrics windows.
        cov = self.world.coverage()
        self.coverage_curve.append((self.t, cov))
        if self.t - self._win_start >= 1.0:
            ws = self.t - self._win_start
            busiest = max(self._win_pair.values(), default=0) / ws   # per-link peak (vs the cap)
            self.bandwidth_curve.append(
                (self.t, self._win_stats.bytes_per_s(ws), busiest, c.comms.bandwidth_bps))
            self._win_stats = WindowStats()
            self._win_pair = {}
            self._win_start = self.t

        if c.record:
            self.frames.append(Frame(
                t=self.t,
                drones=[(d.pos[0], d.pos[1], d.role) for d in self.drones],
                goals=[(d.goal[0], d.goal[1]) if d.goal is not None else None for d in self.drones],
                evader=(self.evader.pos[0], self.evader.pos[1]),
                known=self.world.known.copy(),
                coverage=cov, links=delivered, phase=self.phase, captured=self.captured))
        self.t += c.dt

    def run(self) -> SimResult:
        steps = int(self.cfg.max_time_s / self.cfg.dt)
        for _ in range(steps):
            self.step()
            if (not self.cfg.explore_only) and self.captured \
                    and self.t - (self.capture_time or 0) > 1.0:
                break
        return SimResult(
            captured=self.captured, capture_time=self.capture_time,
            final_coverage=self.world.coverage(),
            coverage_curve=self.coverage_curve, bandwidth_curve=self.bandwidth_curve,
            min_distance=self.min_distance, frames=self.frames)
