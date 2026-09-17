"""Exploration-goal selection strategies for the swarm simulator.

Strategies only choose exploration goals. Motion, sensing, communications, and
pursuit remain owned by their existing simulator components.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .drone import Drone
from .world import World


# Mirrors swarm_autonomy_msgs/msg/DroneRole.msg.  Kept here so strategies do
# not need to import the simulator (which owns the pursuit implementation).
ROLE_SCOUT, ROLE_BLOCKER, ROLE_INTERCEPTOR = 1, 2, 3


class ExplorationGoalStrategy(Protocol):
    """Select exploration goals for the supplied drones."""

    def assign_goals(
        self,
        world: World,
        drones: list[Drone],
        claimed_frontiers: set[tuple[int, int]],
    ) -> None:
        """Update drone goals and frontier claims for one exploration cycle."""


class NearestFrontierStrategy:
    """Preserve the simulator's original greedy nearest-frontier policy."""

    def assign_goals(
        self,
        world: World,
        drones: list[Drone],
        claimed_frontiers: set[tuple[int, int]],
    ) -> None:
        if not any(d.local_known is not None and world.frontier_cells(d.local_known)
                   for d in drones):
            return
        claimed_frontiers.clear()
        for d in drones:
            if d.role in (ROLE_BLOCKER, ROLE_INTERCEPTOR):
                continue
            if d.local_known is None:
                continue
            frontiers = world.frontier_cells(d.local_known)
            if not frontiers:
                continue
            need = d.goal is None or d.at_goal(1.2)
            if not need and d.goal is not None:
                gi = world._to_cell(*d.goal)
                if gi in frontiers and gi not in claimed_frontiers:
                    claimed_frontiers.add(gi)
                    continue
            best, bestd = None, 1e18
            for (i, j) in frontiers:
                if (i, j) in claimed_frontiers:
                    continue
                cx, cy = world.cell_center(i, j)
                dist = np.linalg.norm(d.pos - np.array([cx, cy]))
                if dist < bestd:
                    best, bestd = (i, j), dist
            if best is not None:
                claimed_frontiers.add(best)
                d.set_goal(world.cell_center(*best))
                d.role = ROLE_SCOUT


@dataclass(frozen=True)
class DARTSARScore:
    """Normalized Stage-1 utility components for one frontier candidate."""

    information_gain: float
    distance_cost: float
    utility: float


class DARTSARStrategy:
    """Stage-1 DART-SAR frontier selection from each UAV's local map only."""

    information_weight = 0.7
    distance_weight = 0.3

    def score_frontiers(
        self,
        world: World,
        drone: Drone,
        frontiers: list[tuple[int, int]] | None = None,
    ) -> dict[tuple[int, int], DARTSARScore]:
        """Return local-map information gain, travel cost, and utility per frontier."""
        if drone.local_known is None:
            return {}
        frontiers = world.frontier_cells(drone.local_known) if frontiers is None else frontiers
        if not frontiers:
            return {}

        gains: list[int] = []
        distances: list[float] = []
        for frontier in frontiers:
            center = world.cell_center(*frontier)
            gains.append(world.observable_unknown_free_count(
                drone.local_known, *center, drone.sensor_radius))
            distances.append(float(np.linalg.norm(drone.pos - np.array(center))))

        max_gain = max(gains)
        min_distance, max_distance = min(distances), max(distances)
        distance_span = max_distance - min_distance
        scores: dict[tuple[int, int], DARTSARScore] = {}
        for frontier, gain, distance in zip(frontiers, gains, distances):
            information_gain = gain / max_gain if max_gain else 0.0
            distance_cost = ((distance - min_distance) / distance_span
                             if distance_span else 0.0)
            utility = (self.information_weight * information_gain
                       - self.distance_weight * distance_cost)
            scores[frontier] = DARTSARScore(information_gain, distance_cost, utility)
        return scores

    def assign_goals(
        self,
        world: World,
        drones: list[Drone],
        claimed_frontiers: set[tuple[int, int]],
    ) -> None:
        if not any(d.local_known is not None and world.frontier_cells(d.local_known)
                   for d in drones):
            return
        claimed_frontiers.clear()
        for drone in drones:
            if drone.role in (ROLE_BLOCKER, ROLE_INTERCEPTOR) or drone.local_known is None:
                continue
            frontiers = world.frontier_cells(drone.local_known)
            if not frontiers:
                continue
            need = drone.goal is None or drone.at_goal(1.2)
            if not need and drone.goal is not None:
                current = world._to_cell(*drone.goal)
                if current in frontiers and current not in claimed_frontiers:
                    claimed_frontiers.add(current)
                    continue
            scores = self.score_frontiers(
                world, drone, [frontier for frontier in frontiers
                               if frontier not in claimed_frontiers])
            if scores:
                best = max(scores, key=lambda frontier: scores[frontier].utility)
                claimed_frontiers.add(best)
                drone.set_goal(world.cell_center(*best))
                drone.role = ROLE_SCOUT


@dataclass(frozen=True)
class DARTSARStage2Score:
    """Stage-2 utility components for one locally generated frontier."""

    information_gain: float
    distance_cost: float
    redundancy_ratio: float
    marginal_contribution: float
    utility: float


class DARTSARStage2Strategy(DARTSARStrategy):
    """DART-SAR Stage 2: penalize candidate sensing already reported by teammates."""

    marginal_weight = 0.6
    information_weight = 0.1
    distance_weight = 0.3

    def score_frontiers(
        self,
        world: World,
        drone: Drone,
        frontiers: list[tuple[int, int]] | None = None,
    ) -> dict[tuple[int, int], DARTSARStage2Score]:
        """Score local frontiers using only received teammate map information."""
        stage1_scores = super().score_frontiers(world, drone, frontiers)
        if not stage1_scores or drone.teammate_known is None:
            teammate_known = np.zeros((world.nx, world.ny), dtype=np.int8)
        else:
            teammate_known = drone.teammate_known

        raw_marginal: dict[tuple[int, int], float] = {}
        redundancy: dict[tuple[int, int], float] = {}
        for frontier, score in stage1_scores.items():
            visible = list(world.visible_cells(*world.cell_center(*frontier), drone.sensor_radius))
            known_by_teammates = sum(teammate_known[i, j] != 0 for i, j in visible)
            ratio = known_by_teammates / len(visible) if visible else 0.0
            ratio = min(1.0, max(0.0, ratio))
            redundancy[frontier] = ratio
            raw_marginal[frontier] = score.information_gain * (1.0 - ratio)

        maximum = max(raw_marginal.values(), default=0.0)
        scores: dict[tuple[int, int], DARTSARStage2Score] = {}
        for frontier, stage1_score in stage1_scores.items():
            marginal = raw_marginal[frontier] / maximum if maximum else 0.0
            utility = (self.marginal_weight * marginal
                       + self.information_weight * stage1_score.information_gain
                       - self.distance_weight * stage1_score.distance_cost)
            scores[frontier] = DARTSARStage2Score(
                stage1_score.information_gain, stage1_score.distance_cost,
                redundancy[frontier], marginal, utility)
        return scores

    def assign_goals(
        self,
        world: World,
        drones: list[Drone],
        claimed_frontiers: set[tuple[int, int]],
    ) -> None:
        if not any(d.local_known is not None and world.frontier_cells(d.local_known)
                   for d in drones):
            return
        claimed_frontiers.clear()
        for drone in drones:
            if drone.role in (ROLE_BLOCKER, ROLE_INTERCEPTOR) or drone.local_known is None:
                continue
            frontiers = world.frontier_cells(drone.local_known)
            if not frontiers:
                continue
            need = drone.goal is None or drone.at_goal(1.2)
            if not need and drone.goal is not None:
                current = world._to_cell(*drone.goal)
                if current in frontiers and current not in claimed_frontiers:
                    claimed_frontiers.add(current)
                    continue
            scores = self.score_frontiers(
                world, drone, [frontier for frontier in frontiers
                               if frontier not in claimed_frontiers])
            if scores:
                best = max(scores, key=lambda frontier: scores[frontier].utility)
                claimed_frontiers.add(best)
                drone.set_goal(world.cell_center(*best))
                drone.role = ROLE_SCOUT
