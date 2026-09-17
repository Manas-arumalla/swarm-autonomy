#!/usr/bin/env python3
"""Reproducible multi-seed exploration comparison for the simulator."""

from __future__ import annotations

import argparse
import csv
import os
import statistics
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "sim"))

from swarm_sim.exploration import DARTSARStage2Strategy, DARTSARStrategy, NearestFrontierStrategy
from swarm_sim.simulator import SimConfig, Simulator
from swarm_sim.world import default_city


THRESHOLDS = (50, 75, 90, 95, 100)
METRICS = tuple([f"time_to_{threshold}_pct_s" for threshold in THRESHOLDS]
                + ["final_coverage_pct", "total_uav_travel_m", "communication_bytes",
                   "goal_assignments", "goal_changes_after_existing_goal",
                   "unique_frontier_cells_selected", "repeated_frontier_selections",
                   "average_travel_per_assigned_goal_m", "total_overlap_cell_visits",
                   "overlap_ratio"])
FIELDNAMES = ("strategy", "seed", *METRICS, "goal_history")


def run_one(strategy_name: str, seed: int, drones: int, duration_s: float) -> dict[str, object]:
    """Run one fixed-condition exploration trial and return measured metrics."""
    strategy = {"nearest": NearestFrontierStrategy,
                "dartsar": DARTSARStrategy,
                "dartsar-stage2": DARTSARStage2Strategy}[strategy_name]()
    cfg = SimConfig(num_drones=drones, max_time_s=duration_s, seed=seed,
                    record=False, explore_only=True)
    sim = Simulator(default_city(seed), cfg, exploration_strategy=strategy)
    reached = {threshold: None for threshold in THRESHOLDS}
    travel_m = 0.0
    prior_goals = [None] * len(sim.drones)
    goal_assignments = 0
    goal_changes = 0
    selected_frontiers: list[tuple[int, int]] = []
    goal_history: list[str] = []
    physical_sensed = [np.zeros((sim.world.nx, sim.world.ny), dtype=bool)
                       for _ in sim.drones]

    for _ in range(int(duration_s / cfg.dt)):
        before = [drone.pos.copy() for drone in sim.drones]
        sim.step()
        for drone, mask in zip(sim.drones, physical_sensed):
            # Diagnostic-only: replay this UAV's own sensor observation into a
            # private mask. Communication never writes to this mask.
            sim.world.sense_into(mask, drone.pos[0], drone.pos[1], drone.sensor_radius)
        travel_m += sum(float(np.linalg.norm(drone.pos - start))
                        for drone, start in zip(sim.drones, before))
        for drone in sim.drones:
            goal = None if drone.goal is None else sim.world._to_cell(*drone.goal)
            previous = prior_goals[drone.drone_id]
            if goal is not None and goal != previous:
                goal_assignments += 1
                goal_changes += previous is not None
                selected_frontiers.append(goal)
                goal_history.append(f"{sim.t:.1f}:{drone.drone_id}:{goal[0]}:{goal[1]}")
            prior_goals[drone.drone_id] = goal
        coverage_pct = 100.0 * sim.world.coverage()
        for threshold in THRESHOLDS:
            if reached[threshold] is None and coverage_pct >= threshold:
                reached[threshold] = sim.t

    physical_visit_counts = sum(physical_sensed)
    free = sim.world.occ == 0
    total_sensed_free_visits = int(physical_visit_counts[free].sum())
    overlap_cell_visits = int(np.maximum(physical_visit_counts[free] - 1, 0).sum())

    return {
        "strategy": strategy_name,
        "seed": seed,
        **{f"time_to_{threshold}_pct_s": reached[threshold] for threshold in THRESHOLDS},
        "final_coverage_pct": 100.0 * sim.world.coverage(),
        "total_uav_travel_m": travel_m,
        "communication_bytes": sum(stats.bytes_delivered for stats in sim.stats.values()),
        "goal_assignments": goal_assignments,
        "goal_changes_after_existing_goal": goal_changes,
        "unique_frontier_cells_selected": len(set(selected_frontiers)),
        "repeated_frontier_selections": len(selected_frontiers) - len(set(selected_frontiers)),
        "average_travel_per_assigned_goal_m": travel_m / goal_assignments if goal_assignments else None,
        "total_overlap_cell_visits": overlap_cell_visits,
        "overlap_ratio": (overlap_cell_visits / total_sensed_free_visits
                          if total_sensed_free_visits else None),
        "goal_history": ";".join(goal_history),
    }


def write_summary(rows: list[dict[str, object]], path: str) -> None:
    """Write means and sample standard deviations for each strategy and metric."""
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=("strategy", "metric", "n", "mean", "stddev"))
        writer.writeheader()
        for strategy in ("nearest", "dartsar", "dartsar-stage2"):
            group = [row for row in rows if row["strategy"] == strategy]
            for metric in METRICS:
                values = [float(row[metric]) for row in group if row[metric] is not None]
                writer.writerow({
                    "strategy": strategy,
                    "metric": metric,
                    "n": len(values),
                    "mean": statistics.mean(values) if values else None,
                    "stddev": statistics.stdev(values) if len(values) > 1 else 0.0 if values else None,
                })


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Compare nearest and DART-SAR exploration.")
    parser.add_argument("--seed-start", type=int, default=1)
    parser.add_argument("--seed-end", type=int, default=30)
    parser.add_argument("--drones", type=int, default=4)
    parser.add_argument("--time", type=float, default=90.0)
    parser.add_argument("--workers", type=int, default=1,
                        help="parallel independent trials (default: 1)")
    parser.add_argument("--raw-csv", default=os.path.join(ROOT, "experiments", "exploration_comparison_raw.csv"))
    parser.add_argument("--summary-csv", default=os.path.join(ROOT, "experiments", "exploration_comparison_summary.csv"))
    args = parser.parse_args(argv)
    if args.seed_end < args.seed_start:
        parser.error("--seed-end must be at least --seed-start")

    trials = [(strategy, seed, args.drones, args.time)
              for strategy in ("nearest", "dartsar", "dartsar-stage2")
              for seed in range(args.seed_start, args.seed_end + 1)]
    if args.workers == 1:
        rows = []
        for trial in trials:
            print(f"{trial[0]} seed {trial[1]}", flush=True)
            rows.append(run_one(*trial))
    else:
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            rows = list(pool.map(_run_trial, trials))
    rows.sort(key=lambda row: (str(row["strategy"]), int(row["seed"])))

    for path in (args.raw_csv, args.summary_csv):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(args.raw_csv, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)
    write_summary(rows, args.summary_csv)
    print(f"wrote {args.raw_csv}")
    print(f"wrote {args.summary_csv}")
    return 0


def _run_trial(trial: tuple[str, int, int, float]) -> dict[str, object]:
    return run_one(*trial)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
