#!/usr/bin/env python3
"""Communication-robustness experiment for DART-SAR exploration."""

from __future__ import annotations

import argparse
import csv
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "sim"))

from swarm_sim.exploration import (
    DARTSARStage2Strategy,
    DARTSARStrategy,
    NearestFrontierStrategy,
)
from swarm_sim.simulator import SimConfig, Simulator
from swarm_sim.world import default_city
from swarm_autonomy_comms.link_model import LinkConfig


THRESHOLDS = (50, 75, 90, 95, 100)

STRATEGIES = {
    "nearest": NearestFrontierStrategy,
    "dartsar": DARTSARStrategy,
    "dartsar-stage2": DARTSARStage2Strategy,
}

METRICS = (
    "time_to_50_pct_s",
    "time_to_75_pct_s",
    "time_to_90_pct_s",
    "time_to_95_pct_s",
    "time_to_100_pct_s",
    "final_coverage_pct",
    "total_uav_travel_m",
    "communication_bytes",
    "total_overlap_cell_visits",
    "overlap_ratio",
)

FIELDNAMES = ("strategy", "base_loss", "seed", *METRICS)


def run_one(
    strategy_name: str,
    base_loss: float,
    seed: int,
    drones: int,
    duration_s: float,
) -> dict[str, object]:

    strategy = STRATEGIES[strategy_name]()

    comms = LinkConfig(
        bandwidth_bps=40_000.0,
        max_range_m=22.0,
        soft_range_m=14.0,
        base_loss=base_loss,
        edge_loss=0.6,
        bucket_capacity_bytes=25_000.0,
    )

    cfg = SimConfig(
        num_drones=drones,
        max_time_s=duration_s,
        seed=seed,
        record=False,
        explore_only=True,
        comms=comms,
    )

    sim = Simulator(
        default_city(seed),
        cfg,
        exploration_strategy=strategy,
    )

    reached = {threshold: None for threshold in THRESHOLDS}
    travel_m = 0.0

    physical_sensed = [
        np.zeros((sim.world.nx, sim.world.ny), dtype=bool)
        for _ in sim.drones
    ]

    for _ in range(int(duration_s / cfg.dt)):

        before = [drone.pos.copy() for drone in sim.drones]

        sim.step()

        for drone, mask in zip(sim.drones, physical_sensed):
            sim.world.sense_into(
                mask,
                drone.pos[0],
                drone.pos[1],
                drone.sensor_radius,
            )

        travel_m += sum(
            float(np.linalg.norm(drone.pos - start))
            for drone, start in zip(sim.drones, before)
        )

        coverage_pct = 100.0 * sim.world.coverage()

        for threshold in THRESHOLDS:
            if reached[threshold] is None and coverage_pct >= threshold:
                reached[threshold] = sim.t

    physical_visit_counts = sum(physical_sensed)

    free = sim.world.occ == 0

    total_sensed_free_visits = int(
        physical_visit_counts[free].sum()
    )

    overlap_cell_visits = int(
        np.maximum(
            physical_visit_counts[free] - 1,
            0,
        ).sum()
    )

    overlap_ratio = (
        overlap_cell_visits / total_sensed_free_visits
        if total_sensed_free_visits
        else None
    )

    return {
        "strategy": strategy_name,
        "base_loss": base_loss,
        "seed": seed,
        "time_to_50_pct_s": reached[50],
        "time_to_75_pct_s": reached[75],
        "time_to_90_pct_s": reached[90],
        "time_to_95_pct_s": reached[95],
        "time_to_100_pct_s": reached[100],
        "final_coverage_pct": 100.0 * sim.world.coverage(),
        "total_uav_travel_m": travel_m,
        "communication_bytes": sum(
            stats.bytes_delivered
            for stats in sim.stats.values()
        ),
        "total_overlap_cell_visits": overlap_cell_visits,
        "overlap_ratio": overlap_ratio,
    }


def _run_trial(
    trial: tuple[str, float, int, int, float]
) -> dict[str, object]:
    return run_one(*trial)


def write_summary(rows: list[dict[str, object]], path: str) -> None:

    fields = (
        "strategy",
        "base_loss",
        "metric",
        "n",
        "mean",
        "stddev",
    )

    with open(path, "w", newline="") as handle:

        writer = csv.DictWriter(
            handle,
            fieldnames=fields,
        )

        writer.writeheader()

        for strategy in STRATEGIES:

            for loss in sorted(
                {float(row["base_loss"]) for row in rows}
            ):

                group = [
                    row
                    for row in rows
                    if row["strategy"] == strategy
                    and float(row["base_loss"]) == loss
                ]

                for metric in METRICS:

                    values = [
                        float(row[metric])
                        for row in group
                        if row[metric] is not None
                    ]

                    mean = (
                        float(np.mean(values))
                        if values
                        else None
                    )

                    stddev = (
                        float(np.std(values, ddof=1))
                        if len(values) > 1
                        else 0.0
                        if values
                        else None
                    )

                    writer.writerow({
                        "strategy": strategy,
                        "base_loss": loss,
                        "metric": metric,
                        "n": len(values),
                        "mean": mean,
                        "stddev": stddev,
                    })


def main(argv: list[str]) -> int:

    parser = argparse.ArgumentParser(
        description="Measure exploration robustness under communication packet loss."
    )

    parser.add_argument(
        "--losses",
        type=float,
        nargs="+",
        default=[0.02, 0.10, 0.30, 0.50],
        help="base packet-loss probabilities",
    )

    parser.add_argument(
        "--seed-start",
        type=int,
        default=1,
    )

    parser.add_argument(
        "--seed-end",
        type=int,
        default=30,
    )

    parser.add_argument(
        "--drones",
        type=int,
        default=4,
    )

    parser.add_argument(
        "--time",
        type=float,
        default=90.0,
    )

    parser.add_argument(
        "--workers",
        type=int,
        default=1,
    )

    parser.add_argument(
        "--raw-csv",
        default=os.path.join(
            ROOT,
            "experiments",
            "comms_robustness_raw.csv",
        ),
    )

    parser.add_argument(
        "--summary-csv",
        default=os.path.join(
            ROOT,
            "experiments",
            "comms_robustness_summary.csv",
        ),
    )

    args = parser.parse_args(argv)

    if args.seed_end < args.seed_start:
        parser.error(
            "--seed-end must be at least --seed-start"
        )

    if any(loss < 0.0 or loss > 1.0 for loss in args.losses):
        parser.error(
            "--losses values must be between 0 and 1"
        )

    trials = [
        (
            strategy,
            loss,
            seed,
            args.drones,
            args.time,
        )
        for strategy in STRATEGIES
        for loss in args.losses
        for seed in range(
            args.seed_start,
            args.seed_end + 1,
        )
    ]

    if args.workers == 1:

        rows = []

        for trial in trials:

            print(
                f"{trial[0]} loss={trial[1]:.2f} seed={trial[2]}",
                flush=True,
            )

            rows.append(
                run_one(*trial)
            )

    else:

        with ProcessPoolExecutor(
            max_workers=args.workers
        ) as pool:

            rows = list(
                pool.map(
                    _run_trial,
                    trials,
                )
            )

    rows.sort(
        key=lambda row: (
            str(row["strategy"]),
            float(row["base_loss"]),
            int(row["seed"]),
        )
    )

    for path in (
        args.raw_csv,
        args.summary_csv,
    ):
        os.makedirs(
            os.path.dirname(
                os.path.abspath(path)
            ),
            exist_ok=True,
        )

    with open(
        args.raw_csv,
        "w",
        newline="",
    ) as handle:

        writer = csv.DictWriter(
            handle,
            fieldnames=FIELDNAMES,
        )

        writer.writeheader()
        writer.writerows(rows)

    write_summary(
        rows,
        args.summary_csv,
    )

    print(
        f"wrote {args.raw_csv}"
    )

    print(
        f"wrote {args.summary_csv}"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(
        main(sys.argv[1:])
    )