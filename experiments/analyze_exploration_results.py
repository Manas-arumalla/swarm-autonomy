#!/usr/bin/env python3
"""Paired statistical analysis for the reproducible exploration experiment."""

from __future__ import annotations

import csv
import math
import os
import statistics
from collections import defaultdict

from scipy import stats


ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
RAW_CSV = os.path.join(ROOT, "experiments", "exploration_comparison_raw.csv")
OUTPUT_CSV = os.path.join(ROOT, "experiments", "exploration_statistical_analysis.csv")
EXPECTED_SEEDS = set(range(1, 31))
STRATEGIES = ("nearest", "dartsar", "dartsar-stage2")
COMPARISONS = (("nearest", "dartsar"), ("nearest", "dartsar-stage2"),
               ("dartsar", "dartsar-stage2"))
METRICS = (
    "time_to_50_pct_s", "time_to_75_pct_s", "time_to_90_pct_s", "time_to_95_pct_s",
    "time_to_100_pct_s", "final_coverage_pct", "total_uav_travel_m",
    "communication_bytes", "goal_assignments", "repeated_frontier_selections",
    "average_travel_per_assigned_goal_m", "total_overlap_cell_visits", "overlap_ratio",
)
PERCENT_CHANGE_METRICS = (
    "total_uav_travel_m", "total_overlap_cell_visits", "overlap_ratio",
    "time_to_50_pct_s", "time_to_75_pct_s", "time_to_90_pct_s", "time_to_95_pct_s",
    "time_to_100_pct_s",
)
OUTPUT_COLUMNS = (
    "record_type", "metric", "strategy_a", "strategy_b", "n_paired", "mean_strategy_a",
    "mean_strategy_b", "mean_paired_difference_b_minus_a", "median_paired_difference_b_minus_a",
    "stddev_paired_difference", "ci95_lower", "ci95_upper", "wilcoxon_p_value",
    "standardized_mean_paired_difference", "mean_percent_change_b_vs_a",
)


def as_float(value: str | None) -> float | None:
    return None if value in (None, "") else float(value)


def load_and_validate(path: str) -> dict[str, dict[int, dict[str, str]]]:
    data: dict[str, dict[int, dict[str, str]]] = defaultdict(dict)
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            strategy, seed = row["strategy"], int(row["seed"])
            if strategy not in STRATEGIES:
                raise ValueError(f"unexpected strategy: {strategy}")
            if seed in data[strategy]:
                raise ValueError(f"duplicate row for {strategy}, seed {seed}")
            data[strategy][seed] = row
    for strategy in STRATEGIES:
        seeds = set(data[strategy])
        if seeds != EXPECTED_SEEDS:
            raise ValueError(f"{strategy} seeds differ from 1–30: {sorted(seeds)}")
    if len({frozenset(data[strategy]) for strategy in STRATEGIES}) != 1:
        raise ValueError("strategies do not share identical pairing seeds")
    return data


def paired_values(data, strategy_a: str, strategy_b: str, metric: str):
    pairs = []
    for seed in sorted(EXPECTED_SEEDS):
        a = as_float(data[strategy_a][seed][metric])
        b = as_float(data[strategy_b][seed][metric])
        if a is not None and b is not None:
            pairs.append((a, b))
    return pairs


def paired_record(data, strategy_a: str, strategy_b: str, metric: str) -> dict[str, object]:
    pairs = paired_values(data, strategy_a, strategy_b, metric)
    a_values, b_values = zip(*pairs) if pairs else ((), ())
    differences = [b - a for a, b in pairs]
    n = len(differences)
    mean_difference = statistics.mean(differences) if n else None
    stddev = statistics.stdev(differences) if n > 1 else None
    if n > 1 and stddev is not None:
        margin = stats.t.ppf(0.975, n - 1) * stddev / math.sqrt(n)
        ci_lower, ci_upper = mean_difference - margin, mean_difference + margin
        effect_size = mean_difference / stddev if stddev else None
    else:
        ci_lower = ci_upper = effect_size = None
    wilcoxon_p = None
    if n and any(difference != 0 for difference in differences):
        wilcoxon_p = stats.wilcoxon(differences, alternative="two-sided", method="auto").pvalue
    return {
        "record_type": "paired_comparison", "metric": metric,
        "strategy_a": strategy_a, "strategy_b": strategy_b, "n_paired": n,
        "mean_strategy_a": statistics.mean(a_values) if n else None,
        "mean_strategy_b": statistics.mean(b_values) if n else None,
        "mean_paired_difference_b_minus_a": mean_difference,
        "median_paired_difference_b_minus_a": statistics.median(differences) if n else None,
        "stddev_paired_difference": stddev, "ci95_lower": ci_lower, "ci95_upper": ci_upper,
        "wilcoxon_p_value": wilcoxon_p,
        "standardized_mean_paired_difference": effect_size,
        "mean_percent_change_b_vs_a": None,
    }


def percent_change_record(data, metric: str) -> dict[str, object]:
    pairs = paired_values(data, "dartsar", "dartsar-stage2", metric)
    changes = [(stage2 - stage1) / stage1 * 100.0 for stage1, stage2 in pairs if stage1 != 0]
    return {
        "record_type": "stage1_to_stage2_percent_change", "metric": metric,
        "strategy_a": "dartsar", "strategy_b": "dartsar-stage2", "n_paired": len(changes),
        "mean_strategy_a": None, "mean_strategy_b": None,
        "mean_paired_difference_b_minus_a": None, "median_paired_difference_b_minus_a": None,
        "stddev_paired_difference": None, "ci95_lower": None, "ci95_upper": None,
        "wilcoxon_p_value": None, "standardized_mean_paired_difference": None,
        "mean_percent_change_b_vs_a": statistics.mean(changes) if changes else None,
    }


def main() -> int:
    data = load_and_validate(RAW_CSV)
    records = [paired_record(data, strategy_a, strategy_b, metric)
               for strategy_a, strategy_b in COMPARISONS for metric in METRICS]
    records.extend(percent_change_record(data, metric) for metric in PERCENT_CHANGE_METRICS)
    with open(OUTPUT_CSV, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=OUTPUT_COLUMNS)
        writer.writeheader()
        writer.writerows(records)

    print("Paired seeds verified: 1–30 for nearest, dartsar, and dartsar-stage2.")
    print("Stage 1 → Stage 2 mean paired percent changes:")
    for record in records[-len(PERCENT_CHANGE_METRICS):]:
        value = record["mean_percent_change_b_vs_a"]
        print(f"  {record['metric']}: {value:.2f}% (n={record['n_paired']})" if value is not None
              else f"  {record['metric']}: unavailable")
    print(f"Wrote {OUTPUT_CSV}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
