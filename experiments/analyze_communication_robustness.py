#!/usr/bin/env python3

import argparse
import csv
import math
import os
from itertools import combinations

import numpy as np
from scipy import stats


EXPECTED_STRATEGIES = [
    "nearest",
    "dartsar",
    "dartsar-stage2",
]

EXPECTED_LOSSES = [
    0.02,
    0.10,
    0.30,
    0.50,
]

METRICS = [
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
]

COMPARISONS = [
    ("nearest", "dartsar"),
    ("nearest", "dartsar-stage2"),
    ("dartsar", "dartsar-stage2"),
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Statistical analysis of communication robustness experiment."
    )

    parser.add_argument(
        "--input",
        default="experiments/comms_robustness_raw.csv",
    )

    parser.add_argument(
        "--output",
        default="experiments/comms_robustness_statistical_analysis.csv",
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

    return parser.parse_args()


def finite_float(value):
    try:
        x = float(value)

        if math.isfinite(x):
            return x

    except (TypeError, ValueError):
        pass

    return math.nan


def load_data(path):
    if not os.path.exists(path):
        raise FileNotFoundError(
            "Input file not found: {}".format(path)
        )

    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)

        if reader.fieldnames is None:
            raise ValueError("CSV has no header.")

        required = {
            "strategy",
            "base_loss",
            "seed",
        }

        missing = required - set(reader.fieldnames)

        if missing:
            raise ValueError(
                "CSV is missing required columns: {}".format(
                    ", ".join(sorted(missing))
                )
            )

        missing_metrics = [
            metric
            for metric in METRICS
            if metric not in reader.fieldnames
        ]

        if missing_metrics:
            raise ValueError(
                "CSV is missing metric columns: {}".format(
                    ", ".join(missing_metrics)
                )
            )

        rows = []

        for row in reader:
            try:
                strategy = row["strategy"].strip()
                loss = float(row["base_loss"])
                seed = int(row["seed"])
            except (TypeError, ValueError):
                continue

            record = {
                "strategy": strategy,
                "base_loss": loss,
                "seed": seed,
            }

            for metric in METRICS:
                record[metric] = finite_float(
                    row.get(metric)
                )

            rows.append(record)

    return rows


def validate_data(
    rows,
    seed_start,
    seed_end,
):
    print("\n=== DATA VALIDATION ===")

    expected_seeds = set(
        range(
            seed_start,
            seed_end + 1,
        )
    )

    problems = []

    for strategy in EXPECTED_STRATEGIES:

        for loss in EXPECTED_LOSSES:

            matching = [
                row
                for row in rows
                if row["strategy"] == strategy
                and math.isclose(
                    row["base_loss"],
                    loss,
                    abs_tol=1e-8,
                )
            ]

            seeds = {
                row["seed"]
                for row in matching
            }

            missing = sorted(
                expected_seeds - seeds
            )

            if missing:
                problems.append(
                    "{} loss={} missing seeds: {}".format(
                        strategy,
                        loss,
                        missing,
                    )
                )

    if problems:

        print(
            "WARNING: incomplete combinations found."
        )

        for problem in problems:
            print("  " + problem)

    else:

        print(
            "All expected strategy/loss/seed combinations are present."
        )

    print(
        "Strategies: {}".format(
            ", ".join(EXPECTED_STRATEGIES)
        )
    )

    print(
        "Loss levels: {}".format(
            ", ".join(
                "{:.2f}".format(x)
                for x in EXPECTED_LOSSES
            )
        )
    )

    print(
        "Seeds: {}-{}".format(
            seed_start,
            seed_end,
        )
    )

    print(
        "Raw rows loaded: {}".format(
            len(rows)
        )
    )


def make_lookup(rows):
    lookup = {}

    for row in rows:

        key = (
            row["strategy"],
            round(row["base_loss"], 8),
            row["seed"],
        )

        lookup[key] = row

    return lookup


def get_paired_values(
    lookup,
    strategy_a,
    strategy_b,
    loss,
    metric,
    seeds,
):
    values_a = []
    values_b = []

    paired_seeds = []

    for seed in seeds:

        row_a = lookup.get(
            (
                strategy_a,
                round(loss, 8),
                seed,
            )
        )

        row_b = lookup.get(
            (
                strategy_b,
                round(loss, 8),
                seed,
            )
        )

        if row_a is None or row_b is None:
            continue

        value_a = row_a[metric]
        value_b = row_b[metric]

        # Missing time-to-100 values are excluded.
        if not math.isfinite(value_a):
            continue

        if not math.isfinite(value_b):
            continue

        values_a.append(value_a)
        values_b.append(value_b)
        paired_seeds.append(seed)

    return (
        np.asarray(values_a, dtype=float),
        np.asarray(values_b, dtype=float),
        paired_seeds,
    )


def confidence_interval(differences):
    """
    95% CI for the mean paired difference
    using the Student-t distribution.
    """

    n = len(differences)

    if n < 2:
        return math.nan, math.nan

    mean_difference = np.mean(
        differences
    )

    std_difference = np.std(
        differences,
        ddof=1,
    )

    if std_difference == 0:

        return (
            float(mean_difference),
            float(mean_difference),
        )

    standard_error = (
        std_difference
        / math.sqrt(n)
    )

    critical_value = stats.t.ppf(
        0.975,
        df=n - 1,
    )

    margin = (
        critical_value
        * standard_error
    )

    return (
        float(mean_difference - margin),
        float(mean_difference + margin),
    )


def wilcoxon_test(differences):
    """
    Two-sided paired Wilcoxon signed-rank test.
    """

    if len(differences) == 0:
        return math.nan

    if np.allclose(
        differences,
        0.0,
    ):
        return 1.0

    try:

        result = stats.wilcoxon(
            differences,
            zero_method="wilcox",
            alternative="two-sided",
            method="auto",
        )

        return float(
            result.pvalue
        )

    except ValueError:

        return math.nan


def paired_effect_size(differences):
    """
    Standardized mean paired difference:

        mean difference / SD of differences
    """

    n = len(differences)

    if n < 2:
        return math.nan

    mean_difference = np.mean(
        differences
    )

    std_difference = np.std(
        differences,
        ddof=1,
    )

    if std_difference == 0:

        if mean_difference == 0:
            return 0.0

        return math.copysign(
            math.inf,
            mean_difference,
        )

    return float(
        mean_difference
        / std_difference
    )


def percent_change(
    mean_a,
    mean_b,
):
    if mean_a == 0:
        return math.nan

    return (
        100.0
        * (mean_b - mean_a)
        / mean_a
    )


def analyze_pair(
    lookup,
    strategy_a,
    strategy_b,
    loss,
    metric,
    seeds,
):
    (
        values_a,
        values_b,
        paired_seeds,
    ) = get_paired_values(
        lookup,
        strategy_a,
        strategy_b,
        loss,
        metric,
        seeds,
    )

    n = len(values_a)

    if n == 0:

        return {
            "base_loss": loss,
            "comparison": "{} vs {}".format(
                strategy_a,
                strategy_b,
            ),
            "strategy_a": strategy_a,
            "strategy_b": strategy_b,
            "metric": metric,
            "n": 0,
            "mean_a": math.nan,
            "mean_b": math.nan,
            "mean_difference_b_minus_a": math.nan,
            "ci95_low": math.nan,
            "ci95_high": math.nan,
            "wilcoxon_p": math.nan,
            "standardized_paired_effect": math.nan,
            "percent_change_b_vs_a": math.nan,
        }

    differences = (
        values_b - values_a
    )

    mean_a = float(
        np.mean(values_a)
    )

    mean_b = float(
        np.mean(values_b)
    )

    mean_difference = float(
        np.mean(differences)
    )

    ci_low, ci_high = confidence_interval(
        differences
    )

    p_value = wilcoxon_test(
        differences
    )

    effect = paired_effect_size(
        differences
    )

    pct = percent_change(
        mean_a,
        mean_b,
    )

    return {
        "base_loss": loss,
        "comparison": "{} vs {}".format(
            strategy_a,
            strategy_b,
        ),
        "strategy_a": strategy_a,
        "strategy_b": strategy_b,
        "metric": metric,
        "n": n,
        "mean_a": mean_a,
        "mean_b": mean_b,
        "mean_difference_b_minus_a": mean_difference,
        "ci95_low": ci_low,
        "ci95_high": ci_high,
        "wilcoxon_p": p_value,
        "standardized_paired_effect": effect,
        "percent_change_b_vs_a": pct,
    }

def apply_holm_correction(results):
    """
    Apply Holm-Bonferroni correction to all Wilcoxon p-values.

    The correction is applied across all statistical comparisons
    in the experiment.
    """

    valid_results = [
        result
        for result in results
        if math.isfinite(result["wilcoxon_p"])
    ]

    m = len(valid_results)

    if m == 0:
        return

    # Sort hypotheses by raw p-value.
    ordered = sorted(
        valid_results,
        key=lambda result: result["wilcoxon_p"]
    )

    adjusted = [0.0] * m

    # Holm step-down adjustment.
    running_max = 0.0

    for i, result in enumerate(ordered):

        raw_p = result["wilcoxon_p"]

        adjusted_p = (m - i) * raw_p

        adjusted_p = min(
            adjusted_p,
            1.0
        )

        # Enforce monotonicity.
        adjusted_p = max(
            adjusted_p,
            running_max
        )

        running_max = adjusted_p

        adjusted[i] = adjusted_p

    for result, adjusted_p in zip(
        ordered,
        adjusted
    ):
        result["holm_adjusted_p"] = adjusted_p

def write_csv(
    path,
    results,
):
    directory = os.path.dirname(path)

    if directory:
        os.makedirs(
            directory,
            exist_ok=True,
        )

    fieldnames = [
        "base_loss",
        "comparison",
        "strategy_a",
        "strategy_b",
        "metric",
        "n",
        "mean_a",
        "mean_b",
        "mean_difference_b_minus_a",
        "ci95_low",
        "ci95_high",
        "wilcoxon_p",
        "holm_adjusted_p",
        "standardized_paired_effect",
        "percent_change_b_vs_a",
    ]

    with open(
        path,
        "w",
        newline="",
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames,
        )

        writer.writeheader()

        for result in results:
            writer.writerow(
                result
            )


def print_summary(results):
    print("\n=== STATISTICAL ANALYSIS ===")

    important_metrics = [
        "time_to_50_pct_s",
        "time_to_75_pct_s",
        "time_to_90_pct_s",
        "time_to_95_pct_s",
        "time_to_100_pct_s",
        "total_uav_travel_m",
        "total_overlap_cell_visits",
        "overlap_ratio",
    ]

    for result in results:

        if result["metric"] not in important_metrics:
            continue

        if result["n"] == 0:
            continue

        print(
            "\nLoss {:.2f} | {} | {}".format(
                result["base_loss"],
                result["comparison"],
                result["metric"],
            )
        )

        print(
            "  n = {}".format(
                result["n"]
            )
        )

        print(
            "  A mean = {:.6f}".format(
                result["mean_a"]
            )
        )

        print(
            "  B mean = {:.6f}".format(
                result["mean_b"]
            )
        )

        print(
            "  B-A = {:.6f}".format(
                result[
                    "mean_difference_b_minus_a"
                ]
            )
        )

        print(
            "  95% CI = [{:.6f}, {:.6f}]".format(
                result["ci95_low"],
                result["ci95_high"],
            )
        )

        print(
            "  Wilcoxon p = {:.6g}".format(
                result["wilcoxon_p"]
            )
        )

        print(
            "  Standardized paired effect = {:.6f}".format(
                result[
                    "standardized_paired_effect"
                ]
            )
        )

        print(
            "  Percent change = {:.3f}%".format(
                result[
                    "percent_change_b_vs_a"
                ]
            )
        )


def main():

    args = parse_args()

    print(
        "Loading raw experiment data:"
    )

    print(
        "  {}".format(
            args.input
        )
    )

    rows = load_data(
        args.input
    )

    validate_data(
        rows,
        args.seed_start,
        args.seed_end,
    )

    lookup = make_lookup(
        rows
    )

    seeds = list(
        range(
            args.seed_start,
            args.seed_end + 1,
        )
    )

    results = []

    for loss in EXPECTED_LOSSES:

        for strategy_a, strategy_b in COMPARISONS:

            for metric in METRICS:

                result = analyze_pair(
                    lookup,
                    strategy_a,
                    strategy_b,
                    loss,
                    metric,
                    seeds,
                )

                results.append(
                    result
                )

    apply_holm_correction(results)
    
    write_csv(
        args.output,
        results,
    )

    print(
        "\nGenerated {} statistical comparisons.".format(
            len(results)
        )
    )

    print(
        "Output file:"
    )

    print(
        "  {}".format(
            args.output
        )
    )

    print_summary(
        results
    )


if __name__ == "__main__":
    main()