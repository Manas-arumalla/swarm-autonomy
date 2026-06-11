#!/usr/bin/env python3
"""Generate the Swarm Autonomy benchmark plots.

Runs headless sweeps over the swarm simulator and writes three figures:

  1. coverage_vs_time.png   — map coverage % over time, solo vs swarm (3/5/8)
  2. interception_rate.png  — capture success rate vs drone count & target speed
  3. bandwidth_vs_cap.png   — delivered inter-drone bytes/s against the link cap

    python3 sim/benchmarks.py            # all three, into experiments/plots/

These exercise the same CBBA / comms / pursuit code as the live demo; they are
the quantitative backing for the "swarm beats solo" and "decentralized
interception" claims.
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "experiments"))

from swarm_sim.simulator import Simulator, SimConfig          # noqa: E402
from swarm_sim.world import default_city                       # noqa: E402
from swarm_autonomy_style import apply, C, footer                    # noqa: E402

OUT = "experiments/plots"
PURSUER_MAX_SPEED = 4.0   # Drone.max_speed — the kinematic limit the heatmap axis crosses


def _sim(num_drones, seed, evader_speed=2.6, max_time=90.0, explore_only=False):
    # Decoupled randomness: the city layout and the channel/spawn stream use different seeds so
    # "variance over seeds" is not secretly the same draw driving both.
    cfg = SimConfig(num_drones=num_drones, seed=seed, evader_speed=evader_speed,
                    max_time_s=max_time, record=False, explore_only=explore_only)
    return Simulator(default_city(1000 + seed), cfg).run()


def coverage_vs_time(ax):
    # Pure exploration (no pursuit phase, no early capture break), so every run spans the full
    # 60 s — no forward-filled tails from early-terminated runs.
    grid = np.arange(0, 60, 0.5)
    palette = {1: C["baseline"], 3: C["method"], 5: C["accent"], 8: C["alt"]}
    for n in (1, 3, 5, 8):
        acc = []
        for s in range(6):
            res = _sim(n, seed=s, max_time=60.0, explore_only=True)
            ts = np.array([t for t, _ in res.coverage_curve])
            cv = np.array([c for _, c in res.coverage_curve])
            acc.append(np.interp(grid, ts, cv))
        acc = np.array(acc) * 100
        mean, sd = acc.mean(axis=0), acc.std(axis=0)
        col = palette[n]
        ax.fill_between(grid, mean - sd, mean + sd, color=col, alpha=0.15)
        ax.plot(grid, mean, color=col, label=f"{n} drone" + ("s" if n > 1 else ""))
    ax.set_xlabel("time (s)")
    ax.set_ylabel("free-space coverage (%)")
    ax.set_title("Cooperative exploration scales with drone count")
    ax.legend(title="swarm size (mean ± 1σ over 6 maps)")


def interception_rate(ax):
    # Full pursuer-count x evader-speed grid. No aggregation across speed: the evader-speed axis
    # deliberately crosses the pursuers' max speed (4 m/s) so the kinematic cliff is VISIBLE
    # instead of being averaged into a healthy-looking bar.
    counts = [2, 3, 4, 6]
    speeds = [2.0, 3.0, 4.0, 5.0, 7.0]
    seeds = range(8)
    rate = np.zeros((len(speeds), len(counts)))
    for i, sp in enumerate(speeds):
        for j, n in enumerate(counts):
            caught = sum(_sim(n, seed=s, evader_speed=sp, max_time=80.0).captured for s in seeds)
            rate[i, j] = 100.0 * caught / len(list(seeds))
    im = ax.imshow(rate, origin="lower", aspect="auto", cmap="viridis", vmin=0, vmax=100)
    for i in range(len(speeds)):
        for j in range(len(counts)):
            v = rate[i, j]
            ax.text(j, i, f"{v:.0f}%", ha="center", va="center", fontsize=9.5,
                    color="white" if v < 55 else "black")
    ax.set_xticks(range(len(counts)), counts)
    ax.set_yticks(range(len(speeds)), [f"{s:g}" for s in speeds])
    ax.set_xlabel("number of pursuers")
    ax.set_ylabel("evader speed (m/s)")
    ax.set_title(f"Interception success — pursuers capped at {PURSUER_MAX_SPEED:g} m/s")
    ax.axhline(speeds.index(4.0) + 0.5, color="white", lw=1.2, ls="--", alpha=0.85)
    ax.text(len(counts) - 0.55, speeds.index(4.0) + 0.62, "evader outruns pursuers ↑",
            ha="right", va="bottom", fontsize=8.5, color="white")
    ax.grid(False)
    cb = ax.figure.colorbar(im, ax=ax, shrink=0.9)
    cb.set_label(f"capture rate (% of {len(list(seeds))} maps)")


def bandwidth_vs_cap(ax):
    # Busiest single directed link per 1 s window vs the PER-LINK cap (an apples-to-apples
    # comparison), with the all-links total for context. The token bucket allows a one-off
    # burst of bucket_capacity bytes, which the first windows can show.
    runs = [_sim(5, seed=s, max_time=60.0).bandwidth_curve for s in (1, 2, 3)]
    cap = runs[0][0][3] if runs[0] else 0
    t_end = min(run[-1][0] for run in runs)        # clip to the shortest run (no fabricated tail)
    grid = np.arange(1.0, t_end, 1.0)
    busiest = np.array([np.interp(grid, [r[0] for r in run], [r[2] for r in run])
                        for run in runs])
    total = np.array([np.interp(grid, [r[0] for r in run], [r[1] for r in run])
                      for run in runs])
    ax.plot(grid, total.mean(axis=0), color=C["ref"], lw=1.2, alpha=0.8,
            label="all links, total")
    ax.fill_between(grid, busiest.min(axis=0), busiest.max(axis=0), color=C["method"], alpha=0.15)
    ax.plot(grid, busiest.mean(axis=0), color=C["method"], label="busiest single link")
    ax.axhline(cap, color=C["bad"], ls="--", label="per-link cap")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("delivered bandwidth (bytes/s)")
    ax.set_title("Per-link delivered bandwidth vs. the link cap")
    ax.legend(title="mean over 3 runs (band = min–max)")


def main() -> int:
    import matplotlib
    matplotlib.use("Agg")
    apply()
    import matplotlib.pyplot as plt

    os.makedirs(OUT, exist_ok=True)
    for name, fn in [("coverage_vs_time", coverage_vs_time),
                     ("interception_rate", interception_rate),
                     ("bandwidth_vs_cap", bandwidth_vs_cap)]:
        print(f"computing {name} ...")
        fig, ax = plt.subplots(figsize=(7.2, 4.6))
        fn(ax)
        footer(fig)
        path = os.path.join(OUT, name + ".png")
        fig.savefig(path)
        plt.close(fig)
        print(f"  wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
