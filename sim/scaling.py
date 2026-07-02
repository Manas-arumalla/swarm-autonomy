#!/usr/bin/env python3
"""Swarm-size scaling sweep: how coverage, comms load, and interception scale with N.

Runs the headless simulator at swarm sizes 1..20 (seeded, several maps per point) and writes
``experiments/plots/scaling_vs_n.png``:

  1. time to 90% coverage vs N          (pure exploration; does adding drones keep paying off?)
  2. delivered comms traffic vs N       (total offered load grows ~O(N^2) with the team;
                                         the busiest single link stays under the per-link cap)
  3. interception vs N                  (capture rate and mean time-to-capture)

Also runs one multi-minute end-to-end mission (explore -> detect -> pursue -> capture) and prints
its event timeline. Everything is deterministic; map seeds and channel seeds are decoupled.

    python3 sim/scaling.py
"""

from __future__ import annotations

import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "experiments"))

import numpy as _np                                            # noqa: E402
from swarm_sim.simulator import Simulator, SimConfig          # noqa: E402
from swarm_sim.world import Rect, World, default_city          # noqa: E402
from swarm_autonomy_style import apply, C, footer              # noqa: E402


def big_city(seed: int = 0, size: float = 100.0) -> World:
    """A larger city on the same block pattern as ``default_city`` — used by the end-to-end
    mission so exploration and pursuit genuinely take minutes, and as a check that nothing in
    the stack assumes the 40 m map."""
    rng = _np.random.default_rng(seed)
    buildings = []
    for bx in range(4, int(size) - 4, 8):
        for by in range(4, int(size) - 4, 8):
            jitter = rng.uniform(-0.5, 0.5, size=2)
            w = rng.uniform(3.0, 4.5)
            h = rng.uniform(3.0, 4.5)
            buildings.append(Rect(bx + jitter[0], by + jitter[1],
                                  bx + w + jitter[0], by + h + jitter[1]))
    return World(width=size, height=size, cell=1.0, buildings=buildings)

OUT = "experiments/plots"
NS = [1, 2, 3, 5, 8, 12, 16, 20]
SEEDS = range(6)


def _sim(n, seed, explore_only=False, max_time=90.0, evader_speed=2.6, record=False):
    cfg = SimConfig(num_drones=n, seed=seed, evader_speed=evader_speed,
                    max_time_s=max_time, record=record, explore_only=explore_only)
    return Simulator(default_city(1000 + seed), cfg).run()


def _t90(res):
    """First sim time at which team coverage reaches 90% of free space (None if never)."""
    for t, cov in res.coverage_curve:
        if cov >= 0.90:
            return t
    return None


def sweep():
    rows = []
    for n in NS:
        t90s, totals, busiest, caps = [], [], [], []
        caught, cap_times = 0, []
        for s in SEEDS:
            r = _sim(n, s, explore_only=True, max_time=90.0)
            t = _t90(r)
            if t is not None:
                t90s.append(t)
            bw = r.bandwidth_curve
            if bw:
                totals.append(float(np.mean([b[1] for b in bw])))
                busiest.append(float(np.mean([b[2] for b in bw])))
                caps.append(bw[0][3])
            p = _sim(n, s, explore_only=False, max_time=120.0)
            if p.captured:
                caught += 1
                cap_times.append(p.capture_time)
        rows.append({
            "n": n,
            "t90_mean": float(np.mean(t90s)) if t90s else float("nan"),
            "t90_sd": float(np.std(t90s)) if t90s else float("nan"),
            "bw_total": float(np.mean(totals)) if totals else 0.0,
            "bw_busiest": float(np.mean(busiest)) if busiest else 0.0,
            "bw_cap": float(np.mean(caps)) if caps else 0.0,
            "cap_rate": 100.0 * caught / len(list(SEEDS)),
            "cap_t_mean": float(np.mean(cap_times)) if cap_times else float("nan"),
            "cap_t_sd": float(np.std(cap_times)) if cap_times else float("nan"),
        })
        r0 = rows[-1]
        print(f"N={n:2d}: t90 {r0['t90_mean']:5.1f}±{r0['t90_sd']:4.1f} s | "
              f"comms total {r0['bw_total']:7.0f} B/s busiest-link {r0['bw_busiest']:5.0f} B/s | "
              f"capture {r0['cap_rate']:3.0f}% at {r0['cap_t_mean']:5.1f} s", flush=True)
    return rows


def plot(rows):
    import matplotlib
    matplotlib.use("Agg")
    apply()
    import matplotlib.pyplot as plt

    ns = [r["n"] for r in rows]
    fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.4))

    ax = axes[0]
    m = np.array([r["t90_mean"] for r in rows])
    sd = np.array([r["t90_sd"] for r in rows])
    ax.fill_between(ns, m - sd, m + sd, color=C["method"], alpha=0.15)
    ax.plot(ns, m, "o-", color=C["method"])
    ax.set_xlabel("swarm size N")
    ax.set_ylabel("time to 90% coverage (s)")
    ax.set_title("Exploration keeps scaling…\nwith diminishing returns")

    ax = axes[1]
    ax.plot(ns, [r["bw_total"] for r in rows], "o-", color=C["accent"],
            label="all links, total")
    ax.plot(ns, [r["bw_busiest"] for r in rows], "o-", color=C["method"],
            label="busiest single link")
    ax.axhline(rows[0]["bw_cap"], color=C["bad"], ls="--", label="per-link cap")
    ax.set_yscale("log")
    ax.set_xlabel("swarm size N")
    ax.set_ylabel("delivered bandwidth (bytes/s, log)")
    ax.set_title("Total gossip grows with the team;\nper-link load stays capped")
    ax.legend(fontsize=8.5)

    ax = axes[2]
    ax.bar(ns, [r["cap_rate"] for r in rows], width=[max(0.6, n * 0.12) for n in ns],
           color=C["method"], alpha=0.85)
    ax.set_ylim(0, 105)
    ax.set_xlabel("swarm size N")
    ax.set_ylabel("capture rate (%)", color=C["method"])
    ax2 = ax.twinx()
    ax2.plot(ns, [r["cap_t_mean"] for r in rows], "o-", color=C["alt"])
    ax2.set_ylabel("mean time to capture (s)", color=C["alt"])
    ax2.grid(False)
    ax.set_title("Interception vs swarm size")

    footer(fig)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "scaling_vs_n.png")
    fig.savefig(path, bbox_inches="tight")
    print(f"wrote {path}", flush=True)


def mission():
    """One multi-minute end-to-end mission, timeline printed from the recorded frames.

    Runs on a 100 x 100 m city (6x the benchmark map's area) with three drones, so the full
    explore -> map -> detect -> pursue -> capture arc genuinely takes minutes — and doubles as a
    check that nothing in the stack assumes the 40 m benchmark map."""
    print("\n=== multi-minute end-to-end mission (N=3, 100 m city) ===", flush=True)
    t0 = time.time()
    cfg = SimConfig(num_drones=3, seed=3, evader_speed=2.6, max_time_s=600.0,
                    record=True, explore_only=False)
    r = Simulator(big_city(1003), cfg).run()
    first_pursue = next((f.t for f in r.frames if f.phase == "pursue"), None)
    n_pursue = sum(1 for f in r.frames if f.phase == "pursue")
    n_total = len(r.frames) or 1
    cov50 = next((t for t, c in r.coverage_curve if c >= 0.5), None)
    cov90 = _t90(r)
    print(f"  t={0.0:6.1f} s  mission start (3 drones, 100 m city, explore)")
    if cov50 is not None:
        print(f"  t={cov50:6.1f} s  50% of the city mapped")
    if cov90 is not None:
        print(f"  t={cov90:6.1f} s  90% of the city mapped")
    if first_pursue is not None:
        print(f"  t={first_pursue:6.1f} s  first target detection -> swarm switches to pursuit")
    if r.captured:
        print(f"  t={r.capture_time:6.1f} s  TARGET CAPTURED (closest {r.min_distance:.1f} m)")
    else:
        print(f"  target not captured within 600 s (closest approach {r.min_distance:.1f} m)")
    print(f"  {100 * n_pursue / n_total:.0f}% of the mission in active pursuit  |  "
          f"final coverage {100 * r.final_coverage:.1f}%  |  wall time {time.time() - t0:.0f} s")


def main() -> int:
    rows = sweep()
    plot(rows)
    mission()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
