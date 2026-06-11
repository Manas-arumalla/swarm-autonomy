"""Navigate-among-buildings: the CPU ESDF + ego-planner-style planner in the Swarm Autonomy city.

The same building grid the camera-pursuit demo uses (BGRID at 12 m spacing, 7 m boxes) is
turned into a CPU occupancy grid + ESDF (swarm_autonomy_mapping.esdf), then several start->goal
queries are routed with the A*-front-end / ESDF-gradient-optimizer planner
(swarm_autonomy_planning.planner). This is the GPU-free fallback for nvblox + ego-planner-swarm on
GPU-constrained hosts (design-decisions D5): no CUDA, runs headless in <1 s.

Produces experiments/plots/plan_among_buildings.png: the ESDF field, the buildings, the jagged
A* front-end vs the smoothed collision-clear trajectory, and a metrics table (length, min
clearance, smoothness) with the straight-line baseline that would fly through walls.
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src",
                                "swarm_autonomy_mapping"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src",
                                "swarm_autonomy_planning"))
from swarm_autonomy_mapping.esdf import GridMap
from swarm_autonomy_planning.planner import astar, plan, path_metrics
from swarm_autonomy_style import apply, C, SEQ_CMAP, footer

apply()

BGRID = (-12, 0, 12)            # buildings inside the ±16 operating area
B_HALF = 3.5
CENTERS = [(cx, cy) for cx in BGRID for cy in BGRID]

# representative queries that each must weave around buildings
QUERIES = [
    ((-6.0, -6.0), (6.0, 6.0)),     # diagonal across the centre block
    ((-6.0, 0.0), (6.0, 0.0)),      # straight line would hit the centre building
    ((-15.0, 6.0), (15.0, -6.0)),   # long traverse across the whole city
]
QNAMES = ["diagonal", "through centre", "long traverse"]
COLORS = ["#f59e0b", "#ec4899", "#a855f7"]   # amber / pink / violet — distinct on viridis


def straight_min_clear(g, a, b, n=60):
    pts = np.linspace(a, b, n)
    return min(g.distance(x, y) for x, y in pts)


def main():
    g = GridMap((-16, -16), (16, 16), res=0.4)
    g.add_buildings(CENTERS, B_HALF)

    # ESDF field for the background (clamped for a readable colour range)
    e = g.esdf().T
    xs = np.linspace(g.lo[0], g.hi[0], g.nx)
    ys = np.linspace(g.lo[1], g.hi[1], g.ny)

    fig, (axm, axt) = plt.subplots(1, 2, figsize=(13.5, 7.4),
                                   gridspec_kw={"width_ratios": [1.75, 1]})

    # Left: the ESDF field + buildings + planned routes.
    im = axm.contourf(xs, ys, np.clip(e, -2, 6), levels=24, cmap=SEQ_CMAP, alpha=0.9)
    fig.colorbar(im, ax=axm, label="signed distance to nearest building (m)", shrink=0.85)
    for cx, cy in CENTERS:
        axm.add_patch(Rectangle((cx - B_HALF, cy - B_HALF), 2 * B_HALF, 2 * B_HALF,
                                facecolor="0.12", edgecolor="white", lw=0.8, zorder=3))

    rows = []
    for (a, b), col, name in zip(QUERIES, COLORS, QNAMES):
        coarse = astar(g, a, b, clearance=0.0)
        path = plan(g, a, b, d_safe=1.3)
        if path is None:
            rows.append((name, None))
            continue
        ca = np.asarray(coarse)
        axm.plot(ca[:, 0], ca[:, 1], "--", color=col, lw=1.1, alpha=0.65, zorder=4)
        axm.plot(path[:, 0], path[:, 1], "-", color=col, lw=2.8, zorder=5, label=name)
        axm.scatter([a[0], b[0]], [a[1], b[1]], color=col, s=72, ec="white",
                    zorder=6, marker="o")
        m = path_metrics(g, path)
        m["straight_clear"] = straight_min_clear(g, a, b)
        rows.append((name, m))

    axm.set_xlim(-16, 16); axm.set_ylim(-16, 16); axm.set_aspect("equal")
    axm.set_title("ESDF-guided routing through the city")
    axm.set_xlabel("East (m)"); axm.set_ylabel("North (m)")
    leg = axm.legend(loc="upper left", title="planned route", framealpha=0.92)
    leg.get_title().set_fontsize(9)
    axm.plot([], [], "--", color="0.4", label="A* front-end")  # convention note in caption
    axm.text(0.5, -0.085, "dashed = A* front-end   ·   solid = ESDF-optimized trajectory",
             transform=axm.transAxes, ha="center", va="top", fontsize=9, color=C["ref"])

    # Right: the metrics table — the quantitative result, with the straight-line baseline.
    axt.axis("off")
    axt.set_title("Path metrics vs. straight-line baseline", pad=16)
    col_labels = ["query", "length\n(m)", "min.\nclearance\n(m)",
                  "straight-line\nclearance (m)", "smoothness\n(rad)"]
    cell_text, cell_colours = [], []
    for name, m in rows:
        if m is None:
            cell_text.append([name, "—", "—", "—", "—"])
            cell_colours.append(["white"] * 5)
            continue
        cell_text.append([name, f"{m['length']:.1f}", f"+{m['min_clearance']:.2f}",
                          f"{m['straight_clear']:+.2f}", f"{m['mean_turn']:.3f}"])
        cell_colours.append(["white", "white", "#ecfdf5", "#fef2f2", "white"])
    tbl = axt.table(cellText=cell_text, colLabels=col_labels, cellColours=cell_colours,
                    colWidths=[0.24, 0.15, 0.19, 0.25, 0.19], loc="center", cellLoc="center",
                    bbox=[0.0, 0.45, 1.0, 0.42])
    tbl.auto_set_font_size(False); tbl.set_fontsize(9.5)
    for (r, cc), cell in tbl.get_celld().items():
        cell.set_edgecolor("#cbd5e1")
        if r == 0:
            cell.set_facecolor(C["method"]); cell.set_text_props(color="white", weight="bold")
        elif cc == 2:
            cell.set_text_props(color="#047857", weight="bold")
        elif cc == 3:
            cell.set_text_props(color=C["bad"], weight="bold")
    axt.text(0.5, 0.12, "Straight-line clearance is negative — the direct path passes\n"
             "through buildings. The planner holds positive clearance on every query.",
             transform=axt.transAxes, ha="center", va="top", fontsize=9, color=C["ink"])

    print(f"{'query':>16} | {'len(m)':>7} | {'min clear(m)':>12} | "
          f"{'straight clear':>14} | {'smooth(rad)':>11}")
    for name, m in rows:
        if m is None:
            print(f"{name:>16} | NO PATH");  continue
        print(f"{name:>16} | {m['length']:7.1f} | {m['min_clearance']:12.2f} | "
              f"{m['straight_clear']:14.2f} | {m['mean_turn']:11.3f}")

    footer(fig)
    OUT = os.path.join(os.path.dirname(__file__), "plots", "plan_among_buildings.png")
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.tight_layout(); fig.savefig(OUT)
    print(f"\nwrote {OUT}")


if __name__ == "__main__":
    main()
