"""Cooperative multi-drone exploration of the Swarm Autonomy city (RACER role, CPU fallback).

A decentralized, comms-realistic model with three key properties:

1. OCCLUSION SENSING: each drone ray-casts a 360° lidar-like scan (swarm_autonomy_mapping.frontier
   states), so it only learns cells it has LINE OF SIGHT to — buildings cast shadows, and the
   shadow boundaries are exactly the frontiers worth flying to.
2. COMMS-GATED MAP SHARING: there is NO shared oracle map. Each drone holds its OWN belief and
   periodically broadcasts the cells it has newly seen; every link is gated by the real
   swarm_autonomy_comms.LinkModel (range + i.i.d. loss + a token-bucket bandwidth cap), so what a
   neighbour learns — and the bandwidth it costs — is metered.
3. DIVISION OF LABOUR: a drone routes (CPU ESDF planner) to the nearest frontier in its own
   belief that it does not believe a teammate has claimed — claims piggyback on the same gated
   link, so coordination degrades gracefully when the radio is poor.

Produces plots/coop_exploration.png: team coverage-vs-time for 1/2/3 drones (speedup
vs solo) AND a bandwidth-throttled 3-drone run (coordination falls off when comms is starved),
with the delivered-bytes budget reported. Pure/headless, a few seconds, deterministic.
"""
import math
import os
import random
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
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src",
                                "swarm_autonomy_comms"))
from swarm_autonomy_mapping.esdf import GridMap
from swarm_autonomy_mapping.frontier import (cluster_frontiers, coverage_fraction,
                                       UNKNOWN, FREE, OCC)
from swarm_autonomy_planning.planner import plan
from swarm_autonomy_comms.link_model import LinkModel, LinkConfig
from swarm_autonomy_style import apply, C, footer

apply()

BGRID = (-12, 0, 12)
B_HALF = 3.5
CENTERS = [(cx, cy) for cx in BGRID for cy in BGRID]
EXT = 16.0
RES = 0.5
SENSOR_R = 5.0
N_RAYS = 90
SPEED = 3.0
DT = 0.3
MAX_STEPS = 320
COV_TARGET = 0.95
COMMS_EVERY = 3                 # broadcast a map delta every N steps
CLAIM_KEEP_OUT = 4.0           # avoid frontiers within this of a teammate's known claim


def build_truth():
    g = GridMap((-EXT, -EXT), (EXT, EXT), res=RES)
    g.add_buildings(CENTERS, B_HALF)
    occ = g.occ
    xs = g.lo[0] + (np.arange(g.nx) + 0.5) * RES
    ys = g.lo[1] + (np.arange(g.ny) + 0.5) * RES
    return g, occ, ~occ, xs, ys


def reveal_occluded(belief, sensed, truth_occ, g, pos, radius, n_rays):
    """Ray-march a 360° scan from `pos`; reveal cells with line of sight, stop each ray at the
    first occupied cell (so buildings shadow what's behind them). Updates belief + sensed."""
    for a in np.linspace(0.0, 2 * math.pi, n_rays, endpoint=False):
        dx, dy = math.cos(a), math.sin(a)
        r = 0.0
        while r <= radius:
            x = pos[0] + dx * r
            y = pos[1] + dy * r
            if not g.in_bounds_world(x, y):
                break
            i = int((x - g.lo[0]) / RES); j = int((y - g.lo[1]) / RES)
            if 0 <= i < g.nx and 0 <= j < g.ny:
                if truth_occ[i, j]:
                    belief[i, j] = OCC; sensed[i, j] = True
                    break                       # ray blocked beyond this wall
                belief[i, j] = FREE; sensed[i, j] = True
            r += RES * 0.5


def plan_grid_from_belief(state):
    g = GridMap((-EXT, -EXT), (EXT, EXT), res=RES)
    g.occ = (state == OCC); g._esdf = None
    return g


def cell_to_world(i, j):
    return (-EXT + (i + 0.5) * RES, -EXT + (j + 0.5) * RES)


def run(n_drones, rng, link_cfg, share=True):
    g, truth_occ, truth_free, xs, ys = build_truth()
    beliefs = [np.full((g.nx, g.ny), UNKNOWN, dtype=int) for _ in range(n_drones)]
    sensed = [np.zeros((g.nx, g.ny), dtype=bool) for _ in range(n_drones)]
    pending = [set() for _ in range(n_drones)]            # newly-seen cells awaiting broadcast
    goals = [None] * n_drones                             # each drone's own current goal
    known_claims = [dict() for _ in range(n_drones)]      # drone -> last-heard teammate goals
    paths = [None] * n_drones
    pis = [0] * n_drones

    # drones LAUNCH TOGETHER from one corner of the clear y=-6 street; without map/claim
    # sharing they would chase the same frontiers, so coordination (comms) is what divides
    # the city — making the comms quality actually matter.
    pos = []
    for k in range(n_drones):
        x = -10.0 + 1.2 * k
        pos.append(np.array([x, -6.0], float))
    pos = np.array(pos)

    # directed per-pair links, each its own metered channel
    links = {(i, j): LinkModel(link_cfg, random.Random(1000 + 17 * i + j))
             for i in range(n_drones) for j in range(n_drones) if i != j}

    def scan(d):
        before = sensed[d].copy()
        reveal_occluded(beliefs[d], sensed[d], truth_occ, g, pos[d], SENSOR_R, N_RAYS)
        new = np.argwhere(sensed[d] & ~before)
        for i, j in new:
            pending[d].add((int(i), int(j)))

    for d in range(n_drones):
        scan(d)

    team_sensed = np.zeros((g.nx, g.ny), dtype=bool)
    cov, bytes_delivered = [], 0
    trails = [[tuple(pos[d])] for d in range(n_drones)]

    for step in range(MAX_STEPS):
        team_sensed = np.zeros((g.nx, g.ny), dtype=bool)
        for d in range(n_drones):
            team_sensed |= sensed[d]
        c = float((team_sensed & truth_free).sum()) / max(1, int(truth_free.sum()))
        cov.append(c)
        if c >= COV_TARGET:
            break

        # --- comms: broadcast each drone's map delta + current goal through the gated link ---
        if share and n_drones > 1 and step % COMMS_EVERY == 0:
            t = step * DT
            for i in range(n_drones):
                if not pending[i]:
                    continue
                payload = list(pending[i])
                nbytes = max(8, len(payload) * 3 + 8)     # cells (3B each) + a goal (8B)
                delivered_to_any = False
                for j in range(n_drones):
                    if j == i:
                        continue
                    rng_m = float(np.hypot(*(pos[i] - pos[j])))
                    res = links[(i, j)].try_deliver(t, rng_m, nbytes)
                    if res.delivered:
                        for (ci, cj) in payload:
                            if beliefs[j][ci, cj] == UNKNOWN:
                                beliefs[j][ci, cj] = beliefs[i][ci, cj]
                        if goals[i] is not None:
                            known_claims[j][i] = goals[i]
                        bytes_delivered += res.bytes
                        delivered_to_any = True
                if delivered_to_any:
                    pending[i].clear()

        # --- per-drone decentralized frontier choice + routing ---
        for d in range(n_drones):
            clusters = cluster_frontiers(beliefs[d], min_size=3)
            if not clusters:
                goals[d] = None
                continue
            fronts = [cell_to_world(ci, cj) for ci, cj, _ in clusters]
            others = [known_claims[d][o] for o in known_claims[d] if known_claims[d][o]]
            best, bgoal = None, None
            for fw in fronts:
                if any(np.hypot(fw[0] - o[0], fw[1] - o[1]) < CLAIM_KEEP_OUT for o in others):
                    continue                                  # believed claimed by a teammate
                dist = float(np.hypot(fw[0] - pos[d][0], fw[1] - pos[d][1]))
                if best is None or dist < best:
                    best, bgoal = dist, fw
            if bgoal is None:                                 # all claimed → take plain nearest
                bgoal = min(fronts, key=lambda fw: np.hypot(fw[0] - pos[d][0], fw[1] - pos[d][1]))
            exhausted = paths[d] is None or pis[d] >= len(paths[d])
            moved = goals[d] is None or np.hypot(bgoal[0] - goals[d][0], bgoal[1] - goals[d][1]) > 1.5
            if exhausted or moved:
                p = plan(plan_grid_from_belief(beliefs[d]), tuple(pos[d]), bgoal,
                         d_safe=1.0, spacing=1.0)
                paths[d] = p if p is not None else np.array([pos[d], bgoal])
                pis[d] = 1
            goals[d] = bgoal

        # --- advance + rescan ---
        for d in range(n_drones):
            if goals[d] is None or paths[d] is None:
                continue
            budget = SPEED * DT
            P = np.asarray(paths[d])
            while budget > 1e-6 and pis[d] < len(P):
                v = P[pis[d]] - pos[d]
                dist = float(np.hypot(*v))
                if dist <= budget:
                    pos[d] = np.array(P[pis[d]], float); pis[d] += 1; budget -= dist
                else:
                    pos[d] = pos[d] + v / dist * budget; budget = 0.0
            scan(d)
            trails[d].append(tuple(pos[d]))

    return {"cov": cov, "trails": trails, "beliefs": beliefs, "team": team_sensed,
            "bytes": bytes_delivered, "g": g}


def main():
    good = LinkConfig(bandwidth_bps=50_000, max_range_m=80, base_loss=0.02)
    poor = LinkConfig(bandwidth_bps=1_200, max_range_m=22, base_loss=0.25, edge_loss=0.7)

    res = {}
    for n in (1, 2, 3):
        res[n] = run(n, random.Random(0), good, share=True)
    res["3poor"] = run(3, random.Random(0), poor, share=True)

    def t90(cov):
        return next((i * DT for i, c in enumerate(cov) if c >= 0.9), None)
    for k in (1, 2, 3, "3poor"):
        c = res[k]["cov"]
        tag = f"{k} drones" if k != "3poor" else "3 drones (throttled comms)"
        print(f"{tag:>26}: final {c[-1]*100:5.1f}%  t90={t90(c)}s  "
              f"bytes={res[k]['bytes']/1000:.1f} kB")

    fig, ax = plt.subplots(1, 2, figsize=(13, 5.6))

    # Left: coverage vs. time. Solo baseline in slate, swarm in teal shades, the throttled-comms
    # ablation dotted.
    series = [(1, C["baseline"], "-", "1 drone (solo)"),
              (2, C["accent"], "-", "2 drones"),
              (3, C["method"], "-", "3 drones"),
              ("3poor", C["method"], ":", "3 drones · throttled comms")]
    for k, col, ls, lbl in series:
        cov = res[k]["cov"]
        ax[0].plot([i * DT for i in range(len(cov))], [c * 100 for c in cov],
                   color=col, ls=ls, label=lbl)
    ax[0].axhline(90, ls="--", color=C["ref"], lw=0.9, alpha=0.7)
    ax[0].text(1, 91, "90% coverage", fontsize=8, color=C["ref"])
    ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("team coverage (%)")
    ax[0].set_title("Cooperative exploration: coverage vs. time")
    ax[0].legend(loc="lower right")

    # Inset: time to 90% coverage per configuration (the headline speedup).
    inset = ax[0].inset_axes([0.10, 0.13, 0.40, 0.34])
    keys = [1, 2, 3, "3poor"]; labels = ["1", "2", "3", "3·thr"]
    cols = [C["baseline"], C["accent"], C["method"], C["method"]]
    t90s = [t90(res[k]["cov"]) for k in keys]
    bars = inset.bar(range(len(keys)), t90s, color=cols, width=0.7)
    bars[-1].set_hatch("///"); bars[-1].set_edgecolor("white")
    for b, v in zip(bars, t90s):
        inset.text(b.get_x() + b.get_width() / 2, v + 0.5, f"{v:.0f}", ha="center",
                   va="bottom", fontsize=7.5)
    inset.set_xticks(range(len(keys))); inset.set_xticklabels(labels, fontsize=7)
    inset.set_ylabel("t→90% (s)", fontsize=7.5); inset.tick_params(labelsize=7)
    inset.set_title("time to 90%", fontsize=8); inset.grid(axis="x", alpha=0)

    # Right: the 3-drone team map + per-drone trails.
    r = res[3]; team = r["team"]
    ax[1].imshow(team.T, origin="lower", cmap="Greens", alpha=0.45,
                 extent=[-EXT, EXT, -EXT, EXT])
    for cx, cy in CENTERS:
        ax[1].add_patch(Rectangle((cx - B_HALF, cy - B_HALF), 2 * B_HALF, 2 * B_HALF,
                                  facecolor="0.18", edgecolor="white", lw=0.8, zorder=3))
    for trail, col in zip(r["trails"], (C["method"], C["accent"], C["alt"])):
        T = np.asarray(trail)
        ax[1].plot(T[:, 0], T[:, 1], color=col, lw=1.9, zorder=4)
        ax[1].scatter(T[0, 0], T[0, 1], color=col, s=62, marker="s", ec="white", zorder=5)
        ax[1].scatter(T[-1, 0], T[-1, 1], color=col, s=90, marker="*", ec="white", zorder=5)
    ax[1].set_xlim(-EXT, EXT); ax[1].set_ylim(-EXT, EXT); ax[1].set_aspect("equal")
    ax[1].set_title("3-drone explored map + trajectories")
    ax[1].set_xlabel("East (m)"); ax[1].set_ylabel("North (m)")
    ax[1].text(0.5, -0.10,
               f"{r['cov'][-1]*100:.0f}% covered · {r['bytes']/1000:.0f} kB shared · "
               "green = sensed free · ■ start · ★ end",
               transform=ax[1].transAxes, ha="center", va="top", fontsize=8.5, color=C["ref"])

    footer(fig)
    OUT = os.path.join(os.path.dirname(__file__), "plots", "coop_exploration.png")
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.tight_layout(); fig.savefig(OUT)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
