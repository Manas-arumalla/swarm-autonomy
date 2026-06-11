"""Quantify the MPC vs reactive controller SMOOTHNESS under a noisy goal estimate.

The oscillation in the pursuit came from a jittering camera estimate of the target.
Here we reproduce that in isolation: a target moves on a path while the controller
only sees a NOISY estimate of it (Gaussian jitter, like the camera). We close the
loop with each controller and measure command jitter (the oscillation), velocity
reversals, and tracking error. Produces experiments/plots/control_compare.png.
"""
import math
import os
import random
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "ros2_ws", "src", "swarm_autonomy_coordination"))
from swarm_autonomy_coordination.mpc_pursuit import mpc_velocity
from swarm_autonomy_coordination.swarm_control import control_velocity
from swarm_autonomy_style import apply, C, footer

apply()

DT, VMAX, STEPS = 0.1, 4.5, 220
rng = random.Random(7)


def target(k):
    """A moving target: cruise, turn, dawdle — exercises tracking + hover."""
    t = k * DT
    if t < 8:
        return (1.8 * t, 0.0), (1.8, 0.0)
    if t < 14:
        return (14.4, 1.8 * (t - 8)), (0.0, 1.8)
    return (14.4, 10.8), (0.0, 0.0)            # stops -> hover (where reactive oscillates)


def run(kind):
    p, v = [0.0, 0.0], [0.0, 0.0]
    cmd = [0.0, 0.0]
    traj, cmds = [], []
    for k in range(STEPS):
        (tx, ty), (tvx, tvy) = target(k)
        gx = tx + rng.gauss(0, 0.6)            # NOISY estimate (camera jitter)
        gy = ty + rng.gauss(0, 0.6)
        gv = (tvx, tvy)
        if kind == "mpc":
            vx, vy = mpc_velocity((p[0], p[1]), (v[0], v[1]), (gx, gy), gv, vmax=VMAX, amax=3.0)
        else:  # reactive lead-pursuit + adaptive smoothing (as in the pursuit)
            vE, vN = control_velocity((p[0], p[1]), (v[0], v[1]), (gx, gy), [], VMAX)
            dg = math.hypot(gx - p[0], gy - p[1])
            a = 0.45 if dg < 4.0 else 0.95
            cmd[0] += a * (vE - cmd[0]); cmd[1] += a * (vN - cmd[1])
            vx, vy = cmd[0], cmd[1]
        cmds.append((vx, vy))
        v = [vx, vy]
        p = [p[0] + v[0] * DT, p[1] + v[1] * DT]
        traj.append((p[0], p[1]))
    return traj, cmds


def metrics(cmds, traj):
    jit = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(cmds, cmds[1:])]
    rev = sum(1 for a, b in zip(cmds, cmds[1:]) if a[0] * b[0] + a[1] * b[1] < 0)
    terr = [math.hypot(traj[k][0] - target(k)[0][0], traj[k][1] - target(k)[0][1])
            for k in range(len(traj))]
    return (sum(jit) / len(jit), rev, sum(terr[40:]) / len(terr[40:]))


mt, mc = run("mpc")
rt, rc = run("reactive")
mj, mr, me = metrics(mc, mt)
rj, rr, re = metrics(rc, rt)

print(f"               cmd-jitter   vel-reversals   track-err")
print(f"  reactive:      {rj:.3f}          {rr:5d}        {re:.2f} m")
print(f"  MPC:           {mj:.3f}          {mr:5d}        {me:.2f} m")
print(f"  -> MPC is {rj/mj:.1f}x smoother, {rr/max(mr,1):.1f}x fewer reversals")

tgt = [target(k)[0] for k in range(STEPS)]
fig, ax = plt.subplots(1, 2, figsize=(12.5, 5.2))

# Left: pursuer paths tracking the (noisily estimated) moving target.
ax[0].plot([t[0] for t in tgt], [t[1] for t in tgt], "--", color=C["ref"], lw=2,
           label="true target path")
ax[0].plot([t[0] for t in rt], [t[1] for t in rt], "-", color=C["baseline"], alpha=0.9,
           label="reactive")
ax[0].plot([t[0] for t in mt], [t[1] for t in mt], "-", color=C["method"],
           label="MPC (Swarm Autonomy)")
ax[0].set_title("Pursuer path while tracking a noisy target estimate")
ax[0].set_xlabel("x (m)"); ax[0].set_ylabel("y (m)")
ax[0].legend(loc="lower right"); ax[0].set_aspect("equal")

# Right: the three guidance metrics, each normalized to the reactive baseline (=1.0),
# so "how much better is the MPC" is one comparable axis. Lower is better on all three.
metrics = [("command\njitter", rj, mj), ("velocity\nreversals", rr, mr),
           ("tracking\nerror", re, me)]
xpos = np.arange(len(metrics)); w = 0.36
rbars = ax[1].bar(xpos - w / 2, [1.0] * len(metrics), w, color=C["baseline"], label="reactive")
mbars = ax[1].bar(xpos + w / 2, [m / b if b else 0 for _, b, m in metrics], w,
                  color=C["method"], label="MPC (Swarm Autonomy)")
for (lbl, b, m), rb, mb in zip(metrics, rbars, mbars):
    ax[1].text(rb.get_x() + rb.get_width() / 2, 1.02, f"{b:.2f}".rstrip("0").rstrip("."),
               ha="center", va="bottom", fontsize=8, color=C["ref"])
    ax[1].text(mb.get_x() + mb.get_width() / 2, m / b + 0.02,
               f"{m:.2f}".rstrip("0").rstrip("."), ha="center", va="bottom",
               fontsize=8.5, color=C["method"], fontweight="bold")
ax[1].set_xticks(xpos); ax[1].set_xticklabels([m[0] for m in metrics])
ax[1].set_ylabel("relative to reactive baseline")
ax[1].set_ylim(0, 1.25)
ax[1].axhline(1.0, color=C["ref"], lw=0.8, ls="--", alpha=0.6)
ax[1].set_title(f"MPC vs. reactive guidance  ·  {rj / mj:.1f}× smoother commands")
ax[1].legend(loc="upper right")
ax[1].grid(axis="x", alpha=0)

footer(fig)
OUT = os.path.join(os.path.dirname(__file__), "plots", "control_compare.png")
os.makedirs(os.path.dirname(OUT), exist_ok=True)
fig.tight_layout(); fig.savefig(OUT)
print(f"  wrote {OUT}")
