"""GPS-denied handover on stereo visual-inertial odometry.

Renders the recorded stereo-VIO handover run (`data/vio_handover.csv`): the VIO position
estimate aligned into the PX4 NED frame, and the external-vision error against the PX4 state
estimate over the flight. Two panels:

  (left)  the VIO estimate traces a clean 5 m-radius circle — the recovered metric scale.
          Monocular VIO reconstructed only a ~1 m circle (5x under-scale), which drove the
          GPS-off divergence; stereo plus a trajectory-fit (Umeyama) frame alignment fix it.
  (right) the |VIO - PX4| position error stays bounded (mean 0.26 m, max 0.70 m), so the
          vehicle holds the trajectory on vision alone after the GPS cutoff.

The recorded run was captured live in PX4 SITL; the committed CSV makes the figure reproducible
without the simulator. Writes experiments/plots/vio_stereo_handover.png.
"""
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from swarm_autonomy_style import apply, C, SEQ_CMAP, footer

apply()

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "data", "vio_handover.csv")

t, est_e, est_n, err = [], [], [], []
with open(CSV) as f:
    for row in csv.DictReader(f):
        t.append(float(row["t"]))
        est_e.append(float(row["est_E"]))
        est_n.append(float(row["est_N"]))
        err.append(float(row["err"]) if row["err"] not in ("", "nan") else float("nan"))

t = np.array(t); est_e = np.array(est_e); est_n = np.array(est_n); err = np.array(err)
valid = ~np.isnan(err)
vt, ve = t[valid], err[valid]
mean_e, max_e = float(ve.mean()), float(ve.max())

fig, ax = plt.subplots(1, 2, figsize=(12.5, 5.2))

# Left: the VIO estimate (East, North), coloured by time, with an ideal 5 m reference circle.
cx, cy = float(est_e.mean()), float(est_n.mean())
theta = np.linspace(0, 2 * np.pi, 200)
ax[0].plot(cx + 5.0 * np.cos(theta), cy + 5.0 * np.sin(theta), "--", color=C["ref"], lw=1.4,
           label="ideal 5 m circle")
sc = ax[0].scatter(est_e, est_n, c=t, cmap=SEQ_CMAP, s=16, zorder=3)
ax[0].set_aspect("equal")
ax[0].set_title("Stereo VIO estimate recovers metric scale")
ax[0].set_xlabel("East (m)"); ax[0].set_ylabel("North (m)")
ax[0].legend(loc="upper right")
fig.colorbar(sc, ax=ax[0], label="time (s)")

# Right: external-vision error over the flight.
ax[1].fill_between(vt, ve, color=C["method"], alpha=0.12)
ax[1].plot(vt, ve, "-", color=C["method"], lw=1.8, label="stereo + Umeyama alignment")
ax[1].axhline(mean_e, ls="--", color=C["ref"], lw=1.0, label=f"mean {mean_e:.2f} m")
ax[1].set_ylim(0, max(max_e * 1.35, 1.0))
ax[1].set_title("External-vision error stays bounded under GPS-denied flight")
ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("|VIO − PX4| position error (m)")
ax[1].legend(loc="upper left")
ax[1].text(0.97, 0.05,
           f"mean {mean_e:.2f} m · max {max_e:.2f} m\nstable 5 m circle on VIO only, no runaway\n"
           "(monocular diverged at handover)",
           transform=ax[1].transAxes, ha="right", va="bottom", fontsize=8.5, color=C["ink"],
           bbox=dict(boxstyle="round,pad=0.4", fc="#ecfdf5", ec=C["method"], alpha=0.95))

footer(fig)
OUT = os.path.join(HERE, "plots", "vio_stereo_handover.png")
os.makedirs(os.path.dirname(OUT), exist_ok=True)
fig.tight_layout(); fig.savefig(OUT)
print(f"points={len(t)}  err mean {mean_e:.2f} m  max {max_e:.2f} m  "
      f"circle x-span={est_e.max()-est_e.min():.1f} m")
print(f"wrote {OUT}")
