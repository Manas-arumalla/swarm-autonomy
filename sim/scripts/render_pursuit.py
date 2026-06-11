"""Render the Gazebo vision-pursuit recording (/tmp/pursuit_rec.csv) into a top-down
demo GIF: N camera drones (blue, with trails) cooperatively cornering a fleeing target
(red) they detect by camera (green X = swarm's shared visual estimate).
Output: experiments/plots/gazebo_pursuit.gif
"""
import csv
import math
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = "/tmp/pursuit_rec.csv"
OUT = os.path.join(HERE, "..", "..", "experiments", "plots", "gazebo_pursuit.gif")

rows = list(csv.DictReader(open(CSV)))
N = sum(1 for k in rows[0] if k.startswith("d") and k.endswith("E"))
# downsample to keep the GIF light (~20 fps target from ~5Hz log -> take all; cap frames)
step = max(1, len(rows) // 300)
rows = rows[::step]

def f(x):
    return float(x) if x not in ("", None) else None

cols = ["#1f77ff", "#00b8d4", "#7c4dff", "#00c853", "#ff9100", "#d500f9"]
fig, ax = plt.subplots(figsize=(7, 7))
fig.patch.set_facecolor("#0e1117")
ax.set_facecolor("#0e1117")

# faint city blocks (swarm_autonomy_city: 7x7 m buildings on a 12 m grid, B_HALF=3.5)
for bx in range(-24, 25, 12):
    for by in range(-24, 25, 12):
        ax.add_patch(plt.Rectangle((bx - 3.5, by - 3.5), 7, 7, color="#2a2f3a", zorder=0))

trails = [[] for _ in range(N)]
drone_dots = [ax.plot([], [], "o", color=cols[i % len(cols)], ms=9, zorder=5)[0] for i in range(N)]
trail_lines = [ax.plot([], [], "-", color=cols[i % len(cols)], lw=1.2, alpha=0.5, zorder=3)[0] for i in range(N)]
tgt_dot, = ax.plot([], [], "o", color="#ff1744", ms=13, zorder=6, label="fleeing target")
est_dot, = ax.plot([], [], "x", color="#69f0ae", ms=11, mew=2.5, zorder=6, label="swarm visual estimate")
title = ax.set_title("", color="white", fontsize=12)
status = ax.text(0.02, 0.97, "", transform=ax.transAxes, color="white", va="top", fontsize=10)

R = 32
ax.set_xlim(-R, R); ax.set_ylim(-R, R); ax.set_aspect("equal")
ax.tick_params(colors="#888"); [s.set_color("#444") for s in ax.spines.values()]
ax.legend(loc="upper right", facecolor="#1a1f29", labelcolor="white", fontsize=9, framealpha=0.9)

def upd(k):
    r = rows[k]
    for i in range(N):
        x, y = f(r[f"d{i}E"]), f(r[f"d{i}N"])
        if x is None:
            continue
        trails[i].append((x, y)); trails[i][:] = trails[i][-40:]
        drone_dots[i].set_data([x], [y])
        tx, ty = zip(*trails[i]); trail_lines[i].set_data(tx, ty)
    tgt_dot.set_data([f(r["tgtE"])], [f(r["tgtN"])])
    if f(r["estE"]) is not None:
        est_dot.set_data([f(r["estE"])], [f(r["estN"])])
    dmin = min(math.hypot(f(r[f"d{i}E"]) - f(r["tgtE"]), f(r[f"d{i}N"]) - f(r["tgtN"]))
               for i in range(N) if f(r[f"d{i}E"]) is not None)
    cap = int(r["captured"])
    title.set_text(f"Swarm Autonomy — camera-guided swarm pursuit ({N} drones, no central node)")
    status.set_text(f"t={float(r['t']):4.1f}s   closest={dmin:4.1f} m" +
                    ("   *** INTERCEPTED ***" if cap else ""))
    status.set_color("#69f0ae" if cap else "white")
    return drone_dots + trail_lines + [tgt_dot, est_dot, title, status]

os.makedirs(os.path.dirname(OUT), exist_ok=True)
ani = animation.FuncAnimation(fig, upd, frames=len(rows), interval=60, blit=False)
ani.save(OUT, writer=animation.PillowWriter(fps=18), dpi=90)
print(f"saved {OUT}  ({len(rows)} frames, {N} drones)")
