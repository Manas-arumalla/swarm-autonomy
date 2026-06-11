"""Render simulator frames to an animated GIF (headless, matplotlib + imageio).

No ffmpeg dependency: frames are drawn to an Agg canvas and stitched into a GIF.
"""

from __future__ import annotations

import numpy as np

from .world import World
from .simulator import Frame, ROLE_SCOUT, ROLE_BLOCKER, ROLE_INTERCEPTOR

_ROLE_COLOR = {
    0: "#888888",                 # idle
    ROLE_SCOUT: "#1f77b4",        # blue
    ROLE_BLOCKER: "#2ca02c",      # green
    ROLE_INTERCEPTOR: "#d62728",  # red
}
_ROLE_NAME = {0: "idle", ROLE_SCOUT: "scout", ROLE_BLOCKER: "blocker", ROLE_INTERCEPTOR: "interceptor"}


def render_gif(world: World, frames: list[Frame], path: str,
               stride: int = 2, fps: int = 15, dpi: int = 90) -> str:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    import imageio.v2 as imageio

    sel = frames[::stride]
    images = []
    for fr in sel:
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.set_xlim(0, world.width)
        ax.set_ylim(0, world.height)
        ax.set_aspect("equal")
        ax.set_facecolor("#0e1117")

        # Discovered (known-free) cells as a faint overlay.
        known = fr.known
        ys, xs = np.where((known.T == 1))  # transpose: array is [i=x, j=y]
        ax.scatter((xs + 0.5) * world.cell, (ys + 0.5) * world.cell,
                   s=6, c="#26324a", marker="s", linewidths=0)

        for b in world.buildings:
            ax.add_patch(Rectangle((b.x0, b.y0), b.x1 - b.x0, b.y1 - b.y0,
                                   color="#3a3f4b"))

        # Comms links delivered this frame.
        for (s, d) in fr.links:
            xs2 = [fr.drones[s][0], fr.drones[d][0]]
            ys2 = [fr.drones[s][1], fr.drones[d][1]]
            ax.plot(xs2, ys2, color="#445", lw=0.6, alpha=0.5, zorder=1)

        for (x, y, role) in fr.drones:
            ax.scatter([x], [y], s=70, c=_ROLE_COLOR.get(role, "#888"),
                       edgecolors="white", linewidths=0.6, zorder=3)
            circ = plt.Circle((x, y), 7.0, color=_ROLE_COLOR.get(role, "#888"),
                              fill=False, lw=0.4, alpha=0.25)
            ax.add_patch(circ)

        ex, ey = fr.evader
        ax.scatter([ex], [ey], s=130, marker="*",
                   c="#ffcc00", edgecolors="black", linewidths=0.6, zorder=4)

        title = f"t={fr.t:4.1f}s  phase={fr.phase}  coverage={fr.coverage*100:4.1f}%"
        if fr.captured:
            title += "  — TARGET INTERCEPTED"
        ax.set_title(title, color="white", fontsize=10)
        ax.tick_params(colors="#888")
        for spine in ax.spines.values():
            spine.set_color("#333")

        fig.canvas.draw()
        buf = np.asarray(fig.canvas.buffer_rgba())
        images.append(buf[..., :3].copy())
        plt.close(fig)

    imageio.mimsave(path, images, fps=fps, loop=0)
    return path
