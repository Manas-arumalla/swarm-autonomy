"""Shared Matplotlib style for all Swarm Autonomy figures.

Importing this module and calling :func:`apply` gives every figure a single,
consistent, publication-quality look — one colour palette, one DPI, modern
spines/grid — so the result figures read as a coherent set rather than a pile of
one-off scripts.

    from swarm_autonomy_style import apply, C
    apply()
    ...
    ax.plot(t, y, color=C["method"], label="Swarm Autonomy")
"""

from __future__ import annotations

import matplotlib as mpl

# Semantic colour palette. Use these names, not raw hex, so series stay consistent
# across figures (the Swarm Autonomy method is always teal, baselines always slate, etc.).
C = {
    "method":   "#0f766e",  # deep teal  — the Swarm Autonomy method / primary result
    "baseline": "#94a3b8",  # slate grey — solo / straight-line / reactive baseline
    "accent":   "#b45309",  # amber      — secondary configuration
    "alt":      "#7c3aed",  # violet     — third series / query
    "bad":      "#dc2626",  # red        — failure / collision / fleeing target
    "ref":      "#64748b",  # muted      — reference lines (mean, cap, threshold)
    "good":     "#16a34a",  # green      — success / captured
    "ink":      "#0f172a",  # near-black — text / annotations
}
SEQ_CMAP = "viridis"        # sequential fields (ESDF, time-coloured tracks)

_RC = {
    "figure.dpi":         150,
    "savefig.dpi":        200,
    "figure.facecolor":   "white",
    "savefig.facecolor":  "white",
    "savefig.bbox":       "tight",
    "font.family":        "DejaVu Sans",
    "font.size":          11,
    "axes.titlesize":     12.5,
    "axes.titleweight":   "semibold",
    "axes.titlepad":      9,
    "axes.labelsize":     11,
    "axes.labelweight":   "medium",
    "axes.edgecolor":     "#475569",
    "axes.linewidth":     0.9,
    "xtick.labelsize":    9.5,
    "ytick.labelsize":    9.5,
    "xtick.color":        "#334155",
    "ytick.color":        "#334155",
    "legend.fontsize":    9.5,
    "legend.frameon":     True,
    "legend.framealpha":  0.92,
    "legend.edgecolor":   "#cbd5e1",
    "axes.grid":          True,
    "grid.alpha":         0.25,
    "grid.linewidth":     0.6,
    "grid.color":         "#94a3b8",
    "axes.spines.top":    False,
    "axes.spines.right":  False,
    "axes.prop_cycle":    mpl.cycler(color=[C["method"], C["accent"], C["alt"], C["bad"]]),
    "lines.linewidth":    2.0,
    "lines.solid_capstyle": "round",
}


def apply() -> None:
    """Apply the shared Swarm Autonomy Matplotlib style globally."""
    mpl.rcParams.update(_RC)


def footer(fig, text: str = "Swarm Autonomy — decentralized multi-drone autonomy") -> None:
    """Add a small, consistent provenance caption to the bottom-left of a figure."""
    fig.text(0.008, 0.006, text, fontsize=7.5, color="#94a3b8", ha="left", va="bottom")
