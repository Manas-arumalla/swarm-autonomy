#!/usr/bin/env python3
"""Run one Swarm Autonomy swarm scenario and save a GIF + print metrics.

    python3 sim/run_sim.py --drones 4 --time 90 --out experiments/plots/pursuit.gif

Drives the real CBBA / comms / pursuit / PID modules. No ROS, no Gazebo needed.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from swarm_sim.simulator import Simulator, SimConfig          # noqa: E402
from swarm_sim.world import default_city                       # noqa: E402


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description="Swarm Autonomy swarm simulation")
    ap.add_argument("--drones", type=int, default=4)
    ap.add_argument("--time", type=float, default=90.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default="experiments/plots/pursuit.gif")
    ap.add_argument("--no-gif", action="store_true", help="metrics only, skip rendering")
    ap.add_argument("--stride", type=int, default=2)
    args = ap.parse_args(argv)

    cfg = SimConfig(num_drones=args.drones, max_time_s=args.time, seed=args.seed,
                    record=not args.no_gif)
    sim = Simulator(default_city(args.seed), cfg)
    print(f"running: {args.drones} drones, {args.time:.0f}s sim, seed={args.seed} ...")
    res = sim.run()

    print("\n=== result ===")
    print(f"  target intercepted : {res.captured}"
          + (f"  at t={res.capture_time:.1f}s" if res.captured else ""))
    print(f"  closest approach   : {res.min_distance:.2f} m "
          f"(capture radius {cfg.capture_radius} m)")
    print(f"  final coverage     : {res.final_coverage*100:.1f}% of free space")
    print(f"  sim steps recorded : {len(res.frames)}")

    if not args.no_gif:
        from swarm_sim.render import render_gif
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        render_gif(sim.world, res.frames, args.out, stride=args.stride)
        print(f"  wrote animation    : {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
