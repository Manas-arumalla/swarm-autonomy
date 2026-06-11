# Experiments

Deterministic, headless benchmark scripts. Each turns a capability into a quantitative figure with a
baseline and, where relevant, an ablation. None require a GPU, ROS, or a simulator in the loop;
outputs are written to [`plots/`](plots/). Regenerate everything at once with:

```bash
./make_figures.sh
```

| Script | Produces | Measures |
|---|---|---|
| `control_compare.py` | `control_compare.png` | MPC vs. reactive pursuit guidance: command jitter, velocity reversals, tracking error. |
| `coop_exploration.py` | `coop_exploration.png` | Cooperative-exploration coverage vs. time for 1/2/3 drones, plus a throttled-comms ablation. |
| `plan_among_buildings.py` | `plan_among_buildings.png` | CPU ESDF + planner routes through the city: path length, clearance, smoothness vs. a straight-line baseline. |
| `vio_stereo_handover.py` | `vio_stereo_handover.png` | Stereo VIO external-vision error during the GPS-denied handover (reads a captured bridge log). |
| `fit_camera_extrinsic.py` | (console) | Re-fits the pursuit camera→body rotation from labelled pixel↔world calibration samples. |

The swarm-scale sweeps (coverage scaling 1→3→5→8 drones, interception success vs. pursuer count and
evader speed, delivered bandwidth vs. cap) are produced by
[`../sim/benchmarks.py`](../sim/benchmarks.py). `swarm_autonomy_style.py` defines the shared figure style
all scripts apply.

Full methodology, metric definitions, and results are in [`../docs/benchmarks.md`](../docs/benchmarks.md).
