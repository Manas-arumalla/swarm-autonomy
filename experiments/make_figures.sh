#!/usr/bin/env bash
# Regenerate every result figure in experiments/plots/.
# Pure Python (numpy/scipy/matplotlib) — no GPU, no ROS, no Gazebo. Takes about a minute.
#
#   ./experiments/make_figures.sh
#
# The cooperative-pursuit GIF (experiments/plots/pursuit.gif) is produced by sim/run_sim.py
# (headless swarm simulator) and is not regenerated here.
set -euo pipefail
cd "$(dirname "$0")/.."

PYTHONPATH="ros2_ws/src/swarm_autonomy_mapping:ros2_ws/src/swarm_autonomy_planning:ros2_ws/src/swarm_autonomy_comms:ros2_ws/src/swarm_autonomy_coordination:ros2_ws/src/swarm_autonomy_control${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONPATH

# Self-contained deterministic experiments (each writes a PNG to experiments/plots/).
for fig in control_compare plan_among_buildings coop_exploration vio_stereo_handover; do
  echo "== experiments/${fig}.py =="
  python3 "experiments/${fig}.py"
done

# Swarm-scale sweeps over the headless simulator (coverage scaling, interception, bandwidth).
echo "== sim/benchmarks.py =="
python3 sim/benchmarks.py

echo "done — see experiments/plots/"
