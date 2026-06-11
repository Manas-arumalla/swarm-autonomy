# sim/

Two simulation paths, by fidelity.

## 1. `swarm_sim/` — headless swarm simulator (no ROS or Gazebo required)

A dependency-light, physics-lite multi-drone simulator that drives the **real** Swarm Autonomy algorithm
modules — the comms link model (`swarm_autonomy_comms`), CBBA allocation and pursuit geometry
(`swarm_autonomy_coordination`), and the PID controller (`swarm_autonomy_control`) — through the full scenario:
cooperative exploration of an unknown city, then decentralized scout/blocker/interceptor role
allocation and interception of a fleeing target. Each drone learns the target only from its own line
of sight or a neighbour observation that survives the bandwidth-limited comms gating, so the
decentralization is real.

```bash
# One scenario -> animated GIF + metrics
python3 sim/run_sim.py --drones 4 --time 90 --out experiments/plots/pursuit.gif

# Benchmark sweeps -> coverage_vs_time, interception_rate, bandwidth_vs_cap
python3 sim/benchmarks.py

# Tests
python3 -m pytest sim/swarm_sim/test -q
```

Outputs land in `experiments/plots/`. This path produces the demo GIF and benchmark figures without
the heavy PX4 stack.

## 2. PX4 SITL + Gazebo Harmonic — full-fidelity flight

Real PX4 flight dynamics in Gazebo, driven over uXRCE-DDS by the ROS 2 nodes in `ros2_ws/`. Install
with `scripts/setup.sh` (requires apt/sudo and a multi-GB PX4 build), then:

```bash
sim/launch_sim.sh                                          # PX4 SITL + Gazebo + XRCE agent
ros2 launch swarm_autonomy_bringup multi_drone.launch.py num_drones:=3
```

See `sim/launch_sim.sh`, `sim/worlds/`, and the EKF2 GPS-denied parameters for the world and flight
configuration.
