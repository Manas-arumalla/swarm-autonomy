# Architecture

Swarm Autonomy runs an identical autonomy stack on every drone and coordinates the swarm peer-to-peer.
This document describes the per-drone pipeline, the two cross-cutting design principles, the
message interfaces, and how the pieces are tested.

## Per-drone pipeline

Each vehicle lives in its own ROS 2 namespace (`/drone_N`) and runs the full sense → map → plan →
control loop onboard:

```
[Gazebo: camera + IMU]
      │ image, imu
      ▼
 [OpenVINS VIO] ── odom ──▶ [VIO→EKF2 bridge] ──▶ PX4 EKF2 (GPS off) ──┐ state
      │                                                                │
      ▼                                                                ▼
 [ESDF / occupancy mapping] ◀───────────────────────────────── [drone state]
      │ local map
      ▼
 [map merge] ◀── neighbour map deltas ──┐
      │ shared map                       │   All inter-drone traffic — poses, map deltas,
      ▼                                   │   task bids, target observations — passes through
 [ego-planner / RACER] ◀── goal/role ────┤   the comms middleware, which gates on range,
      │ trajectory                        │   rate, and dropout and logs delivered bandwidth.
      ▼                                   │
 [controller] ──▶ PX4 offboard           │
                                          │
 [coordination: CBBA] ◀── bids ───────────┤
 [pursuit behaviour] ◀── target obs ──────┘
      ▲
 [target detection from camera]
```

| Stage | Component | Implementation |
|---|---|---|
| State estimation | OpenVINS VIO → PX4 EKF2 external vision | `swarm_autonomy_perception` (bridge); EKF2 with `EKF2_GPS_CTRL=0` |
| Mapping | Euclidean Signed Distance Field + occupancy | `swarm_autonomy_mapping` (CPU ESDF; nvblox GPU backend drop-in) |
| Planning | A\* front-end + ESDF-gradient elastic-band; RACER for exploration | `swarm_autonomy_planning` |
| Coordination | CBBA role allocation, pursuit/interception geometry | `swarm_autonomy_coordination` |
| Control | PX4 offboard PID / MPC velocity guidance | `swarm_autonomy_control`, `swarm_autonomy_coordination` |
| Communication | Range/rate/dropout link model + bandwidth logging | `swarm_autonomy_comms` |

## Principle 1 — a single inter-drone comms choke point

Every message that crosses between drones is routed through one module,
[`swarm_autonomy_comms`](../ros2_ws/src/swarm_autonomy_comms/). Producers publish to `comms/out/<topic>`; the
middleware applies the gating policy and re-publishes the survivors on `comms/in/<topic>`; nothing
talks drone-to-drone directly.

The gating policy lives in a dependency-free module, `link_model.py`, and models three effects:

- **Range** — packets beyond `max_range_m` are dropped; a soft falloff above `soft_range_m` raises
  the loss probability toward the edge of coverage.
- **Rate** — a per-link token bucket caps delivered throughput at `bandwidth_bps`; packets that do
  not fit the budget are dropped rather than queued indefinitely.
- **Dropout** — an i.i.d. base loss plus the range-dependent term models a lossy channel.

Because there is exactly one choke point, "bandwidth-limited radio" becomes a **measured** quantity:
delivered bytes/s can be plotted against the cap, and the cooperative-exploration benchmark can
show coverage *degrading* as the link is throttled (see [benchmarks.md](benchmarks.md)).

> **Status.** The headless simulator routes every message type through this gating in-process, and
> the ROS 2 `comms_middleware` node brokers all four traffic classes (pose, target observation,
> map delta, task bid) from a single topic registry — adding a traffic class is one registry line.
> Inter-drone ranges come from the pose traffic itself.

## Principle 2 — pure algorithmic cores, thin ROS nodes

Each algorithm is implemented as a plain, importable Python module with no ROS or simulator
dependency, and unit-tested with `pytest`. A sibling `*_node.py` does only the ROS plumbing
(subscriptions, parameters, message conversion). New logic goes in the pure module and is tested
there, so the system's correctness does not depend on standing up a simulator.

| Pure core | Tested behaviour |
|---|---|
| `swarm_autonomy_comms/link_model.py` | Range/rate/dropout gating, token-bucket throughput, determinism under a seeded RNG. |
| `swarm_autonomy_coordination/cbba.py` | Conflict-free decentralized task assignment (CBBA). |
| `swarm_autonomy_coordination/pursuit.py` | Interception point, containment ring, target prediction. |
| `swarm_autonomy_coordination/mpc_pursuit.py` | Condensed-QP MPC guidance (convergence, velocity limits, smoothness). |
| `swarm_autonomy_coordination/target_tracker.py` | Constant-velocity Kalman track with outlier gating and dropout coasting. |
| `swarm_autonomy_coordination/swarm_control.py` | Reciprocal collision-avoidance heuristic (ORCA-*inspired* steering; not the full VO half-plane LP — see notes). |
| `swarm_autonomy_mapping/esdf.py` | Signed distance field, bilinear sampling, gradients. |
| `swarm_autonomy_mapping/frontier.py` | Frontier detection and clustering, coverage metric. |
| `swarm_autonomy_mapping/grid_io.py`, `merge.py` | Occupancy↔delta serialization, log-odds map fusion. |
| `swarm_autonomy_planning/planner.py` | A\* routing + ESDF elastic-band optimization, clearance guarantees. |
| `swarm_autonomy_control/pid.py` | PID with integral clamping and output saturation. |

> **Integration status.** These cores are real and unit-tested, and they drive the headless
> simulator and experiments directly. The ROS 2 node wiring is *partial*: `cbba`, `pursuit`,
> `link_model`, `esdf`, `planner`, `grid_io`/`merge`, and `pid` have node wrappers, while
> `mpc_pursuit`, `target_tracker`, `swarm_control`, and `frontier` are currently exercised by the
> sim and the standalone `sim/scripts/*` rather than a ROS node. See the README *implementation
> status* note for the full picture.

This pattern also makes the cores reusable outside ROS — for example, the headless swarm simulator
in [`sim/swarm_sim/`](../sim/) drives the *same* `link_model`, `cbba`, `pursuit`, and `pid` modules
to produce the demo GIF and benchmark figures without the PX4/Gazebo stack.

## Message interfaces

Shared interfaces are defined once in [`swarm_autonomy_msgs`](../ros2_ws/src/swarm_autonomy_msgs/) and depended
on by every other package:

| Message | Purpose |
|---|---|
| `NeighborPose` | A drone's pose shared with the swarm. |
| `MapDelta` | A compact occupancy update: origin, voxel indices, quantized log-odds, byte count. |
| `TaskBid` | A CBBA bid (agent, task, score) for decentralized allocation. |
| `TargetObservation` | A camera-derived target estimate shared over the comms link. |
| `CommsStats` | Per-link delivered/dropped counts and bandwidth, for the benchmark plots. |
| `DroneRole` | The assigned role (scout / blocker / interceptor). |

`px4_msgs` is an `exec_depend` only; the nodes guard the import (`try/except ImportError` → log and
idle), so `colcon build` and CI succeed without the PX4 overlay installed.

## Packages

| Package | Role |
|---|---|
| `swarm_autonomy_msgs` | Shared message interfaces (ament_cmake). |
| `swarm_autonomy_comms` | The comms choke point: link model + bandwidth-logging middleware. |
| `swarm_autonomy_coordination` | CBBA allocation, pursuit geometry, MPC guidance, target tracker, swarm control. |
| `swarm_autonomy_control` | PX4 offboard PID position controller. |
| `swarm_autonomy_perception` | OpenVINS bringup, VIO→EKF2 bridge, target detection. |
| `swarm_autonomy_mapping` | CPU ESDF + occupancy grid, frontier detection, map-delta serialization, shared-map merge. |
| `swarm_autonomy_planning` | CPU A\* + ESDF elastic-band planner; ego-planner-swarm / RACER integration. |
| `swarm_autonomy_bringup` | Launch files, per-drone namespacing, parameter layering. |

## Determinism and reproducibility

Simulation and algorithm code never call global `random`/`time`; randomness comes from an injected,
seeded `random.Random`, so every benchmark and CI run is reproducible bit-for-bit. Parameters are
centralized in `swarm_autonomy_bringup/config/drone_params.yaml`, keyed by node name, and the launch files
layer per-drone overrides (`drone_id`, `num_drones`, `profile`) on top.
