# Design Decisions

The rationale behind the major architecture and component choices I made.

## Integrate proven systems rather than reinvent

My contribution here is **integration**: OpenVINS (VIO), nvblox (mapping), ego-planner-swarm
(planning), RACER (cooperative exploration), and CBBA (allocation) are mature, published systems.
The parts I wrote from scratch are the **comms middleware** — the single bandwidth choke point —
and the **coordination/pursuit** layer. Everything else is vendored under
`ros2_ws/src/third_party` by `scripts/setup.sh`. This keeps my engineering focused on the
system-integration and coordination problems rather than on re-deriving SLAM or trajectory
optimization.

## Communications as a single inter-drone choke point

All inter-drone traffic — poses, map deltas, task bids, target observations — flows through
`swarm_autonomy_comms`. A single choke point makes "bandwidth-limited radio" a **measured** quantity
(delivered bytes/s against a configured cap) and keeps the gating policy in one dependency-free,
unit-tested module (`link_model.py`). It also enables a clean ablation: throttling the link and
measuring the coverage penalty, which would be impossible if drones exchanged data through ad-hoc
topics.

## Deterministic coordination core; learned communication as an extension

CBBA is decentralized and convergent with provably conflict-free assignment — a solid foundation for
the decentralized role-allocation claim. Learned multi-agent communication (MARL) is a **planned**
research extension, fenced off as such; it is **not yet implemented** — CBBA is the shipped allocator.

## RACER-style (not FUEL) for cooperative exploration

FUEL is a single-drone explorer; RACER is the decentralized *multi-UAV* cooperative explorer with
intermittent communication and map sharing — a direct match to the multi-drone goal, so I
modelled the cooperative explorer on it (frontier detection + claim sharing over the gated link).
NOTE: FUEL itself is not wired in; the coverage benchmark's "solo" baseline is the **single-drone
run of the same frontier explorer** (N = 1), which is the direct apples-to-apples comparison.

## GPS-denied flight via external-vision fusion

EKF2 external-vision fusion (`EKF2_GPS_CTRL=0`, EV fusion enabled) realizes GPS-denied flight from
OpenVINS odometry. Restoring metric scale required a stereo camera and a trajectory-fit frame
alignment; the full investigation is in [engineering-notes.md](engineering-notes.md).

## CPU mapping/planning as the portable default

The nvblox GPU ESDF and a planned RL extension would target a 12 GB+ CUDA GPU. To keep the mapping, planning, and
exploration results reproducible on any machine, I implemented the ESDF and the planner as a
pure-NumPy/SciPy **CPU implementation**, fully unit-tested. The nvblox GPU backend is an
interface-compatible drop-in behind `map_merge_node` — an upgrade for scale and resolution rather
than a different architecture. On GPU-constrained hosts, the CPU path runs the full pipeline; on a
12 GB+ GPU, nvblox replaces the ESDF backend without touching the planner, coordination, or comms
layers.
