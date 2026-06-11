# Engineering Notes

Three technical deep-dives into the problems I found most instructive while building this system:
restoring metric scale for the GPS-denied state-estimation handover, isolating the precision limit
of vision-only pursuit, and hardening the live multi-drone pursuit until it survived a city full of
occlusions. They share a single debugging method I applied throughout the project — instrument
against ground truth, form a hypothesis, and disprove it with data before changing the system.

---

## 1. Restoring metric scale for the GPS-denied VIO handover

**Symptom.** With OpenVINS providing odometry and PX4 EKF2 fusing it as external vision, disabling
GPS caused the position estimate to diverge: the controller chased a setpoint that drifted away,
and the vehicle ran off.

**Investigation.** I logged the VIO state against the true (GPS-on) trajectory and found the cause
was not a timestamping, frame, or yaw-fusion bug, as I first suspected — it was **scale**. The
drone flew a 5 m-radius circle, but the monocular VIO reconstructed a circle of roughly **1 m
radius**: a 5× under-scaling. In a monocular-inertial system, metric scale is observable only
through linear acceleration, and a gentle constant-speed survey circle has almost none
(centripetal acceleration ≈ v²/r ≈ 0.13 m/s²). With scale unobservable, the accelerometer bias
estimate absorbed the ambiguity and the filter drifted. Feeding that under-scaled estimate into
EKF2 with GPS off meant the controller over-drove to reach its setpoint, producing the runaway.

**Fix.** Two changes:

1. **Stereo VIO.** Adding a second camera (0.12 m baseline) makes scale observable from the stereo
   baseline directly, independent of acceleration. The reconstructed circle then matches the true
   5 m radius and the accelerometer bias stays bounded.
2. **Trajectory-fit frame alignment.** My first bridge aligned the VIO frame to PX4's NED frame
   from a single instantaneous yaw sample, which left a few-degrees rotation error and an
   oscillating position offset. I replaced it with a 2-D similarity fit (Umeyama) over a short arc
   of matched VIO↔PX4 positions — using trajectory *shape* rather than one attitude sample — and
   solving for rotation, translation, and any residual scale dropped the fused external-vision
   error to **0.26 m mean / 0.70 m max**.

A secondary finding: VIO initialization is sensitive to excitation. Initializing during the takeoff
transient diverged (the accelerometer bias saturated); initializing once the vehicle was in steady,
excited circular flight produced a clean, bounded filter. The bringup sequence reflects this.

**Result.** The drone holds a stable 5 m circle on vision alone after the GPS cutoff, with no
runaway. See [`vio_stereo_handover.png`](../experiments/plots/vio_stereo_handover.png).

---

## 2. The precision limit of vision-only pursuit

**Symptom.** In camera-guided pursuit, the swarm detects the fleeing target and closes on it, but
follows within a few metres rather than locking on exactly.

**Investigation.** The temptation was to tune the controller, but instrumenting the perception
against ground truth pointed elsewhere. I tested and rejected four hypotheses:

| Hypothesis | Test | Verdict |
|---|---|---|
| Camera mis-calibration | Re-fit the camera→body rotation and FOV from labelled pixel↔world pairs (`fit_camera_extrinsic.py`) | Rejected — a rotation re-fit barely moved the error |
| Off-nadir geometry | Bin estimate error by off-nadir angle | Rejected — error does not shrink as a drone passes overhead |
| Depth/scale ambiguity | Reason about the ground-plane back-projection | Rejected — altitude already provides scale, so stereo would not help |
| Drone localization bug | Log the drone's own `world_pos` belief vs. simulator ground truth over a wide single-drone trajectory | Rejected for that regime — the belief tracked truth with ≈ zero mean error (but see §3: longer multi-drone flights told a different story) |

**Conclusion.** The few-metre follow error is the **accumulated precision of monocular vision-only
pursuit** — small localization noise, ground back-projection sensitivity at chase distances, and
fast relative motion compounding — rather than a single defect. The Kalman target tracker
(`target_tracker.py`) with outlier gating and dropout coasting mitigates the random component and
keeps the swarm from chasing spurious jumps, but tightening the follow to sub-metre would require a
better sensing chain (a stabilized gimbal or a depth sensor), not a parameter change.

**Takeaway.** Disproving the obvious explanations with ground-truth data was more valuable than a
lucky parameter sweep: it located the limit in the sensor model and produced a tracker that makes
the existing sensing as robust as it can be.

---

## 3. Hardening live pursuit in an occluded city

Flying the camera-guided pursuit for long sessions in the full PX4 + Gazebo city exposed a chain of
failure modes that short runs never showed. I instrumented every run (phase timeline, distance to
the true target, per-frame detection error against ground truth) and fixed the causes one at a
time. The headline numbers, measured on identical 5–6 minute scenarios before and after:

| Metric | Before | After |
|---|---|---|
| Time actively tracking the target | 12 % | ~50 % |
| Time blind-searching | 64 % | 16–26 % |
| Median distance to target while tracking | 20.9 m | 12.1 m |
| Worst boundary excursion | 80 m (unbounded) | ≤ 28 m (geofenced) |
| Live capture (≤ 3 m) achieved | no | yes, repeatedly |

The individual findings, each verified against ground truth before and after the fix:

- **Stale references caused the overshoot.** The guidance ran at 50 Hz but the target goal
  refreshed at the detection-bound loop rate, and I had capped goal extrapolation at 50 ms — the
  interceptor aimed at where the target *was*, crossed it, and lost it behind a building before
  reacquiring. Dead-reckoning each goal along its velocity for its full age (bounded at 1 s)
  removed the crossing oscillation. I also added a terminal velocity-matching cost to the MPC so a
  plan *arrives the way its goal moves* — braking to a stop at a static containment slot,
  station-keeping on a mover — and gave only the interceptor the target-velocity feedforward
  (feeding it to the static-slot blockers had been dragging their reference points and making them
  orbit the ring).
- **The estimator's own position error is a detection error.** Per-frame detection error against
  ground truth was a suspicious *flat* ~5.7 m regardless of camera tilt or image position — the
  signature of a systematic bias, not noise. The drone's EKF local-position belief had drifted
  tens of metres under GPS noise during long aggressive flights, and every camera back-projection
  inherits the drone's self-position error 1:1. For this demo the coordination runs on simulator
  ground-truth self-pose (disclosed in the code — the *target* is still found by camera only);
  on hardware this becomes a state-estimation workstream, not a perception one.
- **The camera must outrank the filter.** The tracker's Mahalanobis gate protects an established
  track from single-frame outliers, but its failure mode is rejecting *genuine* re-sightings
  whenever the coasted estimate is wrong: the swarm would see the target and keep flying to a
  phantom. I added standard track management — the track restarts at the live sightings whenever
  the filter materially disagrees with them or has been blind for seconds — and, while
  reacquiring, even banked, image-corner glimpses are accepted (with a large noise scale) so one
  sliver of red is enough to turn the swarm.
- **Occlusion needs to be modelled, not suffered.** Three additions: a conservative 3-D
  line-of-sight test against the city grid; visibility-aware containment (a slot whose sightline
  to the target is cut by a building scores far lower in the allocation, so blockers hold vantages
  that keep eyes on); and the coasted estimate is projected out of building footprints — a ground
  target cannot be inside a wall, so the motion model should never put it there.
- **Search is a coverage problem.** A reactive ring around the last-known position circles stale
  space. The search is now structural: the last drone holds station high over the area centre
  (its footprint covers most of the map — a permanent reacquisition anchor) while the others mow
  gap-separated serpentine strips; altitude is phase-scheduled (search high for footprint, track
  low for precision) and every drone cruises at its own level, which makes mid-air collisions
  geometrically impossible regardless of the horizontal logic.
- **Independent safety layers.** A command-layer geofence (the flight-stack pattern) bounds the
  operational volume against *any* upstream fault, an ESDF repulsion from the shared mapping core
  deflects below-roof flight around buildings, a below-roof speed cap gives that repulsion real
  authority, and a slew-rate limit on the final command keeps the sum of guidance + avoidance
  terms continuous. None of these depend on the others being correct — that's the point.

**Takeaway.** Every one of these failures looked, from the outside, like "the drones are being
dumb." Ground-truth instrumentation turned each into a one-line cause — a stale reference, a
biased prior, an over-trusted gate, an unmodelled occlusion — and the fixes are the standard tools
of the trade applied at the right layer. The remaining follow distance is dominated by the
detection→decision→flight latency against a moving target, which is exactly where I would next
spend effort (predictive interception on the planner side, a gimballed sensor on the hardware
side).

---

## Component notes

**Comms as the measurement boundary.** Routing all inter-drone traffic through a single gated link
model turns "bandwidth-limited radio" into a measured quantity. The cooperative-exploration
benchmark uses this directly: throttling the link raises coverage time by a measurable +27%, which
would be impossible to quantify if drones exchanged data through ad-hoc topics.

**CPU mapping and planning.** The ESDF and the A\*-plus-elastic-band planner are implemented in pure
NumPy/SciPy so they run, and are unit-tested, without a GPU. They produce the navigation and
exploration results directly, and present the same `map_merge_node` interface the nvblox GPU backend
plugs into, so the GPU path is an upgrade rather than a rewrite.

## Known limitations

- Vision-only pursuit follows within ~12 m median of a moving target in the occluded city; tighter
  lock is bounded by the sensing chain and perception→control latency (see §2 and §3).
- The live demo reads drone *self*-position from simulator ground truth (§3); the target is found
  by camera only. Flying the full loop on the EKF belief is the next state-estimation milestone.
- Mapping/planning results use the CPU implementation; the nvblox GPU backend is interface-ready
  but not yet benchmarked here.
- The VIO handover is validated on a survey circle; aggressive/3-D trajectories and long GPS-denied
  endurance are untested.
- Simulation only; the hardware transfer path is analyzed in [sim-to-real.md](sim-to-real.md).
