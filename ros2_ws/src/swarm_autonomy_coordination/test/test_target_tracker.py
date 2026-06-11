"""Unit tests for the constant-velocity target Kalman tracker."""
import math
import random

from swarm_autonomy_coordination.target_tracker import TargetTracker


def test_converges_to_static_target():
    tr = TargetTracker(r_meas=1.5)
    rng = random.Random(0)
    truth = (10.0, -4.0)
    for k in range(40):
        z = (truth[0] + rng.gauss(0, 1.5), truth[1] + rng.gauss(0, 1.5))
        tr.update([z], t=k * 0.1)
    e, n = tr.position()
    assert math.hypot(e - truth[0], n - truth[1]) < 1.0        # filtered below the 1.5 m meas noise


def test_tracks_constant_velocity_mover():
    tr = TargetTracker(r_meas=1.0, q_accel=1.0)
    rng = random.Random(1)
    vel = (1.5, -0.5)
    for k in range(60):
        t = k * 0.1
        tx, ty = 2.0 + vel[0] * t, 5.0 + vel[1] * t
        tr.update([(tx + rng.gauss(0, 1.0), ty + rng.gauss(0, 1.0))], t)
    vE, vN = tr.velocity()
    assert math.hypot(vE - vel[0], vN - vel[1]) < 0.6          # recovers the velocity
    # and predicts ahead roughly correctly
    pe, pn = tr.predict_to(6.0 + 0.5)
    assert abs(pe - (2.0 + vel[0] * 6.5)) < 1.5


def test_rejects_outlier_jumps():
    tr = TargetTracker(r_meas=1.0, gate=3.0)
    for k in range(20):                                        # establish a clean track at origin
        tr.update([(0.0, 0.0)], t=k * 0.1)
    before = tr.position()
    tr.update([(40.0, -30.0)], t=2.1)                          # a wild 50 m outlier detection
    after = tr.position()
    assert math.hypot(after[0] - before[0], after[1] - before[1]) < 1.0   # barely moved


def test_fusing_multiple_detections_beats_one():
    rng = random.Random(2)
    truth = (8.0, 8.0)
    one = TargetTracker(r_meas=2.0)
    many = TargetTracker(r_meas=2.0)
    for k in range(25):
        t = k * 0.1
        one.update([(truth[0] + rng.gauss(0, 2), truth[1] + rng.gauss(0, 2))], t)
        many.update([(truth[0] + rng.gauss(0, 2), truth[1] + rng.gauss(0, 2)) for _ in range(3)], t)
    e1 = math.hypot(one.position()[0] - truth[0], one.position()[1] - truth[1])
    em = math.hypot(many.position()[0] - truth[0], many.position()[1] - truth[1])
    assert em <= e1 + 0.3                                      # 3 views no worse (usually better)


def test_predict_through_dropout():
    tr = TargetTracker(q_accel=1.0)
    for k in range(20):
        tr.update([(k * 0.15, 0.0)], t=k * 0.1)               # moving +E at 1.5 m/s
    p = tr.position()[0]
    nxt = tr.predict_to(2.0 + 0.5)                            # 0.5 s with NO detection
    assert nxt[0] > p                                          # kept moving, didn't freeze


def test_noise_scales_trusts_the_centred_detection():
    # two conflicting fixes each tick: a TRUSTED near-nadir one at the true spot, and a noisy
    # image-edge one pulling away. Per-measurement noise should keep the track near the trusted fix.
    tr = TargetTracker(r_meas=1.5, gate=8.0)                  # wide gate so neither is rejected
    truth = (5.0, 5.0)
    for k in range(40):
        meas = [truth, (truth[0] - 12.0, truth[1] + 8.0)]    # [centred-accurate, edge-far]
        tr.update(meas, t=k * 0.1, noise_scales=[1.0, 8.0])  # distrust the edge fix
    e, n = tr.position()
    assert math.hypot(e - truth[0], n - truth[1]) < 2.0      # stayed near the trusted detection

    # without the weighting, the same edge fix drags the track much further off
    tr2 = TargetTracker(r_meas=1.5, gate=8.0)
    for k in range(40):
        tr2.update([truth, (truth[0] - 12.0, truth[1] + 8.0)], t=k * 0.1)
    e2, n2 = tr2.position()
    assert math.hypot(e2 - truth[0], n2 - truth[1]) > math.hypot(e - truth[0], n - truth[1])
