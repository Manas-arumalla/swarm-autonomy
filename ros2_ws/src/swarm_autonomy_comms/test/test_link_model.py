"""Unit tests for the ROS-free link model."""

import random

from swarm_autonomy_comms.link_model import (
    DropReason,
    LinkConfig,
    LinkModel,
    WindowStats,
    _TokenBucket,
)


def test_out_of_range_is_dropped():
    lm = LinkModel(LinkConfig(max_range_m=80.0), random.Random(1))
    res = lm.try_deliver(t=0.0, range_m=120.0, nbytes=100)
    assert not res.delivered
    assert res.reason == DropReason.OUT_OF_RANGE


def test_in_range_lossless_when_no_random_loss_and_budget_available():
    # base_loss=0 removes the random component; large bandwidth removes rate limit.
    lm = LinkModel(LinkConfig(base_loss=0.0, bandwidth_bps=1e9), random.Random(1))
    res = lm.try_deliver(t=0.0, range_m=10.0, nbytes=100)
    assert res.delivered
    assert res.reason == DropReason.DELIVERED


def test_rate_limit_blocks_oversized_burst():
    cfg = LinkConfig(base_loss=0.0, bandwidth_bps=1000.0, bucket_capacity_bytes=1000.0)
    lm = LinkModel(cfg, random.Random(1))
    # First packet drains the bucket; an immediate second one (no refill time) fails.
    assert lm.try_deliver(0.0, 10.0, 1000).delivered
    res = lm.try_deliver(0.0, 10.0, 1000)
    assert not res.delivered
    assert res.reason == DropReason.RATE_LIMITED


def test_token_bucket_refills_over_time():
    b = _TokenBucket(rate=1000.0, capacity=1000.0)
    b.refill(0.0)
    assert b.consume(1000.0)
    assert not b.consume(500.0)
    b.refill(1.0)  # 1 s -> +1000 bytes, capped at capacity
    assert b.consume(500.0)


def test_range_loss_increases_with_distance():
    cfg = LinkConfig(soft_range_m=50.0, max_range_m=80.0, base_loss=0.0, edge_loss=0.6)
    lm = LinkModel(cfg, random.Random(1))
    assert lm._range_loss(10.0) == 0.0
    assert lm._range_loss(50.0) == 0.0
    mid = lm._range_loss(65.0)
    assert 0.0 < mid < 0.6
    assert abs(lm._range_loss(80.0) - 0.6) < 1e-9


def test_delivery_rate_degrades_with_range_statistically():
    # Far-but-in-range link should deliver fewer packets than a near link.
    cfg = LinkConfig(bandwidth_bps=1e9, soft_range_m=50.0, max_range_m=80.0,
                     base_loss=0.0, edge_loss=0.6)
    near = LinkModel(cfg, random.Random(42))
    far = LinkModel(cfg, random.Random(42))
    n_near = sum(near.try_deliver(i * 1.0, 10.0, 100).delivered for i in range(1000))
    n_far = sum(far.try_deliver(i * 1.0, 75.0, 100).delivered for i in range(1000))
    assert n_near > n_far
    assert n_near > 950  # essentially lossless up close


def test_window_stats_accounting():
    st = WindowStats()
    cfg = LinkConfig(base_loss=0.0, bandwidth_bps=1e9)
    lm = LinkModel(cfg, random.Random(1))
    for i in range(5):
        st.add(lm.try_deliver(i * 1.0, 10.0, 200))
    assert st.sent == 5
    assert st.delivered == 5
    assert st.bytes_delivered == 1000
    assert st.bytes_per_s(1.0) == 1000.0
    assert st.mean_range() == 10.0
