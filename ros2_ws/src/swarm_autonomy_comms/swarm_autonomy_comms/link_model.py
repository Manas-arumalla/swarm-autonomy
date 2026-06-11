"""Pure, ROS-free model of a bandwidth-limited multi-drone radio link.

Every inter-drone message is run through :meth:`LinkModel.try_deliver`,
which gates on:

* **range**  — packets beyond ``max_range_m`` are dropped; a soft falloff above
  ``soft_range_m`` raises the drop probability toward the edge of coverage.
* **rate**   — a per-link token bucket caps throughput at ``bandwidth_bps``;
  packets that don't fit the budget are dropped (no infinite queue).
* **dropout** — an i.i.d. ``base_loss`` plus the range-dependent term models a
  lossy channel.

Determinism: randomness comes from an injected :class:`random.Random`, so tests
and replays are reproducible (workflow/CI never call the global RNG).

Kept dependency-free so it can be unit-tested with plain pytest and reused
from headless simulation environments without modification.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import random


@dataclass
class LinkConfig:
    bandwidth_bps: float = 50_000.0      # per-link delivered-throughput cap (bytes/s)
    max_range_m: float = 80.0            # hard cutoff: no delivery beyond this
    soft_range_m: float = 50.0           # range-dependent loss starts here
    base_loss: float = 0.02              # baseline i.i.d. packet loss [0,1]
    edge_loss: float = 0.6               # extra loss at max_range_m
    bucket_capacity_bytes: float = 25_000.0  # token-bucket burst size


@dataclass
class _TokenBucket:
    """Classic token bucket; tokens are bytes, refilled at ``rate`` bytes/s."""

    rate: float
    capacity: float
    tokens: float = field(default=0.0)
    last_t: float | None = None

    def refill(self, t: float) -> None:
        if self.last_t is None:
            self.tokens = self.capacity
        else:
            dt = max(0.0, t - self.last_t)
            self.tokens = min(self.capacity, self.tokens + dt * self.rate)
        self.last_t = t

    def consume(self, nbytes: float) -> bool:
        if nbytes <= self.tokens:
            self.tokens -= nbytes
            return True
        return False


class DropReason:
    DELIVERED = "delivered"
    OUT_OF_RANGE = "out_of_range"
    RATE_LIMITED = "rate_limited"
    RANDOM_LOSS = "random_loss"


@dataclass
class DeliveryResult:
    delivered: bool
    reason: str
    range_m: float
    bytes: int


@dataclass
class WindowStats:
    sent: int = 0
    delivered: int = 0
    dropped: int = 0
    bytes_delivered: int = 0
    range_sum: float = 0.0

    def add(self, res: DeliveryResult) -> None:
        self.sent += 1
        self.range_sum += res.range_m
        if res.delivered:
            self.delivered += 1
            self.bytes_delivered += res.bytes
        else:
            self.dropped += 1

    def bytes_per_s(self, window_s: float) -> float:
        return self.bytes_delivered / window_s if window_s > 0 else 0.0

    def mean_range(self) -> float:
        return self.range_sum / self.sent if self.sent else 0.0


class LinkModel:
    """Gates messages on one directed link (sender -> receiver)."""

    def __init__(self, cfg: LinkConfig, rng: random.Random | None = None):
        self.cfg = cfg
        self.rng = rng or random.Random(0)
        self._bucket = _TokenBucket(rate=cfg.bandwidth_bps, capacity=cfg.bucket_capacity_bytes)

    def _range_loss(self, range_m: float) -> float:
        if range_m <= self.cfg.soft_range_m:
            return self.cfg.base_loss
        span = max(1e-6, self.cfg.max_range_m - self.cfg.soft_range_m)
        frac = (range_m - self.cfg.soft_range_m) / span
        return min(1.0, self.cfg.base_loss + self.cfg.edge_loss * frac)

    def try_deliver(self, t: float, range_m: float, nbytes: int) -> DeliveryResult:
        if range_m > self.cfg.max_range_m or math.isinf(range_m):
            return DeliveryResult(False, DropReason.OUT_OF_RANGE, range_m, nbytes)

        if self.rng.random() < self._range_loss(range_m):
            return DeliveryResult(False, DropReason.RANDOM_LOSS, range_m, nbytes)

        self._bucket.refill(t)
        if not self._bucket.consume(float(nbytes)):
            return DeliveryResult(False, DropReason.RATE_LIMITED, range_m, nbytes)

        return DeliveryResult(True, DropReason.DELIVERED, range_m, nbytes)
