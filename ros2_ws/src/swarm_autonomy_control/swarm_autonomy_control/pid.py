"""Minimal PID with anti-windup and output clamping.

Pure and ROS-free so the position controller's gains can be tuned and regression-
tested offline. One PID per axis composes into the geometric position controller
in ``offboard_control``.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PID:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    out_min: float = -1e9
    out_max: float = 1e9
    integral_limit: float = 1e9

    _i: float = 0.0
    _prev_err: float | None = None

    def reset(self) -> None:
        self._i = 0.0
        self._prev_err = None

    def step(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return self._clamp(self.kp * error)
        self._i += error * dt
        self._i = max(-self.integral_limit, min(self.integral_limit, self._i))
        d = 0.0 if self._prev_err is None else (error - self._prev_err) / dt
        self._prev_err = error
        return self._clamp(self.kp * error + self.ki * self._i + self.kd * d)

    def _clamp(self, u: float) -> float:
        return max(self.out_min, min(self.out_max, u))
