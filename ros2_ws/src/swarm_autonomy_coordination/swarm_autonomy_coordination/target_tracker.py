"""ROS-free constant-velocity Kalman tracker for the fleeing target.

The pursuers detect the target by camera and back-project to a ground position, but a single
back-projection is noisy and jumps several metres frame-to-frame (camera tilt during manoeuvres,
attitude/image lag, blob-centroid noise). Chasing the raw estimate makes the swarm follow a
teleporting phantom and peel off the target.

This filter fuses ALL pursuers' detections over time into one stable track:
  * a constant-velocity model predicts the target through detection dropouts (so the swarm keeps
    pursuing — and leads — even when nobody currently sees it, the "stays on it if it speeds up"
    requirement),
  * Mahalanobis GATING rejects outlier detections (the 5-12 m jumps) instead of lurching to them,
  * fusing several pursuers' views per tick shrinks the variance faster than any one camera.

State x = [E, N, vE, vN]. Pure/unit-tested like cbba/pursuit/mpc_pursuit; the Gazebo pursuit and
any headless sim use the same filter.
"""

from __future__ import annotations

import math

import numpy as np


class TargetTracker:
    def __init__(self, q_accel: float = 2.0, r_meas: float = 1.5, gate: float = 3.0,
                 vmax: float = 6.0):
        """q_accel: target accel std (m/s^2) driving process noise; r_meas: per-detection
        position std (m); gate: Mahalanobis distance for outlier rejection; vmax: speed clamp."""
        self.q = float(q_accel)
        self.R = np.eye(2) * (float(r_meas) ** 2)
        self.gate2 = float(gate) ** 2
        self.vmax = float(vmax)
        self._p_cap = 1.0e4                # bound position/velocity variance growth on long dropouts
        self.x = None                      # [E, N, vE, vN]
        self.P = None
        self.t = None
        self.H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])

    def initialized(self) -> bool:
        return self.x is not None

    def _clamp_velocity(self) -> None:
        """Hold the velocity estimate within vmax so a coasting track never extrapolates absurdly."""
        sp = math.hypot(self.x[2], self.x[3])
        if sp > self.vmax:
            self.x[2] *= self.vmax / sp
            self.x[3] *= self.vmax / sp

    def _predict(self, dt: float) -> None:
        dt = max(1e-3, min(1.0, dt))
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        # constant-acceleration process noise (white-accel model)
        q = self.q ** 2
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        Qb = np.array([[dt4 / 4, dt3 / 2], [dt3 / 2, dt2]]) * q
        Q = np.zeros((4, 4))
        Q[np.ix_([0, 2], [0, 2])] = Qb
        Q[np.ix_([1, 3], [1, 3])] = Qb
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        # coasting must respect vmax (no runaway extrapolation on dropout) and stay bounded/symmetric
        self._clamp_velocity()
        self.P = 0.5 * (self.P + self.P.T)
        np.fill_diagonal(self.P, np.minimum(np.diag(self.P), self._p_cap))

    def predict_to(self, t: float):
        """Advance the track to time t WITHOUT a measurement (dropout). Returns (E,N) or None."""
        if self.x is None:
            return None
        self._predict(t - self.t)
        self.t = t
        return (float(self.x[0]), float(self.x[1]))

    def update(self, measurements, t: float, noise_scales=None):
        """Fuse a list of (E, N) detections taken at time t. Outliers are gated out.
        `noise_scales` (optional, same length) multiplies each measurement's std — pass a
        large scale for ill-conditioned (image-edge / off-nadir) detections so an accurate
        near-nadir fix dominates the track. Returns the filtered (E, N) or None."""
        pairs = [(m, (noise_scales[i] if noise_scales else 1.0))
                 for i, m in enumerate(measurements) if m is not None]
        if self.x is None:
            if not pairs:
                return None
            e = float(np.mean([m[0] for m, _ in pairs]))
            n = float(np.mean([m[1] for m, _ in pairs]))
            self.x = np.array([e, n, 0.0, 0.0])
            self.P = np.diag([self.R[0, 0], self.R[1, 1], 16.0, 16.0])
            self.t = t
            return (e, n)

        self._predict(t - self.t)
        self.t = t
        for m, scale in pairs:
            z = np.array([float(m[0]), float(m[1])])
            y = z - self.H @ self.x
            Rm = self.R * float(scale) ** 2
            S = self.H @ self.P @ self.H.T + Rm
            try:
                Sinv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                continue
            if float(y @ Sinv @ y) > self.gate2:      # outlier -> reject the jump
                continue
            K = self.P @ self.H.T @ Sinv
            self.x = self.x + K @ y
            ImKH = np.eye(4) - K @ self.H
            self.P = ImKH @ self.P @ ImKH.T + K @ Rm @ K.T   # Joseph form: stays symmetric & PSD
        self._clamp_velocity()                                # clamp implausible velocity
        return (float(self.x[0]), float(self.x[1]))

    def position(self):
        return None if self.x is None else (float(self.x[0]), float(self.x[1]))

    def velocity(self):
        return (0.0, 0.0) if self.x is None else (float(self.x[2]), float(self.x[3]))
