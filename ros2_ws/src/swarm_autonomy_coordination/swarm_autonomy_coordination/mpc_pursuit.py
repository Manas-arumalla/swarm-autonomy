"""Model-Predictive Control for smooth pursuit interception.

A condensed **linear MPC** for a double-integrator drone — state = (position,
velocity), input = acceleration — tracking a predicted (constant-velocity) target
trajectory over a finite horizon. Instead of *reacting* to the current target
position (which oscillates), it solves a small box-constrained QP

    min_U  sum_k  q*||p_k - p_target_k||^2 + qv*||v_k - v_target||^2 + r*||a_k||^2
           + qf*||v_N - v_target||^2      (terminal velocity matching = planned braking)
    s.t.   |a_k| <= a_max          (acceleration limit)
           double-integrator dynamics

and returns the SMOOTH first-step velocity command — the optimiser folds the future into the
present, so it tracks a moving target without reacting to per-frame jitter. The terminal
velocity-matching cost makes the plan *arrive like the target moves*: braking to a stop at a
static slot, or station-keeping at the target's velocity for a mover — instead of carrying
``v_max`` through a distant set-point and overshooting.

The x and y axes decouple (the target trajectory is per-axis), so each control step solves two
tiny 1-D box-constrained QPs (via scipy). ROS-free and unit-tested; the same law runs in the
Gazebo pursuit bridge and the headless sim.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.optimize import minimize

# Cache the (horizon, dt)-dependent prediction matrices and the QP Hessian, since
# they are constant across control steps — only the linear term changes.
_CACHE: dict = {}


def _matrices(N: int, dt: float, q: float, r: float, qv: float, qf: float):
    key = (N, round(dt, 4), q, r, qv, qf)
    cached = _CACHE.get(key)
    if cached is not None:
        return cached
    # p_k = p0 + k*dt*v0 + dt^2 * sum_{j<k} a_j*(k-j-0.5)        (k = 1..N)
    # v_k = v0 + dt * sum_{j<k} a_j
    Gp = np.zeros((N, N))
    Gv = np.zeros((N, N))
    for k in range(1, N + 1):
        for j in range(k):
            Gp[k - 1, j] = (k - j - 0.5) * dt * dt
            Gv[k - 1, j] = dt
    kv = np.arange(1, N + 1) * dt           # k*dt
    gN = Gv[-1]                              # terminal velocity row: v_N = v0 + gN @ U
    H = 2.0 * (q * Gp.T @ Gp + qv * Gv.T @ Gv + r * np.eye(N) + qf * np.outer(gN, gN))
    H = 0.5 * (H + H.T)                      # symmetric
    cached = (Gp, Gv, kv, gN, H)
    _CACHE[key] = cached
    return cached


def _solve_axis(p0, v0, p_tgt, v_tgt, vmax, amax, N, dt, q, r, qv, qf):
    """Solve the 1-D box-constrained MPC QP; return the first velocity v_1."""
    Gp, Gv, kv, gN, H = _matrices(N, dt, q, r, qv, qf)
    base_p = p0 + kv * v0                    # position rollout with zero accel
    base_v = np.full(N, v0)
    p_ref = p0 + v_tgt * kv + (p_tgt - p0)   # predicted target pos at each step (const vel)
    # J(U) = q||Gp U + base_p - p_ref||^2 + qv||Gv U + base_v - v_tgt||^2 + r||U||^2
    #        + qf*(v0 + gN U - v_tgt)^2                       (terminal velocity matching)
    f = 2.0 * (q * Gp.T @ (base_p - p_ref) + qv * Gv.T @ (base_v - v_tgt)
               + qf * gN * (v0 - v_tgt))

    def fun(u):
        return 0.5 * u @ (H @ u) + f @ u

    def jac(u):
        return H @ u + f

    u0 = np.zeros(N)
    bounds = [(-amax, amax)] * N
    res = minimize(fun, u0, jac=jac, method="L-BFGS-B", bounds=bounds,
                   options={"maxiter": 40})
    a0 = float(res.x[0])
    v1 = v0 + a0 * dt
    return max(-vmax, min(vmax, v1))


def mpc_velocity(pos, vel, goal, goal_vel=(0.0, 0.0), vmax=4.5, amax=3.0,
                 dt=0.12, horizon=15, q=1.0, r=0.04, qv=0.2, qf=1.5):
    """Smooth MPC velocity command to intercept a goal moving at ``goal_vel``.

    Returns (vx, vy). Solves one small QP per axis; falls back to a clamped
    proportional command if the solver hiccups (keeps the loop alive). ``qf``
    weights the terminal velocity-matching (braking) cost.
    """
    try:
        vx = _solve_axis(pos[0], vel[0], goal[0], goal_vel[0], vmax, amax,
                         horizon, dt, q, r, qv, qf)
        vy = _solve_axis(pos[1], vel[1], goal[1], goal_vel[1], vmax, amax,
                         horizon, dt, q, r, qv, qf)
    except Exception:
        ex, ey = goal[0] - pos[0], goal[1] - pos[1]
        d = math.hypot(ex, ey) or 1.0
        sp = min(vmax, d) / d
        vx, vy = goal_vel[0] + ex * sp, goal_vel[1] + ey * sp
    sp = math.hypot(vx, vy)
    if sp > vmax:
        vx, vy = vx / sp * vmax, vy / sp * vmax
    return (vx, vy)
