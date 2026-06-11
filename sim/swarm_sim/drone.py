"""Kinematic drone driven by the real Swarm Autonomy PID controller.

FIDELITY NOTE: the per-axis :class:`swarm_autonomy_control.pid.PID` turns position error into a
velocity *command* (clamped to ``max_speed``); the achieved velocity then tracks that command
through an **acceleration limit** (``max_accel``), so the vehicle cannot reverse or reach full
speed instantaneously — a first-order stand-in for PX4's velocity-tracking lag. There is still no
attitude loop, mass, or thrust dynamics; PX4/Gazebo add those. Use this for coordination/comms/
allocation studies, not for control-fidelity claims; the timing numbers remain optimistic
relative to hardware (just no longer infinitely so). A simple repulsive term steers the drone
around buildings (standing in for the ego-planner local avoidance used in the full stack).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from swarm_autonomy_control.pid import PID  # the real controller module

from .world import World


@dataclass
class Drone:
    drone_id: int
    pos: np.ndarray                      # shape (2,)
    max_speed: float = 4.0
    max_accel: float = 6.0               # m/s^2: bounds dv per step (no instant reversals)
    sensor_radius: float = 7.0
    vel: np.ndarray = field(default_factory=lambda: np.zeros(2))
    goal: np.ndarray | None = None
    role: int = 0                        # mirrors swarm_autonomy_msgs DroneRole constants
    _pid_x: PID = field(init=False)
    _pid_y: PID = field(init=False)

    def __post_init__(self) -> None:
        self.pos = np.asarray(self.pos, dtype=float)
        self._pid_x = PID(kp=1.4, kd=0.15, out_min=-self.max_speed, out_max=self.max_speed)
        self._pid_y = PID(kp=1.4, kd=0.15, out_min=-self.max_speed, out_max=self.max_speed)

    def set_goal(self, goal) -> None:
        self.goal = None if goal is None else np.asarray(goal, dtype=float)

    def _repulsion(self, world: World) -> np.ndarray:
        """Sum of short-range pushes away from nearby building edges."""
        force = np.zeros(2)
        influence = 2.5
        for b in world.buildings:
            cx = min(max(self.pos[0], b.x0), b.x1)
            cy = min(max(self.pos[1], b.y0), b.y1)  # closest point on the rect
            d = np.array([self.pos[0] - cx, self.pos[1] - cy])
            dist = np.linalg.norm(d)
            if 1e-6 < dist < influence:
                force += (d / dist) * (influence - dist) / influence * self.max_speed
        return force

    def step(self, world: World, dt: float) -> None:
        if self.goal is None:
            cmd = self.vel * 0.5
        else:
            vx = self._pid_x.step(self.goal[0] - self.pos[0], dt)
            vy = self._pid_y.step(self.goal[1] - self.pos[1], dt)
            cmd = np.array([vx, vy]) + 1.5 * self._repulsion(world)
            speed = np.linalg.norm(cmd)
            if speed > self.max_speed:
                cmd = cmd / speed * self.max_speed
        # Acceleration limit: the achieved velocity tracks the command, it doesn't jump to it.
        dv = cmd - self.vel
        dv_max = self.max_accel * dt
        dv_norm = np.linalg.norm(dv)
        if dv_norm > dv_max:
            dv = dv / dv_norm * dv_max
        self.vel = self.vel + dv

        new = self.pos + self.vel * dt
        # Don't penetrate buildings or leave the map; slide along instead.
        if world.is_free(new[0], new[1], margin=0.2) and world.in_bounds(new[0], new[1]):
            self.pos = new
        else:
            for axis in (0, 1):  # try axis-separated motion to slide on walls
                trial = self.pos.copy()
                trial[axis] = new[axis]
                if world.is_free(trial[0], trial[1], margin=0.2) and world.in_bounds(*trial):
                    self.pos = trial
                    break

    def at_goal(self, tol: float = 1.0) -> bool:
        return self.goal is not None and np.linalg.norm(self.pos - self.goal) < tol

    def sees(self, world: World, point: np.ndarray) -> bool:
        return (np.linalg.norm(self.pos - point) <= self.sensor_radius
                and not world.blocked(self.pos[0], self.pos[1], point[0], point[1]))
