"""Bounded point-mass UAV model and guidance-based trajectory rollout.

The rollout now enforces the *hard* clearance, not just "no penetration".
Previously the tracker chased a look-ahead point and cut corners, so the
executed trajectory could come closer to an obstacle than the planner allowed
(0.27 m against a 0.35 m constraint).  Two mechanisms fix that:

1. the look-ahead shrinks near obstacles, so corner cutting is small to begin
   with (:meth:`GuidanceTracker.effective_lookahead`);
2. every proposed step is validated against ``hard_clearance``; if it violates
   it, the step is truncated to the largest safe fraction.  The reported
   execution clearance therefore cannot drop below the constraint.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .config import DynamicsConfig
from .guidance import GuidanceTracker


@dataclass
class UAVState:
    position: np.ndarray
    velocity: np.ndarray

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=float).reshape(3).copy()
        self.velocity = np.asarray(self.velocity, dtype=float).reshape(3).copy()


class KinematicModel:
    def __init__(self, config: Optional[DynamicsConfig] = None):
        self.config = config or DynamicsConfig()

    def step(self, state: UAVState, desired_velocity) -> UAVState:
        desired = np.asarray(desired_velocity, dtype=float).reshape(3).copy()
        speed = float(np.linalg.norm(desired))
        if speed > self.config.max_speed:
            desired *= self.config.max_speed / speed
        delta = desired - state.velocity
        max_delta = self.config.max_acceleration * self.config.dt
        delta_norm = float(np.linalg.norm(delta))
        if delta_norm > max_delta:
            delta *= max_delta / delta_norm
        velocity = state.velocity + delta
        position = state.position + velocity * self.config.dt
        return UAVState(position, velocity)

    # ------------------------------------------------------------------ safety
    def _safe_fraction(self, world, a: np.ndarray, b: np.ndarray,
                       hard_clearance: float, iterations: int = 10) -> float:
        """Largest ``t`` in [0, 1] with the segment ``a -> a + t (b - a)`` safe."""
        step = max(world.resolution * 0.25, 0.05)

        def safe(point):
            if world.clearance(point) < hard_clearance:
                return False
            return not world.segment_collision(a, point, radius=hard_clearance, step=step)

        if safe(b):
            return 1.0
        low, high = 0.0, 1.0
        for _ in range(iterations):
            mid = 0.5 * (low + high)
            if safe(a + mid * (b - a)):
                low = mid
            else:
                high = mid
        return low

    def rollout(self, start, goal, world=None, tracker: GuidanceTracker | None = None,
                max_steps: int = 500, hard_clearance: Optional[float] = None,
                stall_limit: int = 40) -> Tuple[np.ndarray, np.ndarray]:
        hard = self.config.hard_clearance if hard_clearance is None else float(hard_clearance)
        state = UAVState(np.asarray(start, float), np.zeros(3))
        goal = np.asarray(goal, dtype=float).reshape(3)
        trajectory = [state.position.copy()]
        rewards = []
        stalls = 0
        for _ in range(max_steps):
            clearance_now = float(world.clearance(state.position)) if world is not None else float("inf")
            if tracker is not None:
                tracker.project_position(state.position)
                look = tracker.effective_lookahead(clearance_now)
            else:
                look = 0.0

            best = None
            for scale in (1.0, 0.6, 0.3, 0.0):
                target = tracker.target(look * scale) if tracker is not None else goal
                delta = target - state.position
                norm = float(np.linalg.norm(delta))
                if norm < 1e-9:
                    continue
                candidate = self.step(state, delta / norm * self.config.max_speed)
                if world is not None:
                    candidate.position = np.clip(candidate.position, world.origin, world.maximum)
                    fraction = self._safe_fraction(world, state.position, candidate.position, hard)
                else:
                    fraction = 1.0
                if fraction >= 1.0 - 1e-9:
                    best = (fraction, candidate)
                    break
                if best is None or fraction > best[0]:
                    best = (fraction, candidate)

            if best is None:
                # Already at the target: hold position.
                trajectory.append(state.position.copy())
                rewards.append(tracker.potential_reward(state.position) if tracker is not None else 0.0)
                if np.linalg.norm(state.position - goal) <= self.config.goal_tolerance:
                    break
                continue

            fraction, candidate = best
            if fraction < 1.0 - 1e-9:
                position = state.position + fraction * (candidate.position - state.position)
                candidate = UAVState(position, (position - state.position) / self.config.dt)
            moved = float(np.linalg.norm(candidate.position - state.position))
            state = candidate
            stalls = stalls + 1 if moved < 1e-4 else 0
            trajectory.append(state.position.copy())
            rewards.append(tracker.potential_reward(state.position) if tracker is not None else 0.0)
            if np.linalg.norm(state.position - goal) <= self.config.goal_tolerance:
                break
            if stalls >= stall_limit:
                break
        return np.asarray(trajectory), np.asarray(rewards)
