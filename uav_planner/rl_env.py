"""Gym-like NumPy environment for training a multi-UAV policy.

It deliberately has no Gym dependency.  The action is a normalized 3-D desired
velocity per UAV, making it easy to connect either a hand-written controller or
the optional attention critic.  Global A* paths can be passed as guidance
references, so this environment is the bridge between the deterministic
planner and a future MADDPG experiment.
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from .config import DynamicsConfig, RewardConfig
from .dynamics import KinematicModel, UAVState
from .environment import VoxelMap
from .guidance import GuidanceTracker, paper_guidance_reward


class MultiUAV3DEnv:
    def __init__(self, world: VoxelMap, starts: Sequence, goals: Sequence,
                 reference_paths: Optional[Sequence[np.ndarray]] = None,
                 dynamics: Optional[DynamicsConfig] = None,
                 rewards: Optional[RewardConfig] = None,
                 max_steps: int = 150):
        self.world = world
        self.starts = np.asarray(starts, dtype=float).reshape(-1, 3)
        self.goals = np.asarray(goals, dtype=float).reshape(-1, 3)
        if len(self.starts) != len(self.goals):
            raise ValueError("starts and goals must have equal length")
        self.n_agents = len(self.starts)
        self.reference_paths = reference_paths
        self.dynamics = dynamics or DynamicsConfig()
        self.reward_cfg = rewards or RewardConfig()
        self.model = KinematicModel(self.dynamics)
        self.max_steps = int(max_steps)
        self.states = []
        self.trackers = []
        self.step_count = 0
        self.reset()

    @property
    def observation_dim(self) -> int:
        # own position, goal delta, guidance delta, velocity, clearance
        return 3 + 3 + 3 + 3 + 1

    def reset(self):
        self.states = [UAVState(p, np.zeros(3)) for p in self.starts]
        self.trackers = [GuidanceTracker(path) for path in self.reference_paths] if self.reference_paths else []
        self.step_count = 0
        return self.observations()

    def _guidance(self, i):
        if not self.trackers:
            return self.goals[i]
        clearance = float(self.world.clearance(self.states[i].position))
        return self.trackers[i].update(self.states[i].position, clearance)

    def observations(self) -> np.ndarray:
        extent = np.maximum(self.world.maximum - self.world.origin, 1e-6)
        result = []
        positions = np.stack([s.position for s in self.states])
        for i, state in enumerate(self.states):
            guidance = self._guidance(i)
            result.append(np.concatenate([
                (state.position - self.world.origin) / extent,
                (self.goals[i] - state.position) / extent,
                (guidance - state.position) / extent,
                state.velocity / max(self.dynamics.max_speed, 1e-6),
                [self.world.clearance(state.position) / max(extent.max(), 1.0)],
            ]))
        return np.asarray(result, dtype=np.float32)

    def step(self, actions):
        actions = np.asarray(actions, dtype=float).reshape(self.n_agents, 3)
        before = np.stack([s.position.copy() for s in self.states])
        for i, action in enumerate(actions):
            desired = np.clip(action, -1.0, 1.0) * self.dynamics.max_speed
            previous = self.states[i]
            candidate = self.model.step(previous, desired)
            candidate.position = np.clip(candidate.position, self.world.origin, self.world.maximum)
            if self.world.segment_collision(previous.position, candidate.position,
                                            radius=self.dynamics.hard_clearance):
                candidate.position = previous.position.copy()
                candidate.velocity = np.zeros(3)
            self.states[i] = candidate
        after = np.stack([s.position for s in self.states])
        rewards = np.zeros(self.n_agents, dtype=float)
        for i in range(self.n_agents):
            old_distance = float(np.linalg.norm(self.goals[i] - before[i]))
            new_distance = float(np.linalg.norm(self.goals[i] - after[i]))
            rewards[i] += self.reward_cfg.progress * (old_distance - new_distance)
            rewards[i] += self.reward_cfg.guidance_gain * paper_guidance_reward(
                after[i], self._guidance(i), gain=1.0, radius=self.reward_cfg.guidance_radius)
            clearance = self.world.clearance(after[i])
            if clearance < 0:
                rewards[i] -= self.reward_cfg.collision_penalty
            elif clearance < 0.5:
                rewards[i] -= self.reward_cfg.obstacle_penalty * (0.5 - clearance)
            if new_distance <= self.dynamics.goal_tolerance:
                rewards[i] += self.reward_cfg.arrival
        if self.n_agents > 1:
            pairwise = np.linalg.norm(after[:, None] - after[None, :], axis=-1)
            for i in range(self.n_agents):
                nearest = np.min(pairwise[i][np.arange(self.n_agents) != i])
                if nearest < self.reward_cfg.separation_penalty:
                    rewards[i] -= self.reward_cfg.separation_penalty - nearest
        self.step_count += 1
        distances = np.linalg.norm(self.goals - after, axis=1)
        terminated = bool(np.all(distances <= self.dynamics.goal_tolerance))
        truncated = self.step_count >= self.max_steps
        info = {"step": self.step_count, "distances": distances,
                "min_clearance": float(min(self.world.clearance(p) for p in after))}
        return self.observations(), rewards.astype(np.float32), terminated, truncated, info
