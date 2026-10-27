"""Continuous-space RRT* fallback for maps where voxel resolution is coarse."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .astar3d import PathResult
from .environment import VoxelMap


@dataclass
class RRTConfig:
    iterations: int = 3500
    step_size: float = 2.0
    goal_bias: float = 0.12
    neighbor_radius: float = 4.0
    seed: int = 7
    hard_clearance: float = 0.35


class RRTStar3D:
    def __init__(self, world: VoxelMap, config: Optional[RRTConfig] = None,
                 robot_radius: Optional[float] = None):
        self.world = world
        self.config = config or RRTConfig()
        self.robot_radius = float(self.config.hard_clearance if robot_radius is None
                                  else max(robot_radius, self.config.hard_clearance))
        self.rng = np.random.default_rng(self.config.seed)

    def plan(self, start, goal) -> PathResult:
        start, goal = np.asarray(start, float).reshape(3), np.asarray(goal, float).reshape(3)
        if self.world.clearance(start) < self.robot_radius or self.world.clearance(goal) < self.robot_radius:
            raise ValueError("start or goal is in collision")
        nodes = [start.copy()]
        node_array = np.asarray(nodes)
        parents = [-1]
        costs = [0.0]
        best = None
        for _ in range(self.config.iterations):
            sample = goal if self.rng.random() < self.config.goal_bias else self.rng.uniform(self.world.origin, self.world.maximum)
            nearest = int(np.argmin(np.linalg.norm(node_array - sample, axis=1)))
            direction = sample - node_array[nearest]
            norm = float(np.linalg.norm(direction))
            if norm < 1e-9:
                continue
            new = node_array[nearest] + direction * min(1.0, self.config.step_size / norm)
            if self.world.clearance(new) < self.robot_radius or self.world.segment_collision(node_array[nearest], new, self.robot_radius):
                continue
            offsets = np.linalg.norm(node_array - new, axis=1)
            nearby = [i for i in np.nonzero(offsets <= self.config.neighbor_radius)[0]]
            parent = nearest
            cost = costs[parent] + float(np.linalg.norm(new - node_array[parent]))
            for i in nearby:
                candidate = costs[i] + float(np.linalg.norm(new - node_array[i]))
                if candidate < cost and not self.world.segment_collision(node_array[i], new, self.robot_radius):
                    parent, cost = i, candidate
            nodes.append(new)
            node_array = np.asarray(nodes)
            parents.append(parent)
            costs.append(cost)
            new_index = len(nodes) - 1
            for i in nearby:
                candidate = cost + float(np.linalg.norm(node_array[i] - new))
                if candidate < costs[i] and not self.world.segment_collision(node_array[i], new, self.robot_radius):
                    parents[i], costs[i] = new_index, candidate
            if np.linalg.norm(new - goal) <= self.config.step_size and not self.world.segment_collision(new, goal, self.robot_radius):
                total = cost + float(np.linalg.norm(goal - new))
                if best is None or total < best[0]:
                    best = (total, new_index)
        if best is None:
            raise RuntimeError("RRT* did not connect to the goal; increase iterations")
        points = [goal]
        idx = best[1]
        while idx >= 0:
            points.append(nodes[idx])
            idx = parents[idx]
        points = np.asarray(points[::-1])
        return PathResult(points, float(best[0]), len(nodes), planner="rrtstar3d",
                          metadata={"length": float(best[0]), "nodes": float(len(nodes)),
                                    "min_clearance": self.world.path_clearance(points),
                                    "hard_clearance": float(self.robot_radius),
                                    "constraint_ok": float(
                                        self.world.path_clearance(points) >= self.robot_radius - 1e-9)})
