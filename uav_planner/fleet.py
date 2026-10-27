"""Fleet-level planning and dynamic guidance rollout."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

from .astar3d import AStar3D, CostWeights, PathResult, plan_fleet
from .config import PlannerConfig
from .dynamics import KinematicModel
from .environment import Cell, VoxelMap
from .guidance import GuidanceTracker
from .smoothing import smooth_collision_aware


@dataclass
class FleetPlan:
    paths: List[np.ndarray]
    results: List[PathResult]
    trajectories: Optional[List[np.ndarray]] = None


class FleetPlanner:
    """High-level composition root: global plan, smooth, then track."""

    def __init__(self, world: VoxelMap, weights: Optional[CostWeights] = None,
                 max_expansions: int = 250_000, safety_margin: Optional[float] = None,
                 heuristic: str = "euclidean", diagonal: bool = True):
        self.world = world
        self.weights = weights or CostWeights()
        self.max_expansions = max_expansions
        self.safety_margin = safety_margin
        self.heuristic = heuristic
        self.diagonal = diagonal

    @classmethod
    def from_config(cls, world: VoxelMap, config: PlannerConfig) -> "FleetPlanner":
        return cls(world, config.to_weights(), config.max_expansions,
                   config.safety_margin, config.heuristic, config.diagonal_cost)

    def plan(self, starts: Sequence[Cell], goals: Sequence[Cell],
             smooth: bool = True) -> FleetPlan:
        results = plan_fleet(self.world, starts, goals, self.weights,
                             diagonal=self.diagonal, max_expansions=self.max_expansions,
                             heuristic=self.heuristic, safety_margin=self.safety_margin)
        paths = [self._post_process(r.points, smooth) for r in results]
        for path, result in zip(paths, results):
            # Report the metrics of the path that is actually delivered.
            result.metadata["min_clearance"] = self.world.path_clearance(path)
            result.metadata["length"] = float(
                np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) if len(path) > 1 else 0.0
            result.metadata["waypoints"] = float(len(path))
            result.metadata["constraint_ok"] = float(
                result.metadata["min_clearance"] >= self.weights.hard_clearance - 1e-9)
        return FleetPlan(paths=paths, results=results)

    def _post_process(self, points: np.ndarray, smooth: bool) -> np.ndarray:
        if not smooth or len(points) < 3:
            return np.asarray(points, dtype=float).copy()
        hard = self.weights.hard_clearance
        candidate = smooth_collision_aware(points, self.world, safety_radius=hard)
        if self.world.path_clearance(candidate) >= hard - 1e-9:
            return candidate
        return np.asarray(points, dtype=float).copy()

    def rollout(self, fleet_plan: FleetPlan, goals: Sequence[np.ndarray],
                model: Optional[KinematicModel] = None, max_steps: int = 500,
                hard_clearance: Optional[float] = None) -> FleetPlan:
        model = model or KinematicModel()
        hard = self.weights.hard_clearance if hard_clearance is None else float(hard_clearance)
        trajectories = []
        for path, goal in zip(fleet_plan.paths, goals):
            tracker = GuidanceTracker(path, lookahead=model.config.lookahead)
            trajectory, _ = model.rollout(path[0], goal, self.world, tracker,
                                          max_steps=max_steps, hard_clearance=hard)
            trajectories.append(trajectory)
        fleet_plan.trajectories = trajectories
        return fleet_plan
