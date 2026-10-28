"""Configuration objects shared by the planners and the simulator.

:class:`PlannerConfig` is the *single* source of truth for planning parameters.
:class:`CostWeights` is the compact object the planners consume; it is derived
from ``PlannerConfig`` via :meth:`PlannerConfig.to_weights`, so the two can no
longer drift apart (the previous revision had seven fields nobody read).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple


@dataclass
class CostWeights:
    """Objective coefficients.

    ``hard_clearance`` is a true safety constraint (infeasible cells are
    rejected); the other weights only bias *which* safe path is chosen.
    """

    length: float = 1.0
    obstacle: float = 8.0
    separation: float = 6.0
    clearance: float = 2.0
    minimum_separation: float = 2.5
    hard_clearance: float = 0.35

    def replace(self, **changes) -> "CostWeights":
        from dataclasses import replace
        return replace(self, **changes)


@dataclass
class PlannerConfig:
    """Parameters for a voxel planner.

    ``resolution`` is metres per cell.  ``hard_clearance`` is a true safety
    constraint; the other weights are soft costs used to choose between safe
    alternatives.  ``safety_margin`` is the extra inflation applied to the
    passable mask; ``None`` means half a cell diagonal, which is what makes a
    26-neighbour search provably safe without per-edge sampling.
    """

    resolution: float = 1.0
    shape: Tuple[int, int, int] = (32, 32, 14)
    length_weight: float = 1.0
    obstacle_weight: float = 8.0
    separation_weight: float = 6.0
    clearance: float = 2.0
    hard_clearance: float = 0.35
    minimum_separation: float = 2.5
    max_expansions: int = 250_000
    diagonal_cost: bool = True
    heuristic: str = "euclidean"
    safety_margin: Optional[float] = None

    def to_weights(self) -> CostWeights:
        return CostWeights(
            length=self.length_weight,
            obstacle=self.obstacle_weight,
            separation=self.separation_weight,
            clearance=self.clearance,
            minimum_separation=self.minimum_separation,
            hard_clearance=self.hard_clearance,
        )


@dataclass
class DynamicsConfig:
    """Simple bounded velocity/acceleration model used for demonstrations."""

    dt: float = 0.2
    max_speed: float = 4.0
    max_acceleration: float = 5.0
    goal_tolerance: float = 1.0
    lookahead: float = 2.5
    hard_clearance: float = 0.35


@dataclass
class RewardConfig:
    """Reward terms useful when the planner is used as an RL environment."""

    arrival: float = 20.0
    progress: float = 1.0
    guidance_gain: float = 2.0
    guidance_radius: float = 1.5
    obstacle_penalty: float = 3.0
    collision_penalty: float = 25.0
    separation_penalty: float = 2.0


@dataclass
class ScenarioConfig:
    """Reproducible scenario settings.

    ``obstacle_radius`` is the sphere radius range, ``corridor_ratio`` the
    fraction of obstacles biased towards a start-goal corridor and
    ``max_repairs`` how many obstacles may be dropped to restore feasibility.
    """

    seed: int = 7
    n_uavs: int = 1
    n_obstacles: int = 12
    n_walls: Optional[int] = None
    obstacle_radius: Tuple[float, float] = (1.5, 3.0)
    corridor_ratio: float = 0.55
    corridor_offset: float = 3.0
    max_repairs: int = 64
    planner: PlannerConfig = field(default_factory=PlannerConfig)
