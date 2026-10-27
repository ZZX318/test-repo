"""A small but complete research-grade 3D multi-UAV planning toolkit.

The package follows the pipeline used by the AGP-MADDPG reference material:
voxel world -> global planner -> dynamic guidance -> kinematic tracking ->
safety/quality metrics.  NumPy is the only required dependency.
"""

from .config import CostWeights, DynamicsConfig, PlannerConfig, RewardConfig, ScenarioConfig
from .environment import VoxelMap
from .astar3d import AStar3D, AStarStepper3D, PathResult, plan_fleet
from .fleet import FleetPlan, FleetPlanner
from .guidance import GuidanceTracker
from .dynamics import KinematicModel, UAVState
from .metrics import evaluate, evaluate_plan, format_metrics, format_report
from .scenario import Scenario, make_demo, make_wall_demo
from .rl_env import MultiUAV3DEnv
from .algorithms import ALGORITHMS, plan_path

__all__ = [
    "AStar3D", "AStarStepper3D", "CostWeights", "DynamicsConfig", "FleetPlan",
    "FleetPlanner", "GuidanceTracker", "KinematicModel", "MultiUAV3DEnv",
    "PathResult", "PlannerConfig", "RewardConfig", "Scenario", "ScenarioConfig",
    "UAVState", "VoxelMap", "evaluate", "evaluate_plan", "format_metrics",
    "format_report", "make_demo", "make_wall_demo", "plan_fleet",
    "ALGORITHMS", "plan_path",
]
