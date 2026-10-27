"""Safety and quality metrics for a fleet rollout.

``evaluate`` reports both the geometric path and the executed trajectory, and
it always states whether ``min_clearance`` honours ``hard_clearance``.  The old
version silently evaluated the trajectory, which is how a 0.35 m constraint got
reported as 0.27 m without anyone noticing.
"""

from __future__ import annotations

from typing import Dict, Optional, Sequence

import numpy as np

from .environment import VoxelMap


def evaluate(paths: Sequence[np.ndarray], goals: Sequence[np.ndarray], world: VoxelMap,
             minimum_separation: float = 2.5, goal_tolerance: float = 1.0,
             hard_clearance: float = 0.35, sample_step: float = 0.25) -> Dict[str, float]:
    paths = [np.asarray(p, float).reshape(-1, 3) for p in paths]
    goals = np.asarray(goals, float).reshape(-1, 3)
    if len(paths) != len(goals):
        raise ValueError("paths and goals must have the same length")
    final_dist = np.array([np.linalg.norm(p[-1] - g) for p, g in zip(paths, goals)])
    # Interpolated clearance: node-only sampling overestimates safety on long edges.
    clearances = np.array([world.path_clearance(p, step=sample_step) for p in paths])
    min_sep = float("inf")
    if len(paths) > 1:
        # Compare synchronised samples after padding with each final position.
        t = max(len(p) for p in paths)
        padded = np.stack([np.vstack([p, np.repeat(p[-1][None, :], t - len(p), axis=0)])
                           for p in paths])
        distances = np.linalg.norm(padded[:, None] - padded[None, :], axis=-1)
        min_sep = float(np.min(distances[np.triu_indices(len(paths), k=1)]))
    lengths = np.array([np.linalg.norm(np.diff(p, axis=0), axis=1).sum() if len(p) > 1 else 0.0
                        for p in paths])
    straight = np.array([np.linalg.norm(g - p[0]) for p, g in zip(paths, goals)])
    ratio = lengths / np.maximum(straight, 1e-9)
    speed_change = []
    for p in paths:
        if len(p) < 3:
            speed_change.append(0.0)
            continue
        velocity = np.diff(p, axis=0)
        speeds = np.linalg.norm(velocity, axis=1)
        # Scale-free curvature: |dv| / |v|.  The old raw |dv| grew with waypoint
        # density, so it could not be compared between planners.
        turn = np.linalg.norm(np.diff(velocity, axis=0), axis=1)
        speed_change.append(float(np.mean(turn) / max(float(np.mean(speeds)), 1e-9)))
    worst = float(np.min(clearances))
    return {
        "success_rate": float(np.mean(final_dist <= goal_tolerance)),
        "collision_rate": float(np.mean(clearances < 0.0)),
        "clearance_intrusion_rate": float(np.mean(clearances < 0.5)),
        "min_clearance": worst,
        "hard_clearance": float(hard_clearance),
        "clearance_violation": float(worst < float(hard_clearance) - 1e-9),
        "min_separation": min_sep,
        "separation_violation": float(min_sep < minimum_separation),
        "mean_path_length": float(np.mean(lengths)),
        "mean_path_ratio": float(np.mean(ratio)),
        "path_ratio": float(np.mean(ratio)),
        "max_path_ratio": float(np.max(ratio)),
        "mean_final_distance": float(np.mean(final_dist)),
        "mean_smoothness": float(np.mean(speed_change)),
        "mean_steps": float(np.mean([len(p) - 1 for p in paths])),
        "waypoints": [float(len(p)) for p in paths],
        "path_ratios": [float(v) for v in ratio],
    }


def evaluate_plan(plan, goals: Sequence[np.ndarray], world: VoxelMap,
                  hard_clearance: float = 0.35, minimum_separation: float = 2.5,
                  goal_tolerance: float = 1.0) -> Dict[str, Dict[str, float]]:
    """Both layers, side by side: the geometric path and the flown trajectory."""
    report = {"plan": evaluate(plan.paths, goals, world, minimum_separation,
                               goal_tolerance, hard_clearance)}
    if getattr(plan, "trajectories", None) is not None:
        report["execution"] = evaluate(plan.trajectories, goals, world, minimum_separation,
                                       goal_tolerance, hard_clearance)
    return report


_ROWS = [
    ("success_rate", "{:.1%}"), ("collision_rate", "{:.1%}"),
    ("min_clearance", "{:.2f} m"), ("min_separation", "{:.2f} m"),
    ("mean_path_length", "{:.2f} m"), ("mean_path_ratio", "{:.2f}"),
    ("max_path_ratio", "{:.2f}"), ("mean_smoothness", "{:.3f}"),
    ("mean_steps", "{:.1f}"),
]


def format_metrics(metrics: Dict[str, float], indent: str = "") -> str:
    lines = []
    for key, fmt in _ROWS:
        value = metrics.get(key, float("nan"))
        lines.append(f"{indent}{key:28s}: {fmt.format(value)}")
    hard = metrics.get("hard_clearance")
    if hard is not None:
        ok = "OK" if not metrics.get("clearance_violation") else "VIOLATED"
        lines.append(f"{indent}{'hard_clearance':28s}: {hard:.2f} m -> {ok}")
    return "\n".join(lines)


def format_report(report: Dict[str, Dict[str, float]]) -> str:
    blocks = []
    for layer in ("plan", "execution"):
        if layer in report:
            title = "规划层 (A* 几何路径)" if layer == "plan" else "执行层 (动力学跟踪轨迹)"
            blocks.append(f"[{title}]\n" + format_metrics(report[layer]))
    return "\n\n".join(blocks)
