"""Collision-aware path post-processing.

Both passes take a ``safety_radius``.  It used to be hard-coded to 0.25 m while
``hard_clearance`` was 0.35 m, so smoothing alone could produce a path that
violated the planner's own constraint.  :class:`FleetPlanner` now passes
``hard_clearance`` through, and re-validates the result before accepting it.
"""

from __future__ import annotations

import numpy as np

from .environment import VoxelMap


def shortcut(path, world: VoxelMap, passes: int = 2, seed: int = 7,
             safety_radius: float = 0.25) -> np.ndarray:
    """Remove unnecessary waypoints only when the replacement is collision-free."""
    points = np.asarray(path, dtype=float).reshape(-1, 3).copy()
    if len(points) <= 2:
        return points
    rng = np.random.default_rng(seed)
    for _ in range(max(1, passes)):
        changed = True
        while changed:
            changed = False
            indices = list(range(1, len(points) - 1))
            rng.shuffle(indices)
            for i in indices:
                if world.segment_collision(points[i - 1], points[i + 1], radius=safety_radius):
                    continue
                points = np.delete(points, i, axis=0)
                changed = True
                break
    return points


def chaikin(path, iterations: int = 2) -> np.ndarray:
    """Smooth a polyline while preserving its exact endpoints."""
    points = np.asarray(path, dtype=float).reshape(-1, 3)
    for _ in range(max(0, iterations)):
        if len(points) < 3:
            break
        result = [points[0]]
        for a, b in zip(points[:-1], points[1:]):
            result.extend([0.75 * a + 0.25 * b, 0.25 * a + 0.75 * b])
        result.append(points[-1])
        points = np.asarray(result)
    return points


def smooth_collision_aware(path, world: VoxelMap, shortcut_passes: int = 2,
                           chaikin_iterations: int = 2,
                           safety_radius: float = 0.25) -> np.ndarray:
    """Shortcut + Chaikin, keeping every segment at least ``safety_radius`` clear."""
    candidate = shortcut(path, world, shortcut_passes, safety_radius=safety_radius)
    curved = chaikin(candidate, chaikin_iterations)
    if any(world.segment_collision(a, b, radius=safety_radius) for a, b in zip(curved[:-1], curved[1:])):
        return candidate
    if world.path_clearance(curved) < safety_radius - 1e-9:
        return candidate
    return curved
