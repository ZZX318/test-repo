"""Analytic 3D obstacle geometry and continuous collision checks.

Every distance helper accepts either a single point ``(3,)`` or a batch
``(N, 3)``.  Batched evaluation is what makes the collision layer affordable:
one vectorised call per obstacle replaces thousands of scalar calls, which was
the dominant cost in the previous revision.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Union

import numpy as np


def _vec3(value) -> np.ndarray:
    return np.asarray(value, dtype=float).reshape(3)


def _as_batch(value) -> tuple[np.ndarray, bool]:
    """Return ``(N, 3)`` points and whether the caller passed a single point."""
    arr = np.asarray(value, dtype=float)
    single = arr.ndim == 1
    return arr.reshape(-1, 3), single


@dataclass(frozen=True)
class Sphere:
    center: np.ndarray
    radius: float

    def __post_init__(self):
        object.__setattr__(self, "center", _vec3(self.center))
        if self.radius <= 0:
            raise ValueError("sphere radius must be positive")


@dataclass(frozen=True)
class Box:
    minimum: np.ndarray
    maximum: np.ndarray

    def __post_init__(self):
        lo, hi = _vec3(self.minimum), _vec3(self.maximum)
        if np.any(hi <= lo):
            raise ValueError("box maximum must be greater than minimum")
        object.__setattr__(self, "minimum", lo)
        object.__setattr__(self, "maximum", hi)


@dataclass(frozen=True)
class Cylinder:
    center_xy: np.ndarray
    z_min: float
    z_max: float
    radius: float

    def __post_init__(self):
        c = np.asarray(self.center_xy, dtype=float).reshape(2)
        if self.radius <= 0 or self.z_max <= self.z_min:
            raise ValueError("invalid cylinder dimensions")
        object.__setattr__(self, "center_xy", c)


Obstacle = Union[Sphere, Box, Cylinder]


def signed_distance(point, obstacle: Obstacle):
    """Signed distance to the obstacle surface (negative inside).

    Accepts ``(3,)`` (returns ``float``) or ``(N, 3)`` (returns ``(N,)``).
    """
    p, single = _as_batch(point)
    if isinstance(obstacle, Sphere):
        d = np.linalg.norm(p - obstacle.center, axis=1) - obstacle.radius
    elif isinstance(obstacle, Box):
        lo, hi = obstacle.minimum, obstacle.maximum
        q = np.maximum(np.maximum(lo - p, 0.0), p - hi)
        outside = np.linalg.norm(q, axis=1)
        inside = np.min(np.minimum(p - lo, hi - p), axis=1)
        d = np.where(inside > 0.0, -inside, outside)
    elif isinstance(obstacle, Cylinder):
        radial = np.linalg.norm(p[:, :2] - obstacle.center_xy, axis=1) - obstacle.radius
        vertical = np.maximum(np.maximum(obstacle.z_min - p[:, 2], 0.0),
                              p[:, 2] - obstacle.z_max)
        outside = np.hypot(np.maximum(radial, 0.0), vertical)
        inside_mask = (radial <= 0.0) & (p[:, 2] >= obstacle.z_min) & (p[:, 2] <= obstacle.z_max)
        inside = np.minimum(np.minimum(-radial, p[:, 2] - obstacle.z_min),
                            obstacle.z_max - p[:, 2])
        # Above/below a cylinder is outside even when the projection lies in
        # its circular cross-section; the signed distance is the vertical gap.
        d = np.where(inside_mask, -inside, np.where(radial > 0.0, outside, vertical))
    else:
        raise TypeError(f"unsupported obstacle type: {type(obstacle)!r}")
    return float(d[0]) if single else d


def clearance(point, obstacles: Iterable[Obstacle]):
    """Minimum signed distance over ``obstacles`` (``inf`` when there are none)."""
    p, single = _as_batch(point)
    result = np.full(len(p), np.inf)
    for obs in obstacles:
        np.minimum(result, signed_distance(p, obs), out=result)
    return float(result[0]) if single else result


def sample_segment(a, b, step: float = 0.5) -> np.ndarray:
    """Points along ``a -> b`` including both ends, spaced at most ``step``."""
    a, b = _vec3(a), _vec3(b)
    length = float(np.linalg.norm(b - a))
    n = max(int(np.ceil(length / max(float(step), 1e-3))), 1)
    t = np.linspace(0.0, 1.0, n + 1)[:, None]
    return a + t * (b - a)


def segment_clearance(a, b, obstacles: Iterable[Obstacle], step: float = 0.5) -> float:
    """Smallest analytic clearance along the closed segment ``a -> b``."""
    obstacles = list(obstacles)
    if not obstacles:
        return float("inf")
    return float(np.min(clearance(sample_segment(a, b, step), obstacles)))


def segment_collision(a, b, obstacles: Iterable[Obstacle], radius: float = 0.0,
                      step: float = 0.5) -> bool:
    """True when any point of ``a -> b`` is closer than ``radius`` to an obstacle.

    Sampling is conservative because ``step`` is smaller than the voxel size,
    and the whole sample batch is evaluated in one vectorised call.
    """
    return segment_clearance(a, b, obstacles, step) < float(radius)


def polyline_clearance(path, obstacles: Iterable[Obstacle], step: float = 0.25) -> float:
    """Smallest clearance along a polyline, sampled between waypoints too."""
    path = np.asarray(path, dtype=float).reshape(-1, 3)
    obstacles = list(obstacles)
    if len(path) == 0:
        return float("inf")
    if not obstacles:
        return float("inf")
    if len(path) == 1:
        return float(clearance(path[0], obstacles))
    chunks = [sample_segment(a, b, step) for a, b in zip(path[:-1], path[1:])]
    return float(np.min(clearance(np.vstack(chunks), obstacles)))


def path_length(path: np.ndarray) -> float:
    path = np.asarray(path, dtype=float).reshape(-1, 3)
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) if len(path) > 1 else 0.0
