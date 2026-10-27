"""Projection-based dynamic guidance points from the AGP reference method.

Two changes relative to the previous revision:

* ``update`` is split into ``project_position`` (advance the arc coordinate) and
  ``target`` (pure query).  A controller can therefore ask for a *shorter*
  look-ahead when the long one would cut a corner, without corrupting the
  monotonic progress state.
* the effective look-ahead shrinks when the UAV is close to an obstacle
  (``effective_lookahead``).  Corner cutting scales with look-ahead, so this is
  the cheapest way to keep the tracker inside the planned corridor.
"""

from __future__ import annotations

import numpy as np


def arc_length(path: np.ndarray) -> np.ndarray:
    path = np.asarray(path, dtype=float).reshape(-1, 3)
    if len(path) == 0:
        return np.zeros(0)
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])


def point_at(path: np.ndarray, s: float, cumulative: np.ndarray | None = None) -> np.ndarray:
    path = np.asarray(path, dtype=float).reshape(-1, 3)
    if len(path) == 1:
        return path[0].copy()
    cumulative = arc_length(path) if cumulative is None else cumulative
    s = float(np.clip(s, 0.0, cumulative[-1]))
    i = int(np.searchsorted(cumulative, s, side="right") - 1)
    i = min(max(i, 0), len(path) - 2)
    span = cumulative[i + 1] - cumulative[i]
    t = 0.0 if span <= 1e-12 else (s - cumulative[i]) / span
    return path[i] + t * (path[i + 1] - path[i])


def project(position, path, cumulative: np.ndarray | None = None):
    """Return arc coordinate, projected point and distance to a polyline."""
    p = np.asarray(position, dtype=float).reshape(3)
    path = np.asarray(path, dtype=float).reshape(-1, 3)
    cumulative = arc_length(path) if cumulative is None else cumulative
    if len(path) == 1:
        return 0.0, path[0].copy(), float(np.linalg.norm(p - path[0]))
    best = (float("inf"), 0.0, path[0].copy())
    for i, (a, b) in enumerate(zip(path[:-1], path[1:])):
        ab = b - a
        denom = float(ab @ ab)
        t = 0.0 if denom <= 1e-12 else float(np.clip((p - a) @ ab / denom, 0.0, 1.0))
        q = a + t * ab
        d = float(np.linalg.norm(p - q))
        if d < best[0]:
            best = (d, float(cumulative[i] + t * np.sqrt(denom)), q)
    return best[1], best[2], best[0]


def resample(path, spacing: float = 1.0) -> np.ndarray:
    path = np.asarray(path, dtype=float).reshape(-1, 3)
    cumulative = arc_length(path)
    if len(path) < 2 or cumulative[-1] <= 1e-12:
        return path.copy()
    count = max(int(np.ceil(cumulative[-1] / spacing)), 1)
    return np.stack([point_at(path, s, cumulative) for s in np.linspace(0, cumulative[-1], count + 1)])


class GuidanceTracker:
    """Monotonic, look-ahead guidance tracker with potential shaping."""

    def __init__(self, path, lookahead: float = 2.5, reach_radius: float = 1.2,
                 smoothing: float = 0.35):
        self.path = resample(path, max(0.25, min(1.0, lookahead / 2.0)))
        if len(self.path) == 0:
            raise ValueError("guidance path cannot be empty")
        self.cumulative = arc_length(self.path)
        self.lookahead = float(lookahead)
        self.reach_radius = float(reach_radius)
        self.smoothing = float(np.clip(smoothing, 0.05, 1.0))
        self.progress_s = 0.0
        self.current = self.path[0].copy()
        self.offset = 0.0
        self.max_offset = 0.0
        self._previous_potential = None

    # --------------------------------------------------------------- queries
    def effective_lookahead(self, clearance: float | None = None) -> float:
        """Look-ahead shortened near obstacles (corner cutting scales with it)."""
        if clearance is None:
            return self.lookahead
        scaled = 0.8 * max(float(clearance), 0.0) + 0.4
        return float(np.clip(scaled, 0.5, self.lookahead))

    def target(self, lookahead: float | None = None) -> np.ndarray:
        look = self.lookahead if lookahead is None else float(lookahead)
        return point_at(self.path, self.progress_s + look, self.cumulative)

    def project_position(self, position) -> float:
        s_near, _, distance = project(position, self.path, self.cumulative)
        # Forward-only progress prevents oscillation when the vehicle cuts a corner.
        self.progress_s = max(self.progress_s, s_near)
        self.offset = float(distance)
        self.max_offset = max(self.max_offset, self.offset)
        return self.progress_s

    def update(self, position, clearance: float | None = None) -> np.ndarray:
        self.project_position(position)
        target = self.target(self.effective_lookahead(clearance))
        self.current = (1.0 - self.smoothing) * self.current + self.smoothing * target
        return self.current.copy()

    def potential_reward(self, position, gamma: float = 0.95) -> float:
        phi = -float(np.linalg.norm(np.asarray(position, dtype=float) - self.current))
        if self._previous_potential is None:
            self._previous_potential = phi
            return 0.0
        reward = gamma * phi - self._previous_potential
        self._previous_potential = phi
        return float(reward)

    @property
    def done(self) -> bool:
        return self.progress_s >= self.cumulative[-1] - self.reach_radius

    @property
    def progress(self) -> float:
        return self.progress_s / max(float(self.cumulative[-1]), 1e-12)


def paper_guidance_reward(position, guidance_point, gain: float = 2.0, radius: float = 1.5) -> float:
    d = float(np.linalg.norm(np.asarray(position, dtype=float) - np.asarray(guidance_point, dtype=float)))
    return float(gain * max(0.0, 1.0 - (d / max(radius, 1e-9)) ** 2))
