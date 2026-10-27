"""Voxel world used by both A* and RRT*.

The map stores a boolean occupancy grid for fast search and keeps the analytic
obstacles for accurate clearance metrics and continuous validation.

Two derived rasters make the A* inner loop a pure array lookup:

``clearance_grid()``
    Analytic clearance at every cell centre (metres), cached.
``passable(clearance, margin)``
    Boolean mask of cells whose centre is at least ``clearance + margin`` away
    from every obstacle.  Because the analytic clearance is 1-Lipschitz, a
    margin of ``0.5 * sqrt(3) * resolution`` guarantees that *every* point of a
    cell and of any edge joining two passable neighbours is at least
    ``clearance`` away from an obstacle.  That is what lets the search skip the
    per-edge sampling loop without losing the hard constraint.
"""

from __future__ import annotations

from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .geometry import Box, Cylinder, Obstacle, Sphere, clearance, polyline_clearance, signed_distance

Cell = Tuple[int, int, int]


class VoxelMap:
    def __init__(self, shape=(32, 32, 14), resolution: float = 1.0,
                 origin=(0.0, 0.0, 0.0)):
        self.shape = tuple(int(v) for v in shape)
        if len(self.shape) != 3 or min(self.shape) < 2:
            raise ValueError("shape must contain three dimensions >= 2")
        self.resolution = float(resolution)
        if self.resolution <= 0:
            raise ValueError("resolution must be positive")
        self.origin = np.asarray(origin, dtype=float).reshape(3)
        self.occupied = np.zeros(self.shape, dtype=bool)
        self.obstacles: List[Obstacle] = []
        self._distance_cache = None
        self._clearance_cache = None
        self._passable_cache: Dict[Tuple[float, float], np.ndarray] = {}
        self._cells = None

    # ------------------------------------------------------------------ basics
    @property
    def maximum(self) -> np.ndarray:
        return self.origin + (np.asarray(self.shape, dtype=float) - 1.0) * self.resolution

    @property
    def cell_centres(self) -> np.ndarray:
        """``(N, 3)`` world coordinates of every cell centre (cached)."""
        if self._cells is None:
            idx = np.indices(self.shape, dtype=float).reshape(3, -1).T
            self._cells = self.origin + idx * self.resolution
        return self._cells

    def inside(self, cell: Cell) -> bool:
        return all(0 <= int(cell[i]) < self.shape[i] for i in range(3))

    def free(self, cell: Cell) -> bool:
        return self.inside(cell) and not bool(self.occupied[tuple(cell)])

    def world_to_grid(self, point, clip: bool = False) -> Cell:
        p = (np.asarray(point, dtype=float).reshape(3) - self.origin) / self.resolution
        c = np.rint(p).astype(int)
        if clip:
            c = np.clip(c, 0, np.asarray(self.shape) - 1)
        return tuple(int(v) for v in c)

    def grid_to_world(self, cell: Cell) -> np.ndarray:
        return self.origin + np.asarray(cell, dtype=float) * self.resolution

    # --------------------------------------------------------------- obstacles
    def add_obstacle(self, obstacle: Obstacle) -> None:
        self.obstacles.append(obstacle)
        # One vectorised distance evaluation over the whole grid instead of a
        # Python loop over ~14k cells per obstacle.
        inside = signed_distance(self.cell_centres, obstacle) <= 0.0
        self.occupied |= inside.reshape(self.shape)
        self._invalidate()

    def remove_obstacle(self, index: int = -1) -> None:
        """Drop one obstacle and rebuild the occupancy grid."""
        self.obstacles.pop(index)
        self._rebuild_occupancy()

    def clear_obstacles(self) -> None:
        self.obstacles.clear()
        self._rebuild_occupancy()

    def _rebuild_occupancy(self) -> None:
        self.occupied = np.zeros(self.shape, dtype=bool)
        for obs in self.obstacles:
            inside = signed_distance(self.cell_centres, obs) <= 0.0
            self.occupied |= inside.reshape(self.shape)
        self._invalidate()

    def _invalidate(self) -> None:
        self._distance_cache = None
        self._clearance_cache = None
        self._passable_cache.clear()

    def add_sphere(self, center, radius: float) -> None:
        self.add_obstacle(Sphere(center, radius))

    def add_box(self, minimum, maximum) -> None:
        self.add_obstacle(Box(minimum, maximum))

    def add_cylinder(self, center_xy, z_min, z_max, radius) -> None:
        self.add_obstacle(Cylinder(center_xy, z_min, z_max, radius))

    def add_random_obstacles(self, count: int, rng: np.random.Generator,
                             radius_range=(1.2, 2.8), z_margin: int = 2,
                             avoid_points: Sequence[np.ndarray] = (),
                             max_attempts: int = 200,
                             corridors: Sequence[Tuple[np.ndarray, np.ndarray]] = (),
                             corridor_ratio: float = 0.5,
                             corridor_offset: float = 3.0,
                             avoid_regions: Sequence[Tuple[np.ndarray, np.ndarray]] = ()) -> None:
        """Add spheres away from endpoints and the map boundary.

        A fraction ``corridor_ratio`` of the spheres is biased towards the
        straight corridor between a random ``(start, goal)`` pair.  Uniform
        sampling alone almost never blocks a path: the corridor bias is what
        turns "obstacle count" into an actual detour cost.

        ``avoid_regions`` are ``(min, max)`` boxes that must stay open - the
        holes in the gapped walls.  Keeping them clear is what stops the
        repair loop from having to delete half the obstacle field.

        Rejection sampling keeps scenario construction deterministic while
        preventing random obstacles from invalidating starts/goals.
        """
        lo = np.asarray([2, 2, z_margin], dtype=int)
        hi = np.asarray(self.shape) - np.asarray([2, 2, z_margin])
        hi = np.maximum(hi, lo + 1)
        avoid = [np.asarray(p, dtype=float).reshape(3) for p in avoid_points]
        corridors = [(np.asarray(a, float).reshape(3), np.asarray(b, float).reshape(3))
                     for a, b in corridors]
        regions = [(np.asarray(a, float).reshape(3), np.asarray(b, float).reshape(3))
                   for a, b in avoid_regions]
        added = 0
        attempts = 0
        limit = max_attempts * max(int(count), 1)
        while added < int(count) and attempts < limit:
            attempts += 1
            radius = float(rng.uniform(*radius_range))
            use_corridor = bool(corridors) and rng.random() < corridor_ratio
            if use_corridor:
                a, b = corridors[int(rng.integers(len(corridors)))]
                t = float(rng.uniform(0.15, 0.85))
                base = a + t * (b - a)
                # Offset perpendicular to the corridor so the sphere straddles it.
                direction = b - a
                norm = float(np.linalg.norm(direction))
                direction = direction / norm if norm > 1e-9 else np.array([0.0, 1.0, 0.0])
                side = np.cross(direction, np.array([0.0, 0.0, 1.0]))
                if np.linalg.norm(side) < 1e-6:
                    side = np.array([0.0, 1.0, 0.0])
                side = side / np.linalg.norm(side)
                base = base + side * float(rng.uniform(-corridor_offset, corridor_offset)) \
                    + np.array([0.0, 0.0, float(rng.uniform(-corridor_offset, corridor_offset))])
                center = np.clip(base, self.origin + 1.0, self.maximum - 1.0)
            else:
                cell = rng.integers(lo, hi)
                center = self.grid_to_world(tuple(cell))
            if any(np.linalg.norm(center - point) < radius + 2.0 for point in avoid):
                continue
            if any(np.all(center >= lo_r - radius) and np.all(center <= hi_r + radius)
                   for lo_r, hi_r in regions):
                continue
            self.add_sphere(center, radius)
            added += 1

    # ------------------------------------------------------------- clearance
    def clearance(self, point):
        """Analytic clearance at ``point`` (``(3,)`` or ``(N, 3)``)."""
        return clearance(point, self.obstacles)

    def clearance_grid(self) -> np.ndarray:
        """Analytic clearance at every cell centre, in metres (cached)."""
        if self._clearance_cache is None:
            grid = np.full(int(np.prod(self.shape)), np.inf)
            for obs in self.obstacles:
                np.minimum(grid, signed_distance(self.cell_centres, obs), out=grid)
            self._clearance_cache = grid.reshape(self.shape)
        return self._clearance_cache

    def passable(self, clearance_threshold: float = 0.0,
                 margin: Optional[float] = None) -> np.ndarray:
        """Cells whose whole voxel is at least ``clearance_threshold`` from obstacles.

        ``margin`` defaults to half the space diagonal of a cell, which makes the
        mask safe for 26-neighbour moves without any per-edge sampling.
        """
        if margin is None:
            margin = 0.5 * np.sqrt(3.0) * self.resolution
        key = (round(float(clearance_threshold), 6), round(float(margin), 6))
        cached = self._passable_cache.get(key)
        if cached is not None:
            return cached
        mask = self.clearance_grid() >= (float(clearance_threshold) + float(margin))
        self._passable_cache[key] = mask
        return mask

    def clearance_steps(self, cap: int) -> np.ndarray:
        """Clearance quantised to integer multiples of ``resolution`` (for lookups)."""
        grid = self.clearance_grid()
        steps = np.rint(np.where(np.isfinite(grid), grid, cap * self.resolution) / self.resolution)
        return np.clip(steps, 0, cap).astype(np.int32)

    def segment_collision(self, a, b, radius: float = 0.0, step: Optional[float] = None) -> bool:
        from .geometry import segment_collision
        return segment_collision(a, b, self.obstacles, radius=radius,
                                 step=max(self.resolution * 0.5 if step is None else step, 0.02))

    def path_clearance(self, path, step: float = 0.25) -> float:
        """Smallest analytic clearance along a polyline (interpolated, not just nodes)."""
        return polyline_clearance(path, self.obstacles, step=step)

    # ---------------------------------------------------------------- rasters
    def distance_field(self, max_distance: float = 20.0) -> np.ndarray:
        """6-connected distance to occupied cells, in metres (cached)."""
        if self._distance_cache is not None:
            return self._distance_cache
        dist = np.full(self.shape, np.inf, dtype=float)
        q = deque()
        for c in zip(*np.nonzero(self.occupied)):
            dist[c] = 0.0
            q.append(c)
        dirs = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
        while q:
            c = q.popleft()
            value = dist[c]
            if value >= max_distance:
                continue
            for d in dirs:
                n = (c[0] + d[0], c[1] + d[1], c[2] + d[2])
                if self.inside(n) and dist[n] > value + self.resolution:
                    dist[n] = value + self.resolution
                    q.append(n)
        dist[np.isinf(dist)] = max_distance
        self._distance_cache = dist
        return dist

    def path_distance_steps(self, paths: Sequence[np.ndarray], cap: int) -> np.ndarray:
        """6-connected distance (in cells) from every cell to the given paths.

        Replaces a per-node ``min(norm(path - point))`` with one BFS, so the
        separation term costs an array lookup inside A* as well.
        """
        steps = np.full(self.shape, cap, dtype=np.int32)
        q = deque()
        for path in paths:
            path = np.asarray(path, dtype=float).reshape(-1, 3)
            if not len(path):
                continue
            for p in path:
                c = self.world_to_grid(p, clip=True)
                if steps[c] > 0:
                    steps[c] = 0
                    q.append(c)
        dirs = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
        while q:
            c = q.popleft()
            value = int(steps[c])
            if value >= cap:
                continue
            for d in dirs:
                n = (c[0] + d[0], c[1] + d[1], c[2] + d[2])
                if self.inside(n) and steps[n] > value + 1:
                    steps[n] = value + 1
                    q.append(n)
        return steps

    def reachable(self, start, goal, clearance_threshold: float = 0.0,
                  margin: Optional[float] = None) -> bool:
        """Flood fill on the passable mask; used to *verify* scenario feasibility."""
        mask = self.passable(clearance_threshold, margin)
        s = self.world_to_grid(start, clip=True)
        g = self.world_to_grid(goal, clip=True)
        if not mask[s] or not mask[g]:
            return False
        seen = np.zeros(self.shape, dtype=bool)
        seen[s] = True
        q = deque([s])
        dirs = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
        while q:
            c = q.popleft()
            if c == g:
                return True
            for d in dirs:
                n = (c[0] + d[0], c[1] + d[1], c[2] + d[2])
                if self.inside(n) and not seen[n] and mask[n]:
                    seen[n] = True
                    q.append(n)
        return False

    def nearest_passable(self, point, clearance_threshold: float = 0.0,
                         margin: Optional[float] = None) -> Cell:
        """Closest cell satisfying the clearance mask (keeps scenarios feasible)."""
        mask = self.passable(clearance_threshold, margin)
        if not mask.any():
            raise ValueError("no cell satisfies the requested clearance")
        p = np.asarray(point, dtype=float).reshape(3)
        candidates = np.argwhere(mask)
        if len(candidates) == 0:
            raise ValueError("no passable cell available")
        centres = self.origin + candidates * self.resolution
        best = int(np.argmin(np.linalg.norm(centres - p, axis=1)))
        return tuple(int(v) for v in candidates[best])

    # -------------------------------------------------------------- traversal
    def neighbors(self, cell: Cell, diagonal: bool = True):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == dy == dz == 0:
                        continue
                    if not diagonal and sum(abs(v) for v in (dx, dy, dz)) > 1:
                        continue
                    n = (cell[0] + dx, cell[1] + dy, cell[2] + dz)
                    if self.free(n):
                        yield n

    def edge_free(self, a: Cell, b: Cell, distance_field: np.ndarray | None = None,
                  clearance: float = 0.0) -> bool:
        """Sampled voxel edge check (kept for RRT*/tests; A* uses the mask)."""
        field = self.clearance_grid() if distance_field is None else distance_field
        samples = np.rint(np.linspace(np.asarray(a, float), np.asarray(b, float), 5)).astype(int)
        for s in samples:
            cell = tuple(int(v) for v in s)
            if not self.inside(cell) or self.occupied[cell] or field[cell] < clearance:
                return False
        return True

    def inflated(self, margin: float) -> "VoxelMap":
        """Return a copy with analytic obstacles expanded by ``margin``."""
        result = VoxelMap(self.shape, self.resolution, self.origin)
        for obs in self.obstacles:
            if isinstance(obs, Sphere):
                result.add_sphere(obs.center, obs.radius + margin)
            elif isinstance(obs, Box):
                result.add_box(obs.minimum - margin, obs.maximum + margin)
            else:
                result.add_cylinder(obs.center_xy, obs.z_min - margin, obs.z_max + margin,
                                    obs.radius + margin)
        return result
