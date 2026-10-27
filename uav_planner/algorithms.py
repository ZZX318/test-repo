"""Additional planning backends with one common ``PathResult`` interface.

The project keeps the optimized multi-objective A* implementation as the
default and adds deliberately small reference implementations for comparison:
Dijkstra, greedy best-first, bidirectional Dijkstra/A*, and RRT-Connect.  They
are useful in experiments because every method returns the same path/cost/
expanded metadata instead of forcing the UI to know each algorithm's internals.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

from .astar3d import AStar3D, CostWeights, PathResult
from .environment import Cell, VoxelMap
from .geometry import path_length

GRAPH_ALGORITHMS = ("astar", "dijkstra", "greedy", "bidirectional")
SAMPLING_ALGORITHMS = ("rrtstar", "rrtconnect")
ALGORITHMS = GRAPH_ALGORITHMS + SAMPLING_ALGORITHMS

_OFFSETS = tuple((dx, dy, dz) for dx in (-1, 0, 1)
                 for dy in (-1, 0, 1) for dz in (-1, 0, 1)
                 if (dx, dy, dz) != (0, 0, 0))


def _neighbors(cell: Cell, shape, diagonal=True):
    for dx, dy, dz in _OFFSETS:
        if not diagonal and abs(dx) + abs(dy) + abs(dz) > 1:
            continue
        n = (cell[0] + dx, cell[1] + dy, cell[2] + dz)
        if 0 <= n[0] < shape[0] and 0 <= n[1] < shape[1] and 0 <= n[2] < shape[2]:
            yield n


def _grid_cost(world: VoxelMap, a: Cell, b: Cell, clearance_grid: np.ndarray,
               weights: CostWeights) -> float:
    step = float(np.linalg.norm(np.subtract(b, a))) * world.resolution
    d = float(clearance_grid[b])
    penalty = weights.obstacle * max(0.0, (weights.clearance - d) /
                                     max(weights.clearance, 1e-9)) ** 2
    return weights.length * step + penalty


def _rebuild_path(parent: Dict[Cell, Cell], node: Cell) -> List[Cell]:
    result = [node]
    while node in parent:
        node = parent[node]
        result.append(node)
    result.reverse()
    return result


class GreedyBestFirst3D:
    """Fast baseline that prioritizes only the estimated distance to goal."""

    def __init__(self, world: VoxelMap, weights: Optional[CostWeights] = None,
                 diagonal: bool = True, max_expansions: int = 250_000):
        self.world, self.weights = world, weights or CostWeights()
        self.diagonal, self.max_expansions = diagonal, int(max_expansions)

    def plan(self, start: Cell, goal: Cell, other_paths=()) -> PathResult:
        start, goal = tuple(map(int, start)), tuple(map(int, goal))
        mask = self.world.passable(self.weights.hard_clearance)
        if not mask[start] or not mask[goal]:
            raise ValueError("start or goal violates the hard clearance constraint")
        clearance_grid = self.world.clearance_grid()
        h = lambda c: float(np.linalg.norm(np.subtract(goal, c)))
        frontier = [(h(start), 0.0, start)]
        g = {start: 0.0}
        parent = {}
        expanded = 0
        while frontier:
            _, current_g, current = heapq.heappop(frontier)
            if current_g > g.get(current, float("inf")) + 1e-10:
                continue
            if current == goal:
                cells = _rebuild_path(parent, current)
                points = np.stack([self.world.grid_to_world(c) for c in cells])
                return PathResult(points, current_g, expanded, planner="greedy",
                                  metadata={"length": path_length(points),
                                            "min_clearance": self.world.path_clearance(points),
                                            "waypoints": float(len(points)),
                                            "hard_clearance": self.weights.hard_clearance})
            expanded += 1
            if expanded > self.max_expansions:
                break
            for nb in _neighbors(current, self.world.shape, self.diagonal):
                # ``passable`` already includes the half-cell safety margin;
                # avoid sampling analytic geometry in this inner loop.
                if not mask[nb]:
                    continue
                ng = current_g + _grid_cost(self.world, current, nb, clearance_grid, self.weights)
                if ng < g.get(nb, float("inf")):
                    g[nb], parent[nb] = ng, current
                    heapq.heappush(frontier, (h(nb), ng, nb))
        raise RuntimeError("greedy best-first search found no feasible path")


class BidirectionalAStar3D:
    """Bidirectional weighted search on the same conservative voxel mask."""

    def __init__(self, world: VoxelMap, weights: Optional[CostWeights] = None,
                 diagonal: bool = True, max_expansions: int = 250_000):
        self.world, self.weights = world, weights or CostWeights()
        self.diagonal, self.max_expansions = diagonal, int(max_expansions)

    def plan(self, start: Cell, goal: Cell, other_paths=()) -> PathResult:
        start, goal = tuple(map(int, start)), tuple(map(int, goal))
        mask = self.world.passable(self.weights.hard_clearance)
        if not mask[start] or not mask[goal]:
            raise ValueError("start or goal violates the hard clearance constraint")
        clearance_grid = self.world.clearance_grid()
        front_a, front_b = [(0.0, start)], [(0.0, goal)]
        dist_a, dist_b = {start: 0.0}, {goal: 0.0}
        par_a, par_b = {}, {}
        closed_a, closed_b = set(), set()
        best, meet, expanded = float("inf"), None, 0
        while front_a and front_b:
            side_a = front_a[0][0] <= front_b[0][0]
            frontier, dist, other, parent, closed = ((front_a, dist_a, dist_b, par_a, closed_a)
                                                      if side_a else (front_b, dist_b, dist_a, par_b, closed_b))
            cost, current = heapq.heappop(frontier)
            if current in closed or cost > dist[current] + 1e-10:
                continue
            closed.add(current)
            expanded += 1
            if current in other:
                candidate = cost + other[current]
                if candidate < best:
                    best, meet = candidate, current
            if expanded > self.max_expansions:
                break
            for nb in _neighbors(current, self.world.shape, self.diagonal):
                if not mask[nb]:
                    continue
                nc = cost + _grid_cost(self.world, current, nb, clearance_grid, self.weights)
                if nc < dist.get(nb, float("inf")) - 1e-10:
                    dist[nb], parent[nb] = nc, current
                    heapq.heappush(frontier, (nc, nb))
        if meet is None:
            raise RuntimeError("bidirectional search found no feasible path")
        left = _rebuild_path(par_a, meet)
        right = []
        node = meet
        while node in par_b:
            node = par_b[node]
            right.append(node)
        cells = left + right
        points = np.stack([self.world.grid_to_world(c) for c in cells])
        return PathResult(points, float(best), expanded, planner="bidirectional",
                          metadata={"length": path_length(points),
                                    "min_clearance": self.world.path_clearance(points),
                                    "waypoints": float(len(points)),
                                    "hard_clearance": self.weights.hard_clearance})


@dataclass
class RRTConnectConfig:
    iterations: int = 2500
    step_size: float = 2.0
    goal_bias: float = 0.15
    seed: int = 7
    hard_clearance: float = 0.35


class RRTConnect3D:
    """Two-tree RRT-Connect baseline for continuous 3D spaces."""

    def __init__(self, world: VoxelMap, config: Optional[RRTConnectConfig] = None):
        self.world = world
        self.config = config or RRTConnectConfig()
        self.rng = np.random.default_rng(self.config.seed)

    def _extend(self, nodes, parents, target):
        array = np.asarray(nodes)
        nearest = int(np.argmin(np.linalg.norm(array - target, axis=1)))
        direction = target - array[nearest]
        distance = float(np.linalg.norm(direction))
        if distance < 1e-9:
            return nearest
        new = array[nearest] + direction * min(1.0, self.config.step_size / distance)
        if self.world.clearance(new) < self.config.hard_clearance:
            return None
        if self.world.segment_collision(array[nearest], new, self.config.hard_clearance):
            return None
        nodes.append(new)
        parents.append(nearest)
        return len(nodes) - 1

    @staticmethod
    def _chain(nodes, parents, index):
        chain = []
        while index >= 0:
            chain.append(nodes[index])
            index = parents[index]
        return chain[::-1]

    def plan(self, start, goal, other_paths=()) -> PathResult:
        start, goal = np.asarray(start, float).reshape(3), np.asarray(goal, float).reshape(3)
        if self.world.clearance(start) < self.config.hard_clearance or self.world.clearance(goal) < self.config.hard_clearance:
            raise ValueError("start or goal violates the hard clearance constraint")
        nodes_a, parents_a = [start.copy()], [-1]
        nodes_b, parents_b = [goal.copy()], [-1]
        for iteration in range(self.config.iterations):
            sample = goal if self.rng.random() < self.config.goal_bias else self.rng.uniform(self.world.origin, self.world.maximum)
            ia = self._extend(nodes_a, parents_a, sample)
            if ia is None:
                continue
            new_a = nodes_a[ia]
            ib = self._extend(nodes_b, parents_b, new_a)
            if ib is None:
                continue
            new_b = nodes_b[ib]
            if np.linalg.norm(new_a - new_b) <= self.config.step_size and not self.world.segment_collision(new_a, new_b, self.config.hard_clearance):
                path = self._chain(nodes_a, parents_a, ia) + self._chain(nodes_b, parents_b, ib)[::-1]
                points = np.asarray(path)
                return PathResult(points, path_length(points), iteration + 1, planner="rrtconnect",
                                  metadata={"length": path_length(points),
                                            "min_clearance": self.world.path_clearance(points),
                                            "waypoints": float(len(points)),
                                            "hard_clearance": self.config.hard_clearance})
        raise RuntimeError("RRT-Connect did not connect the two trees; increase iterations")


def plan_path(world: VoxelMap, start: Cell, goal: Cell, algorithm: str = "astar",
              weights: Optional[CostWeights] = None, diagonal: bool = True,
              max_expansions: int = 250_000, heuristic: str = "euclidean",
              safety_margin=None, other_paths=(), seed: int = 7) -> PathResult:
    """Dispatch one named algorithm without exposing backend-specific details."""
    if algorithm not in ALGORITHMS:
        raise ValueError(f"unknown algorithm {algorithm!r}; choose from {ALGORITHMS}")
    weights = weights or CostWeights()
    if algorithm == "astar":
        return AStar3D(world, weights, diagonal, max_expansions, heuristic, safety_margin).plan(start, goal, other_paths)
    if algorithm == "dijkstra":
        return AStar3D(world, weights, diagonal, max_expansions, "zero", safety_margin).plan(start, goal, other_paths)
    if algorithm == "greedy":
        return GreedyBestFirst3D(world, weights, diagonal, max_expansions).plan(start, goal, other_paths)
    if algorithm == "bidirectional":
        return BidirectionalAStar3D(world, weights, diagonal, max_expansions).plan(start, goal, other_paths)
    if algorithm == "rrtconnect":
        return RRTConnect3D(world, RRTConnectConfig(seed=seed, hard_clearance=weights.hard_clearance)).plan(
            world.grid_to_world(start), world.grid_to_world(goal), other_paths)
    from .rrtstar3d import RRTConfig, RRTStar3D
    return RRTStar3D(world, RRTConfig(seed=seed, hard_clearance=weights.hard_clearance)).plan(
        world.grid_to_world(start), world.grid_to_world(goal))


def plan_fleet_algorithm(world: VoxelMap, starts: Sequence[Cell], goals: Sequence[Cell],
                         algorithm: str = "astar", weights: Optional[CostWeights] = None,
                         diagonal: bool = True, max_expansions: int = 250_000,
                         heuristic: str = "euclidean", safety_margin=None, seed: int = 7) -> List[PathResult]:
    weights = weights or CostWeights()
    results = []
    for i, (start, goal) in enumerate(zip(starts, goals)):
        results.append(plan_path(world, start, goal, algorithm, weights, diagonal,
                                 max_expansions, heuristic, safety_margin,
                                 [r.points for r in results], seed + i))
    return results
