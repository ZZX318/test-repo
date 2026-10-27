"""Multi-objective 26-neighbour A* planner for a :class:`VoxelMap`.

Design note (performance)
-------------------------
The previous revision called analytic geometry inside the expansion loop:
``segment_collision`` stepped along every candidate edge and evaluated a signed
distance per sample, which cost 82% of the runtime.  The search below touches
only pre-computed rasters:

* ``passable`` - boolean voxel mask, inflated by ``hard_clearance + margin``;
* ``steps``    - clearance quantised to integer cell counts, used to index a
  pre-computed penalty table;
* ``sep``      - distance (in cells) to previously planned UAV paths, produced
  by one BFS instead of a per-node ``min(norm(path - point))``.

Continuous analytic geometry is still used, but only twice per plan: to build
the rasters and to validate the final path.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .config import CostWeights
from .environment import Cell, VoxelMap

__all__ = ["AStar3D", "AStarStepper3D", "CostWeights", "PathResult", "plan_fleet"]


@dataclass
class PathResult:
    """Common result object used by A* and RRT* backends."""

    points: np.ndarray
    cost: float
    expanded: int
    success: bool = True
    planner: str = "astar3d"
    metadata: Dict[str, float] = field(default_factory=dict)

    def __post_init__(self):
        self.points = np.asarray(self.points, dtype=float).reshape(-1, 3)


_OFFSETS = tuple((dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                 for dz in (-1, 0, 1) if (dx, dy, dz) != (0, 0, 0))


def _reconstruct(parent, node):
    path = [node]
    while node in parent:
        node = parent[node]
        path.append(node)
    path.reverse()
    return path


def _path_separation(point: np.ndarray, paths: Sequence[np.ndarray]) -> float:
    if not paths:
        return float("inf")
    query = np.asarray(point, dtype=float).reshape(-1, 3)
    distances = []
    for path in paths:
        path = np.asarray(path, dtype=float).reshape(-1, 3)
        if not len(path):
            continue
        distances.append(float(np.linalg.norm(query[:, None, :] - path[None, :, :], axis=-1).min()))
    return float(min(distances)) if distances else float("inf")


class _LookupGrid:
    """Padded flat arrays + pre-computed cost tables for the search inner loop."""

    def __init__(self, world: VoxelMap, mask: np.ndarray, weights: CostWeights,
                 other_paths: Sequence[np.ndarray] = (), diagonal: bool = True):
        nx, ny, nz = world.shape
        self.nx, self.ny, self.nz = nx, ny, nz
        self.ny_p, self.nz_p = ny + 2, nz + 2
        self.nyz = self.ny_p * self.nz_p
        res = world.resolution

        cap = int(np.ceil(max(weights.clearance, weights.minimum_separation, 1.0) / res)) + 2
        self.cap = cap

        padded = np.zeros((nx + 2, ny + 2, nz + 2), dtype=bool)
        padded[1:-1, 1:-1, 1:-1] = mask
        self.passable = padded.ravel().tolist()

        steps = np.zeros((nx + 2, ny + 2, nz + 2), dtype=np.int32)
        steps[1:-1, 1:-1, 1:-1] = world.clearance_steps(cap)
        self.steps = steps.ravel().tolist()

        clearance = max(weights.clearance, 1e-9)
        self.penalty = [weights.obstacle * max(0.0, (clearance - i * res) / clearance) ** 2
                        for i in range(cap + 1)]

        if other_paths:
            sep = np.full((nx + 2, ny + 2, nz + 2), cap, dtype=np.int32)
            sep[1:-1, 1:-1, 1:-1] = world.path_distance_steps(other_paths, cap)
            self.sep = sep.ravel().tolist()
            min_sep = max(weights.minimum_separation, 1e-9)
            self.sep_penalty = [weights.separation * max(0.0, (min_sep - i * res) / min_sep) ** 2
                                for i in range(cap + 1)]
        else:
            self.sep = None
            self.sep_penalty = None

        self.offsets = []
        for dx, dy, dz in _OFFSETS:
            if not diagonal and abs(dx) + abs(dy) + abs(dz) > 1:
                continue
            delta = dx * self.nyz + dy * self.nz_p + dz
            base = weights.length * res * float(np.sqrt(dx * dx + dy * dy + dz * dz))
            self.offsets.append((dx, dy, dz, delta, base))

        self.goal = (0, 0, 0)
        self.resolution = res
        self.length_weight = weights.length

    def index(self, x: int, y: int, z: int) -> int:
        return (x + 1) * self.nyz + (y + 1) * self.nz_p + (z + 1)

    def cell_of(self, index: int) -> Cell:
        x, rem = divmod(index, self.nyz)
        y, z = divmod(rem, self.nz_p)
        return (x - 1, y - 1, z - 1)

    def heuristic(self, x: int, y: int, z: int) -> float:
        gx, gy, gz = self.goal
        d = sorted((abs(x - gx), abs(y - gy), abs(z - gz)), reverse=True)
        octile = d[0] + (np.sqrt(2.0) - 1.0) * d[1] + (np.sqrt(3.0) - np.sqrt(2.0)) * d[2]
        return float(self.length_weight * self.resolution * octile)


class AStar3D:
    """A* with obstacle clearance and prioritized multi-UAV separation costs."""

    def __init__(self, world: VoxelMap, weights: Optional[CostWeights] = None,
                 diagonal: bool = True, max_expansions: int = 250_000,
                 heuristic: str = "euclidean", safety_margin: Optional[float] = None):
        self.world = world
        self.weights = weights or CostWeights()
        self.diagonal = bool(diagonal)
        self.max_expansions = int(max_expansions)
        if heuristic not in ("euclidean", "zero"):
            raise ValueError("heuristic must be 'euclidean' or 'zero'")
        self.heuristic_mode = heuristic
        self.safety_margin = (0.5 * np.sqrt(3.0) * world.resolution
                              if safety_margin is None else float(safety_margin))

    # ------------------------------------------------------------------ public
    def lookup(self, other_paths: Sequence[np.ndarray] = ()) -> _LookupGrid:
        mask = self.world.passable(self.weights.hard_clearance, self.safety_margin)
        return _LookupGrid(self.world, mask, self.weights, other_paths, self.diagonal)

    def plan(self, start: Cell, goal: Cell,
             other_paths: Optional[Sequence[np.ndarray]] = None) -> PathResult:
        start = tuple(int(v) for v in start)
        goal = tuple(int(v) for v in goal)
        other_paths = [np.asarray(p, dtype=float).reshape(-1, 3) for p in (other_paths or [])]
        if not self.world.inside(start) or not self.world.inside(goal):
            raise ValueError("start and goal must be inside the map")

        table = self.lookup(other_paths)
        if not table.passable[table.index(*start)]:
            raise ValueError("start violates the hard clearance constraint")
        if not table.passable[table.index(*goal)]:
            raise ValueError("goal violates the hard clearance constraint")

        table.goal = goal
        passable, steps, penalty = table.passable, table.steps, table.penalty
        sep, sep_penalty = table.sep, table.sep_penalty
        zero_h = self.heuristic_mode == "zero"

        start_index = table.index(*start)
        goal_index = table.index(*goal)
        g_score = {start_index: 0.0}
        parent: Dict[int, int] = {}
        h0 = 0.0 if zero_h else table.heuristic(*start)
        frontier = [(h0, 0.0, start_index, start[0], start[1], start[2])]
        expanded = 0

        while frontier:
            _, current_g, index, x, y, z = heapq.heappop(frontier)
            if current_g > g_score.get(index, float("inf")) + 1e-10:
                continue
            if index == goal_index:
                cells = [table.cell_of(i) for i in _reconstruct(parent, index)]
                points = np.stack([self.world.grid_to_world(c) for c in cells])
                return PathResult(points, float(current_g), expanded,
                                  metadata=self._metadata(points, current_g, expanded, other_paths))
            expanded += 1
            if expanded > self.max_expansions:
                raise RuntimeError("A* exceeded max_expansions; reduce map or relax costs")
            for dx, dy, dz, delta, base in table.offsets:
                nb = index + delta
                if not passable[nb]:
                    continue
                cost = current_g + base + penalty[steps[nb]]
                if sep is not None:
                    cost += sep_penalty[sep[nb]]
                if cost < g_score.get(nb, float("inf")) - 1e-10:
                    g_score[nb] = cost
                    parent[nb] = index
                    nx_, ny_, nz_ = x + dx, y + dy, z + dz
                    h = 0.0 if zero_h else table.heuristic(nx_, ny_, nz_)
                    heapq.heappush(frontier, (cost + h, cost, nb, nx_, ny_, nz_))
        raise RuntimeError("no feasible 3D path found")

    # ----------------------------------------------------------------- private
    def _metadata(self, points, cost, expanded, other_paths):
        from .geometry import path_length
        hard = self.weights.hard_clearance
        clear = self.world.path_clearance(points, step=max(self.world.resolution * 0.25, 0.1))
        sep = _path_separation(points, other_paths) if other_paths else float("inf")
        return {"length": path_length(points), "min_clearance": clear,
                "min_separation": sep, "expanded": float(expanded),
                "waypoints": float(len(points)), "cost": float(cost),
                "hard_clearance": float(hard),
                "constraint_ok": float(clear >= hard - 1e-9)}


def plan_fleet(world: VoxelMap, starts: Sequence[Cell], goals: Sequence[Cell],
               weights: Optional[CostWeights] = None, diagonal: bool = True,
               max_expansions: int = 250_000, heuristic: str = "euclidean",
               safety_margin: Optional[float] = None) -> List[PathResult]:
    """Prioritized planning: each subsequent UAV avoids earlier paths."""
    if len(starts) != len(goals):
        raise ValueError("starts and goals must have equal length")
    planner = AStar3D(world, weights, diagonal, max_expansions, heuristic, safety_margin)
    results = []
    for start, goal in zip(starts, goals):
        previous = [r.points for r in results]
        results.append(planner.plan(start, goal, previous))
    return results


class AStarStepper3D:
    """State-machine A* used by the interactive 3D teaching view.

    It shares :class:`_LookupGrid` with :class:`AStar3D`, so the numbers shown
    in the viewer (g, h, f) are exactly the numbers the real planner uses.
    """

    def __init__(self, world: VoxelMap, start: Cell, goal: Cell,
                 weights: Optional[CostWeights] = None, heuristic: str = "euclidean",
                 diagonal: bool = True, max_expansions: int = 250_000,
                 safety_margin: Optional[float] = None):
        self.world = world
        self.start = tuple(int(v) for v in start)
        self.goal = tuple(int(v) for v in goal)
        self.weights = weights or CostWeights()
        if heuristic not in ("euclidean", "zero"):
            raise ValueError("heuristic must be 'euclidean' or 'zero'")
        self.heuristic_mode = heuristic
        self.diagonal = diagonal
        self.max_expansions = int(max_expansions)
        self.safety_margin = (0.5 * np.sqrt(3.0) * world.resolution
                              if safety_margin is None else float(safety_margin))
        self.reset()

    def reset(self) -> None:
        mask = self.world.passable(self.weights.hard_clearance, self.safety_margin)
        self.table = _LookupGrid(self.world, mask, self.weights, (), self.diagonal)
        self.table.goal = self.goal
        if not self.table.passable[self.table.index(*self.start)]:
            raise ValueError("start violates the hard clearance constraint")
        if not self.table.passable[self.table.index(*self.goal)]:
            raise ValueError("goal violates the hard clearance constraint")
        start_index = self.table.index(*self.start)
        h0 = 0.0 if self.heuristic_mode == "zero" else self.table.heuristic(*self.start)
        self.frontier = [(h0, 0.0, start_index)]
        self.g = {start_index: 0.0}
        self.parent = {}
        self.open_set = {start_index}
        self.closed = set()
        self.current = None
        self.path = []
        self.expanded = 0
        self.done = False
        self.reached = False

    def set_heuristic(self, mode: str) -> None:
        """Switch between A* (``euclidean``) and Dijkstra (``zero``) live."""
        if mode not in ("euclidean", "zero"):
            raise ValueError("heuristic must be 'euclidean' or 'zero'")
        self.heuristic_mode = mode
        if self.expanded == 0:
            self.reset()
            return
        table = self.table
        zero = mode == "zero"
        rebuilt = []
        for _, cost, index in self.frontier:
            if index in self.closed:
                continue
            x, y, z = table.cell_of(index)
            h = 0.0 if zero else table.heuristic(x, y, z)
            rebuilt.append((cost + h, cost, index))
        heapq.heapify(rebuilt)
        self.frontier = rebuilt

    def node_info(self, index: int) -> Dict[str, float]:
        table = self.table
        x, y, z = table.cell_of(index)
        g = self.g.get(index, float("inf"))
        h = 0.0 if self.heuristic_mode == "zero" else table.heuristic(x, y, z)
        return {"g": g, "h": h, "f": g + h if np.isfinite(g) else float("inf"),
                "clearance": float(table.steps[index]) * self.world.resolution}

    def step(self) -> None:
        if self.done:
            return
        while self.frontier:
            _, current_g, index = heapq.heappop(self.frontier)
            if current_g > self.g.get(index, float("inf")) + 1e-10:
                continue
            break
        else:
            self.done = True
            return
        self.open_set.discard(index)
        self.closed.add(index)
        self.current = index
        if index == self.table.index(*self.goal):
            # ``expanded`` counts *expansions*, so the goal pop is not one -
            # this keeps the viewer's counter equal to AStar3D's.
            self.done = self.reached = True
            cells = [self.table.cell_of(i) for i in _reconstruct(self.parent, index)]
            self.path = [self.world.grid_to_world(c) for c in cells]
            return
        self.expanded += 1
        if self.expanded > self.max_expansions:
            self.done = True
            return
        table = self.table
        passable, steps, penalty = table.passable, table.steps, table.penalty
        zero = self.heuristic_mode == "zero"
        for dx, dy, dz, delta, base in table.offsets:
            nb = index + delta
            if not passable[nb] or nb in self.closed:
                continue
            cost = current_g + base + penalty[steps[nb]]
            if cost < self.g.get(nb, float("inf")) - 1e-10:
                self.g[nb] = cost
                self.parent[nb] = index
                self.open_set.add(nb)
                x, y, z = table.cell_of(nb)
                h = 0.0 if zero else table.heuristic(x, y, z)
                heapq.heappush(self.frontier, (cost + h, cost, nb))

    def run(self):
        while not self.done:
            self.step()
        return self

    def snapshot(self):
        return {"open": frozenset(self.open_set), "closed": frozenset(self.closed),
                "current": self.current, "path": tuple(self.path),
                "expanded": self.expanded, "done": self.done, "reached": self.reached}
