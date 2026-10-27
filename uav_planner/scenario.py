"""Reproducible 3D scenarios for demos and experiments.

The scenario builder is "generate, then *verify*, then repair" instead of the
previous "generate, then pray":

1. endpoints are drawn from the seed across the whole 3D volume, so a path can
   no longer be solved inside a single z-layer;
2. the map contains *gapped walls* - a wall spans the full y-z cross-section and
   leaves one hole whose (y, z) position is seed dependent.  A 26-neighbour
   search cannot slide past it inside the xy plane, so the path must use the z
   axis.  The obstacle budget decides how many walls exist (``n // 12``), which
   is what makes the obstacle count actually change the path length;
3. every endpoint is snapped to a cell that satisfies the hard clearance, and
   the map is flood-filled to prove each pair is connected.  If it is not, the
   last obstacle is dropped and the check repeats - no ``raise`` on a bad seed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .config import ScenarioConfig
from .environment import VoxelMap


@dataclass
class Scenario:
    world: VoxelMap
    starts: list
    goals: list
    info: dict = field(default_factory=dict)


def _sample_endpoints(world: VoxelMap, rng: np.random.Generator, n_uavs: int,
                      clearance: float, margin: float):
    """Draw ``n_uavs`` start/goal pairs spread over x, y *and* z."""
    nx, ny, nz = world.shape
    flip = bool(rng.integers(2))                      # seed decides the direction
    x_start = int(rng.integers(1, 3)) if not flip else int(rng.integers(nx - 3, nx - 1))
    x_goal = int(rng.integers(nx - 3, nx - 1)) if not flip else int(rng.integers(1, 3))

    # Split the y axis into n_uavs lanes and jitter inside each lane, so the
    # fleet starts spread out instead of on one line.
    lane = (ny - 4) / max(n_uavs, 1)
    ys_start = [int(np.clip(2 + lane * (i + 0.5) + rng.uniform(-0.35, 0.35) * lane,
                            1, ny - 2)) for i in range(n_uavs)]
    ys_goal = [int(np.clip(2 + lane * (i + 0.5) + rng.uniform(-0.35, 0.35) * lane,
                           1, ny - 2)) for i in range(n_uavs)]
    zs_start = [int(rng.integers(1, nz - 1)) for _ in range(n_uavs)]
    zs_goal = [int(rng.integers(1, nz - 1)) for _ in range(n_uavs)]

    starts, goals = [], []
    for i in range(n_uavs):
        s = world.nearest_passable(world.grid_to_world((x_start, ys_start[i], zs_start[i])),
                                   clearance, margin)
        g = world.nearest_passable(world.grid_to_world((x_goal, ys_goal[i], zs_goal[i])),
                                   clearance, margin)
        starts.append(world.grid_to_world(s))
        goals.append(world.grid_to_world(g))
    return starts, goals


def _add_gapped_wall(world: VoxelMap, x0: int, gap_y: int, gap_z: int,
                     gap_w: int, gap_h: int):
    """Four boxes leaving exactly one rectangular hole in a y-z wall.

    Returns the ``(min, max)`` box of the hole so the clutter generator can keep
    it open.
    """
    nx, ny, nz = world.shape
    x0 = int(np.clip(x0, 2, nx - 3))
    gap_y = int(np.clip(gap_y, 2, max(2, ny - gap_w - 2)))
    gap_z = int(np.clip(gap_z, 1, max(1, nz - gap_h - 2)))
    gap_w = int(min(gap_w, ny - gap_y - 1))
    gap_h = int(min(gap_h, nz - gap_z - 1))
    if gap_w < 3 or gap_h < 3:
        return None
    world.add_box((x0, 0, 0), (x0 + 1, gap_y, nz - 1))
    world.add_box((x0, gap_y + gap_w, 0), (x0 + 1, ny - 1, nz - 1))
    world.add_box((x0, gap_y, 0), (x0 + 1, gap_y + gap_w, gap_z))
    world.add_box((x0, gap_y, gap_z + gap_h), (x0 + 1, gap_y + gap_w, nz - 1))
    origin = world.origin
    return (origin + np.array([x0 - 1, gap_y - 1, gap_z - 1]) * world.resolution,
            origin + np.array([x0 + 2, gap_y + gap_w + 1, gap_z + gap_h + 1]) * world.resolution)


def _build_walls(world: VoxelMap, rng: np.random.Generator, n_walls: int):
    nx, ny, nz = world.shape
    gap_w = max(4, min(7, ny // 5))
    gap_h = max(4, min(6, nz // 3))
    positions = np.linspace(0.30, 0.78, max(n_walls, 1))
    gaps = []
    for i in range(n_walls):
        x0 = int(round(positions[i] * nx))
        gap_y = int(rng.integers(2, max(3, ny - gap_w - 1)))
        gap_z = int(rng.integers(1, max(2, nz - gap_h)))
        region = _add_gapped_wall(world, x0, gap_y, gap_z, gap_w, gap_h)
        if region is not None:
            gaps.append(region)
    return gaps


def _finish(world: VoxelMap, starts, goals, config: ScenarioConfig, layout: str,
            repairs: int) -> Scenario:
    clearance = float(config.planner.hard_clearance)
    margin = (0.5 * np.sqrt(3.0) * config.planner.resolution
              if config.planner.safety_margin is None else float(config.planner.safety_margin))

    while repairs <= config.max_repairs and world.obstacles:
        if all(world.reachable(s, g, clearance, margin) for s, g in zip(starts, goals)):
            break
        world.remove_obstacle()
        repairs += 1

    if world.obstacles and not all(world.reachable(s, g, clearance, margin)
                                   for s, g in zip(starts, goals)):
        # Last resort: keep the endpoints, clear the field.  Never raise.
        world.clear_obstacles()

    info = {"seed": config.seed, "obstacles": len(world.obstacles),
            "requested_obstacles": config.n_obstacles, "repairs": repairs,
            "occupancy": float(world.occupied.mean()), "clearance": clearance,
            "margin": margin, "layout": layout}
    return Scenario(world, starts, goals, info)


def _make(config: ScenarioConfig, layout: str) -> Scenario:
    planner_cfg = config.planner
    rng = np.random.default_rng(config.seed)
    world = VoxelMap(planner_cfg.shape, planner_cfg.resolution)
    clearance = float(planner_cfg.hard_clearance)
    margin = (0.5 * np.sqrt(3.0) * planner_cfg.resolution
              if planner_cfg.safety_margin is None else float(planner_cfg.safety_margin))

    starts, goals = _sample_endpoints(world, rng, config.n_uavs, clearance, margin)

    walls = 0
    gaps = []
    if layout in ("mixed", "walls"):
        if layout == "walls":
            n_walls = max(1, config.n_obstacles // 8)
        elif config.n_walls is not None:
            n_walls = int(config.n_walls)
        else:
            n_walls = config.n_obstacles // 12
        gaps = _build_walls(world, rng, n_walls)
        walls = len(world.obstacles)

    remaining = max(config.n_obstacles - walls, 0) if layout != "walls" else 0
    if remaining > 0:
        world.add_random_obstacles(
            remaining, rng,
            radius_range=config.obstacle_radius,
            avoid_points=starts + goals,
            corridors=list(zip(starts, goals)),
            corridor_ratio=config.corridor_ratio,
            corridor_offset=config.corridor_offset,
            avoid_regions=gaps,
        )
    return _finish(world, starts, goals, config, layout, 0)


def make_demo(config: Optional[ScenarioConfig] = None) -> Scenario:
    """Default scenario: gapped walls plus corridor-biased clutter."""
    config = config or ScenarioConfig()
    return _make(config, "mixed")


def make_wall_demo(config: Optional[ScenarioConfig] = None) -> Scenario:
    """Walls only - the cleanest way to see the z axis being used."""
    config = config or ScenarioConfig()
    return _make(config, "walls")


def make_sphere_demo(config: Optional[ScenarioConfig] = None) -> Scenario:
    """Random spheres only - the old behaviour, kept for comparison."""
    config = config or ScenarioConfig()
    return _make(config, "spheres")


def _make_fixed(config: ScenarioConfig, layout: str) -> Scenario:
    """Build named benchmark layouts with deterministic endpoints."""
    planner_cfg = config.planner
    rng = np.random.default_rng(config.seed)
    world = VoxelMap(planner_cfg.shape, planner_cfg.resolution)
    clearance = float(planner_cfg.hard_clearance)
    margin = (0.5 * np.sqrt(3.0) * planner_cfg.resolution
              if planner_cfg.safety_margin is None else float(planner_cfg.safety_margin))
    starts, goals = _sample_endpoints(world, rng, config.n_uavs, clearance, margin)
    avoid_regions = []
    nx, ny, nz = world.shape

    if layout == "reference":
        # Geometry adapted from refs/PathPlanning/Search_3D/env3D.py.  It is
        # deliberately kept as a named benchmark instead of being hidden in
        # the random generator, so results can be compared with the upstream
        # implementation.
        blocks = [
            ((4, 12, 0), (5, 20, min(5, nz - 1))),
            ((5.5, 12, 0), (10, 13, min(5, nz - 1))),
            ((10, 12, 0), (14, 13, min(5, nz - 1))),
            ((10, 9, 0), (20, 10, min(5, nz - 1))),
            ((9, 6, 0), (10, 10, min(5, nz - 1))),
        ]
        for lo, hi in blocks:
            world.add_box(lo, hi)
        world.add_sphere((2, 6, 2.5), 1.0)
        world.add_sphere((14, 14, 2.5), 2.0)
    elif layout == "urban":
        # Alternating building blocks leave narrow streets and force vertical
        # choices.  Heights vary with the seed, making the same map family
        # useful for ablations.
        for x in range(7, max(8, nx - 5), 7):
            for y in range(5, max(6, ny - 5), 8):
                h = int(rng.integers(max(4, nz // 3), max(5, nz - 2)))
                width = int(rng.integers(2, 4))
                world.add_box((x, y, 0), (min(x + width, nx - 2), min(y + 5, ny - 2), h))
    elif layout == "forest":
        world.add_random_obstacles(
            max(config.n_obstacles, 18), rng, radius_range=(1.0, 2.2),
            avoid_points=starts + goals,
            corridors=list(zip(starts, goals)), corridor_ratio=0.75,
            corridor_offset=3.5)
    elif layout == "maze":
        # Several alternating slabs create a 3D corridor maze.
        for x in range(6, nx - 4, 6):
            if (x // 6) % 2:
                world.add_box((x, 2, 0), (x + 1, ny - 8, nz - 2))
                world.add_box((x, ny - 5, 0), (x + 1, ny - 2, nz - 5))
            else:
                world.add_box((x, 8, 0), (x + 1, ny - 2, nz - 2))
                world.add_box((x, 2, 0), (x + 1, 5, nz - 5))

    # Fill leftover budget with small clutter, while protecting the designed
    # corridors and endpoints.  Fixed layouts remain deterministic.
    used = len(world.obstacles)
    remaining = max(config.n_obstacles - used, 0)
    if remaining:
        world.add_random_obstacles(remaining, rng, radius_range=config.obstacle_radius,
                                   avoid_points=starts + goals,
                                   corridors=list(zip(starts, goals)),
                                   corridor_ratio=config.corridor_ratio,
                                   corridor_offset=config.corridor_offset,
                                   avoid_regions=avoid_regions)
    return _finish(world, starts, goals, config, layout, 0)


def make_urban_demo(config: Optional[ScenarioConfig] = None) -> Scenario:
    return _make_fixed(config or ScenarioConfig(), "urban")


def make_forest_demo(config: Optional[ScenarioConfig] = None) -> Scenario:
    return _make_fixed(config or ScenarioConfig(), "forest")


def make_maze_demo(config: Optional[ScenarioConfig] = None) -> Scenario:
    return _make_fixed(config or ScenarioConfig(), "maze")


def make_reference_demo(config: Optional[ScenarioConfig] = None) -> Scenario:
    return _make_fixed(config or ScenarioConfig(), "reference")


SCENARIO_LAYOUTS = ("mixed", "walls", "spheres", "urban", "forest", "maze", "reference")
