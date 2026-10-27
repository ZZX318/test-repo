"""Regression tests for the review findings.

Each test maps to one line of the self-check list in
``批改_uav_3d_planner.md`` section 5, so a regression shows up as a red test
instead of a surprise in the report.
"""

import time

import numpy as np

from uav_planner.astar3d import AStar3D, AStarStepper3D, CostWeights
from uav_planner.config import PlannerConfig, ScenarioConfig
from uav_planner.dynamics import KinematicModel
from uav_planner.environment import VoxelMap
from uav_planner.fleet import FleetPlanner
from uav_planner.metrics import evaluate_plan
from uav_planner.scenario import make_demo


def _run(seed=7, obstacles=12, uavs=3, planner=None):
    cfg = ScenarioConfig(seed=seed, n_uavs=uavs, n_obstacles=obstacles,
                         planner=planner or PlannerConfig())
    scenario = make_demo(cfg)
    fleet = FleetPlanner.from_config(scenario.world, cfg.planner)
    plan = fleet.plan([scenario.world.world_to_grid(p) for p in scenario.starts],
                      [scenario.world.world_to_grid(p) for p in scenario.goals])
    fleet.rollout(plan, scenario.goals, KinematicModel(),
                  hard_clearance=cfg.planner.hard_clearance)
    report = evaluate_plan(plan, scenario.goals, scenario.world,
                           hard_clearance=cfg.planner.hard_clearance,
                           minimum_separation=cfg.planner.minimum_separation)
    return scenario, plan, report


def test_obstacle_count_changes_path_length():
    """0 / 12 / 40 obstacles must not all give the same 31.33 m."""
    lengths = [_run(obstacles=n)[2]["plan"]["mean_path_length"] for n in (0, 12, 40)]
    assert lengths[0] < lengths[1] < lengths[2], lengths
    assert lengths[1] - lengths[0] > 1.0
    assert lengths[2] - lengths[1] > 1.0


def test_at_least_one_uav_makes_a_real_detour():
    _, plan, report = _run(seed=7)
    waypoints = [len(p) for p in plan.paths]
    assert max(waypoints) > 10, waypoints
    assert report["plan"]["max_path_ratio"] > 1.15, report["plan"]["max_path_ratio"]


def test_same_seed_is_bit_identical():
    _, _, first = _run(seed=7)
    _, _, second = _run(seed=7)
    assert first["plan"]["mean_path_length"] == second["plan"]["mean_path_length"]
    assert first["plan"]["min_clearance"] == second["plan"]["min_clearance"]
    assert first["execution"]["min_clearance"] == second["execution"]["min_clearance"]


def test_no_seed_raises():
    """Feasibility is guaranteed by resampling, never by ``raise``."""
    for seed in range(6):
        scenario, plan, report = _run(seed=seed)
        assert report["plan"]["success_rate"] == 1.0
        assert scenario.world.obstacles, "obstacle field should not be emptied"


def test_reported_clearance_honours_hard_constraint():
    for seed in (1, 3, 7):
        cfg = PlannerConfig(hard_clearance=0.35)
        _, _, report = _run(seed=seed, planner=cfg)
        assert report["plan"]["min_clearance"] >= cfg.hard_clearance - 1e-9
        assert report["execution"]["min_clearance"] >= cfg.hard_clearance - 1e-9
        assert report["plan"]["clearance_violation"] == 0.0
        assert report["execution"]["clearance_violation"] == 0.0


def test_single_plan_is_fast():
    scenario, _, _ = _run(seed=7, uavs=1)
    planner = AStar3D(scenario.world)
    start = scenario.world.world_to_grid(scenario.starts[0])
    goal = scenario.world.world_to_grid(scenario.goals[0])
    t0 = time.perf_counter()
    planner.plan(start, goal)
    assert time.perf_counter() - t0 < 1.0


def test_passable_mask_is_conservative():
    """Every passable cell must be at least ``hard_clearance`` from an obstacle."""
    scenario, _, _ = _run(seed=5, uavs=1)
    world, hard = scenario.world, 0.35
    mask = world.passable(hard)
    cells = np.argwhere(mask)
    assert len(cells) > 0
    picks = cells[:: max(1, len(cells) // 200)]
    for cell in picks:
        assert world.clearance(world.grid_to_world(tuple(cell))) >= hard - 1e-9


def test_stepper_reports_the_same_numbers_as_the_planner():
    scenario, _, _ = _run(seed=7, uavs=1)
    world = scenario.world
    start = world.world_to_grid(scenario.starts[0])
    goal = world.world_to_grid(scenario.goals[0])
    result = AStar3D(world).plan(start, goal)
    stepper = AStarStepper3D(world, start, goal).run()
    assert stepper.reached
    assert stepper.expanded == result.expanded
    assert abs(stepper.g[stepper.table.index(*goal)] - result.cost) < 1e-6
    info = stepper.node_info(stepper.table.index(*goal))
    assert abs(info["f"] - (info["g"] + info["h"])) < 1e-9


def test_dijkstra_and_astar_agree_on_cost():
    scenario, _, _ = _run(seed=7, uavs=1)
    world = scenario.world
    start = world.world_to_grid(scenario.starts[0])
    goal = world.world_to_grid(scenario.goals[0])
    astar = AStar3D(world, heuristic="euclidean").plan(start, goal)
    dijkstra = AStar3D(world, heuristic="zero").plan(start, goal)
    assert abs(astar.cost - dijkstra.cost) < 1e-6
    assert astar.expanded <= dijkstra.expanded


def test_weights_are_read_from_config():
    """``PlannerConfig`` used to define seven fields nobody read."""
    cfg = PlannerConfig(hard_clearance=0.8, obstacle_weight=20.0, clearance=3.0,
                        minimum_separation=4.0, max_expansions=5000,
                        diagonal_cost=False, safety_margin=0.5)
    weights = cfg.to_weights()
    assert weights.hard_clearance == 0.8
    assert weights.obstacle == 20.0
    assert weights.clearance == 3.0
    assert weights.minimum_separation == 4.0
    world = VoxelMap(cfg.shape, cfg.resolution)
    world.add_box((10, 10, 0), (14, 14, 9))
    planner = FleetPlanner.from_config(world, cfg)
    assert planner.max_expansions == 5000
    assert planner.diagonal is False
    assert planner.safety_margin == 0.5
    assert planner.weights.hard_clearance == 0.8


def test_tight_clearance_shrinks_the_passable_area():
    world = VoxelMap((16, 16, 10), 1.0)
    world.add_sphere((8, 8, 5), 2.0)
    loose = int(world.passable(0.0).sum())
    tight = int(world.passable(1.5).sum())
    assert tight < loose
