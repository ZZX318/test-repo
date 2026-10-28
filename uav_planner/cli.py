"""Command-line experiment runner.

Everything that used to be hard-coded in the source is now an argument:
weights, heuristic, diagonal moves, safety margin, expansion budget, obstacle
count, layout.  Three extra modes exist because a static plot is not a tool:

``--step``     interactive A* viewer (next step / reset / h toggle / g, h, f)
``--compare``  A* vs Dijkstra on the identical map
``--sweep``    obstacle-count sweep, the table that exposed the flat map

Examples
--------
python -m uav_planner.cli --planner astar --seed 7 --uavs 3
python -m uav_planner.cli --weights "obstacle=12,clearance=3" --heuristic zero
python -m uav_planner.cli --sweep 0,12,40
python -m uav_planner.cli --step --seed 7
"""

from __future__ import annotations

import argparse
import json
import time
from typing import Dict, List, Tuple

import numpy as np

from .astar3d import AStar3D, AStarStepper3D, CostWeights, plan_fleet
from .algorithms import ALGORITHMS, GRAPH_ALGORITHMS, plan_fleet_algorithm
from .config import PlannerConfig, ScenarioConfig
from .dynamics import KinematicModel
from .fleet import FleetPlan, FleetPlanner
from .metrics import evaluate, evaluate_plan, format_metrics, format_report
from .rrtstar3d import RRTConfig, RRTStar3D
from .scenario import (SCENARIO_LAYOUTS, Scenario, make_demo, make_forest_demo,
                       make_maze_demo, make_reference_demo, make_sphere_demo,
                       make_urban_demo, make_wall_demo)
from .smoothing import smooth_collision_aware

WEIGHT_FIELDS = {
    "length": "length",
    "obstacle": "obstacle",
    "separation": "separation",
    "clearance": "clearance",
    "min_sep": "minimum_separation",
    "minimum_separation": "minimum_separation",
    "hard": "hard_clearance",
    "hard_clearance": "hard_clearance",
}


# --------------------------------------------------------------------- helpers
def parse_weights(text: str, base: CostWeights) -> CostWeights:
    """``"obstacle=12,hard=0.5"`` -> updated :class:`CostWeights`."""
    changes: Dict[str, float] = {}
    for chunk in text.replace(";", ",").split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if "=" not in chunk:
            raise SystemExit(f"--weights entry '{chunk}' must look like key=value")
        key, value = (part.strip() for part in chunk.split("=", 1))
        if key not in WEIGHT_FIELDS:
            raise SystemExit(f"unknown weight '{key}'; choose from {sorted(WEIGHT_FIELDS)}")
        changes[WEIGHT_FIELDS[key]] = float(value)
    return base.replace(**changes)


def build_planner_config(args) -> PlannerConfig:
    cfg = PlannerConfig(
        resolution=args.resolution,
        shape=tuple(args.shape),
        max_expansions=args.max_expansions,
        diagonal_cost=not args.no_diagonal,
        heuristic=args.heuristic,
        safety_margin=args.margin,
    )
    weights = parse_weights(args.weights, cfg.to_weights()) if args.weights else cfg.to_weights()
    overrides = {
        "length": args.length_weight,
        "obstacle": args.obstacle_weight,
        "separation": args.separation_weight,
        "clearance": args.clearance,
        "minimum_separation": args.min_separation,
        "hard_clearance": args.hard_clearance,
    }
    weights = weights.replace(**{k: float(v) for k, v in overrides.items() if v is not None})
    cfg.length_weight = weights.length
    cfg.obstacle_weight = weights.obstacle
    cfg.separation_weight = weights.separation
    cfg.clearance = weights.clearance
    cfg.minimum_separation = weights.minimum_separation
    cfg.hard_clearance = weights.hard_clearance
    return cfg


def build_scenario_config(args, n_obstacles: int | None = None) -> ScenarioConfig:
    planner = build_planner_config(args)
    return ScenarioConfig(seed=args.seed, n_uavs=args.uavs,
                          n_obstacles=args.obstacles if n_obstacles is None else n_obstacles,
                          planner=planner)


def build_scenario(cfg: ScenarioConfig, layout: str) -> Scenario:
    if layout == "walls":
        return make_wall_demo(cfg)
    if layout == "spheres":
        return make_sphere_demo(cfg)
    if layout == "urban":
        return make_urban_demo(cfg)
    if layout == "forest":
        return make_forest_demo(cfg)
    if layout == "maze":
        return make_maze_demo(cfg)
    if layout == "reference":
        return make_reference_demo(cfg)
    return make_demo(cfg)


def plan_fleet_for(args, scenario: Scenario, cfg: PlannerConfig):
    """Run the selected backend and return ``(fleet_plan, timings)``."""
    weights = cfg.to_weights()
    starts = [scenario.world.world_to_grid(p) for p in scenario.starts]
    goals = [scenario.world.world_to_grid(p) for p in scenario.goals]
    timings: Dict[str, float] = {}
    if args.planner == "astar":
        fleet = FleetPlanner.from_config(scenario.world, cfg)
        t0 = time.perf_counter()
        plan = fleet.plan(starts, goals, smooth=not args.no_smooth)
        timings["planning"] = time.perf_counter() - t0
    elif args.planner in GRAPH_ALGORITHMS or args.planner == "rrtconnect":
        t0 = time.perf_counter()
        results = plan_fleet_algorithm(
            scenario.world,
            [scenario.world.world_to_grid(p) for p in scenario.starts],
            [scenario.world.world_to_grid(p) for p in scenario.goals],
            algorithm=args.planner, weights=weights,
            diagonal=not args.no_diagonal, max_expansions=args.max_expansions,
            heuristic=args.heuristic, safety_margin=args.margin, seed=args.seed)
        paths = [smooth_collision_aware(r.points, scenario.world,
                                         safety_radius=weights.hard_clearance)
                 if not args.no_smooth else r.points.copy() for r in results]
        timings["planning"] = time.perf_counter() - t0
        plan = FleetPlan(paths, results)
        fleet = FleetPlanner(scenario.world, weights)
    else:
        planner = RRTStar3D(scenario.world, RRTConfig(seed=args.seed,
                                                      hard_clearance=weights.hard_clearance))
        t0 = time.perf_counter()
        results = [planner.plan(s, g) for s, g in zip(scenario.starts, scenario.goals)]
        paths = [smooth_collision_aware(r.points, scenario.world,
                                       safety_radius=weights.hard_clearance) for r in results]
        timings["planning"] = time.perf_counter() - t0
        plan = FleetPlan(paths, results)
        fleet = FleetPlanner(scenario.world, weights)
    if plan.trajectories is None:
        t0 = time.perf_counter()
        fleet.rollout(plan, scenario.goals, KinematicModel(),
                      hard_clearance=weights.hard_clearance)
        timings["rollout"] = time.perf_counter() - t0
    return plan, timings


def _json_safe(value):
    """JSON has no Infinity; report unbounded metrics as null instead."""
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, (np.floating, float)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, (np.integer,)):
        return int(value)
    return value


def _describe(scenario: Scenario, plan, report, timings, args) -> str:
    info = scenario.info
    lines = [f"planner={args.planner}, uavs={args.uavs}, "
             f"obstacles={info.get('obstacles')}/{info.get('requested_obstacles')} "
             f"(repairs={info.get('repairs')}), seed={args.seed}, "
             f"occupancy={info.get('occupancy', 0.0):.1%}",
             f"build={timings.get('build', 0.0):.2f}s "
             f"plan={timings.get('planning', 0.0):.2f}s "
             f"rollout={timings.get('rollout', 0.0):.2f}s"]
    lines.append("")
    lines.append(format_report(report))
    lines.append("")
    for i, result in enumerate(plan.results):
        meta = result.metadata
        lines.append(f"UAV {i}: {result.planner}, waypoints={len(plan.paths[i])}, "
                     f"cost={result.cost:.2f}, expanded={result.expanded}, "
                     f"min_clearance={meta.get('min_clearance', float('nan')):.2f} m")
    return "\n".join(lines)


# ------------------------------------------------------------------ subcommands
def cmd_run(args) -> int:
    cfg = build_scenario_config(args)
    t0 = time.perf_counter()
    scenario = build_scenario(cfg, args.layout)
    build = time.perf_counter() - t0
    plan, timings = plan_fleet_for(args, scenario, cfg.planner)
    timings["build"] = build
    report = evaluate_plan(plan, scenario.goals, scenario.world,
                           hard_clearance=cfg.planner.hard_clearance,
                           minimum_separation=cfg.planner.minimum_separation)
    if args.json:
        payload = {"scenario": scenario.info, "timings": timings, "metrics": report}
        print(json.dumps(_json_safe(payload), indent=2))
    else:
        print(_describe(scenario, plan, report, timings, args))
    if args.plot:
        from .visualization import plot_scenario
        plot_scenario(scenario.world, plan.paths, scenario.starts, scenario.goals)
    if args.animate:
        from .visualization import animate_fleet
        animate_fleet(scenario.world, plan.paths, scenario.trajectories,
                      scenario.starts, scenario.goals)
    return 0


def cmd_lab(args) -> int:
    """Open the keyboard-driven map/algorithm laboratory."""
    cfg = build_scenario_config(args)
    from .visualization import interactive_lab
    interactive_lab(cfg, layouts=SCENARIO_LAYOUTS)
    return 0


def cmd_sweep(args) -> int:
    counts: List[int] = [int(v) for v in args.sweep.split(",") if v.strip()]
    print(f"{'obstacles':>10} {'occ':>6} {'mean_len':>9} {'ratio':>6} {'max_ratio':>10} "
          f"{'min_clr':>8} {'ok':>4} {'build':>7} {'plan':>7} {'expanded':>9}")
    for count in counts:
        cfg = build_scenario_config(args, count)
        t0 = time.perf_counter()
        scenario = build_scenario(cfg, args.layout)
        build = time.perf_counter() - t0
        plan, timings = plan_fleet_for(args, scenario, cfg.planner)
        report = evaluate_plan(plan, scenario.goals, scenario.world,
                               hard_clearance=cfg.planner.hard_clearance)
        layer = report["plan"]
        expanded = sum(r.expanded for r in plan.results)
        ok = "OK" if not layer["clearance_violation"] else "FAIL"
        print(f"{scenario.info['obstacles']:>10} {scenario.info['occupancy']:>5.1%} "
              f"{layer['mean_path_length']:>9.2f} {layer['mean_path_ratio']:>6.2f} "
              f"{layer['max_path_ratio']:>10.2f} {layer['min_clearance']:>8.2f} {ok:>4} "
              f"{build:>6.2f}s {timings['planning']:>6.2f}s {expanded:>9}")
    return 0


def cmd_compare(args) -> int:
    """A* vs Dijkstra (``h = 0``) on the same map - the h toggle, in numbers.

    Only UAV 0 is compared: in prioritized planning the later UAVs' costs
    depend on the earlier paths, so a fleet-level cost comparison between two
    heuristics would be apples to oranges.
    """
    cfg = build_scenario_config(args)
    scenario = build_scenario(cfg, args.layout)
    weights = cfg.planner.to_weights()
    starts = [scenario.world.world_to_grid(scenario.starts[0])]
    goals = [scenario.world.world_to_grid(scenario.goals[0])]
    print(f"{'heuristic':>12} {'expanded':>9} {'cost':>9} {'length':>9} {'time':>8} {'optimal':>8}")
    best_cost = None
    for mode in ("euclidean", "zero"):
        t0 = time.perf_counter()
        results = plan_fleet(scenario.world, starts, goals, weights,
                             diagonal=not args.no_diagonal,
                             max_expansions=args.max_expansions,
                             heuristic=mode, safety_margin=args.margin)
        elapsed = time.perf_counter() - t0
        expanded = sum(r.expanded for r in results)
        cost = float(np.mean([r.cost for r in results]))
        length = float(np.mean([r.metadata["length"] for r in results]))
        if best_cost is None:
            best_cost = cost
        optimal = "-" if mode == "zero" else ("yes" if abs(cost - best_cost) < 1e-6 else "no")
        print(f"{mode:>12} {expanded:>9} {cost:>9.2f} {length:>9.2f} {elapsed:>7.2f}s {optimal:>8}")
    print("note: the objective is total cost (length + clearance/separation penalties), "
          "so equal cost does not imply equal geometric length.")
    return 0


def cmd_step(args) -> int:
    cfg = build_scenario_config(args)
    scenario = build_scenario(cfg, args.layout)
    weights = cfg.planner.to_weights()
    start = scenario.world.world_to_grid(scenario.starts[0])
    goal = scenario.world.world_to_grid(scenario.goals[0])
    from .visualization import interactive_astar
    interactive_astar(scenario.world, start, goal, weights,
                      heuristic=cfg.planner.heuristic,
                      diagonal=not args.no_diagonal,
                      safety_margin=cfg.planner.safety_margin)
    return 0


# --------------------------------------------------------------------- parser
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="3D multi-UAV path planning demo")
    p.add_argument("--planner", choices=ALGORITHMS, default="astar")
    p.add_argument("--layout", choices=SCENARIO_LAYOUTS, default="mixed")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--uavs", type=int, default=1)
    p.add_argument("--obstacles", type=int, default=12)
    p.add_argument("--resolution", type=float, default=1.0)
    p.add_argument("--shape", type=int, nargs=3, default=(32, 32, 14),
                   metavar=("NX", "NY", "NZ"))
    p.add_argument("--margin", type=float, default=None,
                   help="extra inflation of the passable mask (default: half a cell diagonal)")
    p.add_argument("--max-expansions", type=int, default=250_000)
    p.add_argument("--heuristic", choices=("euclidean", "zero"), default="euclidean")
    p.add_argument("--no-diagonal", action="store_true", help="6-neighbour search")
    p.add_argument("--no-smooth", action="store_true", help="skip shortcut + Chaikin")

    w = p.add_argument_group("cost weights (override PlannerConfig defaults)")
    w.add_argument("--weights", default=None, help='e.g. "obstacle=12,clearance=3,hard=0.5"')
    w.add_argument("--length-weight", type=float, default=None)
    w.add_argument("--obstacle-weight", type=float, default=None)
    w.add_argument("--separation-weight", type=float, default=None)
    w.add_argument("--clearance", type=float, default=None, help="soft clearance radius")
    w.add_argument("--min-separation", type=float, default=None)
    w.add_argument("--hard-clearance", type=float, default=None, help="hard safety constraint")

    p.add_argument("--plot", action="store_true", help="static 3D view of the paths")
    p.add_argument("--lab", action="store_true",
                   help="keyboard laboratory: m switches maps, p switches algorithms")
    p.add_argument("--dashboard", action="store_true",
                   help="open the browser based experiment cockpit")
    p.add_argument("--port", type=int, default=8765,
                   help="local dashboard port (default: 8765)")
    p.add_argument("--step", action="store_true", help="interactive A* viewer")
    p.add_argument("--animate", action="store_true", help="replay the executed trajectories")
    p.add_argument("--compare", action="store_true", help="A* vs Dijkstra")
    p.add_argument("--sweep", default=None, help="obstacle counts, e.g. 0,12,40")
    p.add_argument("--json", action="store_true", help="print metrics as JSON")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.sweep:
        return cmd_sweep(args)
    if args.lab:
        return cmd_lab(args)
    if args.dashboard:
        from .dashboard import serve
        return serve(port=args.port)
    if args.compare:
        return cmd_compare(args)
    if args.step:
        return cmd_step(args)
    return cmd_run(args)


if __name__ == "__main__":
    raise SystemExit(main())
