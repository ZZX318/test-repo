"""Browser based experiment cockpit for the UAV planner.

The project originally exposed several Matplotlib windows with different
controls. This module provides one stable entry point instead: the Python
planner remains the source of truth and a dependency free HTML canvas is used
for the visual layer.
"""

from __future__ import annotations

import copy
import json
import threading
import webbrowser
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from urllib.parse import parse_qs, urlparse

import numpy as np

from .algorithms import ALGORITHMS
from .astar3d import AStarStepper3D
from .cli import build_planner_config, build_scenario, plan_fleet_for
from .config import ScenarioConfig
from .metrics import evaluate_plan
from .scenario import SCENARIO_LAYOUTS
from .geometry import Box, Cylinder, Sphere

ROOT = Path(__file__).resolve().parent.parent
HTML_PATH = ROOT / "dashboard.html"
WEB_PATH = Path(__file__).resolve().parent / "web"
HTML_BODY = HTML_PATH.read_bytes()
VIEWER_BODY = (WEB_PATH / "viewer.js").read_bytes()
PLAN_LOCK = threading.Lock()
PLAN_RUN_LOCK = threading.Lock()
PLAN_CACHE: OrderedDict[tuple, dict] = OrderedDict()
PLAN_CACHE_LIMIT = 16


def _args(query: dict[str, list[str]]) -> SimpleNamespace:
    """Convert browser query parameters to the CLI's shared argument shape."""
    def value(name, default, cast=str):
        raw = query.get(name, [default])[0]
        try:
            return cast(raw)
        except (TypeError, ValueError):
            return cast(default)

    return SimpleNamespace(
        planner=value("planner", "astar"), layout=value("layout", "mixed"),
        seed=value("seed", 7, int), uavs=max(1, value("uavs", 1, int)),
        obstacles=max(0, value("obstacles", 12, int)),
        resolution=value("resolution", 1.0, float),
        shape=tuple(int(v) for v in value("shape", "32x32x14").lower().replace("×", "x").split("x")),
        margin=None, max_expansions=value("max_expansions", 250_000, int),
        heuristic=value("heuristic", "euclidean"),
        no_diagonal=value("diagonal", "1") in ("0", "false", "no"),
        no_smooth=value("smooth", "1") in ("0", "false", "no"), weights=None,
        length_weight=None, obstacle_weight=None, separation_weight=None,
        clearance=None, min_separation=None,
        hard_clearance=value("hard", 0.35, float),
        json=False, plot=False, animate=False, lab=False, step=False,
    )


def _safe(value):
    if isinstance(value, dict):
        return {str(k): _safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return [_safe(v) for v in value.tolist()]
    if isinstance(value, (np.floating, float)):
        return None if not np.isfinite(value) else float(value)
    if isinstance(value, np.integer):
        return int(value)
    return value


def _obstacle_geometry(scenario) -> dict:
    """Point cloud + analytic primitives shared by plan and step payloads."""
    occupied = np.argwhere(scenario.world.occupied)
    stride = max(1, int(np.ceil(len(occupied) / 4500)))
    points = [scenario.world.grid_to_world(tuple(c)).tolist()
              for c in occupied[::stride]]
    geometry = []
    for obstacle in scenario.world.obstacles:
        if isinstance(obstacle, Sphere):
            geometry.append({"type": "sphere", "center": obstacle.center.tolist(),
                             "radius": obstacle.radius})
        elif isinstance(obstacle, Box):
            geometry.append({"type": "box", "minimum": obstacle.minimum.tolist(),
                             "maximum": obstacle.maximum.tolist()})
        elif isinstance(obstacle, Cylinder):
            geometry.append({"type": "cylinder", "center_xy": obstacle.center_xy.tolist(),
                             "z_min": obstacle.z_min, "z_max": obstacle.z_max,
                             "radius": obstacle.radius})
    return {"points": points, "geometry": geometry}


def plan_payload(query: dict[str, list[str]]) -> dict:
    args = _args(query)
    if args.planner not in ALGORITHMS:
        args.planner = "astar"
    if args.layout not in SCENARIO_LAYOUTS:
        args.layout = "mixed"
    key = tuple(sorted((k, tuple(v)) for k, v in query.items()))
    with PLAN_LOCK:
        cached = PLAN_CACHE.get(key)
        if cached is not None:
            PLAN_CACHE.move_to_end(key)
            return cached
    with PLAN_RUN_LOCK:
        return _plan_payload_uncached(args, key)


def _plan_payload_uncached(args, key) -> dict:
    cfg = ScenarioConfig(seed=args.seed, n_uavs=args.uavs, n_obstacles=args.obstacles,
                         planner=build_planner_config(args))
    scenario = build_scenario(cfg, args.layout)
    plan, timings = plan_fleet_for(args, scenario, cfg.planner)
    report = evaluate_plan(plan, scenario.goals, scenario.world,
                           hard_clearance=cfg.planner.hard_clearance,
                           minimum_separation=cfg.planner.minimum_separation)
    obstacle_view = _obstacle_geometry(scenario)
    obstacle_points = obstacle_view["points"]
    geometry = obstacle_view["geometry"]
    payload = {
        "scenario": scenario.info,
        "bounds": {"origin": scenario.world.origin.tolist(),
                   "maximum": scenario.world.maximum.tolist(),
                   "resolution": scenario.world.resolution},
        "obstacles": obstacle_points,
        "obstacles_geometry": geometry,
        "starts": np.asarray(scenario.starts).tolist(),
        "goals": np.asarray(scenario.goals).tolist(),
        "paths": [np.asarray(p).tolist() for p in plan.paths],
        "trajectories": ([np.asarray(p).tolist() for p in plan.trajectories]
                         if plan.trajectories is not None else []),
        "metrics": report,
        "timings": timings,
        "results": [{"planner": r.planner, "cost": r.cost, "expanded": r.expanded,
                     "metadata": r.metadata} for r in plan.results],
        "controls": {"planner": args.planner, "layout": args.layout,
                     "seed": args.seed, "uavs": args.uavs, "obstacles": args.obstacles},
    }
    payload = _safe(payload)
    with PLAN_LOCK:
        PLAN_CACHE[key] = payload
        PLAN_CACHE.move_to_end(key)
        while len(PLAN_CACHE) > PLAN_CACHE_LIMIT:
            PLAN_CACHE.popitem(last=False)
    return payload


def step_payload(query: dict[str, list[str]]) -> dict:
    """Incremental A*/Dijkstra search frames for the step-by-step teaching view."""
    args = _args(query)
    if args.layout not in SCENARIO_LAYOUTS:
        args.layout = "mixed"
    heuristic = query.get("heuristic", ["euclidean"])[0]
    if heuristic not in ("euclidean", "zero"):
        heuristic = "euclidean"
    cfg = ScenarioConfig(seed=args.seed, n_uavs=1, n_obstacles=args.obstacles,
                         planner=build_planner_config(args))
    scenario = build_scenario(cfg, args.layout)
    world = scenario.world
    weights = cfg.planner.to_weights()
    start = world.world_to_grid(scenario.starts[0])
    goal = world.world_to_grid(scenario.goals[0])
    stepper = AStarStepper3D(
        world, start, goal, weights, heuristic=heuristic,
        diagonal=not args.no_diagonal, max_expansions=args.max_expansions)
    stepper.run()
    total_expanded = int(stepper.expanded)
    reached = bool(stepper.reached)
    stride = max(1, int(np.ceil(total_expanded / 400)))
    stepper.reset()
    frames = []
    while not stepper.done:
        close_nodes, open_nodes = [], []
        for _ in range(stride):
            before_open = set(stepper.open_set)
            stepper.step()
            if stepper.current is not None:
                cpos = world.grid_to_world(
                    stepper.table.cell_of(stepper.current)).tolist()
                close_nodes.append(cpos)
            for idx in stepper.open_set - before_open:
                info = stepper.node_info(idx)
                pos = world.grid_to_world(stepper.table.cell_of(idx)).tolist()
                open_nodes.append([pos[0], pos[1], pos[2], round(info["f"], 2)])
            if stepper.done:
                break
        curinfo = None
        if stepper.current is not None:
            ninfo = stepper.node_info(stepper.current)
            curinfo = {"g": round(ninfo["g"], 2), "h": round(ninfo["h"], 2),
                       "f": round(ninfo["f"], 2)}
        frames.append({
            "close": close_nodes, "open": open_nodes,
            "exp": int(stepper.expanded), "osize": len(stepper.open_set),
            "cur": close_nodes[-1] if close_nodes else None,
            "curinfo": curinfo,
        })
    path = ([np.asarray(p).tolist() for p in stepper.path]
            if stepper.path else [])
    obstacle_view = _obstacle_geometry(scenario)
    payload = {
        "mode": "step",
        "heuristic": heuristic,
        "bounds": {"origin": world.origin.tolist(),
                   "maximum": world.maximum.tolist(),
                   "resolution": world.resolution},
        "obstacles": obstacle_view["points"],
        "obstacles_geometry": obstacle_view["geometry"],
        "start": np.asarray(scenario.starts[0]).tolist(),
        "goal": np.asarray(scenario.goals[0]).tolist(),
        "frames": frames,
        "path": path,
        "total_expanded": total_expanded,
        "reached": reached,
        "stride": stride,
    }
    return _safe(payload)


def compare_payload(query: dict[str, list[str]]) -> dict:
    """Same-map comparison of the four graph-search planners (paths + metrics)."""
    args = _args(query)
    if args.layout not in SCENARIO_LAYOUTS:
        args.layout = "mixed"
    cfg = ScenarioConfig(seed=args.seed, n_uavs=1, n_obstacles=args.obstacles,
                         planner=build_planner_config(args))
    scenario = build_scenario(cfg, args.layout)
    world = scenario.world
    obstacle_view = _obstacle_geometry(scenario)
    names = {"astar": "A*", "dijkstra": "Dijkstra",
             "greedy": "Greedy 贪心", "bidirectional": "Bidirectional 双向"}
    colors = {"astar": "#61efff", "dijkstra": "#ffb347",
              "greedy": "#ff7181", "bidirectional": "#78ffb5"}
    items = []
    for algo in ("astar", "dijkstra", "greedy", "bidirectional"):
        sub = copy.copy(args)
        sub.planner = algo
        plan, timings = plan_fleet_for(sub, scenario, cfg.planner)
        r0 = plan.results[0]
        path = np.asarray(plan.paths[0])
        length = (float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
                  if len(path) > 1 else 0.0)
        items.append({
            "planner": algo, "name": names[algo], "color": colors[algo],
            "path": path.tolist(),
            "expanded": int(sum(r.expanded for r in plan.results)),
            "length": round(length, 2),
            "time": round(timings.get("planning", 0.0), 3),
            "cost": round(float(r0.cost), 2),
        })
    payload = {
        "mode": "compare",
        "bounds": {"origin": world.origin.tolist(),
                   "maximum": world.maximum.tolist(),
                   "resolution": world.resolution},
        "obstacles": obstacle_view["points"],
        "obstacles_geometry": obstacle_view["geometry"],
        "start": np.asarray(scenario.starts[0]).tolist(),
        "goal": np.asarray(scenario.goals[0]).tolist(),
        "items": items,
    }
    return _safe(payload)


class _Handler(BaseHTTPRequestHandler):
    server_version = "UAVPlannerCockpit/1.0"

    def log_message(self, *_args):
        return

    def do_GET(self):  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            self._send(200, "text/html; charset=utf-8", HTML_BODY)
            return
        if parsed.path == "/viewer.js":
            self._send(200, "application/javascript; charset=utf-8", VIEWER_BODY)
            return
        if parsed.path == "/api/meta":
            body = json.dumps({"layouts": SCENARIO_LAYOUTS, "algorithms": ALGORITHMS,
                               "shapes": [[24, 24, 12], [32, 32, 14], [40, 40, 16]]},
                              ensure_ascii=False).encode()
            self._send(200, "application/json; charset=utf-8", body)
            return
        if parsed.path == "/api/plan":
            try:
                result = plan_payload(parse_qs(parsed.query))
                body = json.dumps(result,
                                   ensure_ascii=False).encode()
                self._send(200, "application/json; charset=utf-8", body)
            except Exception as exc:
                body = json.dumps(
                    {"error": f"{type(exc).__name__}: {exc}",
                     "hint": "请降低障碍数或增大地图尺寸后重试"},
                    ensure_ascii=False).encode()
                self._send(400, "application/json; charset=utf-8", body)
            return
        if parsed.path == "/api/step":
            try:
                result = step_payload(parse_qs(parsed.query))
                body = json.dumps(result, ensure_ascii=False).encode()
                self._send(200, "application/json; charset=utf-8", body)
            except Exception as exc:
                body = json.dumps(
                    {"error": f"{type(exc).__name__}: {exc}",
                     "hint": "请降低障碍数或增大地图尺寸后重试"},
                    ensure_ascii=False).encode()
                self._send(400, "application/json; charset=utf-8", body)
            return
        if parsed.path == "/api/compare":
            try:
                result = compare_payload(parse_qs(parsed.query))
                body = json.dumps(result, ensure_ascii=False).encode()
                self._send(200, "application/json; charset=utf-8", body)
            except Exception as exc:
                body = json.dumps(
                    {"error": f"{type(exc).__name__}: {exc}",
                     "hint": "请降低障碍数或增大地图尺寸后重试"},
                    ensure_ascii=False).encode()
                self._send(400, "application/json; charset=utf-8", body)
            return
        self._send(404, "text/plain; charset=utf-8", b"Not Found")

    def _send(self, status: int, content_type: str, body: bytes):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


def serve(host: str = "127.0.0.1", port: int = 8765, open_browser: bool = True) -> int:
    """Start the cockpit and return the selected local port."""
    httpd = ThreadingHTTPServer((host, int(port)), _Handler)
    url = f"http://{host}:{httpd.server_port}/"
    print(f"UAV 实验驾驶舱: {url}")
    print("关闭浏览器页面后，在 VSCode 终端按 Ctrl+C 结束服务。")
    if open_browser:
        threading.Timer(0.25, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
    return 0
