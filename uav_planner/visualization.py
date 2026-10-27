"""Optional matplotlib rendering, including the interactive A* stepper.

The static plot is kept, but the useful part is :func:`interactive_astar`:
it drives :class:`AStarStepper3D` one expansion at a time and shows ``g``,
``h`` and ``f`` for the node being expanded, mirroring the HTML demo.

Keys
----
``space`` / ``right``  one expansion
``a``                  run to completion
``r``                  reset
``h``                  toggle heuristic (A* <-> Dijkstra)
``s``                  cycle the number of steps per key press
``q``                  close
"""

from __future__ import annotations

import numpy as np


def _plt():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover - optional dependency
        raise RuntimeError("plotting requires matplotlib; install with pip install -e '.[plot]'") from exc
    return plt


# Palette shared by the viewer and its legend.
COLOR_CLOSED = "#9aa5b1"
COLOR_OPEN = "#f0a020"
COLOR_CURRENT = "#d62728"
COLOR_PATH = "#2ca02c"
COLOR_OBSTACLE = "#e07070"


def _cjk(size: float = 9.0, weight: str = "normal"):
    """Font that can actually draw the Chinese labels on this machine."""
    from matplotlib import font_manager
    from matplotlib.font_manager import FontProperties

    installed = {f.name for f in font_manager.fontManager.ttflist}
    for family in ("Arial Unicode MS", "Hiragino Sans GB", "STHeiti", "Songti SC"):
        if family in installed:
            return FontProperties(family=[family, "DejaVu Sans"], size=size, weight=weight)
    return FontProperties(size=size, weight=weight)


def _set3d(collection, points):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    collection._offsets3d = (points[:, 0], points[:, 1], points[:, 2])
    return collection


def _draw_obstacles(ax, world, alpha: float = 0.16):
    """Voxel-accurate obstacle rendering: one translucent marker per occupied cell."""
    cells = np.argwhere(world.occupied)
    if len(cells) == 0:
        return None
    points = world.origin + cells * world.resolution
    return ax.scatter(points[:, 0], points[:, 1], points[:, 2],
                      s=max(4.0, 220.0 * world.resolution ** 2 / 8.0),
                      c="tab:red", alpha=alpha, marker="s", linewidths=0, depthshade=False)


def _setup_axes(ax, world, title: str):
    ax.set_xlim(world.origin[0], world.maximum[0])
    ax.set_ylim(world.origin[1], world.maximum[1])
    ax.set_zlim(world.origin[2], world.maximum[2])
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.set_zlabel("z / m")
    ax.set_title(title)


def plot_scenario(world, paths=None, starts=None, goals=None, show: bool = True):
    plt = _plt()
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    _draw_obstacles(ax, world)
    for i, path in enumerate(paths or []):
        path = np.asarray(path)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], linewidth=2, label=f"无人机 {i}")
    if starts is not None and len(starts):
        s = np.asarray(starts)
        ax.scatter(s[:, 0], s[:, 1], s[:, 2], marker="o", c="green", s=45, label="起点")
    if goals is not None and len(goals):
        g = np.asarray(goals)
        ax.scatter(g[:, 0], g[:, 1], g[:, 2], marker="*", c="black", s=70, label="目标")
    _setup_axes(ax, world, "3D Multi-UAV Global Paths")
    ax.legend(loc="upper left")
    fig.tight_layout()
    if show:
        plt.show()
    return fig, ax


def interactive_astar(world, start, goal, weights=None, heuristic: str = "euclidean",
                      diagonal: bool = True, safety_margin=None, steps_per_key: int = 1,
                      max_steps_per_frame: int = 2_000_000):
    """Step-through A* viewer with live ``g``/``h``/``f`` read-out."""
    plt = _plt()
    from .astar3d import AStarStepper3D

    stepper = AStarStepper3D(world, start, goal, weights, heuristic=heuristic,
                             diagonal=diagonal, safety_margin=safety_margin)

    fig = plt.figure(figsize=(12.6, 8.6))
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(left=0.02, right=0.95, top=0.92, bottom=0.22)
    ax.set_box_aspect((4, 4, 3), zoom=1.18)
    _draw_obstacles(ax, world)

    closed = _set3d(ax.scatter([], [], [], s=7, c=COLOR_CLOSED, depthshade=False), [])
    opened = _set3d(ax.scatter([], [], [], s=9, c=COLOR_OPEN, depthshade=False), [])
    current = _set3d(ax.scatter([], [], [], s=90, c=COLOR_CURRENT, marker="o", depthshade=False), [])
    path_line, = ax.plot([], [], [], linewidth=2.4, c=COLOR_PATH)
    start_pt = np.asarray(world.grid_to_world(start), float)
    goal_pt = np.asarray(world.grid_to_world(goal), float)
    ax.scatter(*start_pt, marker="o", c="green", s=70, edgecolors="black", linewidths=0.6)
    ax.scatter(*goal_pt, marker="*", c="black", s=130)
    _setup_axes(ax, world, "A* stepper")

    # 图例写全：四种节点状态 + 障碍 + 起终点，不用去记颜色
    from matplotlib.lines import Line2D
    ax.legend(handles=[
        Line2D([], [], color=COLOR_PATH, lw=2.4, label="green line   path found"),
        Line2D([], [], marker="o", ls="", color=COLOR_OPEN, label="orange dot   open (to explore)"),
        Line2D([], [], marker="o", ls="", color=COLOR_CLOSED, label="gray dot   closed (expanded)"),
        Line2D([], [], marker="o", ls="", color=COLOR_CURRENT, ms=9, label="red dot   current node"),
        Line2D([], [], marker="s", ls="", color=COLOR_OBSTACLE, ms=8, label="pink block   obstacle voxel"),
        Line2D([], [], marker="o", ls="", color="green", label="green circle   start"),
        Line2D([], [], marker="*", ls="", color="black", ms=11, label="black star   goal"),
    ], loc="upper right", prop=_cjk(9), framealpha=0.92,
        title="Legend", title_fontproperties=_cjk(9, "bold"))

    info = fig.text(0.74, 0.02, "", va="bottom", ha="left",
                    fontproperties=_cjk(9.5),
                    bbox=dict(boxstyle="round,pad=0.45", facecolor="white",
                              edgecolor="#c8c8c8", alpha=0.94))

    fig.text(0.03, 0.02,
             "Keys    space/->  step    a  run to end    r  reset    h  toggle h (A* <-> Dijkstra, h=0 is Dijkstra)\n"
             "        s  steps per key (1/2/4/8)    q  close window\n"
             "How     each step pops the node with smallest f from open -> mark closed -> expand neighbors -> push to open. f = g + h",
             va="bottom", ha="left", fontproperties=_cjk(9.5), linespacing=1.7,
             bbox=dict(boxstyle="round,pad=0.6", facecolor="#f7f7f7",
                       edgecolor="#c8c8c8", alpha=0.98))

    state = {"per_key": steps_per_key, "running": False}

    def refresh():
        def cells(indices):
            if not indices:
                return np.zeros((0, 3))
            return np.stack([world.grid_to_world(stepper.table.cell_of(i)) for i in indices])

        _set3d(closed, cells(list(stepper.closed)))
        _set3d(opened, cells([i for i in stepper.open_set if i not in stepper.closed]))
        if stepper.current is not None:
            _set3d(current, cells([stepper.current]))
        if len(stepper.path):
            pts = np.asarray(stepper.path, float)
            path_line.set_data(pts[:, 0], pts[:, 1])
            path_line.set_3d_properties(pts[:, 2])

        if stepper.current is not None:
            info_node = stepper.node_info(stepper.current)
            x, y, z = stepper.table.cell_of(stepper.current)
            body = (f"node ({x},{y},{z})\n"
                    f"g = {info_node['g']:.2f}   cost so far\n"
                    f"h = {info_node['h']:.2f}   estimated remaining\n"
                    f"f = {info_node['f']:.2f}   f = g + h\n"
                    f"clearance = {info_node['clearance']:.2f} m\n"
                    f"open {len(stepper.open_set)}    closed {len(stepper.closed)}")
        else:
            body = "not started\npress [space] to expand the first node"
        info.set_text(body)

        w = stepper.weights
        ax.set_title(f"A* stepper - expanded {stepper.expanded} nodes"
                     + ("  ✓ reached" if stepper.reached else "")
                     + ("" if stepper.frontier or stepper.done else "  (open empty)")
                     + "\n"
                     + f"h={stepper.heuristic_mode}   neigh={'26' if stepper.diagonal else '6'}"
                     + f"   w_obs={w.obstacle:.1f}   w_clear={w.clearance:.1f}"
                     + f"   hard={w.hard_clearance:.2f}   min_sep={w.minimum_separation:.1f}"
                     + f"   steps/key={state['per_key']}",
                     fontproperties=_cjk(11), linespacing=1.6)
        fig.canvas.draw_idle()

    def advance(count: int):
        for _ in range(max(1, count)):
            if stepper.done:
                break
            stepper.step()
        refresh()

    def on_key(event):
        key = event.key
        if key in (" ", "right", "n"):
            advance(state["per_key"])
        elif key == "a":
            # Run to completion (the map is small; ~10 us per expansion).
            for _ in range(max_steps_per_frame):
                if stepper.done:
                    break
                stepper.step()
            refresh()
        elif key == "r":
            stepper.reset()
            path_line.set_data([], [])
            path_line.set_3d_properties([])
            refresh()
        elif key == "h":
            stepper.set_heuristic("zero" if stepper.heuristic_mode == "euclidean" else "euclidean")
            refresh()
        elif key == "s":
            state["per_key"] = 1 if state["per_key"] >= 8 else state["per_key"] * 2
            refresh()
        elif key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    refresh()
    plt.show()
    return stepper


def animate_fleet(world, paths, trajectories=None, starts=None, goals=None,
                  interval: int = 60, show: bool = True):
    """Replay the executed trajectories (or the paths when there are none)."""
    plt = _plt()
    from matplotlib.animation import FuncAnimation

    tracks = trajectories if trajectories is not None else paths
    tracks = [np.asarray(t, float).reshape(-1, 3) for t in tracks]
    if not tracks:
        raise ValueError("nothing to animate")
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    _draw_obstacles(ax, world)
    for i, path in enumerate(paths or []):
        path = np.asarray(path, float)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], linewidth=1.0,
                linestyle="--", alpha=0.6, label=f"plan {i}")
    markers = [ax.scatter(*t[0], s=70, marker="o", depthshade=False) for t in tracks]
    trail, = ax.plot([], [], [], linewidth=2.0, c="tab:blue")
    _setup_axes(ax, world, "无人机轨迹回放")
    ax.legend(loc="upper left")
    frames = max(len(t) for t in tracks)

    def update(frame):
        for marker, track in zip(markers, tracks):
            index = min(frame, len(track) - 1)
            _set3d(marker, track[index:index + 1])
        longest = max(tracks, key=len)
        upto = longest[:min(frame, len(longest)) + 1]
        trail.set_data(upto[:, 0], upto[:, 1])
        trail.set_3d_properties(upto[:, 2])
        ax.set_title(f"UAV Trajectory Replay - Step {frame + 1}/{frames}")
        return markers + [trail]

    anim = FuncAnimation(fig, update, frames=frames, interval=interval, blit=False)
    if show:
        plt.show()
    return anim


def interactive_lab(config=None, layouts=None,
                    algorithms=("astar", "dijkstra", "greedy", "bidirectional",
                                "rrtconnect", "rrtstar")):
    """Keyboard-driven map and algorithm laboratory.

    ``m`` cycles benchmark maps and ``p`` cycles planning backends.  A* and
    Dijkstra remain stepable; the other methods run once and display their
    final path.  This keeps the teaching interaction from being tied to one
    algorithm while preserving the useful single-step view.
    """
    plt = _plt()
    from .algorithms import plan_path
    from .astar3d import AStarStepper3D
    from .config import ScenarioConfig
    from .scenario import (make_demo, make_forest_demo, make_maze_demo,
                           make_reference_demo, make_sphere_demo,
                           make_urban_demo, make_wall_demo)

    config = config or ScenarioConfig()
    layouts = tuple(layouts or ("mixed", "walls", "spheres", "urban", "forest", "maze", "reference"))
    algorithms = tuple(algorithms)
    builders = {"mixed": make_demo, "walls": make_wall_demo, "spheres": make_sphere_demo,
                "urban": make_urban_demo, "forest": make_forest_demo,
                "maze": make_maze_demo, "reference": make_reference_demo}
    state = {"layout": 0, "algorithm": 0, "scenario": None,
             "stepper": None, "result": None, "error": None}

    layout_labels = {
        "mixed": "Mixed obstacles", "walls": "Wall gaps", "spheres": "Sphere forest",
        "urban": "Urban buildings", "forest": "Dense forest", "maze": "3D maze",
        "reference": "Reference map",
    }
    algorithm_labels = {
        "astar": "A*", "dijkstra": "Dijkstra", "greedy": "Greedy best-first",
        "bidirectional": "Bidirectional", "rrtconnect": "RRT-Connect", "rrtstar": "RRT*",
    }

    fig = plt.figure(figsize=(12.5, 8.5))
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(left=0.02, right=0.98, top=0.91, bottom=0.14)
    help_text = fig.text(0.03, 0.025, "", va="bottom", ha="left",
                         fontproperties=_cjk(10),
                         bbox=dict(boxstyle="round,pad=0.5", facecolor="#f7f7f7",
                                   edgecolor="#cccccc", alpha=0.96))

    def current_name():
        return layouts[state["layout"]], algorithms[state["algorithm"]]

    def build(reset_only=False):
        layout, algorithm = current_name()
        state["scenario"] = builders.get(layout, make_demo)(config)
        scenario = state["scenario"]
        start = scenario.world.world_to_grid(scenario.starts[0])
        goal = scenario.world.world_to_grid(scenario.goals[0])
        state["stepper"] = None
        state["result"] = None
        state["error"] = None
        try:
            if algorithm in ("astar", "dijkstra"):
                mode = "zero" if algorithm == "dijkstra" else config.planner.heuristic
                state["stepper"] = AStarStepper3D(
                    scenario.world, start, goal, config.planner.to_weights(),
                    heuristic=mode, diagonal=config.planner.diagonal_cost,
                    max_expansions=config.planner.max_expansions,
                    safety_margin=config.planner.safety_margin)
            else:
                state["result"] = plan_path(
                    scenario.world, start, goal, algorithm,
                    config.planner.to_weights(), config.planner.diagonal_cost,
                    config.planner.max_expansions, config.planner.heuristic,
                    config.planner.safety_margin, seed=config.seed)
        except (RuntimeError, ValueError) as exc:
            state["error"] = f"{type(exc).__name__}: {exc}"
        refresh()

    def scatter_points(points, color, size, marker="o"):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        if len(points):
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=color,
                       s=size, marker=marker, depthshade=False)

    def refresh():
        scenario = state["scenario"]
        if scenario is None:
            return
        layout, algorithm = current_name()
        ax.clear()
        _draw_obstacles(ax, scenario.world, alpha=0.15)
        start = np.asarray(scenario.starts[0], dtype=float)
        goal = np.asarray(scenario.goals[0], dtype=float)
        scatter_points([start], "green", 75)
        scatter_points([goal], "black", 140, "*")
        stepper = state["stepper"]
        result = state["result"]
        if stepper is not None:
            table = stepper.table
            def world_cells(indices):
                if not indices:
                    return np.zeros((0, 3))
                return np.stack([scenario.world.grid_to_world(table.cell_of(i)) for i in indices])
            scatter_points(world_cells(list(stepper.closed)), COLOR_CLOSED, 7)
            scatter_points(world_cells([i for i in stepper.open_set if i not in stepper.closed]), COLOR_OPEN, 9)
            if stepper.current is not None:
                scatter_points(world_cells([stepper.current]), COLOR_CURRENT, 90)
            if stepper.path:
                points = np.asarray(stepper.path, dtype=float)
                ax.plot(points[:, 0], points[:, 1], points[:, 2], c=COLOR_PATH, linewidth=2.6)
            status = (f"expanded {stepper.expanded} | open {len(stepper.open_set)} | "
                      f"closed {len(stepper.closed)} | "
                      f"{'goal reached' if stepper.reached else 'searching'}")
        elif result is not None:
            points = np.asarray(result.points, dtype=float)
            ax.plot(points[:, 0], points[:, 1], points[:, 2], c=COLOR_PATH, linewidth=2.8)
            status = (f"complete | cost {result.cost:.2f} | expanded {result.expanded} | "
                      f"waypoints {len(points)}")
        else:
            status = state["error"] or "ready"
        _setup_axes(ax, scenario.world,
                    f"UAV 3D Planning Lab | Map: {layout_labels.get(layout, layout)} | "
                    f"Algorithm: {algorithm_labels.get(algorithm, algorithm)}\n{status}")
        ax.legend(handles=[], loc="upper right", frameon=False)
        help_text.set_text(
            "【空格/→】下一步   【a】运行到终点   【r】重置   【h】切换 A*/Dijkstra\n"
            "【m】切换地图   【p】切换算法   【q】退出   |   绿色：起点  黑色：目标  粉色：障碍")
        fig.canvas.draw_idle()

    def advance(count=1):
        stepper = state["stepper"]
        if stepper is None:
            return
        for _ in range(count):
            if stepper.done:
                break
            stepper.step()
        refresh()

    def on_key(event):
        key = event.key
        if key in (" ", "right"):
            advance(1)
        elif key == "a" and state["stepper"] is not None:
            state["stepper"].run()
            refresh()
        elif key == "r":
            build(reset_only=True)
        elif key == "m":
            state["layout"] = (state["layout"] + 1) % len(layouts)
            build()
        elif key == "p":
            state["algorithm"] = (state["algorithm"] + 1) % len(algorithms)
            build()
        elif key == "h" and state["stepper"] is not None:
            mode = "zero" if state["stepper"].heuristic_mode == "euclidean" else "euclidean"
            state["stepper"].set_heuristic(mode)
            refresh()
        elif key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    build()
    plt.show()
    return state
