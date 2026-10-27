"""Smoke tests for the interactive viewer (matplotlib, Agg backend).

These exercise the real key handler, so a broken callback or a stale index in
the stepper shows up here instead of when the user presses space.
"""

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from uav_planner.config import ScenarioConfig  # noqa: E402
from uav_planner.scenario import make_demo  # noqa: E402
from uav_planner.visualization import (  # noqa: E402
    animate_fleet, interactive_astar, plot_scenario)


def _scenario():
    sc = make_demo(ScenarioConfig(seed=7, n_uavs=1, n_obstacles=12))
    return sc, sc.world.world_to_grid(sc.starts[0]), sc.world.world_to_grid(sc.goals[0])


def _press(fig, key):
    """Dispatch a key press through the figure's callback registry.

    ``FigureCanvasAgg`` has no ``key_press_event`` helper, so the event is
    pushed through the registry that the GUI backends use as well.
    """
    from matplotlib.backend_bases import KeyEvent

    event = KeyEvent("key_press_event", fig.canvas, key)
    fig.canvas.callbacks.process("key_press_event", event)


def test_interactive_stepper_responds_to_keys():
    sc, start, goal = _scenario()
    stepper = interactive_astar(sc.world, start, goal)
    fig = plt.gcf()
    try:
        for key in (" ", " ", " ", "s", "s", "h", " ", "a"):
            _press(fig, key)
        assert stepper.reached
        assert stepper.expanded > 0
        _press(fig, "h")   # toggle back to euclidean
        assert stepper.heuristic_mode == "euclidean"
        _press(fig, "r")   # reset
        assert stepper.expanded == 0 and not stepper.reached
        _press(fig, "q")
    finally:
        plt.close("all")


def test_static_plot_and_replay_render():
    sc, start, goal = _scenario()
    path = [sc.starts[0], sc.goals[0]]
    fig, _ = plot_scenario(sc.world, [path], [sc.starts[0]], [sc.goals[0]], show=False)
    assert fig is not None
    plt.close("all")
    anim = animate_fleet(sc.world, [path], [np.asarray(path, float)], show=False)
    assert anim is not None
    plt.close("all")
