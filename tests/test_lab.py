import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from uav_planner.config import ScenarioConfig
from uav_planner.visualization import interactive_lab


def test_lab_switches_map_and_algorithm_with_keys():
    state = interactive_lab(
        ScenarioConfig(seed=7, n_uavs=1, n_obstacles=12),
        layouts=("mixed", "urban"),
        algorithms=("astar", "dijkstra", "greedy"),
    )
    fig = plt.gcf()
    from matplotlib.backend_bases import KeyEvent

    def press(key):
        fig.canvas.callbacks.process("key_press_event", KeyEvent("key_press_event", fig.canvas, key))

    try:
        press("m")
        assert state["scenario"].info["layout"] == "urban"
        press("p")
        assert state["stepper"] is not None  # dijkstra remains stepable
        press("p")
        assert state["result"] is not None  # greedy is a completed backend
    finally:
        press("q")
        plt.close("all")
