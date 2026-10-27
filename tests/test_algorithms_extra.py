import numpy as np

from uav_planner.algorithms import ALGORITHMS, plan_path
from uav_planner.environment import VoxelMap


def test_all_registered_algorithms_connect_empty_world():
    world = VoxelMap((16, 16, 10), 1.0)
    start, goal = (1, 1, 1), (13, 13, 7)
    for name in ALGORITHMS:
        result = plan_path(world, start, goal, name, seed=3, max_expansions=10000)
        assert np.allclose(result.points[0], world.grid_to_world(start))
        assert np.allclose(result.points[-1], world.grid_to_world(goal))
        assert result.success, name


def test_named_map_layouts_are_reproducible():
    from uav_planner.config import ScenarioConfig
    from uav_planner.scenario import (make_demo, make_forest_demo, make_maze_demo,
                                      make_reference_demo, make_sphere_demo,
                                      make_urban_demo, make_wall_demo)
    cfg = ScenarioConfig(seed=4, n_uavs=1, n_obstacles=12)
    builders = (make_demo, make_wall_demo, make_sphere_demo, make_urban_demo,
                make_forest_demo, make_maze_demo, make_reference_demo)
    for builder in builders:
        first, second = builder(cfg), builder(cfg)
        assert first.info == second.info
        assert np.array_equal(first.world.occupied, second.world.occupied)
