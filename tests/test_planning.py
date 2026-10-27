import numpy as np

from uav_planner.astar3d import AStar3D, CostWeights
from uav_planner.environment import VoxelMap
from uav_planner.smoothing import smooth_collision_aware


def test_astar_routes_around_box():
    world = VoxelMap((20, 12, 8), 1.0)
    world.add_box((8, 2, 0), (11, 10, 5))
    result = AStar3D(world, CostWeights(hard_clearance=0.0)).plan((2, 6, 2), (17, 6, 2))
    assert result.success
    assert np.allclose(result.points[0], (2, 6, 2))
    assert np.allclose(result.points[-1], (17, 6, 2))
    assert not any(world.segment_collision(a, b) for a, b in zip(result.points[:-1], result.points[1:]))


def test_smoothing_keeps_endpoints_and_is_safe():
    world = VoxelMap((20, 12, 8), 1.0)
    path = np.array([[1, 1, 1], [3, 3, 1], [6, 3, 1], [9, 5, 1]])
    smoothed = smooth_collision_aware(path, world)
    assert np.allclose(smoothed[0], path[0])
    assert np.allclose(smoothed[-1], path[-1])
    assert not any(world.segment_collision(a, b) for a, b in zip(smoothed[:-1], smoothed[1:]))
