import numpy as np

from uav_planner.environment import VoxelMap
from uav_planner.rrtstar3d import RRTConfig, RRTStar3D


def test_rrtstar_connects_in_empty_world():
    world = VoxelMap((18, 18, 10), 1.0)
    result = RRTStar3D(world, RRTConfig(iterations=700, step_size=2.5, seed=3)).plan((1, 1, 1), (15, 15, 7))
    assert np.allclose(result.points[0], (1, 1, 1))
    assert np.allclose(result.points[-1], (15, 15, 7))
    assert all(not world.segment_collision(a, b) for a, b in zip(result.points[:-1], result.points[1:]))
