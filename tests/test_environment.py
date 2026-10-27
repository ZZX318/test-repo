import numpy as np

from uav_planner.environment import VoxelMap


def test_obstacles_and_coordinates():
    world = VoxelMap((12, 12, 8), 1.0)
    world.add_sphere((5, 5, 3), 1.5)
    cell = world.world_to_grid((2, 2, 2))
    assert world.free(cell)
    assert np.allclose(world.grid_to_world(cell), (2, 2, 2))
    assert world.clearance((5, 5, 3)) < 0


def test_segment_collision():
    world = VoxelMap((12, 12, 8), 1.0)
    world.add_box((5, 4, 2), (7, 8, 6))
    assert world.segment_collision((1, 6, 4), (10, 6, 4))
    assert not world.segment_collision((1, 1, 1), (10, 1, 1))


def test_cylinder_signed_distance_outside_caps():
    world = VoxelMap((12, 12, 12), 1.0)
    world.add_cylinder((6, 6), 3.0, 7.0, 2.0)
    assert world.clearance((6, 6, 1)) > 0
    assert world.clearance((6, 6, 5)) < 0
