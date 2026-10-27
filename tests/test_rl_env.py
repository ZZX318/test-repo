import numpy as np

from uav_planner.environment import VoxelMap
from uav_planner.rl_env import MultiUAV3DEnv


def test_numpy_multi_uav_environment_step():
    world = VoxelMap((12, 12, 8), 1.0)
    env = MultiUAV3DEnv(world, [(1, 1, 1), (1, 3, 1)], [(10, 10, 2), (10, 8, 2)])
    obs = env.reset()
    assert obs.shape == (2, env.observation_dim)
    new_obs, reward, terminated, truncated, info = env.step(np.ones((2, 3)))
    assert new_obs.shape == obs.shape
    assert reward.shape == (2,)
    assert isinstance(terminated, bool) and isinstance(truncated, bool)
    assert "min_clearance" in info
