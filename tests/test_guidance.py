import numpy as np

from uav_planner.guidance import GuidanceTracker, paper_guidance_reward, project


def test_projection_and_monotonic_guidance():
    path = np.array([[0, 0, 0], [5, 0, 0], [10, 0, 2]], dtype=float)
    s, q, d = project((2, 2, 0), path)
    assert 0 < s < 5 and d > 0
    tracker = GuidanceTracker(path, lookahead=2.0)
    first = tracker.update((1, 0, 0)).copy()
    tracker.update((0, 0, 0))
    assert tracker.progress >= 0
    assert np.linalg.norm(first - tracker.current) < 5.0
    assert paper_guidance_reward((0, 0, 0), (0, 0, 0)) == 2.0
