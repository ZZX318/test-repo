"""Regression checks for the unified browser cockpit payload."""

from uav_planner.dashboard import compare_payload, plan_payload, step_payload


def test_dashboard_payload_uses_project_backend():
    payload = plan_payload({
        "planner": ["astar"], "layout": ["urban"], "seed": ["7"],
        "uavs": ["2"], "obstacles": ["12"], "hard": ["0.35"],
    })
    assert payload["controls"]["layout"] == "urban"
    assert len(payload["paths"]) == 2
    assert payload["metrics"]["plan"]["success_rate"] == 1.0
    assert payload["bounds"]["resolution"] == 1.0


def test_dashboard_falls_back_from_unknown_controls():
    payload = plan_payload({"planner": ["unknown"], "layout": ["unknown"]})
    assert payload["controls"]["planner"] == "astar"
    assert payload["controls"]["layout"] == "mixed"


def test_step_payload_frames_the_search():
    payload = step_payload({
        "planner": ["astar"], "layout": ["walls"], "seed": ["7"],
        "uavs": ["1"], "obstacles": ["12"], "hard": ["0.35"],
        "heuristic": ["euclidean"],
    })
    assert payload["mode"] == "step"
    assert payload["reached"] is True
    assert payload["total_expanded"] > 0
    assert len(payload["frames"]) > 0
    first = payload["frames"][0]
    assert set(("close", "open", "curinfo", "exp", "osize")).issubset(first)


def test_step_dijkstra_expands_more_than_astar():
    base = {"planner": ["astar"], "layout": ["walls"], "seed": ["7"],
            "uavs": ["1"], "obstacles": ["12"], "hard": ["0.35"]}
    astar = step_payload({**base, "heuristic": ["euclidean"]})
    dijkstra = step_payload({**base, "heuristic": ["zero"]})
    assert dijkstra["total_expanded"] > astar["total_expanded"]


def test_compare_payload_covers_four_planners():
    payload = compare_payload({
        "planner": ["astar"], "layout": ["walls"], "seed": ["7"],
        "uavs": ["1"], "obstacles": ["12"], "hard": ["0.35"]})
    assert payload["mode"] == "compare"
    assert [i["planner"] for i in payload["items"]] == [
        "astar", "dijkstra", "greedy", "bidirectional"]
    for item in payload["items"]:
        assert len(item["path"]) > 0
        assert item["expanded"] >= 0
        assert item["color"]
