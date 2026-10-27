"""CLI-level tests: every knob must be reachable without editing source."""

import json

from uav_planner.cli import build_parser, build_planner_config, main, parse_weights
from uav_planner.astar3d import CostWeights


def test_parse_weights_accepts_aliases():
    weights = parse_weights("obstacle=12, hard=0.6,min_sep=4", CostWeights())
    assert weights.obstacle == 12.0
    assert weights.hard_clearance == 0.6
    assert weights.minimum_separation == 4.0
    assert weights.length == 1.0  # untouched


def test_individual_flags_beat_weights_string():
    args = build_parser().parse_args(["--weights", "hard=0.9", "--hard-clearance", "0.4"])
    cfg = build_planner_config(args)
    assert cfg.hard_clearance == 0.4
    assert cfg.to_weights().hard_clearance == 0.4


def test_defaults_match_planner_config():
    cfg = build_planner_config(build_parser().parse_args([]))
    reference = __import__("uav_planner.config", fromlist=["PlannerConfig"]).PlannerConfig()
    assert cfg.to_weights() == reference.to_weights()
    assert cfg.max_expansions == reference.max_expansions


def test_json_mode_emits_valid_json_without_infinities(capsys):
    code = main(["--json", "--uavs", "1", "--seed", "7"])
    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert "plan" in payload["metrics"]
    assert "execution" in payload["metrics"]
    assert payload["metrics"]["plan"]["hard_clearance"] == 0.35


def test_compare_mode_runs(capsys):
    assert main(["--compare", "--seed", "7"]) == 0
    out = capsys.readouterr().out
    assert "euclidean" in out and "zero" in out


def test_sweep_mode_reports_three_lengths(capsys):
    assert main(["--sweep", "0,12,40"]) == 0
    lines = [line for line in capsys.readouterr().out.strip().splitlines() if line.strip()]
    assert len(lines) == 4  # header + 3 rows
    lengths = [float(line.split()[2]) for line in lines[1:]]
    assert lengths[0] < lengths[1] < lengths[2]
