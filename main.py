"""VSCode-friendly entry point for the UAV 3D planner demo.

Running this file directly without arguments opens the browser based experiment
cockpit. Command-line arguments are passed through to :mod:`uav_planner.cli`.
"""

import sys

from uav_planner.cli import main


if __name__ == "__main__":
    if len(sys.argv) == 1:
        sys.argv.append("--dashboard")
    raise SystemExit(main())
