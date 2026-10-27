"""VSCode-friendly entry point for the UAV 3D planner demo.

Running this file directly without arguments opens the *interactive lab*
(map/algorithm switching plus step / run / reset).  Command-line arguments are passed through to
:mod:`uav_planner.cli` unchanged, so ``--plot``, ``--sweep`` and friends still
work from here.
"""

import sys

from uav_planner.cli import main


if __name__ == "__main__":
    if len(sys.argv) == 1:
        sys.argv.append("--lab")
    raise SystemExit(main())
