"""Run the whole test suite without pytest installed.

The project declares pytest as a dev extra, but the bundled virtualenv may not
have it (installing packages is not always possible offline).  This runner calls
every ``test_*`` function in ``tests/`` and supplies a minimal ``capsys`` stand
in, so the suite is always runnable:

    python tests/run_all.py
"""

from __future__ import annotations

import contextlib
import importlib.util
import inspect
import io
import pathlib
import sys
import time
import traceback
from types import SimpleNamespace

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


class CapSys:
    """Tiny stand-in for pytest's ``capsys`` fixture."""

    def __init__(self):
        self.out = io.StringIO()
        self.err = io.StringIO()

    def readouterr(self):
        result = SimpleNamespace(out=self.out.getvalue(), err=self.err.getvalue())
        self.out = io.StringIO()
        self.err = io.StringIO()
        return result


def main() -> int:
    failed = 0
    passed = 0
    for path in sorted((ROOT / "tests").glob("test_*.py")):
        spec = importlib.util.spec_from_file_location(path.stem, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for name in sorted(dir(module)):
            if not name.startswith("test_"):
                continue
            fn = getattr(module, name)
            if not callable(fn):
                continue
            capsys = CapSys() if "capsys" in inspect.signature(fn).parameters else None
            t0 = time.perf_counter()
            try:
                if capsys is not None:
                    with contextlib.redirect_stdout(capsys.out), \
                            contextlib.redirect_stderr(capsys.err):
                        fn(capsys=capsys)
                else:
                    fn()
                passed += 1
                print(f"PASS {path.name}::{name}  ({time.perf_counter() - t0:.2f}s)")
            except Exception:  # noqa: BLE001 - a test runner must report everything
                failed += 1
                print(f"FAIL {path.name}::{name}")
                traceback.print_exc()
    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
