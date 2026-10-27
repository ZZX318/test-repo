"""把 refs/PathPlanning 的脚本跑一遍并把最后的图存成 PNG。

用途：没有图形界面（SSH、CI、或者只是想快速确认"这脚本到底出不出图"）时，
用这个代替弹窗。它和 run_ref.py 用同一套环境准备，只是把后端换成 Agg 并
在脚本跑完后 savefig。

    python refs/preview.py                      # 默认两张：3D A* + 2D A*
    python refs/preview.py Search_3D/Astar3D.py
    python refs/preview.py Search_2D/Dijkstra.py

输出目录：refs/preview/
"""

from __future__ import annotations

import argparse
import os
import runpy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _bootstrap  # noqa: E402  必须最先：缺 numpy/matplotlib 时自动切到 .venv

_bootstrap.ensure()

import matplotlib  # noqa: E402

matplotlib.use("Agg")           # 必须在 pyplot 之前设置
import matplotlib.pyplot as plt  # noqa: E402

from run_ref import ROOT, resolve  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent / "preview"
DEFAULTS = ("Search_3D/Astar3D.py", "Search_2D/Astar.py")


def render(script: Path, out_dir: Path) -> Path:
    plt.close("all")
    script_dir = script.parent
    os.chdir(script_dir)
    sys.path.insert(0, str(script_dir))
    sys.argv = [str(script)]

    print(f"→ 运行 {script.relative_to(ROOT)} ...", flush=True)
    try:
        runpy.run_path(str(script), run_name="__main__")
    except SystemExit:
        pass

    numbers = plt.get_fignums()
    if not numbers:
        raise RuntimeError(f"{script.name} 没有产生任何 figure")
    fig = plt.figure(numbers[-1])
    fig.set_size_inches(9, 8)

    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / (script.stem + ".png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close("all")
    print(f"  已保存 {out}  ({out.stat().st_size / 1024:.0f} KB)")
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="把 refs 脚本的输出存成 PNG")
    parser.add_argument("scripts", nargs="*", help="默认: " + " ".join(DEFAULTS))
    parser.add_argument("--out", default=str(OUT_DIR))
    options = parser.parse_args(argv)

    names = options.scripts or list(DEFAULTS)
    out_dir = Path(options.out)
    results = []
    for name in names:
        try:
            results.append(render(resolve(name), out_dir))
        except Exception as exc:  # noqa: BLE001 - 一个失败不该挡住其他的
            print(f"  失败: {type(exc).__name__}: {exc}")
    print(f"\n完成 {len(results)}/{len(names)}")
    return 0 if results else 1


if __name__ == "__main__":
    raise SystemExit(main())
