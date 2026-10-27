"""在任意目录下运行 refs/PathPlanning 里的算法脚本。

为什么需要这个启动器
--------------------
上游每个脚本都靠两件事找模块：

1. ``sys.path.append(os.path.dirname(__file__) + "/../../Search_based_Planning/")``
   —— 所以 ``from Search_3D.env3D import env`` 能解析；
2. 脚本自己所在目录在 ``sys.path[0]``
   —— 所以 ``import queue`` 拿到的是**同目录的 queue.py**（上游自己写的优先队列），
   不是标准库的 ``queue``。

第 2 条只在 ``python Astar3D.py`` 这种"直接跑文件"时才成立。用 ``python -m``、
用 IDE 的模块模式、或者把文件 import 进来跑，都会拿到标准库的 ``queue``，
然后报 ``AttributeError: module 'queue' has no attribute 'MinheapPQ'``。

这个启动器就是老老实实复刻 ``python Astar3D.py`` 的行为：切 cwd、把脚本目录
放进 sys.path[0]、再 runpy 执行。Python 没有头文件，同目录那几个 .py 就是
它的"头文件"。

用法
----
    python refs/run_ref.py --list
    python refs/run_ref.py Search_3D/Astar3D.py
    python refs/run_ref.py Search_2D/Dijkstra.py
    python refs/run_ref.py rrt_3D/rrt_star3D.py
"""

from __future__ import annotations

import argparse
import os
import runpy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _bootstrap  # noqa: E402  必须最先：缺 numpy 时自动切到 .venv 的 python

_bootstrap.ensure()

ROOT = Path(__file__).resolve().parent / "PathPlanning"
WORKSPACE = ROOT.parent.parent          # refs/PathPlanning -> refs -> 工作区根目录
VENV_PYTHON = _bootstrap.VENV_PYTHON

SEARCH_DIRS = (
    # 3D 排前面：阶段一的重点是三维 A*
    ROOT / "Search_based_Planning" / "Search_3D",
    ROOT / "Search_based_Planning" / "Search_2D",
    ROOT / "Sampling_based_Planning" / "rrt_3D",
    ROOT / "Sampling_based_Planning" / "rrt_2D",
)


RECOMMENDED = "Search_3D/Astar3D.py"


def discover(include_empty: bool = False):
    """所有可独立运行的脚本（排除被 import 的公共模块和上游的空占位文件）。

    上游有 4 个 0 字节的文件（rrt_star_smart3D.py 等），是作者留的占位符，
    选中它们会"什么也不发生"——所以默认从菜单里剔掉，别让人踩这个坑。
    """
    skip = {"env.py", "env3D.py", "utils.py", "utils3D.py", "plotting.py",
            "plot_util3D.py", "queue.py", "__init__.py"}
    found = []
    for folder in SEARCH_DIRS:
        if not folder.is_dir():
            continue
        for script in sorted(folder.glob("*.py")):
            if script.name in skip or script.name.startswith("_"):
                continue
            if not include_empty and script.stat().st_size == 0:
                continue
            found.append((folder.name, script))
    return found


def empty_placeholders():
    """上游留的 0 字节占位文件，只在 --list 末尾提示一句。"""
    known = {"rrt_star_smart3D.py", "rrt_sharp.py",
             "adaptively_informed_trees.py", "advanced_batch_informed_trees.py"}
    hits = []
    for folder in SEARCH_DIRS:
        if folder.is_dir():
            for script in sorted(folder.glob("*.py")):
                if script.name in known and script.stat().st_size == 0:
                    hits.append(f"{folder.name}/{script.name}")
    return hits


def resolve(name: str) -> Path:
    """``Search_3D/Astar3D.py``、``Astar3D.py``、绝对路径都接受。"""
    candidate = Path(name)
    if candidate.is_file():
        return candidate.resolve()
    direct = ROOT / name
    if direct.is_file():
        return direct.resolve()
    matches = [script for _, script in discover() if script.name == candidate.name]
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        raise SystemExit(f"'{name}' 有多个同名脚本，请带上目录：\n" +
                         "\n".join(f"  {s.parent.name}/{s.name}" for s in matches))
    raise SystemExit(f"找不到 '{name}'。用 --list 看有哪些。")


def print_menu():
    scripts = discover()
    print(f"可用脚本（{ROOT}）\n")
    last = None
    for number, (folder, script) in enumerate(scripts, start=1):
        if folder != last:
            print(f"[{folder}]")
            last = folder
        mark = "  ★ 推荐先跑这个（3D A*）" if script.name == "Astar3D.py" else ""
        print(f"  {number:>2}. {folder}/{script.name}{mark}")
    return scripts


def pick_interactively(scripts):
    """没有参数时给一个编号菜单——省得再回来查用法。"""
    print()
    try:
        answer = input("输入编号运行（直接回车退出）: ").strip()
    except (EOFError, KeyboardInterrupt):
        return None
    if not answer:
        return None
    if not answer.isdigit() or not (1 <= int(answer) <= len(scripts)):
        print(f"没有编号 {answer!r}，退出。")
        return None
    return scripts[int(answer) - 1][1]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="运行 refs/PathPlanning 里的脚本")
    parser.add_argument("script", nargs="?", help="例如 Search_3D/Astar3D.py")
    parser.add_argument("--list", action="store_true", help="只列出可运行脚本")
    parser.add_argument("args", nargs=argparse.REMAINDER, help="透传给脚本的参数")
    options = parser.parse_args(argv)

    target = None
    if options.script:
        target = resolve(options.script)
    else:
        scripts = print_menu()
        empties = empty_placeholders()
        if empties:
            print("\n（上游这 4 个是 0 字节空占位文件，已从菜单剔除：" + "、".join(empties) + "）")
        if options.list:
            return 0
        print("\n这只是菜单，不是程序本身——选一个才会真的跑起来、才会弹图。")
        print("推荐输 1 或 2；Ctrl+C 可退出，或改用:")
        print("    python refs/run_ref.py Search_3D/Astar3D.py")
        if not sys.stdin.isatty():
            return 0
        target = pick_interactively(scripts)
        if target is None:
            return 0

    script_dir = target.parent

    # 复刻 `python script.py` 的环境
    os.chdir(script_dir)
    sys.path.insert(0, str(script_dir))
    sys.argv = [str(target)] + ([] if not options.script else list(options.args))

    print(f"→ {target.relative_to(ROOT)}  (cwd={script_dir.name})")
    try:
        runpy.run_path(str(target), run_name="__main__")
    except SystemExit as exc:
        return int(exc.code or 0)
    except ModuleNotFoundError as exc:
        # 最常见的两个坑：解释器选到系统 Python（没 numpy），或者没装 pyrr。
        missing = getattr(exc, "name", "") or "?"
        print(f"\n❌ 缺依赖：{missing}")
        print(f"   当前解释器：{sys.executable}")
        if VENV_PYTHON.exists() and Path(sys.executable) != VENV_PYTHON:
            print("   这个项目自带虚拟环境，用它跑就没问题：")
            print(f"     {VENV_PYTHON} refs/run_ref.py {options.script or target.name}")
        print("   或者先装依赖：")
        print(f"     {sys.executable} -m pip install -r {WORKSPACE / 'refs' / 'requirements.txt'}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
