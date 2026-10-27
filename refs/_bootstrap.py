"""把任何解释器自动切换到项目 .venv。

为什么需要
----------
这台机器上同时存在三个 Python：

    /Users/zzx/.workbuddy-ai/binaries/python/.../python3   ← PATH 上的 python3，没有 numpy
    /usr/bin/python3                                        ← Apple 的，没有 numpy
    <工作区>/.venv/bin/python                               ← 项目虚拟环境，有 numpy/matplotlib/pyrr

所以 `python refs/run_ref.py ...` 和 VSCode 的 ▶/F5 都会报
`ModuleNotFoundError: No module named 'numpy'`——不是没装，是装在了另一个解释器里。

这个模块在进程启动的最早期检查：如果当前解释器缺依赖、而项目的 .venv 存在，
就用 os.execv 把整个进程换成 .venv 的 python 重新跑一遍自己。argv 原样带过去，
用户看到的还是同一条命令，只是背后换了个解释器。

调用方式（必须是每个入口文件的第一件事）：

    import _bootstrap
    _bootstrap.ensure()
"""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent          # refs/
_WORKSPACE = _HERE.parent                        # 工作区根目录
VENV_PYTHON = _WORKSPACE / ".venv" / "bin" / "python"

_GUARD = "REF_BOOTSTRAPPED"                      # 防止无限 execv
REQUIRED = ("numpy", "matplotlib")


def missing(modules=REQUIRED):
    """当前解释器缺哪些模块（用 find_spec，不真的 import，避免拖慢启动）。"""
    return [name for name in modules if importlib.util.find_spec(name) is None]


def ensure(modules=REQUIRED, quiet: bool = False) -> bool:
    """必要时把当前进程换成 .venv 的 python。返回 True 表示做了切换。"""
    if os.environ.get(_GUARD) == "1":
        return False                                 # 已经换过一次，别再递归
    absent = missing(modules)
    if not absent:
        return False                                 # 当前解释器没问题
    if not VENV_PYTHON.exists():
        return False                                 # 没有 venv 可换，交给调用方报错
    # 注意：venv 的 bin/python 通常是指向基础解释器的符号链接，所以不能比
    # sys.executable 的 realpath（会和 /usr/bin/python3 撞车）。要看 sys.prefix。
    try:
        if Path(sys.prefix).resolve() == (_WORKSPACE / ".venv").resolve():
            return False                             # 已经在 venv 里，只是真缺依赖
    except OSError:
        pass

    if not quiet:
        print(f"[自动切换解释器] {sys.executable} 缺 {', '.join(absent)}")
        print(f"[自动切换解释器] → {VENV_PYTHON}")
        sys.stdout.flush()                           # execv 不会帮你 flush
        sys.stderr.flush()
    os.environ[_GUARD] = "1"
    os.execv(str(VENV_PYTHON), [str(VENV_PYTHON)] + sys.argv)
    return True                                      # 走不到这里


def describe() -> str:
    """给报错信息用的一行说明。"""
    absent = missing()
    if not absent:
        return f"解释器 {sys.executable} 依赖齐全"
    return (f"解释器 {sys.executable} 缺 {', '.join(absent)}；"
            f"项目虚拟环境在 {VENV_PYTHON}")
