"""把 refs/PathPlanning 里 40 个脚本逐个真跑一遍，找"跑不起来的"。

为什么要模拟 debugpy
--------------------
`Search_3D` 下 5 个文件原来写的是裸 `import queue`，靠 sys.path[0] 拿到同目录的
`queue.py`。但 VSCode 调试器（debugpy）启动时会先 import 标准库的 `queue`，
`sys.modules` 一旦有了它，后面 `import queue` 就永远返回标准库那个 →
`AttributeError: module 'queue' has no attribute 'MinheapPQ'`。

所以这里故意在跑脚本前 `import queue`，复现调试器环境。修复后应该全部通过。

    python refs/smoke.py              # 全部
    python refs/smoke.py Search_3D    # 只测某一类
"""

from __future__ import annotations

import concurrent.futures as futures
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _bootstrap  # noqa: E402

_bootstrap.ensure()

from run_ref import ROOT, discover  # noqa: E402

TIMEOUT = 40  # 秒；够长的脚本会显示"仍在运行"，那不是报错
WORKERS = 4

# 用子进程跑，并且在子进程里先 import 标准库 queue —— 精确模拟 debugpy
CHILD = (
    "import queue, runpy, sys;"
    "sys.argv = [{p!r}];"
    "runpy.run_path({p!r}, run_name='__main__')"
)


def one(item):
    folder, script = item
    code = CHILD.format(p=str(script))
    env = dict(os.environ, MPLBACKEND="Agg")
    try:
        r = subprocess.run([sys.executable, "-c", code], cwd=str(script.parent),
                           capture_output=True, text=True, timeout=TIMEOUT, env=env,
                           stdin=subprocess.DEVNULL)
    except subprocess.TimeoutExpired:
        return folder, script.name, "SLOW", f">{TIMEOUT}s 仍在运行（不是报错）"
    if r.returncode == 0:
        return folder, script.name, "OK", ""
    lines = [ln for ln in (r.stderr or "").strip().splitlines() if ln.strip()]
    err = lines[-1] if lines else f"exit={r.returncode}"
    return folder, script.name, "FAIL", err[:110]


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    scripts = discover()
    if argv:
        scripts = [s for s in scripts if s[0] in argv]
    print(f"测试 {len(scripts)} 个脚本（超时 {TIMEOUT}s，并发 {WORKERS}）\n")

    rows = []
    with futures.ThreadPoolExecutor(max_workers=WORKERS) as pool:
        for row in pool.map(one, scripts):
            rows.append(row)
            print(f"{row[2]:>4}  {row[0]:<10} {row[1]:<32} {row[3]}")

    ok = sum(1 for r in rows if r[2] == "OK")
    slow = sum(1 for r in rows if r[2] == "SLOW")
    bad = [r for r in rows if r[2] == "FAIL"]
    print(f"\nOK {ok} / 慢 {slow} / 失败 {len(bad)} / 共 {len(rows)}")
    for r in bad:
        print(f"  失败: {r[0]}/{r[1]}  {r[3]}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
