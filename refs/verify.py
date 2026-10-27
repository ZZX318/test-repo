"""一次性自检：把每条承诺都真的跑一遍，打印 PASS/FAIL。"""

import json
import os
import subprocess
import sys
from pathlib import Path

WS = Path(__file__).resolve().parent.parent   # refs/ -> 工作区根目录
PY = WS / ".venv" / "bin" / "python"
SYSTEM_PYS = ["/usr/bin/python3", "python3"]  # 两个都没有 numpy

results = []


def check(name, ok, detail=""):
    results.append((ok, name, detail))
    print(("PASS " if ok else "FAIL ") + name + (f"  {detail}" if detail else ""))


def run(cmd, env_extra=None):
    env = dict(os.environ, MPLBACKEND="Agg", **(env_extra or {}))
    # stdin 必须切断：否则 run_ref.py 的菜单分支会等 input() 卡死
    return subprocess.run(cmd, cwd=str(WS), capture_output=True, text=True, env=env,
                          stdin=subprocess.DEVNULL)


print("=" * 70)
print("1. 三种解释器跑同一个脚本，都必须成功")
print("=" * 70)
for py in [str(PY)] + SYSTEM_PYS:
    r = run([py, "refs/run_ref.py", "Astar3D.py"])
    ok = r.returncode == 0 and "1.2" in r.stdout or "node expanded" in r.stdout
    tail = [ln for ln in r.stdout.splitlines() if ln and "expanded" in ln][-1:] or [""]
    check(f"{py} -> Astar3D", r.returncode == 0, f"exit={r.returncode} {tail[0]}")

print()
print("=" * 70)
print("2. 自动切换解释器是否真的发生（系统 python 应该先打印 [自动切换]）")
print("=" * 70)
r = run(["/usr/bin/python3", "refs/run_ref.py", "Astar3D.py"])
check("系统 python 触发自动切换", "[自动切换解释器]" in r.stdout)
r = run([str(PY), "refs/run_ref.py", "Astar3D.py"])
check("venv python 不触发切换", "[自动切换解释器]" not in r.stdout)

print()
print("=" * 70)
print("3. --list / 无参数菜单 / 空文件剔除")
print("=" * 70)
r = run([str(PY), "refs/run_ref.py", "--list"])
check("--list 退出码 0", r.returncode == 0)
listed = [ln for ln in r.stdout.splitlines() if ln.strip()[:3].strip().rstrip(".").isdigit()
          and ".py" in ln]
check("--list 有 40 个脚本", len(listed) == 40, f"数到 {len(listed)}")
check("空文件不在列表里", "rrt_star_smart3D.py" not in r.stdout.split("（上游")[0])
check("空文件有单独提示", "0 字节空占位" in r.stdout)
r2 = run([str(PY), "refs/run_ref.py"])
check("无参数非 tty 只打印菜单", r2.returncode == 0 and "这只是菜单" in r2.stdout)

print()
print("=" * 70)
print("4. preview.py 在三种解释器下都能出图")
print("=" * 70)
for png in (WS / "refs" / "preview").glob("*.png"):
    png.unlink()
for py in [str(PY)] + SYSTEM_PYS:
    r = run([py, "refs/preview.py", "Search_2D/Dijkstra.py"])
    check(f"{py} -> preview", r.returncode == 0 and "完成 1/1" in r.stdout)
check("PNG 已生成", (WS / "refs" / "preview" / "Dijkstra.png").exists())

print()
print("=" * 70)
print("5. VSCode 配置")
print("=" * 70)
launch = json.loads((WS / ".vscode" / "launch.json").read_text())
cfgs = launch["configurations"]
check("launch.json 可解析", True, f"{len(cfgs)} 条配置")
missing_python = [c["name"] for c in cfgs if "python" not in c]
check("每条配置都写死 python 路径", not missing_python, str(missing_python))
check("python 路径指向 .venv",
      all(".venv/bin/python" in c.get("python", "") for c in cfgs))
json.loads((WS / ".vscode" / "settings.json").read_text())
check("settings.json 可解析", True)

print()
print("=" * 70)
print("6. 没有残留的 .command / 死链接")
print("=" * 70)
check("根目录没有 .command", not list(WS.glob("*.command")))
for doc in ["README.md", "refs/README.md"]:
    text = (WS / doc).read_text()
    check(f"{doc} 没有 run_gui.command / run_astar3d.command",
          "run_gui.command" not in text and "run_astar3d.command" not in text)

print()
print("=" * 70)
print("7. 主工程没被搞坏")
print("=" * 70)
r = run([str(PY), "tests/run_all.py"])
check("27 项测试全过", r.returncode == 0, r.stdout.strip().splitlines()[-1])
r = run([str(PY), "-m", "uav_planner.cli", "--uavs", "1", "--seed", "7"],
        env_extra={"PYTHONPATH": str(WS)})
check("主工程 CLI 能跑", r.returncode == 0 and "hard_clearance" in r.stdout)

print()
print("=" * 70)
bad = [name for ok, name, _ in results if not ok]
print(f"结果：{len(results) - len(bad)} 通过 / {len(bad)} 失败")
if bad:
    for name in bad:
        print("  失败:", name)
sys.exit(1 if bad else 0)
