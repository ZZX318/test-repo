# refs/ —— 第三方参考工程（只读，不是我们自己的代码）

这里放的是**从网上原样下载的开源工程**，用来对照"真正的工程长什么样"。  
我们自己的代码仍然在上一层的 `uav_planner/`，两者互不 import。

---

## 0. 在 VSCode 里怎么跑（先看这个）

### 0.1 "头文件"在哪？—— Python 没有头文件

C/C++ 才有 `.h`。Python 里 `import` 的目标**就是同目录（或 sys.path 上）的 `.py` 文件本身**。  
这个仓库的"头文件"就是这几个同目录的 `.py`：

```text
Search_based_Planning/Search_3D/
├── Astar3D.py          ← 你要运行的主脚本
├── env3D.py            ← "头文件"：三维场景（起点、终点、blocks、balls），场景写死在这里
├── utils3D.py          ← "头文件"：getDist / cost / children / isCollide / Heuristic
├── plot_util3D.py      ← "头文件"：matplotlib 三维画图
└── queue.py            ← "头文件"：作者自己写的 MinheapPQ（注意：名字撞了标准库）
```

`Astar3D.py` 开头这几行就是在"包含头文件"：

```python
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search_based_Planning/")
from Search_3D.env3D import env                       # ← 相当于 #include "env3D.h"
from Search_3D.utils3D import getDist, cost, children  # ← 相当于 #include "utils3D.h"
```

**所以不用你手动加任何东西**，文件已经齐了。看到 Pylance 画红线是因为它在**静态分析阶段**不会执行 `sys.path.append`，那是运行时才生效的——红线是误报，能跑。

### 0.2 为什么 `import queue` 会拿到作者自己那个

```python
import queue                    # 第 18 行
self.OPEN = queue.MinheapPQ()   # 标准库的 queue 没有 MinheapPQ
```

Python 把**脚本所在目录**放在 `sys.path[0]`，优先级高于标准库，所以这里拿到的是同目录的 `queue.py`。  
这是上游的一个设计味道（用标准库的名字命名自己的模块），副作用是：**必须用"直接跑文件"的方式**，不能用 `python -m`、也不能把这个文件 import 进别的脚本跑，否则会拿到标准库的 `queue` 然后报 `AttributeError: module 'queue' has no attribute 'MinheapPQ'`。

### 0.3 怎么跑（只有一条路，两种入口）

**核心事实：这台机器上有三个 Python，只有 `.venv` 那个装了 numpy。**

```text
/Users/zzx/.workbuddy-ai/binaries/python/3.13.12/bin/python3   ← PATH 上的 python3，无 numpy
/usr/bin/python3                                               ← Apple 的，无 numpy
<工作区>/.venv/bin/python                                      ← 有 numpy 2.0.2 / matplotlib / pyrr
```

所以"我明明装了 numpy"是真的——装在了 `.venv` 里，但 `python` / VSCode ▶ / debugpy 用的都是前两个。

> ⚠️ **这三个都不能删，它们不是"重复安装"。**
>
> | 谁 | 干什么 | 删了会怎样 |
> | --- | --- | --- |
> | `~/.workbuddy-ai/binaries/python/3.13.12`（272 MB） | WorkBuddy 自己的运行时 | AI 助手跑不了代码，整个工具挂掉 |
> | `/Library/Developer/CommandLineTools/.../Python3.framework`（24 MB） | Apple 的 Xcode 命令行工具自带 | SIP 保护改不动；**而且 `.venv` 就建在它上面** |
> | `<工作区>/.venv`（81 MB） | 项目的依赖隔离 | numpy/matplotlib/pyrr 一起没了 |
>
> 关键证据：`.venv/bin/python3` 是个**符号链接**，指向
> `/Library/Developer/CommandLineTools/usr/bin/python3`——
> 虚拟环境不是复制一份解释器，而是"借"基础解释器、只把 site-packages 隔开。
> 删掉 CLT 那个，`.venv` 立刻报废，numpy 也跟着没。

**`refs/_bootstrap.py` 解决这件事**：任何入口在启动最早期检查依赖，缺的话用 `os.execv` 把进程换成 `.venv/bin/python` 重跑一遍自己。命令不用改，解释器自动换。

**① 终端（推荐，一条命令，任意 python 都能用）**

```bash
python refs/run_ref.py Search_3D/Astar3D.py      # 跑 3D A*（会弹图）
python refs/run_ref.py Search_2D/Dijkstra.py     # 跑 2D Dijkstra
python refs/run_ref.py --list                    # 列出全部 40 个脚本
python refs/run_ref.py                           # 不带参数 = 菜单 + 输编号
```

第一次跑会看到两行 `[自动切换解释器] ...`，说明它替你换到了 `.venv`；之后没有这行。

**② VSCode（`Run → Start Debugging`，或右上角 ▶）**

- `refs: 3D A* (Astar3D.py)`
- `refs: 运行当前打开的 .py` ← 通用，打开哪个文件跑哪个

`.vscode/launch.json` 每条配置都写了 `"python": "${workspaceFolder}/.venv/bin/python"`，
加上 `_bootstrap.py` 兜底，**选哪个解释器都不会再报 numpy 缺失**。

**③ 只想要 PNG（不弹窗，SSH / CI 也能用）**

```bash
python refs/preview.py                            # 默认出 Astar3D + Astar 两张
python refs/preview.py Search_3D/DstarLite3D.py   # 指定脚本
```

输出到 `refs/preview/*.png`，后端强制 Agg。

> `run_ref.py` 干的事是复刻 `python Astar3D.py`：切 `cwd`、把脚本目录塞进 `sys.path[0]`、再 `runpy` 执行。所以从哪个目录启动都行。
>
> ⚠️ **不带参数时只是菜单，它自己不算法、不画图。** 屏幕上出现一长串 `rrt_2D/... rrt_3D/...` 就是走到了菜单分支，不是报错。

### 0.4 没有图？先看这张表

| 现象 | 原因 | 怎么办 |
| --- | --- | --- |
| 终端刷出一长串 `rrt_2D/... rrt_3D/...`，没有窗口 | 你跑的是 `run_ref.py` 的**菜单分支**（没带脚本名） | `python refs/run_ref.py Search_3D/Astar3D.py` |
| 输入编号后**立刻回到提示符**，什么都没有 | 选到了上游的**空文件**（0 字节占位符） | 已在菜单里剔除并单独提示；重跑，选 **2. Search_3D/Astar3D.py** |
| 不知道 F5 在哪 | MacBook 可能没有实体 F5（Touch Bar） | 用菜单 **Run → Start Debugging**，或右上角 ▶ |
| `ModuleNotFoundError: No module named 'numpy'` | 用了系统 Python（不是 `.venv`） | **已自动处理**：`_bootstrap.py` 会 execv 到 `.venv`。若仍报错，说明 `.venv` 也没装：`python -m pip install -r refs/requirements.txt` |
| `ModuleNotFoundError: No module named 'pyrr'` | 没装上游的三维依赖 | `python -m pip install -r refs/requirements.txt` |
| 窗口一闪而过 / 看不到 | 脚本跑完 `plt.show()` 后退出，窗口跟着关 | 用 `refs: 运行当前打开的 .py` 调试配置（终端不退出），或 `python refs/preview.py` 存图 |
| 图是**俯视图**，看起来像 2D | 上游把视角写死成 `ax.view_init(elev=90., azim=0.)` | 那是 3D 场景的俯视投影。鼠标拖动可以转；想看立体角度改 `plot_util3D.py` 的 `view_init` |
| `AttributeError: module 'queue' has no attribute 'MinheapPQ'` | 用 `python -m` 或 import 方式跑，`sys.path[0]` 不是脚本目录 | 用 `python <路径>/xxx.py` 或 `run_ref.py` |

### 0.5 已经配好的 VSCode 设置

`.vscode/settings.json` 里加了：

| 设置 | 作用 |
| --- | --- |
| `python.defaultInterpreterPath` | 默认用 `.venv/bin/python`，避开系统 Python 没 numpy 的坑 |
| `python.terminal.executeInFileDir` | ▶ 按钮执行时把终端 cwd 设成文件目录 |
| `python.analysis.extraPaths` | 让 Pylance 认识 `Search_3D.*` / `rrt_3D.*`，少一大半红线 |

还剩一种红线消不掉：`queue.MinheapPQ` 会被标成"未知属性"，因为 Pylance 认为 `queue` 是标准库。**这条是上游命名冲突导致的，不影响运行**，要么无视，要么在 `Astar3D.py` 里把 `import queue` 改成 `from . import queue`（会破坏直接运行的写法，不建议）。

### 0.6 那 40 个脚本都是什么？都要看吗？

**不用。** 上游是一个"算法全家桶"，40 个里真正和阶段一有关的是 3 个。

| 类别 | 数量 | 是什么 | 阶段一要不要看 |
| --- | --- | --- | --- |
| `Search_3D` | 8 | 三维栅格搜索：A\*、双向 A\*、D\*、D\* Lite、Anytime D\*、LPA\*、LRTA\*、RTAA\* | **只看 `Astar3D.py`**（★） |
| `Search_2D` | 13 | 二维栅格搜索，同上家族 + BFS/DFS/贪心 | **看 `Astar.py` + `Dijkstra.py`**（★），这俩只差一个 h |
| `rrt_3D` | 9 | 三维采样：RRT、RRT\*、informed RRT\*、RRT-Connect、BIT\*、ABIT\*、FMT\* 等 | 阶段二/三再看，`rrt_star3D` 是代表 |
| `rrt_2D` | 10 | 二维采样，同上 | 不用看 |

为什么是 40 而不是 44：上游原本 44 个 `.py`，其中 **4 个是 0 字节空占位符**（`rrt_star_smart3D.py`、`rrt_sharp.py`、`adaptively_informed_trees.py`、`advanced_batch_informed_trees.py`），作者没写完就提交了。跑它们什么都不会发生，所以从菜单里剔掉了。

### 0.7 只能看最终结果？没有一步一步走？

分三种情况，别混：

| 脚本 | 有没有过程 | 说明 |
| --- | --- | --- |
| `Search_2D/Astar.py` | ✅ 有 | 跑完调 `plot.animation(path, visited)`，**逐帧回放整个搜索过程**——能看到 open/closed 怎么长。这是上游唯一"能看过程"的 A\* |
| `Search_3D/Astar3D.py` | ❌ 没有 | 第 57 行 `# visualization(self)` 被作者**注释掉了**（每帧重画 1.4 万个格子太慢），所以只有最终一张图 |
| 我们自己的 `main.py --step` | ✅ 真·单步 | 空格扩展一个节点，标题栏实时显示当前节点的 **g、h、f=g+h 和净空**，还能 `h` 键现场切 A\*/Dijkstra 对比扩展数 |

**想要"一步一步 + 看 g/h/f 数值"，用我们自己的**：

```bash
python main.py --step
# 空格=下一步  a=跑到底  r=重置  h=切启发式  s=改步长  q=退出
```

上游那个 3D A\* 想加动画也不是不行：把 `Astar3D.py` 第 57 行的注释去掉即可，但它是每扩展一次就整张重画，2400 次扩展会慢到没法看——正确做法是每隔 N 次画一次，这也是作者当初注释掉它的原因。

---

## 1. PathPlanning（zhm-real）

| 项 | 值 |
| --- | --- |
| 上游 | <https://github.com/zhm-real/PathPlanning> |
| 作者 | Huiming Zhou (zhm-real) |
| 热度 | 9381 stars / 1773 forks（2026-10 查询） |
| 提交 | `89c200e0ab855010e2f892b6465b50dab002963c`（2020-12-14，master 最后一个提交） |
| 许可证 | MIT（见 `PathPlanning/LICENSE`） |
| 体积 | 7.2 MB（含 README 里的 gif 动画） |
| 语言 | Python + NumPy + Matplotlib（+ `pyrr` 做三维矩阵） |

下载方式：`https://codeload.github.com/zhm-real/PathPlanning/tar.gz/refs/heads/master`，解包后剔除了 `.git` / `__pycache__` / `.idea`。**没有改任何算法逻辑**，只打了下面 2 处 numpy 兼容补丁。

### 1.1 它有什么

```text
PathPlanning/
├── Search_based_Planning/
│   ├── Search_2D/     Astar, Dijkstra, Best_First, BFS, DFS,
│   │                  ARA*, Anytime D*, D*, D* Lite, LPA*, LRTA*, RTAA*, Bidirectional A*
│   └── Search_3D/     Astar3D, bidirectional_Astar3D, Dstar3D, DstarLite3D,
│                      Anytime_Dstar3D, LP_Astar3D(LPA*), LRT_Astar3D(LRTA*), RTA_Astar3D(RTAA*)
├── Sampling_based_Planning/
│   ├── rrt_2D/
│   └── rrt_3D/        rrt3D, rrt_star3D, informed_rrt_star3D, rrt_connect3D,
│                      BIT*, ABIT*, FMT*, dynamic_rrt, rrt_star_smart
└── CurvesGenerator/   Dubins / Reeds-Shepp / Bezier / spline 曲线
```

每个算法都配了 matplotlib 动画，`README.md` 里有对应的 gif 和论文链接。

已经生成好的预览图在 `refs/preview/`（`Astar3D.png`、`Astar.png`），是 `preview.py` 用 Agg 后端跑出来的——可以先看图再决定跑哪个。

### 1.2 为什么下它（对上阶段一）

- `Search_3D/Astar3D.py`（115 行）就是**真正的三维 26 邻域 A\***，和我们要讲的 g/h/f 完全对得上：`self.OPEN.put(xj, a + 1 * heuristic_fun(self, xj))` 里 `a` 是 g、`heuristic_fun` 是 h。
- `Search_2D/Astar.py` 与 `Dijkstra.py` 只差一个 h 函数——正好验证"h = 0 就是 Dijkstra，扩展节点更多"。
- `Search_3D/utils3D.py` 的 `children()` 把 26 个邻居全展开、`cost()` 按欧氏距离算步长——可以直接量出**三维栅格里节点数为什么爆炸**（32×32×14 就是 14336 个格子，26 邻域就是 372k 条边）。
- `Astar3D.py` 里有个 `self.settings = 'NonCollisionChecking' / 'CollisionChecking'` 开关，是上游作者留的"边是否做连续碰撞检查"的开关，和我们第 1 节夸过的那处设计是同一个问题。

### 1.3 怎么跑

```bash
# 依赖：numpy / matplotlib / pyrr（上游没给 requirements.txt，这里补了一份）
pip install -r refs/requirements.txt

# 在仓库根目录，任意 cwd 都能跑（见 0.3 ③）
python refs/run_ref.py Search_3D/Astar3D.py      # 3D A*，实测约 2400 次扩展 / 1.2 s
```

也可以按上游原来的方式（必须 `cd` 进去）：

```bash
cd refs/PathPlanning/Search_based_Planning/Search_3D
python Astar3D.py
```

```bash
cd refs/PathPlanning/Search_based_Planning/Search_2D
python Astar.py              # 2D A*
python Dijkstra.py           # 2D Dijkstra，对比扩展数
python Best_First.py         # 只按 h 走，能看出"贪心但非最优"
```

```bash
cd refs/PathPlanning/Sampling_based_Planning/rrt_3D
python rrt_star3D.py         # 实测 70 s，路径长 17.83（三维 RRT* 重连是 O(n²)，慢是正常的）
```

实测记录（本机，Python 3.9 + numpy 2.0.2）：

| 脚本 | 结果 |
| --- | --- |
| `Search_3D/Astar3D.py` | 约 2400 次扩展，1.2 s，找到路径 |
| `Search_2D/Astar.py` / `Dijkstra.py` / `Best_First.py` | 均正常出图 |
| `rrt_3D/rrt_star3D.py` | 70 s，Total distance 17.83 |

> ⚠️ 边界条件：`sys.path.append` 用的是 `os.path.abspath(__file__)`，所以**从任何 cwd 用 `python <路径>/Astar3D.py` 跑都可以**（我实测过）。  
> 真正不行的是 `python -m` 模块模式、以及"被别的脚本 import 进来跑"——那两种情况 `sys.path[0]` 不是脚本目录，`import queue` 会拿到标准库的 `queue`，然后 `MinheapPQ` 找不到。原因见 [0.2](#02-为什么-import-queue-会拿到作者自己那个)。

### 1.4 打的补丁（只有 2 处，都是 numpy ≥ 1.25 的兼容问题）

| 文件 | 原代码 | 改成 | 原因 |
| --- | --- | --- | --- |
| `Search_based_Planning/Search_3D/plot_util3D.py:74` | `if SET != []:` | `if len(SET) > 0:` | `SET` 是 `(n,2,3)` 的 ndarray，新版 numpy 里 `ndarray != []` 直接抛 `ValueError: operands could not be broadcast`，A* 算完路径后在画图这一步崩 |
| `Sampling_based_Planning/rrt_3D/plot_util3D.py:74` | 同上 | 同上 | 同上 |

补丁位置都留了 `# PATCH(numpy>=1.25)` 注释，方便以后 `diff` 上游。

### 1.5 它**没有**什么（别指望它）

- ❌ 没有多机协同 / 优先级规划 / 机间分离代价 —— 全是单机。
- ❌ 没有 AGP 的 guidance point、没有 MADDPG、没有任何强化学习。
- ❌ 没有硬净空约束的概念（只有"碰/不碰"的 0/1 判定），也没有净空指标。
- ❌ 没有单元测试、没有 CLI、没有配置文件；`env3D.py` 里场景是写死的。

所以它是**阶段一的算法参照**，不是 AGP-MADDPG 的复现。论文那边没有官方代码（全文里搜不到 GitHub 链接），复现只能自己写——这一层结论没有变。

---

## 2. 怎么和 `uav_planner/` 对照看

| 维度 | `refs/PathPlanning`（上游） | `uav_planner/`（我们的） |
| --- | --- | --- |
| 邻域 | 26 邻域，显式字典列 26 个方向 | 26 邻域，用 offset + 平坦索引 |
| 碰撞检查 | `isCollide()` 按分辨率采样解析距离，在**扩展循环里** | 预计算 passable 掩码，**循环里只查表** |
| 净空 | 无（只有碰撞/不碰撞） | 软净空代价 + 硬净空约束，两层分开 |
| 多机 | 无 | 优先级规划 + 分离代价 |
| 平滑 | 无 | shortcut + Chaikin，且按硬净空复验 |
| 执行层 | 无 | 限速限加速度 + 引导点跟踪 + 安全截断 |
| 指标 | 无（只有画图） | 成功率/净空/间距/路径效率/平滑度 |
| 测试 | 无 | `tests/` 27 项 |

一句话：**上游是"算法演示"，我们是"能跑的工程"**。上游赢在算法覆盖面（D\* Lite、BIT\*、ABIT\*、FMT\* 都有，我们只有 A\* 和 RRT\*），我们赢在约束、多机、执行层和可测性。阶段一该看上游的 `Astar3D.py`，因为它最干净。
