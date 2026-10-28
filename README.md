# UAV 3D Planner

这是围绕论文复现目标整理的独立代码包。它把论文中的思路拆成可以单独测试和替换的模块：

```text
voxel world -> 26 邻域多目标 A* / RRT* -> 碰撞感知平滑
            -> 动态引导点 -> 限速限加速跟踪 -> 规划层/执行层指标
```

默认规划器是 26 邻域多目标 A*，支持硬净空约束、先规划无人机的路径分离代价、以及连续线段碰撞检查。实验室现在还提供 Dijkstra、Greedy、双向搜索、RRT-Connect 和 RRT*，所有算法都用同一个结果接口，方便在同一张地图上比较。

>

---

## 30 秒上手：我想做什么，就跑哪条

| 我想…… | 命令 |
| --- | --- |
| 打开实验驾驶舱（推荐） | `python main.py`（不带参数 = `--dashboard`） |
| 打开旧版键盘实验室 | `python main.py --lab` |
| 看 A\* 一步步怎么扩展（实时显示 g/h/f） | `python -m uav_planner.cli --step` |
| 看最终几条路径长什么样 | `python -m uav_planner.cli --plot` |
| 看无人机沿路径飞的过程 | `python -m uav_planner.cli --animate` |
| 比较 A\* 和 Dijkstra 差多少 | `python -m uav_planner.cli --compare` |
| 看障碍数对路径长度的影响 | `python -m uav_planner.cli --sweep 0,12,40` |
| 换地图风格 | `--layout walls` / `--layout spheres` / `--layout mixed` |
| 调硬安全距离 | `--hard-clearance 0.6` |
| 一次改多个权重 | `--weights "obstacle=12,clearance=3,hard=0.6"` |
| 拿数据自己画表 | `--json` |
| 跑全部测试 | `python tests/run_all.py` |

### 单步视图：窗口里四块信息分别是什么

| 位置 | 内容 |
| --- | --- |
| 窗口标题 | 已扩展节点数；h 模式、邻域数、`w_obs` / `w_clear` / `hard` / `min_sep`、每次按键步数 |
| 右上角图例 | 绿线=路径，橙点=open 待考察，灰点=closed 已扩展，红点=本步取出，粉块=障碍体素，绿圆=起点，黑星=终点 |
| 左下角 | 全部按键说明 + "怎么读"一句话（每步取出 f 最小 → 标 closed → 扩展邻居） |
| 右下角 | 本步节点的 g（已走代价）、h（估计剩余）、f=g+h、净空、open/closed 规模 |
| 画面中央 | 未开始时的大字提示"按 [空格] 开始第一步" |

鼠标左键拖动旋转、滚轮缩放、中键平移。窗口关闭后重新 `python main.py` 即可再来一次。

---


## 这一版改了什么（对应《批改_uav_3d_planner.md》）

| 批改条目 | 原状况 | 现在 |
| --- | --- | --- |
| 2 地图不构成约束 | 障碍 0→40 路径长度都是 31.33 m，占用率 4.8%，起终点压在 z=2/5 薄层 | 障碍预算决定**带缺口的墙**数量（`n // 12`），墙横跨整个 y-z 截面，缺口 (y, z) 由 seed 决定，26 邻域无法在 xy 平面内绕过；起终点在三维空间内随机分布。0/12/40 → **28.65 / 33.68 / 36.88 m** |
| 2 无可行性保证 | `raise RuntimeError("change seed")` | 生成后**验证**：端点在硬净空内、每个起终点对做 flood fill 连通性检查；不通就丢弃最后一个障碍重试，永不 raise |
| 3.1 硬约束在执行阶段失效 | 规划 0.69–1.41 m，执行 0.27 m < `hard_clearance` 0.35 | 前视距离随净空自适应收缩 + 每步用 `hard_clearance` 做安全截断。报告分**规划层/执行层**两段，各自给出 `hard_clearance -> OK/VIOLATED` |
| 3.2 每次扩展 4.52 ms | `segment_collision` 调 15,355 次，`signed_distance` 调 931,668 次（占 82%） | A* 内层改成纯数组查表（padding 平坦索引 + 预计算惩罚表 + 分离距离场），几何只在建图与最终校验时批量调用一次。**每次扩展 10 µs**（约 450×），单机单次规划 33 ms |
| 4 参数没有出口 | `CostWeights` 写死在 `cli.py`，`PlannerConfig` 7 个字段无人读取 | `PlannerConfig` 成为唯一真源，`to_weights()` 派生 `CostWeights`；全部权重/启发式/对角线/安全裕度/扩展上限都能从命令行传入 |
| 4 没有交互 | 静态 3D 图 + `plt.show()` | `--step` 交互 A*：下一步 / 跑到底 / 重置 / h 切换 / 实时显示 g、h、f=g+h；另有 `--compare`、`--sweep`、`--animate` |

### 自检结果（`--sweep 0,12,40`，seed 7，3 架 UAV）

```text
 obstacles    occ  mean_len  ratio  max_ratio  min_clr   ok   build    plan  expanded
         0  0.0%     28.65   1.00       1.00      inf   OK   0.01s   0.03s       509
        12  7.5%     33.68   1.18       1.50     0.60   OK   0.31s   0.13s      7990
        40 23.6%     36.88   1.29       1.68     0.45   OK   0.20s   0.13s      4969
```

- [x] 障碍数 0 / 12 / 40 三组，`mean_path_length` **不同**（28.65 / 33.68 / 36.88）
- [x] 至少一架 UAV `waypoints > 10` 且 `path_ratio > 1.15`（16 个航点，最大 1.50）
- [x] 同一 seed 重跑两次，指标完全一致（`test_same_seed_is_bit_identical`）
- [x] 任意 seed 不抛异常（seed 0–5 全部通过；可行性靠重采样 + 连通性验证）
- [x] 报告的 `min_clearance` ≥ `hard_clearance`（规划层与执行层都 ≥ 0.35 m）
- [x] 单机单次规划 < 1 s（实测 33 ms，3170 次扩展）
- [x] 所有权重能从命令行传入，不改源码（`--weights` / `--hard-clearance` 等）

---

## 在 VSCode 中运行

1. 用 VSCode 打开整个 `uav_3d_planner` 文件夹，不要只打开 `uav_planner` 子文件夹。
2. 打开 Terminal，创建虚拟环境并安装依赖：

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install -r requirements.txt
   ```

3. `Ctrl+Shift+P` → `Python: Select Interpreter` → 选 `.venv\\Scripts\\python.exe`。
4. 打开根目录 `main.py` 点运行按钮或按 `F5`：**无参数时直接打开实验驾驶舱**。

驾驶舱只服务于本目录的 `uav_planner/`，参考代码目录不会参与默认运行。左侧选择地图、算法、无人机数量、障碍密度和硬净空，点击一次即可同时查看三维路径、执行轨迹、规划层/执行层指标。拖动画布旋转视角，滚轮缩放。

### 交互视图按键

| 键 | 作用 |
| --- | --- |
| `空格` / `→` | 扩展一个节点（显示该节点的 g、h、f=g+h 与净空） |
| `a` | 一路跑到终点 |
| `r` | 重置 |
| `h` | 切换启发式：欧氏 ↔ 0（A\* ↔ Dijkstra），可现场看扩展数差异 |
| `s` | 切换每次按键扩展的节点数（1/2/4/8） |
| `q` | 关闭窗口 |

灰色 = 已关闭集合，橙色 = 开集合，红色 = 当前扩展节点，绿线 = 找到的路径。

---

## 命令行

```bash
pip install -e .
python main.py --planner astar --seed 7 --uavs 3 --obstacles 12
python -m uav_planner.cli --json          # 机器可读输出
```

### 场景

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--seed` | 7 | 决定起终点、墙缺口位置、随机障碍 |
| `--uavs` | 3 | 无人机数量 |
| `--obstacles` | 12 | 障碍预算，其中 `n // 12` 面墙（每面 4 个 box） |
| `--layout` | `mixed` | `mixed` 墙+球 / `walls` 只有墙 / `spheres` 纯随机球（旧行为） |
| `--resolution` / `--shape` | 1.0 / 32 32 14 | 体素分辨率与网格尺寸 |

### 代价与约束

| 参数 | 说明 |
| --- | --- |
| `--weights "obstacle=12,clearance=3,hard=0.6,min_sep=4"` | 一次性覆盖多个权重（别名：`hard`、`min_sep`、`length`、`separation`） |
| `--length-weight` `--obstacle-weight` `--separation-weight` | 单项覆盖，优先级高于 `--weights` |
| `--clearance` | 软净空半径（只影响选哪条路） |
| `--hard-clearance` | 硬安全约束（不满足直接判为不可行） |
| `--min-separation` | 多机最小间距（软代价） |
| `--heuristic euclidean\|zero` | `zero` 即 Dijkstra |
| `--no-diagonal` | 退化为 6 邻域 |
| `--margin` | passable 掩码的额外膨胀，默认半个体素对角线 |
| `--max-expansions` | 扩展上限 |
| `--no-smooth` | 跳过 shortcut + Chaikin |

### 模式

| 参数 | 作用 |
| --- | --- |
| `--step` | 交互式 A* 单步视图（见上） |
| `--compare` | 同一张地图上 A\* vs Dijkstra：扩展数、代价、耗时 |
| `--sweep 0,12,40` | 障碍数扫描，打印上面那张表 |
| `--animate` | 回放执行层轨迹 |
| `--plot` | 静态 3D 路径图 |
| `--json` | 输出 `{scenario, timings, metrics}`，其中 `metrics` 含 `plan` 与 `execution` 两段 |

### 实验室里的地图和算法

实验室不是只绑定一张演示地图。按 `m` 会依次切换：

| 地图键位顺序 | 场景 | 用途 |
|---:|---|---|
| 1 | `mixed` | 缺口墙 + 随机球，默认综合场景 |
| 2 | `walls` | 强制使用三维缺口的墙体场景 |
| 3 | `spheres` | 纯球形障碍，便于和旧版本对比 |
| 4 | `urban` | 不同高度的城市建筑块 |
| 5 | `forest` | 走廊附近高密度球形障碍 |
| 6 | `maze` | 交替墙体形成的三维迷宫 |
| 7 | `reference` | 论文参考风格的基准地图 |

按 `p` 会依次切换：

```text
astar → dijkstra → greedy → bidirectional → rrtconnect → rrtstar
```

其中 A*、Dijkstra 可以按空格逐节点观察；其余算法切换后会直接计算并显示结果。命令行也可以直接运行任意一种：

```bash
python main.py --planner bidirectional --layout urban --plot
python main.py --planner rrtconnect --layout maze --seed 11 --plot
python main.py --planner greedy --layout forest --json
```

新增算法都通过 `uav_planner/algorithms.py` 的统一 `PathResult` 接口返回路径、代价、扩展量和净空指标。后续增加 D* Lite、RRT-Connect 的动态重规划或 Hybrid A* 时，只需要接入这个分发层，地图和可视化不用重写。

### 后续扩展路线

| 阶段 | 内容 | 目的 |
|---|---|---|
| 当前 | 7 套静态 3D 地图 + 6 个搜索/采样算法 | 先把地图、算法、指标放到同一实验台比较 |
| 下一步 | D* Lite、Anytime D*、动态障碍回放 | 让无人机在飞行过程中重新规划 |
| 再下一步 | Hybrid A*、速度/转弯半径/姿态约束 | 从几何路径升级为可执行轨迹 |
| 多机阶段 | CBS、ORCA、时空预约表 | 处理多 UAV 的时间冲突和交叉航路 |
| 学习阶段 | A* 引导 + MAPPO/MADDPG + 安全层 | 在稳定基线之上研究强化学习改进 |

---

## 两个关键设计

**为什么 A\* 内层可以只查表？**
`passable(clearance, margin)` 要求每个单元格中心的解析净空 ≥ `clearance + margin`，默认 `margin = 0.5·√3·resolution`。解析净空是 1-Lipschitz 的，因此任意一条 26 邻域边上任意一点到障碍的距离都 ≥ `clearance`——硬约束由掩码本身保证，不需要逐边采样。几何计算只在建图（`clearance_grid`）和最终路径校验时各做一次。

**为什么执行层守得住？**
`GuidanceTracker.effective_lookahead()` 在净空小的时候收缩前视距离（切角量级≈前视距离），`KinematicModel._safe_fraction()` 再对每一步做安全截断：若整步不满足 `hard_clearance`，二分到最大的安全比例。所以报告的 `min_clearance` 不会低于约束值。

---

## 模块说明

| 模块 | 作用 |
| --- | --- |
| `environment.py` | 体素地图、球/盒/圆柱障碍、解析净空栅格、passable 掩码、距离场、连通性检查 |
| `geometry.py` | 障碍解析距离与连续碰撞检查（全部支持批量点） |
| `config.py` | `PlannerConfig`（唯一参数真源）、`CostWeights`、`DynamicsConfig`、`ScenarioConfig` |
| `astar3d.py` | 26 邻域多目标 A*（查表内层）、优先级多机规划、交互式 `AStarStepper3D` |
| `algorithms.py` | Dijkstra、Greedy、双向搜索、RRT-Connect 和统一算法分发 |
| `rrtstar3d.py` | 连续空间 RRT* 备选规划器 |
| `scenario.py` | 7 套地图布局、走廊偏置障碍、可行性验证/修复 |
| `smoothing.py` | 碰撞感知 shortcut 与 Chaikin 平滑（按 `hard_clearance` 校验） |
| `guidance.py` | AGP 风格投影、弧长前视、自适应前视、势函数奖励 |
| `dynamics.py` | 限速、限加速度点质量模型，带硬净空安全截断 |
| `fleet.py` | 组合全局规划、平滑、动力学 rollout |
| `metrics.py` | 规划层/执行层双份指标，显式报告 `hard_clearance` 是否守住 |
| `visualization.py` | 静态 3D 图、交互式 A* 单步视图、轨迹回放 |
| `rl_env.py` | 无 Gym 依赖的多无人机训练环境，支持 A\* 引导观测和稠密奖励 |
| `learning/attention_critic.py` | 可选 PyTorch 多头注意力 critic，方便后续接 MADDPG |

## 快速接入自己的地图

```python
import numpy as np
from uav_planner.config import PlannerConfig
from uav_planner.environment import VoxelMap
from uav_planner.fleet import FleetPlanner

world = VoxelMap(shape=(40, 40, 16), resolution=0.5)
world.add_sphere((10, 10, 4), radius=2.0)
world.add_box((15, 12, 0), (18, 25, 8))
world.add_cylinder((28, 20), 1, 12, 2)

cfg = PlannerConfig(shape=(40, 40, 16), resolution=0.5, hard_clearance=0.5)
fleet = FleetPlanner.from_config(world, cfg)
plan = fleet.plan([world.world_to_grid((1, 1, 2))], [world.world_to_grid((18, 18, 6))])
fleet.rollout(plan, [np.array((18, 18, 6))], hard_clearance=cfg.hard_clearance)
```

## 测试

```bash
pip install -e '.[dev]'
python3 -m pytest -q
```

虚拟环境里没有 pytest 时（离线装不上）也能跑，自带的 runner 不依赖 pytest：

```bash
python3 tests/run_all.py
```

`tests/test_constraints.py` 逐条对应批改文件第 5 节的自检清单，`tests/test_cli.py` 覆盖命令行参数出口，`tests/test_visualization.py` 会真的模拟按键去驱动交互视图。回归时会直接变红，而不是在报告里才发现。
