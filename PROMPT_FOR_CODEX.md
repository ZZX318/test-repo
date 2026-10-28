
# 给 Codex 的任务提示词：把 UAV 3D 驾驶舱从「伪 3D」改造成真三维

> 用法：把本文件**整体**粘贴给 Codex / `codex` CLI。不要删掉「约束」和「验收标准」两节，
> 那是防止它改着改着把规划算法改坏、或者交一份看起来动了其实没动的 diff 的护栏。

---

## 0. 任务一句话

仓库 `test-repo` 是一个 Python 的 3D 多无人机路径规划实验台。它的浏览器驾驶舱
（`dashboard.html` + `uav_planner/dashboard.py`）名义上是 3D，实际渲染是**俯视正交投影**：
拖动鼠标竖直方向完全不改变画面，世界高度 z 根本不参与成像。请把它改造成真正的
**轨道摄像机（orbit camera）透视三维视图**，并按下面 P0 → P1 → P2 的顺序完成其余改进。

**硬性限制：不引入任何前端依赖**（不要 three.js / d3 / npm / 打包器），只用原生 Canvas 2D。

---

## 1. 先复现 bug（30 秒，别跳过）

`dashboard.html` 第 42 行的 `project()`：

```js
x = q[0]*cy - q[1]*sy;
y = q[0]*sy + q[1]*cy;
z = y*sp + q[2]*cp;                 // z 只被当作排序用的深度
return [W/2 + x*s, H/2 - y*s, z];   // 屏幕坐标里没有 q[2]
```

用 Node 复现：

```js
const cy=Math.cos(.65), sy=Math.sin(.65), cp=Math.cos(.42), sp=Math.sin(.42);
const P=q=>[q[0]*cy-q[1]*sy, -(q[0]*sy+q[1]*cy), (q[0]*sy+q[1]*cy)*sp+q[2]*cp];
console.log(P([0.5, 0.5, 0 ]));   // 高度 0
console.log(P([0.5, 0.5, 13]));   // 高度 13
// 前两个分量完全相同 => 13 m 的高度差在屏幕上等于 0 像素
```

我已经实测过的结论（请把它固化成回归测试）：

| 检查项 | 实测结果 |
|---|---|
| pitch 0.00 → 0.42 → 1.20，同一个点的屏幕位移 | **0.000 px** |
| (16,16,0) 与 (16,16,13) 的投影像素 | **完全相同** |
| 竖直拖拽的唯一可见效果 | 立方体按 z 重排绘制顺序，看起来像闪烁 |

这就是用户反馈的「**只能俯视、只能左右移动**」——竖直拖拽改的 `pitch` 只影响深度排序，
从来没进过成像公式。

---

## 2. P0 — 轨道摄像机（必须做，这是本次核心）

### 2.1 相机状态

```js
const cam = {
  yaw:      0.70,                 // 绕世界 +Z 旋转
  pitch:    0.55,                 // 仰角：0 = 平视地平线，+PI/2 = 正俯视
  distance: 62,                   // 到 target 的距离（透视模式）
  target:   [cx, cy, cz*0.7],     // 世界中心，略偏下
  fov:      48 * Math.PI / 180,
  near:     0.5,
  ortho:    false,                // 正交 / 透视可切换
};
```

**pitch 必须 clamp 到 [0.02, 1.50]（约 1.1°–86°）。** 原因：pitch = ±π/2 时视线与世界 up 轴
平行，`cross(forward, up)` 退化，右向量变 NaN，整幅画面直接消失。这是这类改造最经典的翻车点。

### 2.2 视图基与投影（照抄即可）

```js
const cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
const cy = Math.cos(cam.yaw),   sy = Math.sin(cam.yaw);

const eye = [
  cam.target[0] + cam.distance * cp * sy,
  cam.target[1] - cam.distance * cp * cy,
  cam.target[2] + cam.distance * sp,
];

const f = norm(sub(cam.target, eye));     // forward
const r = norm(cross(f, [0,0,1]));        // right（世界 up = +Z）
const u = cross(r, f);                    // up

const focal = (H / 2) / Math.tan(cam.fov / 2);

function project(p) {
  const d  = sub(p, eye);
  const zv = dot(d, f);
  if (zv < cam.near) return null;         // 相机后方：必须裁剪
  const xv = dot(d, r), yv = dot(d, u);
  return [
    W / 2 + xv * focal / zv,
    H / 2 - yv * focal / zv,
    zv,                                   // 第三位 = 深度，供排序
  ];
}
```

正交模式（`cam.ortho === true`）把 `focal / zv` 换成常数 `orthoScale`，深度仍然用 `zv`。

### 2.3 交互（全部要实现）

| 操作 | 行为 |
|---|---|
| 左键拖拽 | `yaw += dx*0.008`；`pitch = clamp(pitch + dy*0.008, 0.02, 1.50)` —— **两个轴都必须真实改变画面** |
| 滚轮 | `distance *= Math.exp(deltaY * 0.0012)`，clamp 到 [0.35, 4] × 适配距离 |
| 右键 / 中键 / Shift+左键 拖拽 | 平移 `cam.target`：把屏幕位移反投影到相机的 right / up 平面 |
| 双击 / 「重置视角」按钮 / 按 R | 回到默认 yaw/pitch/distance，并按包围盒重新 fit |
| 方向键、+ / - | 旋转、缩放 |
| 触屏 | 单指旋转；双指 pinch 缩放 + 平移 |
| 预设视角按钮 | 俯视 / 正视 / 侧视 / 等轴测 |

**必须用 `requestAnimationFrame` 合帧**：`pointermove` 里只置 `dirty = true`，而不是像现在
这样每次 `pointermove` 都做一次全量重投影 + 全量排序（4500 个立方体 × 8 次 project）。

另外加上 `canvas { touch-action: none; }`，否则触屏拖拽会滚动整个页面而不是转视角。

### 2.4 深度正确性（现在完全没有，这是"贴纸感"的来源）

当前的绘制顺序是「网格 → 按深度排序的立方体 → 路径 → 轨迹 → 标记」，所以**路径永远盖在
建筑上面**，绕到楼后面的那一截看起来像浮在楼前。要求：

1. 把所有待画图元（立方体面、球、线段、标记、坐标轴）放进**同一个** `prims` 数组，
   元素形如 `{ depth, draw() }`；
2. `prims.sort((a, b) => b.depth - a.depth)`，从远到近绘制；
3. 线段按**每一段的中点**深度参与排序（路径通常 20–40 段，代价可接受）；
4. 立方体做**背面剔除**：面法线与 `f` 点积 > 0 才画（现在 `cube()` 固定画 3 个固定朝向的面，
   旋转时朝向是错的）；
5. 半透明障碍先排序再合成，避免 alpha 叠加错乱。

### 2.5 空间参照物（现在一个都没有）

- 世界包围盒线框（12 条边，低透明度）
- 三根坐标轴箭头 + `x / y / z` 标签 + 单位（m）
- 地面网格 **和** 三个墙面网格（现在 `drawGrid()` 只画了地面，深度感全靠猜）
- 比例尺（自动选 1 / 2 / 5 / 10 m 的整数量级）
- z 轴高度刻度

### 2.6 障碍几何保真

`uav_planner/geometry.py` 定义了 Sphere / Box / Cylinder 三类障碍，但 `dashboard.py:81-84`
只把**体素中心点**回传，前端把所有东西都画成同样大小的方块——球形障碍看起来是方块。

- `/api/plan` 增加字段 `obstacles_geometry`：
  `[{"type":"sphere","center":[..],"radius":r}, {"type":"box","minimum":[..],"maximum":[..]}, {"type":"cylinder","center_xy":[..],"z_min":..,"z_max":..,"radius":..}]`
- 前端按类型渲染：球 → 屏幕半径 `radius * focal / zv` + 径向渐变；盒 → 6 面 + 背面剔除；
  柱 → 侧面四边形 + 顶盖椭圆
- 保留现有的体素点数组作为 LOD 兜底

---

## 3. P1 — 可玩性与信息密度

1. **轨迹回放**：播放 / 暂停 / 拖动进度条，逐帧显示每架 UAV 的位置与朝向。后端
   `plan.trajectories` 已经算好了，前端目前只静态画线。
2. **悬停高亮**：鼠标悬停某条路径 → 加粗高亮 + 右侧指标卡切换到该机的 plan / execution 两段指标。
3. **高度剖切**：z 高度滑块，只显示该高度以下的障碍与路径（看内部结构时非常有用）。
4. **导出**：当前视角 PNG；指标 JSON / CSV。
5. **全屏**按钮。
6. 键盘快捷键帮助浮层。

---

## 4. P2 — 后端对齐（消除"死控件"和性能坑）

`uav_planner/dashboard.py:44-51` 里这些是硬编码的，前端根本没有对应控件：

```python
resolution=value("resolution", 1.0, float), shape=(32, 32, 14),
margin=None, max_expansions=250_000, heuristic="euclidean",
no_diagonal=False, no_smooth=False,
```

要求：

1. `_args` 从 query 读取 `shape`（形如 `"32x32x14"`）、`heuristic`、`diagonal`、`smooth`、
   `max_expansions`；
2. 前端补控件：地图尺寸、分辨率、启发式（A* / Dijkstra）、是否允许对角、是否平滑；
3. 新增 `/api/meta` 返回可选布局、算法、shape 上限，前端用来动态生成下拉框；
4. **对 `/api/plan` 相同 query 做进程内 LRU 缓存**（key = 规范化后的 query）。用户反复调视角
   时不该重新规划；这也是目前最容易让人误以为"卡住"的地方；
5. `HTML_PATH.read_bytes()` 每个请求都读盘 → 启动时读一次；静态响应加 gzip；
6. `ThreadingHTTPServer` 并发规划会打满 CPU 且互相拖慢 → 用一把全局 `Lock` 串行化 `/api/plan`；
7. `serve(port=0)` 每次随机端口 → 增加 `--port` 参数，并把实际 URL 打印/写文件；
8. **障碍体素合并**：现在 4500 个独立立方体直接画。做一个贪心合并（同色相邻体素合并成 AABB），
   通常能把图元数降 5–10 倍，这是画质和帧率的关键；
9. **文档修正**：`README.md` 第 12、14 行还在指向 `refs/README.md`、`refs/run_ref.py`、
   `refs/preview/Astar3D.png`，但 `refs/` 目录在 `git status` 里已经是全 `D`（已删除）。
   要么恢复该目录，要么删掉这两段引用。

---

## 5. 约束（违反其中任何一条都视为未完成）

- 不引入任何前端依赖，保持"`python main.py` 一行启动 + 浏览器直开"的体验；
- 中文 UI 保留；README 的「30 秒上手」表需要同步更新；
- **不得改变规划算法的数值行为**：`tests/test_constraints.py` 必须全部继续通过
  （尤其是 `test_same_seed_is_bit_identical`、`test_reported_clearance_honours_hard_constraint`）；
- `python tests/run_all.py` 结束时必须仍然打印 `32 passed, 0 failed`；
- 如果拆分 JS，请拆到 `uav_planner/web/` 并在 `dashboard.py` 里加静态路由；
  即使保持单文件，也不要再把 JS 写成第 41–50 行那种几万字符的单行——
  每个函数独立成行并带注释；
- 单帧图元数控制在 5000 以内（靠体素合并 + LOD 达成）；
- 如果判断某一条不划算，在 PR 描述里写明理由，**不要静默跳过**。

---

## 6. 验收标准（逐条自测，缺一不可）

### A. 数学回归（新增 `tests/test_viewer_math.mjs`，用 `node` 直接跑，exit 0）

把投影数学抽成可被 Node import 的纯函数（例如 `uav_planner/web/viewer.js` 导出
`createCamera` / `projectPoint` / `orbitDrag`），然后断言：

1. 同一 `(x, y)`、`z = 0` 与 `z = 13` 两点，默认 pitch 下屏幕 y 差值 **> 20 px**；
2. pitch 从 0.2 变到 1.2，同一个点的屏幕坐标位移 **> 5 px**；
3. 位于相机后方的点返回 `null`；
4. 改变 yaw 使屏幕 x 坐标位移 **> 1 px**；
5. pitch 取边界值 0.02 与 1.50 时，投影结果 **不含 NaN**。

### B. 人工验收（README 或 PR 里附两张截图：改造前 / 改造后）

1. 能明显看出建筑的高度，并能看到绕到楼后面的那段路径（被楼正确遮挡）；
2. 竖直拖拽时画面明显俯仰；
3. 右键拖拽平移、滚轮缩放、双击重置全部可用；
4. 存在「俯视 / 正视 / 侧视 / 等轴测」预设且切换正确。

### C. 功能

1. 回放可播放/暂停/拖动；
2. 悬停高亮并按机切换指标；
3. `/api/plan` 返回合法的 `obstacles_geometry`，球形障碍渲染成球而不是方块。

### D. 回归

```bash
python tests/run_all.py          # 必须 32 passed, 0 failed
node tests/test_viewer_math.mjs  # 必须 exit 0
```

### E. 性能

默认场景（32×32×14、12 障碍、3 UAV）在 1920×1080 下拖拽时 **≥ 45 fps**
（用 `performance.now()` 统计 120 帧平均）。

---

## 7. 非目标（不要做，做了是浪费）

- 不要用 WebGL / three.js / Babylon 重写渲染层；
- 不要改 A* / RRT* / RRT-Connect 的数值行为或启发函数；
- 不要做多机时间维的冲突消解（CBS / ORCA），那是独立课题；
- 不要动 matplotlib 的 `--lab` 视图——mplot3d 的旋转本身是对的，问题只在 HTML 驾驶舱；
  可以在 README 里补一句说明两者区别。

---

## 8. 建议的提交切分

| PR / commit | 内容 |
|---|---|
| 1 | P0 相机 + 投影数学 + `tests/test_viewer_math.mjs` |
| 2 | 统一深度排序 + 背面剔除 + 坐标轴/网格/包围盒 |
| 3 | `obstacles_geometry` + 按类型渲染 + 体素合并 |
| 4 | P1 回放 / 高亮 / 剖切 / 导出 |
| 5 | P2 后端参数出口 + 缓存 + gzip + `--port` + README 修正 |

每个 commit 都必须能独立通过第 6 节的 A 和 D 两项。