# JPS Planner 优化与公式推导说明

这份文档对应当前工程中的两层规划逻辑：

1. `src/ne_global_planner/src/ne_grid_search.cpp` 中已经实现的 JPS 离散搜索。
2. `src/ne_global_planner/include/ne_global_planner/ne_traj_optimize.hpp` 与 `src/ne_global_planner/include/ne_global_planner/SplineTrajectory.hpp` 中的连续轨迹平滑/优化框架。

需要先说明一点：
当前仓库里真正“完整实现”的部分是 JPS 搜索和三次样条能量计算；`ne_traj_optimize.hpp` 里的 L-BFGS 连续优化流程还处于骨架阶段，变量名和求解流程尚未收口。因此下面会把“当前已实现的数学”与“代码显然打算采用的优化模型”分开写。

## 1. 整体思路

这套规划器可以理解为：

$$
\text{ESDF地图} \xrightarrow{\text{JPS}} \text{离散无碰路径} \xrightarrow{\text{样条参数化}} \text{连续轨迹} \xrightarrow{\text{代价优化}} \text{平滑可执行轨迹}
$$

其中：

- JPS 负责在栅格图中快速找到一条可行路径。
- ESDF 提供障碍物距离，用于安全性判定。
- 样条层把离散路径点变成连续轨迹。
- L-BFGS 预期用于优化中间控制点和总时间。

## 2. JPS 搜索层是怎么“优化”的

这里的“优化”不是连续优化，而是搜索空间剪枝。

### 2.1 普通 A* 的状态转移

若把每个栅格都作为节点，8 邻域扩展，则 A* 的状态转移为

$$
g(n') = g(n) + c(n,n')
$$

总代价为

$$
f(n)=g(n)+h(n)
$$

当前代码中的启发函数见 `getHeur()`：

$$
h(x,y)=\sqrt{(x-x_g)^2+(y-y_g)^2}
$$

也就是到目标点的欧氏距离。

对应实现位置：

- `src/ne_global_planner/src/ne_grid_search.cpp` 中 `getHeur()`
- `src/ne_global_planner/include/ne_global_planner/ne_grid_search.hpp` 中 `compare_state`

### 2.2 JPS 的核心：对称性剪枝

JPS 不再盲目扩展 8 个邻居，而是利用“当前节点的进入方向”来裁剪后继。

设当前节点为

$$
n=(x,y)
$$

父节点到当前节点的运动方向为

$$
(d_x,d_y)\in\{-1,0,1\}^2
$$

则 JPS 只保留：

1. 沿当前运动方向继续前进的自然邻居。
2. 遇到障碍物后产生的强迫邻居。

代码里这套邻接关系由 `JPS2DNeib` 预计算：

- `ns` 表示 natural neighbors
- `f1` 表示 forced-neighbor 检查方向
- `f2` 表示若存在强迫邻居时对应需要尝试的跳跃方向

### 2.3 跳点递归 `jump()`

JPS 的关键不是一步一步扩展，而是沿某方向递归“跳”：

设从节点 $(x,y)$ 沿方向 $(d_x,d_y)$ 前进，得到

$$
(x',y')=(x+d_x,\ y+d_y)
$$

若满足以下任一条件，则把 $(x',y')$ 视为 jump point：

1. $(x',y')$ 不可通行，则该方向失败。
2. $(x',y')$ 就是目标点。
3. $(x',y')$ 存在强迫邻居。
4. 若为对角运动，还要检查水平/竖直分量递归是否能产生跳点。

对应代码：

- `jump()`
- `hasForced()`
- `getJPSSucc()`

因此 JPS 的本质是把一长串“方向不变、没有新分支价值”的中间栅格折叠掉，只保留真正影响最优性的关键点。

### 2.4 代价函数

从当前节点到跳点的边代价为欧氏长度：

$$
c(n,n_{jp})=\sqrt{(x_{jp}-x)^2+(y_{jp}-y)^2}
$$

这正是 `getJPSSucc()` 里的计算方式。

于是搜索总代价为

$$
g(n_k)=\sum_{i=0}^{k-1}\sqrt{(x_{i+1}-x_i)^2+(y_{i+1}-y_i)^2}
$$

配合欧氏启发式，整体上仍然是一个 A* 框架，只是状态转移图被 JPS 剪枝后大幅压缩。

### 2.5 ESDF 安全判定

JPS 不是简单判断占用栅格，而是调用 ESDF 地图的碰撞接口：

$$
\text{free}(x,y)\iff \neg \text{isCollision}(p(x,y), d_{\text{safe}})
$$

其中：

- $p(x,y)$ 是格点对应的实际世界坐标；
- $d_{\text{safe}}$ 是安全距离阈值。

因此搜索空间实际上是“经过安全膨胀后的自由空间”：

$$
\mathcal{F}_{safe}=\{p\mid d_{\text{obs}}(p)>d_{\text{safe}}\}
$$

这意味着 JPS 本身已经做了一层几何安全性优化：它搜索的不是原始可通行区域，而是安全裕度内收之后的自由空间。

## 3. JPS 之后的连续优化变量

`ne_traj_optimize.hpp` 的设计意图很明确：先根据 JPS 路径生成中间点，再联合优化中间点和总时间。

### 3.1 优化变量

从 `optimizeSE2Traj()` 看，代码打算把变量写成

$$
\mathbf{x}=[\tau,\ \mathbf{p}_1^\top,\ \mathbf{p}_2^\top,\ \dots,\ \mathbf{p}_{N-1}^\top]^\top
$$

其中：

- $\tau$ 是总时间的无约束参数；
- $\mathbf{p}_i\in\mathbb{R}^2$ 是内部路径点；
- 起点 $\mathbf{p}_0$ 和终点 $\mathbf{p}_N$ 固定。

### 3.2 时间重参数化

代码通过 `logC2()` / `expC2()` 做了一个双向映射，把“优化变量中的实数 $\tau$”映射成“严格正的总时间 $T$”。

定义

$$
T=\operatorname{expC2}(\tau)=
\begin{cases}
\frac{1}{2}\tau^2+\tau+1, & \tau>0 \\
\frac{1}{\frac{1}{2}\tau^2-\tau+1}, & \tau\le 0
\end{cases}
$$

可以验证：

$$
T>0,\quad \forall \tau\in\mathbb{R}
$$

然后均匀分配到每一段：

$$
T_i=\frac{T}{N},\quad i=1,\dots,N
$$

对应代码：

- `expC2()`
- `calTfromTauUni()`

这一步的目的，是把带正约束的时间变量转成无约束变量，便于 L-BFGS 直接优化。

## 4. 三次样条轨迹的数学形式

这部分在 `SplineTrajectory.hpp` 里已经实现得很完整。

### 4.1 每段三次多项式

对第 $i$ 段，定义局部时间

$$
t\in[0,h_i]
$$

轨迹写成

$$
\mathbf{p}_i(t)=\mathbf{c}_{i0}+\mathbf{c}_{i1}t+\mathbf{c}_{i2}t^2+\mathbf{c}_{i3}t^3
$$

其中 $h_i$ 是该段时长。

代码中系数构造为：

$$
\mathbf{c}_{i0}=\mathbf{P}_i
$$

$$
\mathbf{c}_{i1}=\frac{\mathbf{P}_{i+1}-\mathbf{P}_i}{h_i}-\frac{h_i}{6}(2\mathbf{M}_i+\mathbf{M}_{i+1})
$$

$$
\mathbf{c}_{i2}=\frac{1}{2}\mathbf{M}_i
$$

$$
\mathbf{c}_{i3}=\frac{\mathbf{M}_{i+1}-\mathbf{M}_i}{6h_i}
$$

其中：

- $\mathbf{P}_i$ 是路径点；
- $\mathbf{M}_i$ 是节点处二阶导量。

这正对应 `solveSpline()` 中的四行系数构造。

### 4.2 二阶导的线性系统

三次样条满足位置连续、速度连续，并且边界速度固定。

令

$$
\mathbf{D}_i=\frac{\mathbf{P}_{i+1}-\mathbf{P}_i}{h_i}
$$

则内部方程是

$$
h_{i-1}\mathbf{M}_{i-1}+2(h_{i-1}+h_i)\mathbf{M}_i+h_i\mathbf{M}_{i+1}
=6(\mathbf{D}_i-\mathbf{D}_{i-1}),\quad i=1,\dots,N-1
$$

边界条件由起终点速度给出：

$$
2h_0\mathbf{M}_0+h_0\mathbf{M}_1
=6(\mathbf{D}_0-\mathbf{v}_s)
$$

$$
h_{N-1}\mathbf{M}_{N-1}+2h_{N-1}\mathbf{M}_{N}
=6(\mathbf{v}_e-\mathbf{D}_{N-1})
$$

这就是 `solveSpline()` 中 `internal_derivatives_` 的三对角线性方程组来源。

代码用的是手写 Thomas/LU 形式求解：

- `computeLUAndSolve()`
- `solveWithCachedLU()`

## 5. 平滑能量的公式推导

当前实现的能量是加速度平方积分：

$$
J_{energy}=\sum_{i=0}^{N-1}\int_0^{h_i}\|\mathbf{p}_i''(t)\|^2dt
$$

### 5.1 加速度表达式

由

$$
\mathbf{p}_i(t)=\mathbf{c}_{i0}+\mathbf{c}_{i1}t+\mathbf{c}_{i2}t^2+\mathbf{c}_{i3}t^3
$$

得

$$
\mathbf{p}_i''(t)=2\mathbf{c}_{i2}+6\mathbf{c}_{i3}t
$$

因此

$$
\|\mathbf{p}_i''(t)\|^2
=\|2\mathbf{c}_{i2}+6\mathbf{c}_{i3}t\|^2
$$

展开为

$$
4\|\mathbf{c}_{i2}\|^2
+24\,\mathbf{c}_{i2}^\top\mathbf{c}_{i3}\,t
+36\|\mathbf{c}_{i3}\|^2 t^2
$$

### 5.2 对时间积分

对 $t\in[0,h_i]$ 积分：

$$
J_i
=\int_0^{h_i}\left(
4\|\mathbf{c}_{i2}\|^2
+24\,\mathbf{c}_{i2}^\top\mathbf{c}_{i3}t
+36\|\mathbf{c}_{i3}\|^2 t^2
\right)dt
$$

得到

$$
J_i
=4\|\mathbf{c}_{i2}\|^2 h_i
+12\,\mathbf{c}_{i2}^\top\mathbf{c}_{i3} h_i^2
+12\|\mathbf{c}_{i3}\|^2 h_i^3
$$

总能量即

$$
J_{energy}=\sum_{i=0}^{N-1}J_i
$$

这和 `getEnergy()` 一一对应。

## 6. 能量梯度推导

### 6.1 对系数的偏导

对单段能量

$$
J_i
=4\|\mathbf{c}_{i2}\|^2 h_i
+12\,\mathbf{c}_{i2}^\top\mathbf{c}_{i3} h_i^2
+12\|\mathbf{c}_{i3}\|^2 h_i^3
$$

求偏导得

$$
\frac{\partial J_i}{\partial \mathbf{c}_{i2}}
=8\mathbf{c}_{i2}h_i+12\mathbf{c}_{i3}h_i^2
$$

$$
\frac{\partial J_i}{\partial \mathbf{c}_{i3}}
=12\mathbf{c}_{i2}h_i^2+24\mathbf{c}_{i3}h_i^3
$$

对 $\mathbf{c}_{i0},\mathbf{c}_{i1}$ 的偏导为 0。

这就是 `getEnergyPartialGradByCoeffs()` 的来源。

### 6.2 对段时间的偏导

固定系数时，对 $h_i$ 求导：

$$
\frac{\partial J_i}{\partial h_i}
=4\|\mathbf{c}_{i2}\|^2
+24\,\mathbf{c}_{i2}^\top\mathbf{c}_{i3}h_i
+36\|\mathbf{c}_{i3}\|^2 h_i^2
$$

这对应 `getEnergyPartialGradByTimes()`。

## 7. 从系数梯度传播到路径点梯度

样条优化真正想要的变量不是 $\mathbf{c}_{ik}$，而是路径点 $\mathbf{P}_i$ 和时间 $h_i$。

因此需要链式法则：

$$
\frac{dJ}{d\mathbf{P}}
=\frac{\partial J}{\partial \mathbf{C}}
\frac{\partial \mathbf{C}}{\partial \mathbf{P}}
\frac{\partial J}{\partial \mathbf{M}}
\frac{\partial \mathbf{M}}{\partial \mathbf{P}}
$$

$$
\frac{dJ}{dh}
=\frac{\partial J}{\partial h}
\frac{\partial J}{\partial \mathbf{C}}
\frac{\partial \mathbf{C}}{\partial h}
\frac{\partial J}{\partial \mathbf{M}}
\frac{\partial \mathbf{M}}{\partial h}
$$

其中难点在于 $\mathbf{M}$ 不是独立变量，而是由三对角方程

$$
A(h)\mathbf{M}=b(\mathbf{P},h)
$$

隐式决定。

### 7.1 隐函数求导

对上式微分：

$$
dA\cdot \mathbf{M}+A\cdot d\mathbf{M}=db
$$

故

$$
A\cdot d\mathbf{M}=db-dA\cdot \mathbf{M}
$$

若引入伴随变量 $\lambda$，满足

$$
A^\top \lambda = \frac{\partial J}{\partial \mathbf{M}}
$$

则可以避免显式求每个变量对 $\mathbf{M}$ 的 Jacobian，这就是 `propagateGrad()` 里再次调用缓存 LU 分解的根本原因。

代码逻辑是：

1. 先根据 $\partial J / \partial \mathbf{C}$ 组装出对 $\mathbf{P}$、$h$、$\mathbf{M}$ 的显式项。
2. 再解一个和原三对角系统同结构的伴随系统。
3. 把隐式依赖项补回到 $\mathbf{P}$ 和 $h$ 的梯度里。

所以 `propagateGrad()` 本质上是在做一次样条系统的反向传播。

## 8. `ne_traj_optimize.hpp` 预期的总目标函数

从参数定义可以看出，作者想优化的总代价应类似于

$$
J=
\rho_{energy}J_{energy}
+\rho_{T}J_T
+\rho_v J_v
+\rho_{collision}J_{col}
$$

其中可能各项为：

### 8.1 平滑项

$$
J_{energy}=\sum_{i=0}^{N-1}\int_0^{h_i}\|\mathbf{p}_i''(t)\|^2dt
$$

### 8.2 时间项

最自然的写法是

$$
J_T=T=\sum_i h_i
$$

或

$$
J_T=T^2
$$

当前从 `rho_T` 和 `total_time` 看，更像是直接惩罚总时长。

### 8.3 速度惩罚

如果最大速度约束写成软惩罚，通常形式是

$$
J_v=\sum_{i=0}^{N-1}\int_0^{h_i}\phi\left(\|\mathbf{p}_i'(t)\|^2-v_{max}^2\right)dt
$$

其中

$$
\phi(z)=\max(z,0)^2
$$

或其他光滑 hinge/barrier 函数。

### 8.4 碰撞惩罚

利用 ESDF 距离场，常见写法为

$$
J_{col}=\sum_{i=0}^{N-1}\int_0^{h_i}\psi\left(d_{safe}-d_{obs}(\mathbf{p}_i(t))\right)dt
$$

其中

$$
\psi(z)=\max(z,0)^2
$$

若轨迹点离障碍物太近，即 $d_{obs}<d_{safe}$，则开始产生惩罚。

### 8.5 数值积分离散化

参数里有 `int_K = 32`，这基本说明作者打算把积分离散成 K 个采样点：

$$
\int_0^{h_i} q(t)\,dt
\approx
\frac{h_i}{K}\sum_{k=1}^{K} q\left(\frac{k-\frac{1}{2}}{K}h_i\right)
$$

这样就能把速度约束和碰撞约束都写成可求梯度的和式。

## 9. 这份代码当前已经做到哪一步

### 已完成

- ESDF 安全判定下的 JPS 搜索。
- 基于边界速度约束的三次样条构造。
- 样条平滑能量 $J_{energy}$ 的解析计算。
- 能量对样条系数、段时间、路径点的梯度传播框架。

### 未完成或明显还在开发中

- `ne_traj_optimize.hpp` 中 `jps_plan()`、`optimizeSE2Traj()` 尚未闭合。
- `evaluateTrajectory()` 未实现。
- `rho_v`、`rho_collision`、`rho_T` 对应的目标项还没有真正接入求解。
- L-BFGS 回调函数尚未出现，说明总代价与总梯度还没拼装完成。

因此，更准确地说：

当前 `jps_planner` 的“优化”主要体现为：

1. JPS 对 A* 搜索空间的结构化剪枝优化。
2. 样条对离散路径的平滑优化。
3. 连续时空约束优化还在设计中，但参数和接口已经表明了目标方向。

## 10. 如果后面要把它补成完整优化器

可以按下面的数学顺序继续实现：

1. 用 JPS 路径生成初始内部点 $\mathbf{p}_1,\dots,\mathbf{p}_{N-1}$。
2. 用 $\tau \mapsto T$ 的映射生成正的总时间。
3. 通过 `generateTrajectory()` 构造三次样条。
4. 计算
   $$
   J=\rho_{energy}J_{energy}+\rho_TJ_T+\rho_vJ_v+\rho_{collision}J_{col}
   $$
5. 把各项梯度合并到
   $$
   \nabla_{\mathbf{x}}J=
   \left[\frac{\partial J}{\partial \tau},
   \frac{\partial J}{\partial \mathbf{p}_1},
   \dots,
   \frac{\partial J}{\partial \mathbf{p}_{N-1}}\right]^\top
   $$
6. 交给 L-BFGS 迭代求解。

## 11. 一句话总结

当前这部分 `jps_planner` 的优化可以概括为：

$$
\text{JPS 用剪枝优化搜索效率，样条用能量最小化优化轨迹平滑性，时空约束的联合优化框架已经搭好但尚未完全实现。}
$$
