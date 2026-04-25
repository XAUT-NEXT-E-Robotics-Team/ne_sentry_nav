# JPS 与连续轨迹优化公式推导

这份文档对应当前工程中的两层规划逻辑：

1. `src/ne_global_planner/src/ne_grid_search.cpp` 中的 ESDF-JPS 离散搜索。
2. `src/ne_global_planner/include/ne_global_planner/ne_traj_optimize.hpp` 和 `src/ne_global_planner/include/ne_global_planner/SplineTrajectory.hpp` 中的三次样条连续轨迹优化。

整体流程可以写成：

$$
\text{ESDF 地图}
\xrightarrow{\text{JPS}}
\text{离散安全路径}
\xrightarrow{\text{三次样条}}
\text{连续轨迹}
\xrightarrow{\text{L-BFGS}}
\text{平滑、安全、满足速度约束的轨迹}
$$

当前 `ne_traj_optimize.hpp` 已经把 L-BFGS、约束代价、ESDF 距离场、样条梯度反传等模块串起来，但文件里仍有一些拼写、作用域和返回类型问题会阻塞编译。本文先推导代码表达的数学模型，最后单独列出代码核对清单。

## 1. JPS 搜索层

### 1.1 状态、代价与优先级

JPS 搜索的节点状态为：

$$
s=(x,\ y,\ d_x,\ d_y)
$$

其中 $(x,y)$ 是栅格坐标，$(d_x,d_y)$ 是父节点到当前节点的进入方向。

优先队列使用 A* 形式：

$$
f(s)=g(s)+h(s)
$$

`getHeur()` 中的启发函数为欧氏距离：

$$
h(x,y)=\sqrt{(x-x_g)^2+(y-y_g)^2}
$$

跳点边代价同样按欧氏长度计算：

$$
c(s_i,s_j)=\sqrt{(x_j-x_i)^2+(y_j-y_i)^2}
$$

因此路径累计代价为：

$$
g(s_k)=\sum_{i=0}^{k-1} c(s_i,s_{i+1})
$$

### 1.2 ESDF 安全自由空间

`EsdfSearch::isFree()` 并不是只检查占用栅格，而是把栅格点转成世界坐标后调用：

$$
\text{free}(p)\iff \neg\text{isCollision}(p,d_{safe})
$$

也就是：

$$
\mathcal{F}_{safe}=\{p\mid d_{obs}(p)\ge d_{safe}\}
$$

JPS 实际搜索的是被安全距离内收后的自由空间。这个设计让离散路径天然带一层安全裕度。

### 1.3 自然邻居与强迫邻居

`JPS2DNeib` 预计算三类进入方向：

$$
\|d\|_1 \in \{0,1,2\}
$$

分别表示起点、直线运动和对角运动。

每个方向有两组邻接关系：

- `ns`：natural neighbors，自然邻居。
- `f1` / `f2`：forced-neighbor 检查方向和由障碍触发的新跳跃方向。

JPS 的剪枝思想是：若沿当前方向继续走不会产生新的分支价值，就跳过中间格点，只保留跳点。

### 1.4 跳点递归

从 $(x,y)$ 沿 $(d_x,d_y)$ 前进：

$$
(x',y')=(x+d_x,\ y+d_y)
$$

`jump()` 的停止条件是：

1. $(x',y')$ 不在安全自由空间内，则该方向失败。
2. $(x',y')$ 到达目标点，则成功。
3. $(x',y')$ 存在强迫邻居，则成功。
4. 若当前是对角运动，还会递归检查水平和竖直分量是否能产生跳点。

因此 JPS 可以理解为在 A* 的框架内，用方向相关的剪枝规则压缩搜索图。

## 2. 连续优化变量

JPS 给出离散路径后，连续优化固定起点和终点，只优化内部路径点与总时间。

设轨迹被分成 $N$ 段，节点为：

$$
P_0,\ P_1,\ \dots,\ P_{N-1},\ P_N,\quad P_i\in\mathbb{R}^2
$$

其中 $P_0$ 和 $P_N$ 固定，内部点 $P_1,\dots,P_{N-1}$ 是优化变量。

`optimizeSE2Traj()` 中的优化向量为：

$$
x=
\begin{bmatrix}
\tau \\
\operatorname{vec}(P_1,\dots,P_{N-1})
\end{bmatrix}
\in\mathbb{R}^{1+2(N-1)}
$$

这里 $\tau$ 是无约束时间变量，真实总时间由 `expC2()` 映射得到。

## 3. 时间重参数化

为了让 L-BFGS 在无约束空间里工作，同时保证总时间始终为正，代码定义：

$$
T=\operatorname{expC2}(\tau)=
\begin{cases}
\frac{1}{2}\tau^2+\tau+1, & \tau>0 \\
\frac{1}{\frac{1}{2}\tau^2-\tau+1}, & \tau\le 0
\end{cases}
$$

该映射满足：

$$
T>0,\quad \forall \tau\in\mathbb{R}
$$

它的导数是：

$$
\frac{dT}{d\tau}=
\begin{cases}
\tau+1, & \tau>0 \\
\frac{1-\tau}{\left(\frac{1}{2}\tau^2-\tau+1\right)^2}, & \tau\le 0
\end{cases}
$$

这正对应 `getTtoTauGrad()`。

当前代码用均匀分配：

$$
h_i=\frac{T}{N},\quad i=0,\dots,N-1
$$

因此若已有各段时长梯度 $\partial J/\partial h_i$，则：

$$
\frac{\partial J}{\partial T}
=
\sum_{i=0}^{N-1}
\frac{\partial J}{\partial h_i}
\frac{\partial h_i}{\partial T}
=
\frac{1}{N}\sum_{i=0}^{N-1}\frac{\partial J}{\partial h_i}
$$

加上时间正则项 $J_T=\rho_TT$ 后，代码中的总时间梯度为：

$$
\frac{\partial J}{\partial \tau}
=
\left(
\rho_T+\frac{1}{N}\sum_{i=0}^{N-1}\frac{\partial J}{\partial h_i}
\right)
\frac{dT}{d\tau}
$$

## 4. 三次样条轨迹

### 4.1 分段多项式

对第 $i$ 段，局部时间 $t\in[0,h_i]$，轨迹写成三次多项式：

$$
p_i(t)=c_{i0}+c_{i1}t+c_{i2}t^2+c_{i3}t^3
$$

其中 $p_i(t)\in\mathbb{R}^2$，系数 $c_{ik}\in\mathbb{R}^2$。

速度和加速度为：

$$
v_i(t)=p_i'(t)=c_{i1}+2c_{i2}t+3c_{i3}t^2
$$

$$
a_i(t)=p_i''(t)=2c_{i2}+6c_{i3}t
$$

### 4.2 节点二阶导线性系统

令：

$$
D_i=\frac{P_{i+1}-P_i}{h_i}
$$

代码用固定起末速度边界，求解节点二阶导：

$$
M_i=p''(t_i)
$$

边界方程为：

$$
2h_0M_0+h_0M_1=6(D_0-v_s)
$$

$$
h_{N-1}M_{N-1}+2h_{N-1}M_N=6(v_e-D_{N-1})
$$

内部节点满足：

$$
h_{i-1}M_{i-1}+2(h_{i-1}+h_i)M_i+h_iM_{i+1}
=
6(D_i-D_{i-1}),\quad i=1,\dots,N-1
$$

写成矩阵形式：

$$
A(h)M=b(P,h)
$$

`computeLUAndSolve()` 用三对角 Thomas 消元求解，并缓存 LU 相关量，供后续梯度反传复用。

### 4.3 系数构造

求得 $M_i$ 后，每段系数为：

$$
c_{i0}=P_i
$$

$$
c_{i1}=D_i-\frac{h_i}{6}(2M_i+M_{i+1})
$$

$$
c_{i2}=\frac{1}{2}M_i
$$

$$
c_{i3}=\frac{M_{i+1}-M_i}{6h_i}
$$

这正对应 `CubicSplineND::solveSpline()`。

## 5. 平滑能量

当前三次样条的平滑项是加速度平方积分：

$$
E=\sum_{i=0}^{N-1}\int_0^{h_i}\|a_i(t)\|^2dt
$$

由：

$$
a_i(t)=2c_{i2}+6c_{i3}t
$$

展开：

$$
\|a_i(t)\|^2
=
4\|c_{i2}\|^2
+24c_{i2}^{\top}c_{i3}t
+36\|c_{i3}\|^2t^2
$$

积分得到单段能量：

$$
E_i
=
4\|c_{i2}\|^2h_i
+12c_{i2}^{\top}c_{i3}h_i^2
+12\|c_{i3}\|^2h_i^3
$$

总能量：

$$
E=\sum_{i=0}^{N-1}E_i
$$

对应 `CubicSplineND::getEnergy()`。

### 5.1 对系数的偏导

固定时长时：

$$
\frac{\partial E_i}{\partial c_{i2}}
=
8c_{i2}h_i+12c_{i3}h_i^2
$$

$$
\frac{\partial E_i}{\partial c_{i3}}
=
12c_{i2}h_i^2+24c_{i3}h_i^3
$$

对 $c_{i0}$ 和 $c_{i1}$ 的显式偏导为 0。

这对应 `getEnergyPartialGradByCoeffs()`。

### 5.2 对时长的显式偏导

固定系数时：

$$
\frac{\partial E_i}{\partial h_i}
=
4\|c_{i2}\|^2
+24c_{i2}^{\top}c_{i3}h_i
+36\|c_{i3}\|^2h_i^2
$$

这对应 `getEnergyPartialGradByTimes()`。

### 5.3 已消元样条下的直接梯度

`ne_traj_optimize.hpp` 里平滑项没有通过 `propagateGrad()` 反传，而是直接调用：

```cpp
cubic_spline_.getEnergyGradInnerP();
cubic_spline_.getEnergyGradTimes();
```

其内部点梯度为：

$$
\frac{\partial E}{\partial P_i}
=12(c_{i3}-c_{i-1,3}),\quad i=1,\dots,N-1
$$

段时间梯度为：

$$
\frac{\partial E}{\partial h_i}
=
-4\|c_{i2}\|^2
+12c_{i1}^{\top}c_{i3}
$$

这类梯度已经把样条方程 $A(h)M=b(P,h)$ 的最优性关系折叠进去了，和 5.1、5.2 中“固定系数”的偏导不是同一个量。

## 6. 速度与碰撞约束代价

`calculateConstraintCostGrad()` 对每段轨迹做数值积分，积分点使用局部时间：

$$
t_{ij}=\alpha_jh_i,\quad \alpha_j=\frac{j}{K}
$$

采样步长：

$$
\Delta t_i=\frac{h_i}{K}
$$

多项式基函数为：

$$
\beta_0(t)=
\begin{bmatrix}
1&t&t^2&t^3
\end{bmatrix}^{\top}
$$

$$
\beta_1(t)=
\begin{bmatrix}
0&1&2t&3t^2
\end{bmatrix}^{\top}
$$

$$
\beta_2(t)=
\begin{bmatrix}
0&0&2&6t
\end{bmatrix}^{\top}
$$

若第 $i$ 段系数矩阵为：

$$
C_i=
\begin{bmatrix}
c_{i0}^{\top}\\
c_{i1}^{\top}\\
c_{i2}^{\top}\\
c_{i3}^{\top}
\end{bmatrix}
\in\mathbb{R}^{4\times2}
$$

则：

$$
p=C_i^{\top}\beta_0,\quad
v=C_i^{\top}\beta_1,\quad
a=C_i^{\top}\beta_2
$$

### 6.1 速度软约束

最大速度约束写成 violation：

$$
z_v=\|v\|^2-v_{max}^2
$$

当 $z_v>0$ 时，代码使用三次 hinge：

$$
J_v=\rho_v z_v^3
$$

对速度的梯度为：

$$
\frac{\partial J_v}{\partial v}
=
\rho_v\cdot3z_v^2\cdot2v
=
6\rho_v z_v^2v
$$

对应代码中的：

```cpp
grad_v = param.rho_v * 6 * vViola * vViola * vel;
```

### 6.2 ESDF 碰撞软约束

ESDF 给出：

$$
d=d_{obs}(p),\quad \nabla d=\nabla d_{obs}(p)
$$

安全距离 violation：

$$
z_c=d_{safe}-d
$$

当 $z_c>0$ 时产生碰撞代价。代码意图上使用分段惩罚：

$$
\psi(z_c)=
\begin{cases}
z_c^2, & z_c<0.1\\
z_c, & z_c\ge0.1
\end{cases}
$$

因此：

$$
J_{col}=\rho_{collision}\psi(z_c)
$$

若 $z_c<0.1$：

$$
\frac{\partial J_{col}}{\partial p}
=
-2\rho_{collision}z_c\nabla d
$$

若 $z_c\ge0.1$：

$$
\frac{\partial J_{col}}{\partial p}
=
-\rho_{collision}\nabla d
$$

负号来自：

$$
z_c=d_{safe}-d_{obs}(p)
$$

轨迹越靠近障碍物，$d_{obs}$ 越小，惩罚越大。

## 7. 数值积分梯度

对每个采样点，约束代价记为：

$$
q(p,v)
$$

积分近似为：

$$
J_i\approx
\sum_{j=0}^{K}
\omega_j q(p_{ij},v_{ij})\Delta t_i
$$

其中 $\omega_j$ 是梯形积分权重，端点为 $0.5$，中间点为 $1$。

对系数矩阵的梯度为：

$$
\frac{\partial J_i}{\partial C_i}
\mathrel{+}=
\left(
\beta_0\frac{\partial q}{\partial p}^{\top}
+
\beta_1\frac{\partial q}{\partial v}^{\top}
\right)\omega_j\Delta t_i
$$

这对应：

```cpp
gdCpos.block<4,2>(i * 4, 0) +=
    (beta0 * grad_p.transpose() + beta1 * grad_v.transpose()) * omg * step;
```

时长梯度需要考虑两部分：

1. 积分尺度 $\Delta t_i=h_i/K$ 随 $h_i$ 变化。
2. 采样点 $t_{ij}=\alpha_jh_i$ 随 $h_i$ 变化。

由于：

$$
\frac{\partial p}{\partial h_i}=\alpha_jv,\quad
\frac{\partial v}{\partial h_i}=\alpha_ja
$$

所以：

$$
\frac{\partial}{\partial h_i}
\left(q\Delta t_i\right)
=
\frac{q}{K}
+
\Delta t_i\alpha_j
\left(
\frac{\partial q}{\partial p}^{\top}v
+
\frac{\partial q}{\partial v}^{\top}a
\right)
$$

代码中的速度项使用：

$$
\frac{q_v}{K}+\Delta t_i\alpha_j
\frac{\partial q_v}{\partial v}^{\top}a
$$

碰撞项使用：

$$
\frac{q_c}{K}+\Delta t_i\alpha_j
\frac{\partial q_c}{\partial p}^{\top}v
$$

这就是 `gdTpos(i)` 的来源。

## 8. 从系数梯度反传到路径点和时间

约束项先得到的是：

$$
\bar{C}=\frac{\partial J}{\partial C},\quad
\bar{h}_{partial}=\left.\frac{\partial J}{\partial h}\right|_{C\ fixed}
$$

但真正的优化变量是内部点 $P_1,\dots,P_{N-1}$ 和段时长 $h_i$。因为系数依赖：

$$
C=C(P,M,h),\quad A(h)M=b(P,h)
$$

所以需要链式法则。

对样条线性系统微分：

$$
dA\cdot M+A\cdot dM=db
$$

整理得：

$$
A\cdot dM=db-dA\cdot M
$$

若显式反传先得到 $\bar{M}$，引入伴随变量 $\lambda$：

$$
A^{\top}\lambda=\bar{M}
$$

三次样条的 $A$ 是对称三对角矩阵，因此 $A^{\top}=A$。`propagateGrad()` 直接复用 `solveWithCachedLU(lambda)` 解这个伴随系统。

反传流程可以概括为：

1. 由 $\bar{C}$ 对 $c_{i0},c_{i1},c_{i2},c_{i3}$ 的显式依赖，累加到 $\bar{P}$、$\bar{h}$、$\bar{M}$。
2. 解 $A\lambda=\bar{M}$。
3. 由 $b(P,h)-A(h)M$ 对 $P$ 和 $h$ 的依赖，把隐式项补回 $\bar{P}$ 和 $\bar{h}$。
4. 默认丢弃端点梯度，只返回内部点梯度。

这就是 `TrajectoryOptimize::calGradCTtoQT()` 调用 `cubic_spline_.propagateGrad()` 的数学意义。

## 9. 总目标函数与 L-BFGS 梯度

当前代码表达的总代价为：

$$
J
=
J_{constraint}
+
\rho_{energy}E
+
\rho_TT
$$

其中：

$$
J_{constraint}=J_v+J_{col}
$$

且 `calculateConstraintCostGrad()` 内部已经把 $\rho_v$ 和 $\rho_{collision}$ 乘进去了。

### 9.1 内部点梯度

约束项反传得到：

$$
\frac{\partial J_{constraint}}{\partial P}
$$

平滑项直接得到：

$$
\frac{\partial E}{\partial P}
$$

合并后：

$$
\frac{\partial J}{\partial P}
=
\frac{\partial J_{constraint}}{\partial P}
+
\rho_{energy}\frac{\partial E}{\partial P}
$$

这对应：

```cpp
gradPpos = gradPpos_constrain + param.rho_energy * gradP_energy.transpose();
```

### 9.2 段时间梯度

同理：

$$
\frac{\partial J}{\partial h_i}
=
\frac{\partial J_{constraint}}{\partial h_i}
+
\rho_{energy}\frac{\partial E}{\partial h_i}
$$

### 9.3 无约束时间变量梯度

因为：

$$
h_i=\frac{T(\tau)}{N}
$$

最终：

$$
\frac{\partial J}{\partial \tau}
=
\left(
\rho_T+\frac{1}{N}\sum_{i=0}^{N-1}
\frac{\partial J}{\partial h_i}
\right)
\frac{dT}{d\tau}
$$

L-BFGS 看到的完整梯度是：

$$
\nabla_xJ=
\begin{bmatrix}
\frac{\partial J}{\partial \tau}\\
\operatorname{vec}\left(\frac{\partial J}{\partial P_1},\dots,\frac{\partial J}{\partial P_{N-1}}\right)
\end{bmatrix}
$$

## 10. 轨迹质量评估

`evaluateTrajectory()` 以固定步长采样轨迹，统计：

$$
E_{sample}\approx\sum_t\|a(t)\|^2\Delta t
$$

最大速度：

$$
v_{max,sample}=\max_t\|v(t)\|
$$

最小障碍距离：

$$
d_{min}=\min_td_{obs}(p(t))
$$

相对 JPS 路径的平均偏差：

$$
d_{dev}=\frac{1}{M}\sum_{m=1}^{M}\min_j\|p(t_m)-P^{jps}_j\|
$$

这些指标适合用来观察优化是否把路径变得更平滑，同时是否仍然贴近离散可行路径。

## 11. 代码核对清单

下面是根据当前代码逐行核对时看到的收口点。它们不改变上面的数学模型，但会影响编译和运行。

### 11.1 `ne_traj_optimize.hpp`

- `evaluateTrajectory()` 中 `metrtcs.path_deviation` 应为 `metrics.path_deviation`。
- `jps_plan()` 返回类型是 `bool`，但存在裸 `return;`。
- `jps_plan()` 中 `init_state,end_state` 重复声明，且没有 resize 就直接访问 `col(0)`、`col(1)`。
- 构造函数形参是 `jps_path`，但类中没有保存对应成员；后续使用的是 `jps_path_`。
- `jps_paths_`、`params`、`piece_numl`、`current_segment_`、`current_segment`、`inner_pose_node` 等变量名不一致或未定义。
- JPS 路径插值处 `ratio * (...).norm()` 得到的是标量，意图应是 `ratio * (p_{k+1}-p_k)`。
- `inner_pos.resize(2,inner_pose_node)` 的第二个参数应为列数。
- `inner_pos[i] = inner_pose_node(i)` 混用了 Eigen 矩阵索引和 `std::vector` 调用。
- `optimizeSE2Traj()` 中 `end_pos_ = end_pos_` 应为保存 `end_state`。
- `int piece_pos_ = ...` 会遮蔽成员变量，导致 `costFunction()` 无法拿到正确段数。
- `costFunction()` 声明为 `void`，但 L-BFGS 回调需要返回 `double`。
- `costFunction()` 中 `dim_T` 是局部变量，在函数作用域不可见。
- `Eigen::Map<const Eigen::MatrixXd> gradPpos(...)` 被声明为 const 后又被赋值。
- `cubic_spine_` 应为 `cubic_spline_`。
- `calculateConstraintCostGrad()` 的形参 `cost` 又被局部 `double cost` 遮蔽。
- `double s1,double d2,double s3;` 语法错误，且 `s1/s2/s3` 的初始化逻辑需要补齐。
- `for(int i = 0;i < N,++i)` 中逗号应为分号。
- `grad_sdf.serZero()` 应为 `setZero()`。
- `bata2` 应为 `beta2`。
- `v_cost`、`grad_value` 未定义。
- `safe_thresold` 应与参数 `safe_threshold` 保持一致。
- `(if cViola > 0 && sdf_value < 5)` 语法错误。
- 梯形积分里 `j == param.int_K` 在 `j < param.int_K` 循环中永远不会触发，若要包含终点，应循环到 `j <= K` 或改成中点积分。
- `generateTrajectory()` 中 `piece_num` 未定义，`times.reserve(piece_num + 1)` 应使用 `Tpos.size()`。
- `generateTrajectory()` 遍历内部点时应按列遍历 `inner_pos.cols()`，而不是 `inner_pos.size()`。
- `BoundaryConditions` 字段名是 `start_velocity` / `end_velocity`，代码里写成了 `state_vel` / `end_vel`。

### 11.2 `esdf_builder_node.hpp`

- `isCollision()` 中 `return !isInMap(pos) || getDistance(pos) < safe_threshold;+` 末尾多了一个 `+`，会导致编译失败。

### 11.3 建议修复顺序

1. 先补齐类成员：`jps_path_`、`piece_pos_`、`dim_T` 或将其作为局部信息传入回调上下文。
2. 让 `jps_plan()` 只负责生成 `init_state`、`end_state`、`inner_pos` 和 `total_time`，并保证返回 `bool`。
3. 把 `costFunction()` 改成返回 `double`，并在开头 `grad.setZero()`。
4. 修正 `calculateConstraintCostGrad()` 的采样循环、变量名和 ESDF 梯度变量。
5. 修正 `generateTrajectory()` 的 waypoint 组装和边界速度字段。
6. 最后跑一次 `colcon build --packages-select ne_global_planner ne_esdf_builder`，再用简单地图做一次 JPS 加优化闭环测试。

## 12. 总结

当前代码表达的连续优化模型可以浓缩为：

$$
\min_{\tau,P_1,\dots,P_{N-1}}
\left[
J_v+J_{col}
+\rho_{energy}\sum_i\int_0^{h_i}\|p_i''(t)\|^2dt
+\rho_TT(\tau)
\right]
$$

约束项通过采样积分得到系数梯度，再由三次样条的伴随反传映射到内部点和时间；平滑项直接使用三次样条已消元后的解析梯度；总时间通过 `expC2()` 保证正值，并把段时间梯度汇总到无约束变量 $\tau$ 上。
