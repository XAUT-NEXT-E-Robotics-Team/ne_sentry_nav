# ne_sentry_nav
Esdf map for robot navigation
## 感知前端设计 我永远喜欢雪之下雪乃

> 本文档用于说明当前工程中感知前端模块的系统组成、设计动机、运行方式与地图管理方法。
> 文档重点不仅包括“如何使用”，也包括“为何采用当前架构与判定策略”。

## 1. 系统组成

本工程当前的感知前端主要由 3 个核心模块构成：

1. `livox_ros_driver2`
   负责接收 Livox MID360 的原始数据并发布 ROS2 话题。
2. `point_lio`
   负责激光里程计、点云去畸变和世界系点云输出。
3. `rog_map_node`
   负责局部占据地图构建、局部障碍提取，以及 `terrain_analysis` 二维通行栅格输出。

### 1.1 模块分工

| 模块 | 主要职责 | 典型输出 |
| --- | --- | --- |
| `livox_ros_driver2` | 接收 MID360 原始数据 | 激光和 IMU 话题 |
| `point_lio` | 去畸变、里程计、世界系点云 | 位姿、世界系点云 |
| `rog_map_node` | 局部地图、障碍提取、二维栅格 | `/rog_map/occ`、`/terrain_analysis/grid` |

当前入口节点在：
[src/ne_rog_map/rog_map/src/rog_map_node.cpp](/home/ei/ne_nav/src/ne_rog_map/rog_map/src/rog_map_node.cpp:22)

当前主配置文件在：
[src/ne_nav_config.yaml](/home/ei/ne_nav/src/ne_nav_config.yaml:1)

## 2. 数据流与处理链路

当前链路可以理解为：

`MID360 -> livox_ros_driver2 -> Point-LIO -> ROG-Map -> terrain_analysis`

从处理流程上看，可概括为如下流水线：

```text
MID360
  |
  v
[livox_ros_driver2]
  |
  v
[Point-LIO] ----> 输出位姿 / 世界系点云
  |
  v
[ROG-Map] ------> 输出局部占据点云 /rog_map/occ
  |
  v
[terrain_analysis] -> 输出二维障碍栅格 /terrain_analysis/grid
```

主要输出包括：

- `Point-LIO` 输出世界系点云和里程计
- `ROG-Map` 输出局部占据点云，如 `/rog_map/occ`
- `terrain_analysis` 输出二维障碍栅格 `/terrain_analysis/grid`

在当前代码中：

- `rog_map_node` 会读取 `ne_nav_config.yaml`
- `TerrainAnalysis` 默认订阅 `/rog_map/occ`
- `TerrainAnalysis` 最终发布 `/terrain_analysis/grid`

相关配置位置：
[src/ne_nav_config.yaml](/home/ei/ne_nav/src/ne_nav_config.yaml:7)

## 3. 设计动机与方法依据

> 当前前端的目标并非将所有观测点简单映射为障碍，而是在局部范围内尽可能稳定地区分以下几类场景：
> 实体障碍物、可通行坡面、局部噪声，以及存在上方结构但地面仍可通行的区域。
> 这一目标决定了系统必须同时保留三维几何信息与适用于规划的二维表达。

### 3.1 采用局部滑动地图而非固定全局地图的原因

系统优先采用局部滑动地图，而非大尺度全局静态栅格，主要基于以下考虑：

1. 规划真正最关心的是机器人附近这小块区域  
   远处地图再精细，也不会立刻影响下一步避障和控制。
2. 局部地图更适合高分辨率  
   如果把很细的分辨率铺到整张全局地图上，内存和计算量都会迅速变大。
3. 局部地图更适合动态环境  
   机器人运动时，地图窗口跟着移动，老数据会被自然滑出窗口，不容易积累太多过时障碍。

因此，`ROG-Map` 在当前系统中的定位更接近于局部环境建模与在线更新模块，而非离线全局建图模块。

### 3.2 先进行三维占据建模再生成二维栅格的原因

若直接将点云投影为二维障碍图，通常会面临以下问题：

1. 只要有点就变障碍，误报会很多
2. 无法区分“地面上方有结构”和“地面真的被堵住”

先进行局部三维占据建模具有如下优势：

- 能利用射线更新，把击中点和穿过区域区分开
- 能保留垂直方向信息
- 后面在 `(x, y)` 柱子上分析时，可以看清楚这一列到底是“竖着挡住了”，还是“只是上面有东西”

这也是为什么当前链路是：

`Point-LIO -> ROG-Map -> terrain_analysis`

而不是直接采用：

`Point-LIO -> 2D obstacle grid`

### 3.3 采用 `(x, y)` 柱状统计的原因

规划模块通常更适合接收二维或 2.5 维表示，而环境本身具有显著的三维结构。

因此，当前方法采用如下折中策略：

- 先保留 3D 占据关系
- 再在每个 `(x, y)` 网格上统计这一整列的垂直结构

由于rog_map输出的occ和occ_inf的点云是稠密结构化的，基于这一特点，我们可以采用高程分析的方案实现可性分析： 
高度差：最高点高度减最低点高度H
占据率：最高点到最低点之间的点云数n，res为分辨率，占据率= ((n + 1)*res)/H

### 3.4 同时使用高程差与密度指标的原因

仅依赖“是否存在回波”或仅依赖“是否存在高度变化”都不足以形成稳定的障碍判定。

若仅考虑高程差：

- 稀疏噪声
- 远处偶发反射
- 零散边缘点

也可能产生较大的高度跨度，从而导致误检。

若仅考虑密度：

- 地面
- 缓坡
- 一些正常表面

也可能将正常可通行表面误判为障碍。

因此，当前实现采用双阈值判定思路：

- 高程差用于刻画该柱状区域是否存在显著垂直结构
- 密度用于刻画该结构是否具有足够的观测支持，且并非偶然噪声

仅当两者同时满足时，相关区域才更有可能对应真实障碍物。

### 3.5 引入去噪与邻域修正的必要性

即便采用双阈值判定，真实点云仍普遍存在以下问题：

1. 椒盐噪声  
   单点或极少数点随机落在空域里
2. 轮廓缺口  
   障碍边缘因为稀疏采样、抽点或滤波，出现断裂

因此，前端进一步引入了多层稳健化处理：

- 点云层的体素滤波
- 半径离群点滤波
- 单栅格最少点数门槛
- 二维障碍最少邻居数门槛
- 邻域高度差修正

这些处理的目标并非过度平滑地图，而是在抑制偶发误报的同时尽量保持障碍轮廓的连续性。

### 3.6 区分 `unknown` 与 `obstacle` 的原因

在局部感知与规划耦合时，一个关键问题是：

`unknown` 不一定等于 `obstacle`

其根本原因在于：

- `unknown` 表示尚未获得充分观测
- `obstacle` 表示相关区域已被观测并确认为不可通行

在保守策略中，`unknown` 可被视为潜在风险区域；
在更激进的策略中，也可以暂时将其视为可通行区域，并通过局部重规划或后续观测进行修正。

因此，文档中将“未知区”和“障碍区”明确区分，是因为两者对应的环境语义与规划含义并不一致。

## 4. 各模块功能说明

### 4.1 Point-LIO

`Point-LIO` 负责：

- 接收 LiDAR 和 IMU 数据
- 对点云去畸变
- 估计位姿
- 输出世界系点云

默认启动文件：
[src/point_lio/launch/point_lio.launch.py](/home/ei/ne_nav/src/point_lio/launch/point_lio.launch.py:11)

### 4.2 ROG-Map

`ROG-Map` 负责：

- 建立局部滑动占据地图
- 支持加载已有 `PCD` 地图
- 发布局部占据点云和膨胀占据结果

当前配置中：

- `load_pcd_en: true`
- `pcd_name` 指向已有地图

也就是说，当前系统支持“加载已有 PCD 作为地图输入的一部分”。

相关配置：
[src/ne_nav_config.yaml](/home/ei/ne_nav/src/ne_nav_config.yaml:42)

### 4.3 terrain_analysis

`terrain_analysis` 负责：

- 对 `/rog_map/occ` 做点云去噪
- 按 `(x, y)` 网格统计垂直结构
- 根据高程差和密度做障碍判定
- 发布二维栅格 `/terrain_analysis/grid`

当前已经加入的去噪和后处理包括：

- 体素滤波 `denoise_voxel_leaf_size`
- 半径离群点滤波 `denoise_outlier_radius`
- 单栅格最少点数 `min_points_per_cell`
- 二维障碍最少邻居数 `obstacle_min_neighbors`

实现位置：
[src/ne_rog_map/rog_map/src/rog_map/terrain_analysis.cpp](/home/ei/ne_nav/src/ne_rog_map/rog_map/src/rog_map/terrain_analysis.cpp:158)

## 5. 工作空间构建

> 建议首先完成标准 ROS2 工作空间的创建与编译，以保证后续各模块能够在统一环境下运行。

### 5.1 环境准备

本工程当前按 ROS2 Humble 使用最直接。

建议用户先自己创建一个工作空间，例如：

```bash
mkdir -p ~/ne_nav_ws/src
cd ~/ne_nav_ws/src
git clone <your-repo-url> ne_nav
cd ..
```

下面文档统一使用这个约定：

```bash
export WS=~/ne_nav_ws
export PROJ=$WS/src/ne_nav
```

在终端中执行：

```bash
cd $WS
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
```

如果只想重编某个包：

```bash
cd $WS
source /opt/ros/humble/setup.bash
colcon build --packages-select rog_map
. install/setup.bash
```

仓库里也有一个简单构建脚本：

```bash
cd $PROJ
bash build.sh
cd $WS
. install/setup.bash
```

脚本内容见：
[build.sh](/home/ei/ne_nav/build.sh)

## 6. 前端运行方法

> 建议使用 3 个独立终端分别启动驱动、里程计与局部建图模块，以便于调试与状态观察。

### 6.0 启动顺序

```text
终端 1: Livox 驱动
终端 2: Point-LIO
终端 3: ROG-Map + terrain_analysis
```

### 6.1 终端 1：启动 Livox 驱动

```bash
cd $WS
source /opt/ros/humble/setup.bash
. install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

启动文件：
[src/livox_ros_driver2/launch/msg_MID360_launch.py](/home/ei/ne_nav/src/livox_ros_driver2/launch/msg_MID360_launch.py:1)

### 6.2 终端 2：启动 Point-LIO

```bash
cd $WS
source /opt/ros/humble/setup.bash
. install/setup.bash
ros2 launch point_lio point_lio.launch.py rviz:=True
```

### 6.3 终端 3：启动局部地图与地形分析

```bash
cd $WS
source /opt/ros/humble/setup.bash
. install/setup.bash
ros2 run rog_map rog_map_node
```

### 6.4 常用检查命令

如果需要确认各模块是否正常运行，可优先检查以下话题：

```bash
ros2 topic list
ros2 topic echo /terrain_analysis/grid
ros2 topic hz /terrain_analysis/grid
ros2 topic echo /rog_map/occ
```

## 7. 地图保存与加载

> 这一部分需要区分两类地图：
> 一类是 `Point-LIO` 累积形成的全局点云地图，
> 另一类是 `ROG-Map / terrain_analysis` 在运行时维护的局部感知结果。

这里建议区分两类“地图”。

### 7.1 保存 Point-LIO 的全局点云地图

这是当前工程里最直接、最完整的地图保存方式。

`Point-LIO` 支持把累计的世界系点云保存成 `PCD`。相关代码在：

[src/point_lio/src/laserMapping.cpp](/home/ei/ne_nav/src/point_lio/src/laserMapping.cpp:193)
[src/point_lio/src/laserMapping.cpp](/home/ei/ne_nav/src/point_lio/src/laserMapping.cpp:1033)

保存步骤：

1. 打开 Point-LIO 配置文件  
   例如：
   [src/point_lio/config/mid360.yaml](/home/ei/ne_nav/src/point_lio/config/mid360.yaml)

2. 找到 `pcd_save` 配置并修改：

```yaml
pcd_save:
  pcd_save_en: true
  interval: -1
```

说明：

- `pcd_save_en: true` 表示开启保存
- `interval > 0` 时会周期性保存 `scans_x.pcd`
- `interval: -1` 时通常会在程序退出时统一保存

3. 重新启动 `Point-LIO`

4. 正常运行完成后退出节点

5. 地图会保存在 `Point-LIO/PCD/` 目录下，常见文件名为：

- `scans.pcd`
- `scans_1.pcd`
- `scans_2.pcd`

在本仓库下通常对应：

```bash
$WS/build/point_lio/PCD/
```

或编译宏 `ROOT_DIR` 对应的 `PCD/` 目录。

如果想确认实际文件位置，可以运行：

```bash
find $WS -name "scans*.pcd"
```

### 7.2 加载已有 PCD 地图

当前 `ROG-Map` 支持加载已有 `PCD` 地图，配置方式如下：

```yaml
rog_map:
  load_pcd_en: true
  pcd_name: "/path/to/your_map.pcd"
```

对应配置位置：
[src/ne_nav_config.yaml](/home/ei/ne_nav/src/ne_nav_config.yaml:42)

如果要替换地图：

1. 把新的 `PCD` 文件放到合适位置
2. 修改 `pcd_name`
3. 重启 `rog_map_node`

### 7.3 当前是否支持直接保存 ROG-Map 局部地图

短答案：

`暂时没有现成的一键导出接口。`

当前这份工程里，我没有看到现成的“直接把 `ROG-Map` 局部地图一键导出为 `PCD`”接口。

也就是说：

- `ROG-Map` 当前支持加载 `PCD`
- `Point-LIO` 当前支持保存全局点云
- 但 `ROG-Map` 本身目前更偏向运行时发布话题，而不是直接落盘局部地图文件

如果后续确实需要保存 `/rog_map/occ`，可以考虑两种做法：

1. 录 `ros2 bag`
2. 额外写一个订阅 `/rog_map/occ` 并保存成 `PCD` 的小节点

## 8. 参数调节建议

> 当前前端的大多数表现差异可以通过参数调整完成，通常无需首先修改代码实现。

当前和前端效果最相关的配置主要在 `terrain_analysis` 和 `rog_map` 两块。

### 8.1 terrain_analysis

常用项：

- `grid_resolution`
- `obstacle_height_thresh`
- `obstacle_density_thresh`
- `denoise_voxel_leaf_size`
- `denoise_outlier_radius`
- `denoise_outlier_min_neighbors`
- `min_points_per_cell`
- `obstacle_min_neighbors`

### 8.2 rog_map

常用项：

- `resolution`
- `inflation_resolution`
- `map_size`
- `point_filt_num`
- `raycasting.batch_update_size`
- `raycasting.p_occ`
- `raycasting.p_free`
- `raycasting.unk_thresh`

## 9. 常见问题

> 本节汇总运行中较常见的问题与排查方向。

### 9.1 改了 `ne_nav_config.yaml` 但没生效

先确认你已经重启了：

```bash
ros2 run rog_map rog_map_node
```

因为 `rog_map_node` 启动时会读取一次 YAML。路径目前写死在：
[src/ne_rog_map/rog_map/src/rog_map_node.cpp](/home/ei/ne_nav/src/ne_rog_map/rog_map/src/rog_map_node.cpp:23)

如果你准备把项目分享给别人，建议把这里改成：

- 启动参数传入 YAML 路径
- 或者使用相对路径 / 包内配置路径

### 9.2 为什么保存不到地图

最常见原因有：

- 只改了 `ne_nav_config.yaml`，但没改 `Point-LIO` 的 `pcd_save`
- 没有正常退出 `Point-LIO`
- 保存路径和你预期的不一致

### 9.3 为什么前端有很多噪声或轮廓断裂

优先检查：

- `denoise_outlier_radius`
- `denoise_outlier_min_neighbors`
- `min_points_per_cell`
- `obstacle_min_neighbors`
- `point_filt_num`

这些参数过强时，容易把障碍边缘打断。

## 10. 总结

当前前端的整体思路可以概括为：

1. `livox_ros_driver2` 把传感器消息送进来。
2. `Point-LIO` 负责“我现在在哪”和“我看到了什么”。
3. `ROG-Map` 只记机器人附近那一小块区域。
4. `terrain_analysis` 把 3D 点云压成规划更容易使用的 2D 栅格。
5. 如果要存整张地图，优先用 `Point-LIO` 的 `PCD` 保存功能。

上述链路兼顾了局部实时性、三维结构表达能力，以及对规划模块更友好的二维输出形式。
