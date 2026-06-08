# 机器人导论期末项目 — 基于栅格地图的 RRT* 路径规划与 PID 跟踪控制

本项目基于 ROS 1 和 Gazebo 9 仿真环境，在 TurtleBot3 机器人上实现了 RRT* 路径规划算法和 PID 路径跟踪控制器。

## 环境配置

| 依赖 | 版本 |
|------|------|
| ROS | ROS 1 (Melodic) |
| Python | 2.7 |
| Gazebo | 9 |
| 机器人平台 | TurtleBot3 (Waffle) |

## 运行步骤

### 1. 编译工作空间

```bash
# src/CMakeLists.txt内容有问题的话
# 先进入 src/
# 执行 catkin_init_workspace

cd final_project
catkin_make
source ./devel/setup.bash
```

### 2. 设置机器人型号

```bash
export TURTLEBOT3_MODEL=waffle
```

可选型号：`burger`、`waffle`、`waffle_pi`。若使用 `burger`，需在 `planner.py` 中将 `robot_radius` 改为 `0.105`。(需调参)

### 3. 启动仿真环境

```bash
roslaunch turtle obs_world.launch
```

这一步会自动启动：

- Gazebo 仿真世界 + 随机障碍物（20 个）
- TurtleBot3 机器人（初始位置 0,0）
- 地图服务器（延迟 10 秒加载地图）
- 路径规划节点 `planner.py`
- 路径控制节点 `controller.py`
- RViz 可视化界面

### 4. 设置目标点

在 RViz 中：

1. 等待地图加载完成（约 10 秒后地图会显示在 RViz 中）
2. 点击顶部工具栏的 **"2D Nav Goal"** 按钮
3. 在地图上点击目标位置（拖拽可设置朝向）

### 5. 观察运行

- RViz 中会显示 RRT* 搜索树（绿色线段）和规划路径（红色）
- 机器人会自动沿路径行驶到目标点
- 终端会输出规划时间、路径长度、迭代次数、树节点数等信息
- 行驶过程中终端会实时输出 PID 调试信息（轨迹偏差、速度平滑度等）
- 到达终点后终端会输出轨迹追踪精度和速度平滑度统计

<a href="运行示例.mp4">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://img.shields.io/badge/-%F0%9F%93%BD%20%E6%92%AD%E6%94%BE%20%E8%A7%86%E9%A2%91-8b949e?style=flat&logo=github&logoColor=white">
    <img alt="播放演示视频" src="https://img.shields.io/badge/%F0%9F%93%BD%20%E6%92%AD%E6%94%BE%20%E8%A7%86%E9%A2%91-238636?style=for-the-badge&logo=github&logoColor=white">
  </picture>
</a>

### 6. 路径可视化（可选）

运行结束后可使用 `plot_trajectories.py` 对比规划路径与实际轨迹：

```bash
python plot_trajectories.py
```

该脚本会读取 `planned_path.txt` 和 `actual_path.txt`，生成轨迹对比图、偏差分析图、误差分布直方图和统计摘要。

## 项目结构

```
final_project/
├── src/
│   ├── Robot-Planner/              # 核心：路径规划与控制器
│   │   ├── launch/obs_world.launch # 主启动文件
│   │   ├── maps/                   # 栅格地图文件
│   │   └── scripts/
│   │       ├── planner.py          # RRT* 路径规划器
│   │       ├── controller.py       # PID 路径跟踪控制器
│   │       └── gazebo_to_tf.py     # Gazebo 坐标到 TF 变换
│   ├── random_map_generator/       # 随机障碍物地图生成器
│   ├── turtlebot3/                 # TurtleBot3 机器人功能包
│   ├── turtlebot3_msgs/            # TurtleBot3 自定义消息
│   └── turtlebot3_simulations/     # TurtleBot3 Gazebo 仿真包
├── plot_trajectories.py            # 路径可视化脚本
├── planned_path.txt                # 规划路径数据
├── actual_path.txt                 # 实际轨迹数据
└── main.tex                        # 实验报告
```

## 核心算法参数

### RRT* 路径规划（planner.py）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `step_size` | 0.1 m | 树扩展步长 |
| `max_iterations` | 10000 | 最大迭代次数 |
| `goal_sample_rate` | 0.2 | 采样偏向目标节点的概率 |
| `goal_threshold` | 0.5 m | 到达目标的距离阈值 |
| `search_radius` | 0.75 m | RRT* 重连搜索半径 |
| `robot_radius` | 0.55 m | 机器人半径（碰撞膨胀用，可调节） |

### PID 路径跟踪（controller.py）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kp_lin` | 0.5 | 线速度比例增益 |
| `ki_lin` | 0.01 | 线速度积分增益 |
| `kd_lin` | 0.01 | 线速度微分增益 |
| `kp_ang` | 0.2 | 角速度比例增益 |
| `ki_ang` | 0.001 | 角速度积分增益 |
| `kd_ang` | 0.005 | 角速度微分增益 |

## 未来优化

- **RRT* 双向搜索优化**：当前 RRT* 仅从起点开始向终点延伸，可修改为从起点和终点一起开始延伸（Bi-RRT*），更快完成路径规划
- **PID 参数继续调节**：当前 PID 参数为初步调优结果，可进一步细化以降低轨迹偏差和提升速度平滑度
- **动态障碍物**：当前仅支持静态栅格地图，可扩展为动态环境下的实时路径规划
- **路径平滑**：RRT* 生成的路径为折线段，可加入路径平滑算法（如 Bezier 曲线拟合）使机器人运动更自然
- **自适应步长**：在空旷区域增大步长加速搜索，在障碍物密集区域减小步长提高精度

