<div align="center">
  <h1>SCAN-Planner ROS 2</h1>
  <h2>面向路线引导四足长程导航的空间碰撞感知局部规划器</h2>
  <a href="https://arxiv.org/abs/2606.19555"><img alt="论文" src="https://img.shields.io/badge/论文-arXiv-b31b1b?logo=arxiv&logoColor=white"/></a>
  <a href="https://www.bilibili.com/video/BV15a7P6UEXb/"><img alt="视频" src="https://img.shields.io/badge/视频-Bilibili-FB7299?logo=bilibili&logoColor=white"/></a>
  <a href="https://wuyi2121.github.io/SCAN-Planner/"><img alt="项目主页" src="https://img.shields.io/badge/项目主页-Website-4A90E2?logo=googlechrome&logoColor=white"/></a>
</div>

<p align="center">
  <img src="assets/images/abstract_real.jpg" width="100%"/>
</p>

SCAN-Planner 是一款面向四足机器人导航的空间碰撞感知局部规划器。本分支是原生 ROS 2 自移植版本，适配 Ubuntu 22.04、ROS 2 Humble、C++17 和 `colcon` 构建系统。

本仓库是 [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的衍生 ROS 2 移植版。核心算法、项目设计与原始研究工作归功于 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin；ROS 2 适配由本仓库维护者完成，不代表原作者的官方发布或认可。

## 构建

安装 ROS 2 Humble 及包依赖后，在工作空间根目录下执行构建：

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libarmadillo-dev libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

默认构建 CPU 端本地感知后端，如需构建 OpenGL 后端可执行：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_GPU=ON
```
仓库不再链接自带的 x86_64 架构 GLFW 动态库，GPU 构建依赖系统安装的 GLFW、GLEW 和 OpenGL 相关包。

## 快速启动

启动自定义确定性仿真器与规划器：

```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false
```


```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=1 sensor_type:=lidar \
  controller_mode:=closed_loop use_gpu:=false \
  use_pcd_map:=true pcd_map_file:=/home/xiaoqi_wen/Desktop/scan/SCAN-Planner/map.pcd
```



在另一个终端启动 RViz2：

```bash
source install/setup.bash
ros2 launch scan_planner rviz.launch.py
```

RViz2 配置已适配 ROS 2 Humble：Go2 的 RobotModel 使用现有的 `meshes/base.dae`，Sliding Map Bounds 订阅 `/grid_map/sliding_map_bbox`，Goal 订阅 `/goal_point`。



导航模式说明：
- `navi_mode:=1`：使用 RViz2 的 2D 目标点工具选择导航目标
- `navi_mode:=2`：按照 ROS 2 参数文件中预设的 `fsm.waypoints` 路径点序列导航
- `navi_mode:=3`：订阅 `/global_path` 话题（`nav_msgs/msg/Path`）获取一条全局参考路径，并沿它在局部范围内避障

`navi_mode:=3` 的参考路径可以带自己的坐标系（例如 `map`）：规划器会通过 TF 把它换算到建图坐标系（`grid_map.frame_id`，默认 `world`）。换算所需的 TF 长时间缺失、或参考系发生明显跳变（定位修正）时，规划器会安全停机并触发重规划，相关阈值见 `fsm.reference_tf_*` 参数。该模式下规划器还会把当前局部轨迹发布到 `/local_path` 话题，便于观察或录制。

控制器模式分为 `open_loop`（开环）和 `closed_loop`（闭环）两种。本次移植保留的核心启动参数包括：`is_real_world`、`navi_mode`、`sensor_type`、`controller_mode`、`use_gpu`、`use_pcd_map`、`pcd_map_file` 和 `reference_path_file`。

当 `use_pcd_map:=true` 时，必须提供已有的 PCD 点云地图文件：

```bash
ros2 launch scan_planner run.launch.py \
  use_pcd_map:=true pcd_map_file:=/absolute/path/to/map.pcd
```

`navi_mode:=3` 可以用自带的示例发布器快速试跑（航点写在 `config/reference_path.map.yaml`，坐标系为 `world`）。传入 `reference_path_file` 后，启动文件会额外拉起 `reference_path_publisher`，等里程计就绪后把这条路径发到全局路径话题上：

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=3 \
  reference_path_file:=$(ros2 pkg prefix scan_planner)/share/scan_planner/config/reference_path.map.yaml
```

`reference_path_file` 只在 `navi_mode:=3` 下可用，且必须指向一个已存在的 ROS 2 参数文件。实机上通常不需要它——外部全局规划器会自行发布 `/global_path`。

实际硬件部署时，激光惯导里程计（LIO）、相机和宇树（Unitree）驱动均为外部依赖。默认启动（`is_real_world:=true`、`navi_mode:=3`）会按 A2 上实际运行的 LIO 话题名把规划器输入映射到 `/lio/robo/odom`、`/lio/imu/odom`、`/lio/cloud_world`，以及 RealSense 对齐深度图话题 `/camera/aligned_depth_to_color/image_raw`；该 LIO 的点云已在世界系下，因此不再做外参变换。可根据实际安装的驱动栈修改话题重映射配置。

初始位置默认值按导航模式区分：`navi_mode:=1` 为 `(-19.0, 1.0, 0.25)`，其余模式为 `(-5.5, 5.5, 0.5)`；显式传入 `init_x` / `init_y` / `init_z` 时以下传值为准。

## 配置与接口

规划器、控制器和仿真器的参数分别位于：
- `src/planner/plan_manage/config/planner.yaml`
- `src/planner/plan_manage/config/controllers.yaml`
- `src/planner/plan_manage/config/simulator.yaml`

ROS 2 参数名称使用点号分隔，例如 `grid_map.resolution`。预设路径点是由 xyz 三元组组成的浮点数组：

```yaml
scan_planner_node:
  ros__parameters:
    fsm.navi_mode: 2
    fsm.waypoints: [0.0, 0.0, 0.3, 5.0, 1.0, 0.3]
```

自定义消息类型为 `scan_planner_msgs/msg/Bspline` 和 `scan_planner_msgs/msg/DataDisp`。规划器相关话题均为相对话题，支持重映射，核心输出话题包括 `planning/bspline`、`planning/data_display`、`planning/go2_execution_frozen` 和 `local_path`。`navi_mode:=3` 的输入话题为 `initial_path`，`run.launch.py` 默认把它重映射为 `/global_path`。

`navi_mode:=3` 额外的参考坐标系与安全阈值参数：

```yaml
scan_planner_node:
  ros__parameters:
    # 允许的最大 TF 年龄(秒)：比这更旧的变换认为不可用
    fsm.reference_tf_max_age: 0.3
    # 宽限时间(秒)：TF 连续拿不到超过这么久就急停
    fsm.reference_tf_grace: 0.5
    # 判定“参考系跳变”的平移阈值(米)
    fsm.reference_tf_jump_translation: 0.2
    # 判定“参考系跳变”的偏航阈值(度)
    fsm.reference_tf_jump_yaw_deg: 5.0
```

关键点记录器现在是原生的 `rclpy` 可执行程序：

```bash
ros2 run scan_planner keypoint_recorder.py --output keypoints.yaml
```

## 测试

构建后可以运行测试。参考路径 / 参考坐标系相关的单元测试与启动测试位于 `src/planner/plan_manage/test/`，用标签过滤只跑这一组：

```bash
colcon test --packages-select scan_planner --ctest-args -R reference
colcon test-result --verbose
```

注意其中的启动测试（`test_dynamic_reference_tf_launch.py`、`test_reference_path_publisher_launch.py`）依赖实时性，机器负载较高时可能出现偶发失败；空载重跑即可确认。

## Gazebo Fortress / Go2 仿真

Go2 四足机器人物理模型基于 Gazebo Fortress、`ros_gz_sim` 和 `gz_ros2_control` 构建，对外提供 12 关节的 `joint_trajectory_controller`、`/joint_states` 话题、IMU 数据、四个足端接触力话题以及 `/clock` 时钟话题：

```bash
ros2 launch go2_description go2_sim.launch.py
```

如需在不启动物理仿真时查看模型，可运行：

```bash
ros2 launch go2_description go2_rviz.launch.py
```
旧版 Gazebo Classic 的轨迹/力可视化插件、外力插件以及宇树 ROS 1 专用插件已不在本 ROS 2 仿真版本中提供。


## 致谢

首先感谢原项目 [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的作者 Han Zheng、Zhe Chen、Yiwen Fu、Ming Yang 和 Tong Qin 开源其研究成果与实现。本仓库在保留原项目 Apache-2.0 许可证的前提下完成 ROS 2 移植，完整署名见 [NOTICE](NOTICE)。

SCAN-Planner 的实现借鉴了 EGO-Planner、ROG-Map、MARSIM、Mockamap 和 Leg-KILO 的算法思路与开源代码，真实机器人定位方案基于 Elevator-LIO / FAST-LIO2 实现。

## 许可证

本仓库遵循 [Apache License 2.0](LICENSE)。分发或派生本项目时，请保留 [NOTICE](NOTICE) 中的原项目署名与许可证信息。
