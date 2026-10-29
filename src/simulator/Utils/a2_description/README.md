# A2 description for ROS 2 Humble

本包包含 Unitree A2 的 URDF 模型与网格，供 SCAN-Planner 的仿真在 RViz2 里显示 A2 外形。

单独查看模型与关节：

```bash
ros2 launch a2_description a2_rviz.launch.py
```

在仿真的规划流程里使用（`robot:=a2` 同时套用 A2 机型档案与 A2 模型）：

```bash
ros2 launch scan_planner run.launch.py is_real_world:=false navi_mode:=1 \
  sensor_type:=lidar controller_mode:=closed_loop use_gpu:=false robot:=a2
```

## 来源与改动

来源：A2 上 `/home/unitree/agile-nav/static/robot/`（`urdf/a2.urdf` + `meshes/*.STL`）。
相对原始文件的改动只有三处：

1. 网格路径 `../meshes/*.STL` → `package://a2_description/meshes/*.STL`（17 处）。
2. 顶部新增虚拟根 `base` 及固定关节 `base → base_link`。仿真的 TF 约定是
   `world → base`（见 `config/simulator.yaml`、`config/controllers.yaml` 的
   `child_frame_id: base`），模型必须以 `base` 为根才不会与 TF 脱链。
3. SolidWorks 导出遗留的空材质名 `<material name="">` → `a2_mat_N`（17 处）。

## 结构

- `base`（虚拟根，1 mm 方块）→ `base_link`（机身，19.65 kg）
  → `{FL,FR,RL,RR}_hip → _thigh → _calf → _foot`（每腿 3 个旋转关节 + 1 个固定脚）
  → `camera_link` / `front_lidar_link` / `rear_lidar_link`（传感器 frame）
- 12 个可动关节；关节名与 Go2 不同，因此 `go2_gait_publisher` 不会驱动它——
  在仿真里 A2 呈现为**静态站姿**（对照碰撞 footprint 足够）。

## 注意

本包只装了 `urdf` 与 `meshes`，没有 Gazebo 相关部分（`go2_description` 才有
`xacro` / `worlds` / `config` / `ros2_control`）。
