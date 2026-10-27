# Foxglove 布局（`foxglove_layout.json`）

用 [Foxglove](https://foxglove.dev/) 打开录好的 bag，导入这份布局，3D + 曲线一次排好。

## 用法
1. 打开 Foxglove（桌面版或 `app.foxglove.dev`）→ **Open data source → Open local file**，选 bag 目录里的 `metadata.yaml`（或整个 sqlite3 / mcap 目录）。
2. 左上 **Layouts → Import from file…**，选本目录的 `foxglove_layout.json`。
3. 点播放，拖时间轴复盘。

## 布局里有三块

| 面板 | 看什么 |
|---|---|
| **3D**（上，约 65%） | 灰 = 原始占据 `/grid_map/occupancy`；红 = 膨胀 `/grid_map/occupancy_inflate`；绿 = 局部路径 `/local_path`；浅灰 = 全局路径 `/global_path`；橙 = 机身两圆柱 `/self_inflation`；黄点 = 目标 |
| **Plot 指令 vs 实测** | `/cmd_vel.linear.x`（红）与 `/loc/robo/odom.twist.twist.linear.x`（蓝）—— 两条贴不上就是"想走不动" |
| **Plot 状态/使能/冻结** | `/loc/status.state`、`/motion_enabled.data`、`/planning/go2_execution_frozen.data` |

## 导入失败 / 面板空白时（手工 2 分钟搞定）
- **3D 一片空**：把面板设置里的 **Frame**（json 里的 `followTf`）改成你 bag 里的固定坐标系（`map` / `odom` / `world` 之一；用 `ros2 run tf2_ros tf2_echo map odom` 或 TF 面板确认）。
- **点云分不清**：把 `/grid_map/occupancy_inflate` 的 **Flat color 设为红**，和灰的原始占据对比。
- **看事件标注**：**+ 面板 → Raw Messages**，Topic 选 `/test_event`，就能按时间看到 `--event` 与 `AUTO:stuck/move`。
- **想看"事件竖线"**：Foxglove 没这个面板，用它自带的 **Log 面板**看 `/rosout`，或直接看离线报告 `analysis/figs/05_lanes.png`。

## 已知限制
- **`/planning/bspline` 是自定义消息**（控制点 + knots），Foxglove 不能直接画成曲线；用 `/local_path` 代替（内容基本一致），或看离线分析的 `figs/01_traj.png`（那里把 B 样条求值成曲线了）。
- **`/loc/status`、`/planning/data_display` 也是自定义类型**：Plot 里按字段路径画（如 `/loc/status.state`）需要 Foxglove 能拿到该消息定义 —— 用 **`--storage mcap`** 录包会把定义一起存进去；sqlite3 的包需要本地有同样的 ROS 环境/类型支持。
- 点云很大：`/lio/cloud_world` 默认关着，需要再打开。

## 和离线分析的关系
- Foxglove：**交互式回放/现场看**（拉时间轴、转视角）。
- `analyze_run.py`：**出指标和图**（`report.md` + `figs/*`），一条命令、可归档。
- 建议：先用 `analyze_run.py` 定位"哪一段、哪个话题异常"，再用 Foxglove 拉时间轴细看那一小段。
