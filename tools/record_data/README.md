# 真机全量数据录制（record_run.sh）

一条命令录下真机一次运行中的**定位 / 建图 / 全局规划 / 局部规划 / 控制**全部话题，
并自动归档、校验、写索引。

## 依赖

真机上需要两个工作空间都已编译并 source：

| 工作空间 | 内容 | 默认路径 |
|---|---|---|
| `nav_ws` | 定位（super_lio）、建图、全局规划 | `~/nav_ws` |
| `scan_planner_github_ws` | 局部规划 + 控制 | `~/scan_planner_github_ws` |

> 录包终端**也必须 source 局部工作空间**，否则 `scan_planner_msgs` 缺、`/planning/bspline` 会被静默丢掉。

## 用法

```bash
# 录前自检：列出"清单里有、实际不存在"的话题，并打印关键话题频率与 /loc/status
bash record_run.sh --check

# 正片：Ctrl-C 按一次停止
bash record_run.sh stair_corner

# 录 120s 自动停
bash record_run.sh stair_corner --duration 120

# 点云短包（默认 30s）
bash record_run.sh corner_points --points --duration 20

# 精简档（去掉两朵大点云，约省 37%）/ 只录定位+规划+速度 / 开启 zstd 压缩
bash record_run.sh run1 --light
bash record_run.sh run1 --slim        # 只录 定位/全局/局部/速度，约 0.1-0.2 MB/s
bash record_run.sh run1 --compress

# 运行中标注事件（另开终端）
bash record_run.sh --event "点目标A"

# 自动标注"卡住/恢复"（无需手按：订阅 cmd_vel + odom，按"想走不动"判据触发）
bash record_run.sh stair_corner --auto-events
```

可用环境变量覆盖：`NAV_WS` / `LOCAL_WS` / `BAG_ROOT`（默认 `~/bags`）/ `ROS_DOMAIN_ID`（默认 0）
/ `AUTO_CMD_TOPIC`（默认 `/cmd_vel`）/ `AUTO_ODOM_TOPIC`（默认 `/loc/robo/odom`）。

## 行为

> **真机实测带宽**（A2，全量档）：`/grid_map/occupancy_inflate` **40.9 MB/s**、`/lio/cloud_world` 14.7、
> `/grid_map/depth_cloud` 13.1、`/grid_map/occupancy` 5.7 —— 合计约 **75 MB/s（≈4.5 GB/min）**。
> 所以：**默认不压缩**（可直接读、且能按话题做快速局部读）；`--compress` 能把 15 分钟从 61 GB 压到 ~18 GB，
> 但 **`rosbag2_py` / `analyze_run.py` 读不了 file 压缩包**，分析前要：
> `zstd -d -f x.db3.zstd -o x.db3`、`rm x.db3.zstd`、并把 `metadata.yaml` 的 `compression_mode: FILE` 改成 `NONE`。
> `--light` 去掉 `/lio/cloud_world` + `/grid_map/depth_cloud`，约省 37% 带宽。

- 自动 `source` ROS 与两个工作空间；自动生成/沿用 `$BAG_ROOT/qos_overrides.yaml`；
- 录制用 `--qos-profile-overrides-path` 保证**保留型（transient_local）**与
  **best_effort** 话题（`/grid_map/*`）也能录到；
- 默认 `--compression-mode file --compression-format zstd`（不支持就加 `--no-compress`）；
- 停止用 **SIGINT 转发**（等同 Ctrl-C 一次），让 rosbag 正常 finalize；
- 录后自动 `ros2 bag info`，**逐话题核对 Count 并列出缺失/为 0 的话题**，
  写 `meta.yaml`、追加 `$BAG_ROOT/INDEX.md`，收拢 `~/.ros/log/maploc`、`loc_tum.txt`、`match.csv`。

### `--auto-events`（自动事件标注）
与录制同时启动 `auto_event_pub.py`，每 0.2 s 判断一次：

- **想走**：`max(|cmd.vx|, |cmd.vy|) > 0.15`（`/cmd_vel`）
- **在动**：最近 0.5 s 内的**平均位移速度** > `0.03 m/s`（用 `/loc/robo/odom` 位置差分，比 twist 稳）
- 持续 > `1.0 s`"想走但不动" → 发 `/test_event` `AUTO:stuck`；恢复运动 → `AUTO:move`

阈值/话题可用 `--cmd-eps` / `--speed-eps` / `--stuck-time` / `--window`（见 `auto_event_pub.py`）
及环境变量 `AUTO_CMD_TOPIC` / `AUTO_ODOM_TOPIC` 调整。事件日志写在 `$BAG_ROOT/auto_events.log`。

## 离线分析（`analyze_run.py`）

录完后直接分析一个 bag：

```bash
source /opt/ros/humble/setup.bash
source ~/scan_planner_github_ws/install/setup.bash   # 用于解析 Bspline 等自定义类型
python3 analyze_run.py ~/bags/stair_corner_20261027_153000
```

产出（默认写在 `<bag>/analysis/`）：

| 文件 | 内容 |
|---|---|
| `report.md` | 时长/里程/卡住段/速度差/跟踪误差/定位偏差 + 自动结论 |
| `figs/fig1a_localization_traj.png` | **定位轨迹**：WORLD vs MAP（MAP 经 `/tf` 的 map→world 变到 WORLD）；蓝实线 vs 橙虚线 |
| `figs/fig1b_localization_dev.png` | **定位偏差 + TF 修正量**：紫实线=两源偏差、青虚线=map→world 平移 |
| `figs/fig1c_localization_state.png` | **定位状态** `/loc/status.state`（3=跟踪中），掉档一眼可见 |
| `figs/fig1d_localization_rmse.png` | **定位 rmse**（激光帧与先验地图的匹配残差） |
| `figs/fig2_planning.png`(实际文件名 `5_轨迹对比.png`) | **规划测试**：俯视（全局=紫虚线/局部=绿线/实际=蓝带）+ **高度 z 随时间**（实际/局部/全局三条） |
| `figs/fig3_speed.png`(实际文件名 `6_速度对比.png`) | **机体速度三分量**：vx / vy / wz 各自一个子图，比较**下发(cmd_vel)** vs **实际(odom)** |
| `figs/fig4_lanes.png` | **事件泳道**：冻结 / 卡住 / `--event` 与 `AUTO:*` 标注 |

> **配色约定**（贯穿所有图）：蓝=实际/实测、橙=地图定位、紫=全局轨迹/偏差、绿=局部轨迹/计划速度、
> 红=指令速度/卡住、青=TF 修正量/定位状态。线型也区分（实线=实际量，虚线=参考/派生量），
> 两条线重合时也能分辨。

> **坐标系提醒**（真机实测）：WORLD 系 = `/lio/robo/odom` + `/local_path` + `/planning/bspline`；
> MAP 系 = `/loc/robo/odom` + `/global_path`。两者差 ~4.5 m（含约 90° 旋转），**必须录 `/tf`**，
> 分析脚本靠它把 MAP 系对齐到 WORLD 系（实测对齐后两定位源偏差均值 **0.002 m**、全局路径离实际轨迹 **0.01 m**）。

无真机时可造假包做回归：

```bash
python3 make_test_bag.py /tmp/testbag && python3 analyze_run.py /tmp/testbag
```

依赖：`rosbag2_py`（source ROS）**或** `pip install rosbags`（纯 python，免装 ROS）；`numpy`、`matplotlib`
（建议装有中文字体，否则图里的中文会变方框；脚本会自动挑可用字体）。
`analyze_run.py` / `export_tum.py` / `make_test_bag.py` 同目录，公共函数在 `bag_tools.py`。

## 轨迹精度量化（`export_tum.py` + evo）

把 bag 里的轨迹导成 TUM 格式，交给 [evo](https://github.com/MichaelGrupp/evo) 算 **ATE / RPE**：

```bash
source /opt/ros/humble/setup.bash
source ~/scan_planner_github_ws/install/setup.bash
python3 export_tum.py ~/bags/<包>        # 产出 <bag>/tum/{est,ref,global_path,local_path,bsp_last}.tum
# 换轨迹来源：
python3 export_tum.py <包> --est-topic /loc/robo/odom --ref-topic /lio/robo/odom

# evo 建议装在独立 venv，别 pip --user（会把 numpy 升到 2.x，系统 scipy 会挂）
virtualenv ~/venvs/evo && ~/venvs/evo/bin/pip install evo
~/venvs/evo/bin/evo_config set plot_backend Agg        # 无 Qt 环境时必需
~/venvs/evo/bin/evo_traj tum <bag>/tum/est.tum <bag>/tum/ref.tum --ref=<bag>/tum/ref.tum -p --plot_mode=xy
~/venvs/evo/bin/evo_ape  tum <bag>/tum/ref.tum <bag>/tum/est.tum -va --save_plot ape.png
~/venvs/evo/bin/evo_rpe  tum <bag>/tum/ref.tum <bag>/tum/est.tum -va --delta 1 --delta_unit m --save_plot rpe.png
```

**怎么解读**（没有真值 GT 时，这是"两个估计之间"的比较，不是绝对精度）：
- `est=/loc/robo/odom`（地图定位） vs `ref=/lio/robo/odom`（纯 LIO）→ **RPE 小 = 两者一致；
  APE 大 = 地图定位在持续往回拉（修正量/漂移）**。
- 将来若有 RTK / 动捕 / 全站仪真值轨迹，套同样命令即为**绝对精度**。
- 轨迹近似一条直线时 Umeyama 对齐会报 degenerate → 去掉 `-a`，或改用 `--align_origin`。

**本地验证**：假包（注入 `0.03 + 0.01t` 的漂移）跑通，**APE rmse 0.0165 m、RPE(每 1 m) rmse 0.055 m**，
并生成 `ape_raw/map.png`、`rpe_raw/map.png`。

## 交互式回放

### 方案 A：rviz2（零安装，本机已装）
```bash
# 终端 1：回放 bag
ros2 bag play ~/bags/<你的包>
# 终端 2：用项目自带的显示配置（default 或 replay）
ros2 launch scan_planner rviz.launch.py config:=replay
# 等价于：rviz2 -d <share>/scan_planner/rviz/replay.rviz
```
`replay.rviz` 在 `src/planner/plan_manage/launch/`（编译时装进 `share/scan_planner/rviz/`），
显示：原始占据 / 膨胀占据（best_effort，已设对 QoS）/ 局部路径 / 全局路径 /
机身两圆柱 `/self_inflation` / 滑动窗口框 / TF；全局点云与地形图默认关。
Fixed Frame 默认 `map`，不对就改 Global Options。

### 方案 B：Foxglove（3D + 曲线一屏，需装）
用 Foxglove 打开 bag 后导入本目录的 `foxglove_layout.json`：3D（原始/膨胀占据、
局部/全局路径、机身圆柱、目标）+ 两条 Plot（指令 vs 实测、定位状态/使能/冻结）。
获取方式与排错见 `FOXGLOVE.md`。

> `analyze_run.py` 负责**出指标和图**（一条命令、可归档）；rviz2 / Foxglove 负责**交互式回放**。
> 建议先用分析脚本定位"哪一段"，再用回放拉时间轴细看那一小段。

## 话题清单（按 A2 名；真机核对后请改 `DIAG_TOPICS`）

- **定位**：`/loc/robo/odom` `/loc/odom` `/loc/status` `/loc/map` `/loc/keyframes` `/loc/submap` `/lio/robo/odom` `/lio/imu/odom` `/tf` `/tf_static`
- **建图**：`/lio/cloud_world` `/grid_map/occupancy` `/grid_map/occupancy_inflate` `/grid_map/sliding_map_bbox` `/grid_map/depth_cloud`
- **全局**：`/global_path` `/global_planner/raw_map` `/global_planner/map` `/move_base_simple/goal`
- **局部**：`/planning/bspline` `/local_path` `/planning/data_display` `/self_inflation` `/planning/go2_execution_frozen`
- **控制**：`/cmd_vel` `/motion_enabled`
- **事件**：`/test_event`

> 在真机上跑 `bash record_run.sh --check`，它会直接列出实际不存在的话题，照它改即可。
