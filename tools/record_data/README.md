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

# 点云短包（默认 30s，mcap）
bash record_run.sh corner_points --points --duration 20

# 不压缩 / 换存储
bash record_run.sh run1 --no-compress
bash record_run.sh run1 --storage mcap

# 运行中标注事件（另开终端）
bash record_run.sh --event "点目标A"
```

可用环境变量覆盖：`NAV_WS` / `LOCAL_WS` / `BAG_ROOT`（默认 `~/bags`）/ `ROS_DOMAIN_ID`（默认 0）。

## 行为

- 自动 `source` ROS 与两个工作空间；自动生成/沿用 `$BAG_ROOT/qos_overrides.yaml`；
- 录制用 `--qos-profile-overrides-path` 保证**保留型（transient_local）**与
  **best_effort** 话题（`/grid_map/*`）也能录到；
- 默认 `--compression-mode file --compression-format zstd`（不支持就加 `--no-compress`）；
- 停止用 **SIGINT 转发**（等同 Ctrl-C 一次），让 rosbag 正常 finalize；
- 录后自动 `ros2 bag info`，**逐话题核对 Count 并列出缺失/为 0 的话题**，
  写 `meta.yaml`、追加 `$BAG_ROOT/INDEX.md`，收拢 `~/.ros/log/maploc`、`loc_tum.txt`、`match.csv`。

## 话题清单（按 A2 名；真机核对后请改 `DIAG_TOPICS`）

- **定位**：`/loc/robo/odom` `/loc/odom` `/loc/status` `/loc/map` `/loc/keyframes` `/loc/submap` `/lio/robo/odom` `/lio/imu/odom` `/tf` `/tf_static`
- **建图**：`/lio/cloud_world` `/grid_map/occupancy` `/grid_map/occupancy_inflate` `/grid_map/sliding_map_bbox` `/grid_map/depth_cloud`
- **全局**：`/global_path` `/global_planner/raw_map` `/global_planner/map` `/move_base_simple/goal`
- **局部**：`/planning/bspline` `/local_path` `/planning/data_display` `/self_inflation` `/planning/go2_execution_frozen`
- **控制**：`/cmd_vel` `/motion_enabled`
- **事件**：`/test_event`

> 在真机上跑 `bash record_run.sh --check`，它会直接列出实际不存在的话题，照它改即可。
