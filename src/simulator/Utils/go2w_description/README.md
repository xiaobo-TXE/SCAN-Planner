# go2w_description

Go2-W 的仿真模型包。**宇树没有发布 Go2-W 的官方 URDF**，本包由 `go2_description`
派生：

| 做了什么 | 说明 |
|---|---|
| 复制 xacro | `const / materials / leg / ros2_control / gazebo / robot` 六份，只把 `<xacro:include>` 指向本包 |
| 网格 | 仍引用 `package://go2_description/meshes/*.dae`，**不复制** 约 25 MB 网格 |
| 躯干缩放 | `base.dae` 缩放 `1 × 1.387 × 1.25`，`trunk_width/height` 同步 |
| 腿部外移 | `leg_offset_y / hip_offset_y` × 1.387（0.0465 → 0.0645） |
| **未做** | **腿长沿用 Go2**（官方未给 Go2-W 腿长），因此站立高度偏矮于 0.50 m |

因此：**关节名、关节轴与 Go2 完全一致**，`go2_gait_publisher` 可以不加修改地驱动它；
碰撞用的双圆柱参数**不来自本模型**，而来自机型档案 `config/robots/go2w.yaml`。

拿到官方 Go2-W URDF 后，直接替换本包即可（同时确认关节命名与 `base` 根节点约定）。
