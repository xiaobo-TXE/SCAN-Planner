# ============================================================================
# 文件：launch/run.launch.py —— 最常用的启动入口
#
# 【怎么用】
#   ros2 launch scan_planner run.launch.py is_real_world:=false navi_mode:=1 \
#     sensor_type:=lidar controller_mode:=closed_loop use_gpu:=false
#
# 【这个文件做的事情】
#   1. 根据 is_real_world 决定连“真机”还是“仿真”的话题
#   2. 启动规划器 scan_planner_node
#   3. 启动机器人模型显示 robot_state_publisher
#   4. 启动控制器（开环或闭环），仿真时再加上假机器人和步态动画
#   5. 仿真时顺便把 simulator.launch.py（造地图+传感器）也拉起来
#
# 【重要概念】remappings = 话题改名。
#   规划器内部写的是 body_pose，但真机上真实话题叫 /LIO/odom_vehicle，
#   这里就把两者“接”起来。仿真时则接到 /quad_0/body_pose。
# ============================================================================
"""Main ROS 2 launch entry point for simulation and real-robot remapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


# 把字符串形式的开关（如 "true"/"false"）转成 Python 的 True/False。
def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


# 真正干活的函数：在这里根据参数决定启动哪些节点、做哪些话题重映射。
# 用 OpaqueFunction 包裹，是为了能在“运行时”拿到参数值再决定。
def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    go2_share = get_package_share_directory("go2_description")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    is_real = _as_bool(LaunchConfiguration("is_real_world").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if controller_mode not in ("open_loop", "closed_loop"):
        raise RuntimeError("controller_mode must be 'open_loop' or 'closed_loop'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )

    # 下面这段是“二选一”：
    #   if 分支 = 真机：话题接 LIO 里程计/激光、相机内参用真机标定值；
    #   else 分支 = 仿真：话题接 /quad_0/*，点云已经是世界系、不需要外参。
    if is_real:
        body_pose = "/LIO/odom_vehicle"
        sensor_pose = "/LIO/odom_imu"
        cloud = "/LIO/clouds_lidar"
        depth = "/camera/aligned_depth_to_color/image_raw"
        cloud_is_world = False
        need_extrinsic = True
        intrinsics = {
            "grid_map.cx": 317.19183349609375,
            "grid_map.cy": 256.4806823730469,
            "grid_map.fx": 609.5884399414062,
            "grid_map.fy": 609.22021484375,
        }
    else:
        body_pose = "/quad_0/body_pose"
        sensor_pose = "/quad_0/camera_pose" if sensor_type == "depth" else "/quad_0/lidar_pose"
        cloud = "/quad_0/cloud"
        depth = "/quad_0/depth"
        cloud_is_world = True
        need_extrinsic = False
        intrinsics = {}

    common = {"use_sim_time": use_sim_time}
    planner_overrides = {
        **common,
        **intrinsics,
        "fsm.navi_mode": navi_mode,
        "grid_map.sensor_type": sensor_type,
        "grid_map.cloud_is_world": cloud_is_world,
        "grid_map.need_extrinsic": need_extrinsic,
    }
    # ---- 节点 1：规划器本体（最重要）----
    # parameters 按顺序叠加：先 planner.yaml，再（可选）航点文件，最后覆盖项。
    actions = [
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml] + ([keypoints_file] if keypoints_file else []) + [planner_overrides],
            remappings=[
                ("body_pose", body_pose),
                ("sensor_pose", sensor_pose),
                ("cloud", cloud),
                ("depth", depth),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                ("initial_path", "/initial_path"),
            ],
        )
    ]
    actions.append(
        Node(
            package="robot_state_publisher",
    # ---- 节点 2：机器人模型显示 ----
    # 用 xacro 生成 robot_description，让 RViz 能画出 Go2 的外形。
            executable="robot_state_publisher",
            name="go2_robot_state_publisher",
            output="screen",
            parameters=[
                common,
                {
                    "robot_description": Command(
                        ["xacro ", os.path.join(go2_share, "xacro", "robot.xacro"),
                         " use_gazebo:=false"]
                    )
                },
            ],
        )
    )

    if controller_mode == "open_loop":
        actions.append(
            Node(
                package="scan_planner",
    # ---- 控制器（方案 A）：开环 ----
    # 直接把轨迹当成里程计，不做控制。适合单独看规划效果。
                executable="open_loop_controller",
                name="open_loop_controller",
                output="screen",
                parameters=[controllers_yaml, common],
                remappings=[
                    ("planning/bspline", "/planning/bspline"),
                    ("body_pose", body_pose),
                ],
            )
        )
    else:
        actions.append(
            Node(
                package="scan_planner",
    # ---- 控制器（方案 B）：闭环（默认）----
    # 真正算速度指令 cmd_vel。真机发到 /cmd_vel，仿真发到 /quad_0/cmd_vel。
                executable="closed_loop_controller",
                name="closed_loop_controller",
                output="screen",
                parameters=[controllers_yaml, common],
                remappings=[
                    ("body_pose", body_pose),
                    ("cmd_vel", "/cmd_vel" if is_real else "/quad_0/cmd_vel"),
                ],
            )
        )
        if not is_real:
            actions.append(
                Node(
                    package="scan_planner",
    # ---- 假机器人（仅仿真）----
    # 接收 cmd_vel 算出位置，再发 body_pose，和控制器组成闭环。
                    executable="go2_kinematic_sim",
                    name="go2_kinematic_sim",
                    output="screen",
                    parameters=[
                        controllers_yaml,
                        common,
                        {
                            "init_x": float(LaunchConfiguration("init_x").perform(context)),
                            "init_y": float(LaunchConfiguration("init_y").perform(context)),
                            "init_z": float(LaunchConfiguration("init_z").perform(context)),
                            "publish_tf": False,
                        },
                    ],
                    remappings=[
                        ("body_pose", "/quad_0/body_pose"),
                        ("cmd_vel", "/quad_0/cmd_vel"),
                    ],
                )
            )

    if not is_real:
        actions.extend(
            [
                Node(
                    package="scan_planner",
    # ---- 步态动画（仅仿真）----
    # 纯视觉效果：让 RViz 里的 Go2 腿动起来。
                    executable="go2_gait_publisher",
                    name="go2_gait_publisher",
                    output="screen",
                    parameters=[controllers_yaml, common],
                    remappings=[("body_pose", body_pose)],
                ),
    # ---- 拉起仿真环境 ----
    # 包含 simulator.launch.py：造地图 + 模拟传感器 + 里程计可视化。
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(scan_share, "launch", "simulator.launch.py")
                    ),
                    launch_arguments={
                        name: LaunchConfiguration(name)
                        for name in (
                            "is_real_world",
                            "sensor_type",
                            "use_gpu",
                            "use_pcd_map",
                            "pcd_map_file",
                            "map_size_x",
                            "map_size_y",
                            "map_size_z",
                            "use_sim_time",
                        )
                    }.items(),
                ),
            ]
        )
    return actions


# 声明所有可在命令行覆盖的参数（--ros-args 之外的 launch 参数），
# 默认值与 README 里写的一致；最后用 OpaqueFunction 调用上面的 _setup。
def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real_world", default_value="false"),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
            DeclareLaunchArgument("keypoints_file", default_value=""),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("use_pcd_map", default_value="false"),
            DeclareLaunchArgument("pcd_map_file", default_value=""),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("init_x", default_value="-19.0"),
            DeclareLaunchArgument("init_y", default_value="1.0"),
            DeclareLaunchArgument("init_z", default_value="0.3"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
