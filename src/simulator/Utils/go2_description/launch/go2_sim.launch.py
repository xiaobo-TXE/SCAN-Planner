# ============================================================================
# 文件：go2_description/launch/go2_sim.launch.py
#
# 【这个文件是干什么的】
#   在 Gazebo Fortress 里生成一只 Go2，让它的 12 个关节受 ROS 2 控制器驱动。
#
# 【它启动了什么】
#   1) Gazebo 仿真器（含物理引擎、传感器、场景广播插件）
#   2) robot_state_publisher：把 xacro 展开成机器人模型
#   3) ros_gz_bridge：把 Gazebo 的话题桥接到 ROS 2（时钟/IMU/足端接触）
#   4) ros_gz_sim create：把机器人“放进”仿真世界
#   5) 两个控制器：joint_state_broadcaster 和 joint_trajectory_controller
#
# 【注意】这是独立的物理仿真，与本项目规划器用的 go2_kinematic_sim 不是一回事。
# ============================================================================
"""Spawn Go2 in Gazebo Fortress with gz_ros2_control."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # xacro package:// URIs are converted by Gazebo to
    # model://go2_description/..., so Gazebo must search the parent of this
    # package's share directory.  Without this, the robot is spawned but its
    # DAE visual meshes cannot be rendered.
    # 关键：Gazebo 要通过 model://go2_description/... 找外观网格，所以它必须搜索本包 share 的上一级目录
    resource_root = os.path.dirname(get_package_share_directory("go2_description"))
    description_share = FindPackageShare("go2_description")
    ros_gz_share = FindPackageShare("ros_gz_sim")
    model = PathJoinSubstitution([description_share, "xacro", "robot.xacro"])
    world = PathJoinSubstitution([description_share, "worlds", "empty.sdf"])
    bridge_config = PathJoinSubstitution([description_share, "config", "bridge.yaml"])
    # 用 xacro 展开完整模型（use_gazebo 为 true，含控制器与插件）
    robot_description = {
        "robot_description": Command(["xacro ", model, " use_gazebo:=true"]),
        "use_sim_time": True,
    }

    # 启动 Gazebo Fortress（-r 表示启动后立刻开始运行），世界文件用 worlds/empty.sdf
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": ["-r -v 3 ", world], "on_exit_shutdown": "true"}.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    # 把机器人放进仿真世界：从 robot_description 话题读模型，命名为 go2，位置由 x/y/z 参数决定
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "go2",
            "-allow_renaming", "true",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
    )
    # 通过 controller_manager 加载两个控制器
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    # 等机器人“生成完成”后再加载控制器（顺序很重要，否则找不到关节）
    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[joint_state_broadcaster, trajectory_controller],
        )
    )
    # 用参数文件配置 ROS↔Gazebo 的话题桥接
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="go2_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )

    return LaunchDescription(
        [
            # Fortress uses IGN_GAZEBO_RESOURCE_PATH.  Set the newer alias as
            # well, keeping this launch usable with newer Gazebo releases.
            # 设置资源搜索路径。Fortress 用 IGN_GAZEBO_RESOURCE_PATH，新版用 GZ_SIM_RESOURCE_PATH，两个都设上
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=[resource_root, os.pathsep,
                       EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")],
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[resource_root, os.pathsep,
                       EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
            ),
            # 机器人出生位置（可在命令行覆盖）
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.5"),
            gazebo,
            state_publisher,
            bridge,
            spawn,
            start_controllers,
        ]
    )
