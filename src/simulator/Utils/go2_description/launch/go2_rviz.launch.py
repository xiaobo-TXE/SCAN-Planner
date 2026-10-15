# ============================================================================
# 文件：go2_description/launch/go2_rviz.launch.py
#
# 【这个文件是干什么的】
#   不开物理仿真，只用 RViz 看看 Go2 模型长什么样、关节能不能动。
#
# 【它启动了什么】
#   1) robot_state_publisher（模型，use_gazebo 为 false）
#   2) joint_state_publisher_gui（一个滑条窗口，可以手动掰关节）
#   3) rviz2（加载 check_joint.rviz 显示配置）
# ============================================================================
"""Inspect the Go2 model and joints in RViz2 without a physics simulator."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("go2_description")
    model = PathJoinSubstitution([share, "xacro", "robot.xacro"])
    rviz_config = PathJoinSubstitution([share, "launch", "check_joint.rviz"])
    # 用 xacro 展开模型；这里不带 Gazebo 部分，所以在 RViz 里是“静态模型”
    robot_description = {"robot_description": Command(["xacro ", model, " use_gazebo:=false"])}
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                # 提供图形化关节滑条，方便检查模型
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                # 打开 RViz，加载模型检查用的显示配置
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
