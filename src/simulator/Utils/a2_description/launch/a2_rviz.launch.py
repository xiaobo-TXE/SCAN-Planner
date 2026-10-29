# ============================================================================
# 文件：a2_description/launch/a2_rviz.launch.py
#
# 【这个文件是干什么的】
#   不开物理仿真，只用 RViz 看看 A2 模型长什么样、关节能不能动。
#
# 【它启动了什么】
#   1) robot_state_publisher（A2 模型，从 urdf/a2.urdf 读入）
#   2) joint_state_publisher_gui（一个滑条窗口，可以手动掰关节）
#   3) rviz2（加载 a2.rviz 显示配置）
#
# 【和仿真主流程的关系】
#   run.launch.py 用 robot:=a2 时也启动 robot_state_publisher，但读的是同一个
#   urdf/a2.urdf；本文件只是把模型单独拎出来看，方便核对外形与双圆柱 footprint。
# ============================================================================
"""Inspect the A2 model and joints in RViz2 without a physics simulator."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("a2_description")
    model = PathJoinSubstitution([share, "urdf", "a2.urdf"])
    rviz_config = PathJoinSubstitution([share, "launch", "a2.rviz"])
    # xacro 能直接处理纯 URDF，这里只是统一用同一条加载路径
    robot_description = {"robot_description": Command(["xacro ", model])}
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
