# ============================================================================
# 文件：launch/rviz.launch.py —— 启动 RViz2 可视化界面
#
# 【怎么用】  ros2 launch scan_planner rviz.launch.py
# 【作用】    打开 RViz2，并加载本项目预设好的显示配置 default.rviz
#            （哪些话题要显示、什么颜色，都写在那个配置里）
# ============================================================================
"""Start RViz2 with the SCAN-Planner display configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 先拼出 default.rviz 的完整路径，再启动一个 rviz2 节点。
def generate_launch_description():
    config = os.path.join(get_package_share_directory("scan_planner"), "rviz", "default.rviz")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="rviz2",
            # -d 后面跟的就是要加载的显示配置文件路径。
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", config],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
        ]
    )
