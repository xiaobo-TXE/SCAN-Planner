# ============================================================================
# 文件：launch/rviz.launch.py —— 启动 RViz2 可视化界面
#
# 【怎么用】
#   ros2 launch scan_planner rviz.launch.py                    # 默认 default.rviz
#   ros2 launch scan_planner rviz.launch.py config:=replay      # 回放录制包/诊断用
#
# 【作用】打开 RViz2，并加载 share/scan_planner/rviz/<config>.rviz。
#   可选配置（都在 launch/ 下，编译时装进 share/scan_planner/rviz/）：
#     default  —— 规划器/仿真常用显示
#     replay   —— 回放录制包：原始/膨胀占据、局部/全局路径、机身圆柱、TF
# ============================================================================
"""Start RViz2 with one of SCAN-Planner's display configurations."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rviz_node(context, *args, **kwargs):
    """在启动时解析 config 名，找到对应的 .rviz，再起 rviz2。"""
    rviz_dir = os.path.join(get_package_share_directory("scan_planner"), "rviz")
    name = LaunchConfiguration("config").perform(context)
    path = os.path.join(rviz_dir, f"{name}.rviz")
    if not os.path.isfile(path):
        available = sorted(f[:-5] for f in os.listdir(rviz_dir) if f.endswith(".rviz"))
        raise RuntimeError(
            f"找不到显示配置 {path}。可用的 config 有：{available}"
        )
    return [
        Node(
            package="rviz2",
            # -d 后面跟的就是要加载的显示配置文件路径。
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", path],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "config",
                default_value="default",
                description="显示配置名（share/scan_planner/rviz/<name>.rviz）：default 或 replay",
            ),
            OpaqueFunction(function=_rviz_node),
        ]
    )
