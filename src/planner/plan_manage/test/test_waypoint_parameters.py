# ============================================================================
# 文件：test/test_waypoint_parameters.py —— 航点参数冒烟测试
#
# 【这个测试检查什么】
#   同时加载 planner.yaml + keypoints.example.yaml（navi_mode=2 + 航点数组）
#   启动 planner，确认那个“扁平航点数组”能被正确读入、程序能启动。
#   因为 init() 会检查航点个数是不是 3 的倍数，不合法就会启动失败。
# ============================================================================
"""Verify that the flat ROS 2 waypoint array is accepted at startup."""

import os

import launch
import launch_ros.actions
import launch_testing.actions
import pytest


@pytest.mark.launch_test
# 描述测试环境：启动 planner，参数文件按顺序叠加（先 planner.yaml，再航点文件）。
def generate_test_description():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    planner = launch_ros.actions.Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        parameters=[
            os.path.join(root, "config", "planner.yaml"),
            os.path.join(root, "config", "keypoints.example.yaml"),
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]),
        {"planner": planner},
    )


class TestWaypointParameters:
# 断言：planner 进程在 15 秒内成功启动。
    def test_process_starts(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=15)
