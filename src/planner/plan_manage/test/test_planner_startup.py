# ============================================================================
# 文件：test/test_planner_startup.py —— 启动冒烟测试
#
# 【这个测试检查什么】
#   只检查一件事：带着 planner.yaml 启动 scan_planner_node 时，
#   程序能在 15 秒内正常启动（不崩溃、参数都能读出来）。
#   它不检查规划结果对不对——那属于“冒烟测试”，只求“能点着火”。
#
# 【怎么跑】  colcon test --packages-select scan_planner
# ============================================================================
"""Smoke-test that the migrated planner accepts its ROS 2 parameter file."""

import os

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest


@pytest.mark.launch_test
# 描述测试环境：启动一个 planner 节点（只给 planner.yaml），然后开始测试。
def generate_test_description():
    config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "planner.yaml")
    )
    planner = launch_ros.actions.Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        parameters=[config],
        output="screen",
    )
    return (
        launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]),
        {"planner": planner},
    )


class TestPlannerStartup:
# 断言：planner 进程在 15 秒内成功启动。
    def test_process_starts(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=15)
