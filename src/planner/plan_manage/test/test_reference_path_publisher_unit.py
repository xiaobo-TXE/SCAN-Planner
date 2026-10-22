"""参考路径发布器的单元测试（不起 ROS 节点，直接调函数）。

【测什么】
  scripts/reference_path_publisher.py 里的两个纯函数：
    parse_waypoints      参数里的航点表校验与切分
    build_path_message   航点 → nav_msgs/Path 消息

【怎么跑】
  colcon test --packages-select scan_planner --ctest-args -R reference
"""

import importlib.util
import os

import pytest

# 目标脚本不在包里、也不是模块，所以按文件路径动态加载
SCRIPT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts", "reference_path_publisher.py")
)
SPEC = importlib.util.spec_from_file_location("reference_path_publisher", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_parse_waypoints_accepts_xyz_triples():
    """合法的 3 的整数倍长度、且不少于两个点 → 切成三元组列表。"""
    points = MODULE.parse_waypoints([0.0, 1.0, 2.0, 3.0, 4.0, 5.0])
    assert points == [(0.0, 1.0, 2.0), (3.0, 4.0, 5.0)]


@pytest.mark.parametrize(
    "values",
    # 空表 / 点数不足两个 / 长度不是 3 的倍数 / 含 NaN —— 都应报错
    [[], [0.0, 1.0, 2.0], [0.0] * 7, [0.0, 1.0, 2.0, float("nan"), 4.0, 5.0]],
)
def test_parse_waypoints_rejects_invalid_values(values):
    """非法航点表必须抛 ValueError，避免带着坏数据发路径。"""
    with pytest.raises(ValueError):
        MODULE.parse_waypoints(values)


def test_build_path_message_preserves_coordinates_and_frame():
    """组装出的 Path：坐标系、时间戳、点数与坐标都应原样保留，朝向给单位四元数。"""
    from builtin_interfaces.msg import Time

    message = MODULE.build_path_message(
        [(-5.5, 5.5, 0.1), (-5.5, 4.5, 0.3)], "world", Time(sec=12)
    )
    assert message.header.frame_id == "world"
    assert message.header.stamp.sec == 12
    assert len(message.poses) == 2
    assert message.poses[-1].pose.position.y == pytest.approx(4.5)
    assert message.poses[-1].pose.position.z == pytest.approx(0.3)
    assert message.poses[-1].pose.orientation.w == pytest.approx(1.0)
