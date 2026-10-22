"""参考路径发布器的启动测试：验证它“等里程计 → 只发一次”。

【测什么】
  真的把 scripts/reference_path_publisher.py 作为节点起起来，然后：
    1. 在没有任何里程计的情况下，不应该发出任何路径（要等 body_pose）；
    2. 收到第一条里程计、且本测试订阅上话题之后，应恰好发布一次路径；
    3. 之后继续给里程计，也不应该再发第二条。

【怎么跑】
  colcon test --packages-select scan_planner --ctest-args -R reference
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)


@pytest.mark.launch_test
def generate_test_description():
    """起一个发布器节点：坐标系 world、等 0.1 秒、两个航点。"""
    publisher = launch_ros.actions.Node(
        package="scan_planner",
        executable="reference_path_publisher.py",
        name="reference_path_publisher",
        parameters=[
            {
                "frame_id": "world",
                "publish_delay_sec": 0.1,
                "waypoints": [0.0, 0.0, 0.0, 1.0, 0.0, 0.5],
            }
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription(
            [
                publisher,
                launch_testing.actions.ReadyToTest(),
                # 5 秒后自动收尾，避免测试卡住
                launch.actions.TimerAction(
                    period=5.0,
                    actions=[launch.actions.Shutdown(reason="publisher test complete")],
                ),
            ]
        ),
        {"publisher": publisher},
    )


class TestReferencePathPublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)

    @classmethod
    def tearDownClass(cls):
        cls.context.shutdown()

    def test_waits_for_odom_and_publishes_once(self, proc_info, publisher):
        proc_info.assertWaitForStartup(process=publisher, timeout=10)
        node = rclpy.create_node(
            "reference_path_publisher_test", context=self.context
        )
        received = []
        # 订阅端 QoS 必须与发布端一致（可靠 + 掉线补发），否则收不到
        path_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscription = node.create_subscription(
            Path, "initial_path", received.append, path_qos
        )
        odom_publisher = node.create_publisher(
            Odometry, "body_pose", qos_profile_sensor_data
        )
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(node)
        try:
            # 阶段 1：还没有给里程计 → 不应该收到任何路径
            deadline = time.monotonic() + 0.4
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            self.assertEqual(received, [])

            # 阶段 2：开始给里程计 → 应收到恰好一条路径
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not received:
                odom_publisher.publish(Odometry())
                executor.spin_once(timeout_sec=0.05)
            self.assertEqual(len(received), 1)
            self.assertEqual(len(received[0].poses), 2)
            self.assertEqual(received[0].header.frame_id, "world")
            self.assertAlmostEqual(received[0].poses[-1].pose.position.x, 1.0)
            self.assertAlmostEqual(received[0].poses[-1].pose.position.z, 0.5)

            # 阶段 3：继续给里程计 → 不应该再发第二条
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                odom_publisher.publish(Odometry())
                executor.spin_once(timeout_sec=0.05)
            self.assertEqual(len(received), 1)
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_subscription(subscription)
            node.destroy_publisher(odom_publisher)
            node.destroy_node()
