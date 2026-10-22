"""参考路径 + 动态参考坐标系（map→world）的启动测试。

【这个测试在考什么】
  真正把 scan_planner_node 以 navi_mode=3 起起来，然后由测试端扮演“外部”：
  持续发里程计、发一条 map 系下的参考路径、并按需广播 map→world 的 TF，
  检查规划器在各种 TF 状况下的行为是否安全：
    1. TF 缺失/过旧 → 机器人必须停下（发布一条原地不动的轨迹）；
    2. TF 恢复新鲜 → 应该恢复规划；
    3. TF 短暂丢失（未超宽限）→ 不应丢弃仍然有效的世界系轨迹；
    4. 参考系突然跳变 → 应该换一条新轨迹，并且新轨迹跟随跳变后的路线；
    5. 运动中 TF 超时 → 短暂保留后停下，恢复后能继续；
    6. 跳变后重规划失败（路径整体挪到局部栅格之外）→ 必须“失败即停”。

【怎么跑】
  colcon test --packages-select scan_planner --ctest-args -R reference
"""

import math
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from scan_planner_msgs.msg import Bspline
from tf2_ros import TransformBroadcaster


@pytest.mark.launch_test
def generate_test_description():
    """起规划器节点（用仓库自带的 planner.yaml，只把 navi_mode 覆盖成 3）。"""
    config = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "planner.yaml"))
    planner = launch_ros.actions.Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        parameters=[config, {"fsm.navi_mode": 3}],
        output="screen",
    )
    return (
        launch.LaunchDescription([
            planner,
            launch_testing.actions.ReadyToTest(),
            # 18 秒后自动收尾，避免测试挂住
            launch.actions.TimerAction(
                period=18.0,
                actions=[launch.actions.Shutdown(reason="dynamic TF test complete")],
            ),
        ]),
        {"planner": planner},
    )


class TestDynamicReferenceTf(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)

    @classmethod
    def tearDownClass(cls):
        cls.context.shutdown()

    def test_timeout_recovery_and_jump(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=10)
        node = rclpy.create_node("dynamic_reference_tf_test", context=self.context)
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(node)
        # 模拟外部世界：里程计、参考路径、map→world 变换
        odom_pub = node.create_publisher(Odometry, "body_pose", qos_profile_sensor_data)
        path_pub = node.create_publisher(Path, "initial_path", 10)
        tf_pub = TransformBroadcaster(node)
        # 收集规划器发出的轨迹，用来判断“是否停下 / 是否换了新轨迹”
        received = []
        node.create_subscription(Bspline, "planning/bspline", received.append, 10)

        # 参考路径：map 系下沿 x 轴的两个点
        path = Path()
        path.header.frame_id = "map"
        for x in (0.0, 2.0):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.z = 1.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        def pump(seconds, tf_y=None, send_path=False):
            """在给定时间内持续“喂”里程计（可选发路径、可选广播 TF），并转执行器。"""
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                odom = Odometry()
                odom.header.frame_id = "world"
                odom.header.stamp = node.get_clock().now().to_msg()
                odom.pose.pose.position.z = 1.0
                odom.pose.pose.orientation.w = 1.0
                odom_pub.publish(odom)
                if send_path:
                    path.header.stamp = node.get_clock().now().to_msg()
                    path_pub.publish(path)
                if tf_y is not None:
                    tf = TransformStamped()
                    tf.header.frame_id = "map"
                    tf.child_frame_id = "world"
                    tf.header.stamp = node.get_clock().now().to_msg()
                    tf.transform.translation.y = tf_y
                    tf.transform.rotation.w = 1.0
                    tf_pub.sendTransform(tf)
                executor.spin_once(timeout_sec=0.04)

        def stationary(msg):
            """判断一条轨迹是不是“原地不动”（所有控制点位置相同）。"""
            return bool(msg.pos_pts) and all(
                math.isclose(p.x, msg.pos_pts[0].x, abs_tol=1e-6) and
                math.isclose(p.y, msg.pos_pts[0].y, abs_tol=1e-6)
                for p in msg.pos_pts
            )

        try:
            # 场景 1：给了参考路径，但始终没有 TF → 应停下（收到原地不动的轨迹）
            pump(0.3, send_path=True)
            pump(0.8)
            self.assertTrue(any(stationary(msg) for msg in received),
                            "stale/missing TF did not stop the robot")

            # 场景 2：TF 出现且新鲜 → 应恢复规划（出现非原地轨迹）
            pump(2.0, tf_y=0.0)
            moving = [msg for msg in received if not stationary(msg)]
            self.assertTrue(moving, "fresh TF did not resume local planning")
            before_jump = max(msg.traj_id for msg in moving)

            # 场景 3：短时间丢失 TF（未超宽限）→ 不应废弃仍有效的世界系轨迹
            before_dropout = len(received)
            pump(0.2)
            self.assertFalse(any(stationary(msg) for msg in received[before_dropout:]))
            pump(0.2, tf_y=0.0)
            before_jump = max(msg.traj_id for msg in received if not stationary(msg))

            # 场景 4：参考系平移跳变 0.3 米 → 应触发新轨迹，且新轨迹跟随跳变后的路线
            pump(1.0, tf_y=0.3)
            after_jump = [msg for msg in received if msg.traj_id > before_jump]
            self.assertTrue(after_jump, "map -> world jump did not trigger a new trajectory")
            self.assertTrue(any(
                not stationary(msg) and msg.pos_pts[-1].y < -0.1
                for msg in after_jump
            ), "new local trajectory did not follow the shifted map-frame route")

            # 场景 5：运动中 TF 超时 → 先短暂保留，再停下；恢复后能继续
            before_timeout = max(msg.traj_id for msg in received)
            pump(0.95)
            self.assertTrue(any(
                stationary(msg) and msg.traj_id > before_timeout for msg in received
            ), "active-trajectory TF timeout did not stop")
            pump(0.8, tf_y=0.3)
            resumed = [msg for msg in received if not stationary(msg)]
            self.assertGreater(max(msg.traj_id for msg in resumed), before_timeout)

            # 场景 6：把 map 路径整体挪到局部栅格之外 → 跳变后的重规划必须失败即停，
            # 不能继续沿用旧轨迹
            before_failure = max(msg.traj_id for msg in received)
            pump(0.35, tf_y=10.0)
            self.assertTrue(any(
                stationary(msg) and msg.traj_id > before_failure for msg in received
            ), "failed jump replan continued to execute the old trajectory")
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
