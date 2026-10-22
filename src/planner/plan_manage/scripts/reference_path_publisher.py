#!/usr/bin/env python3
"""按参数文件里配置的航点，发布一条三维参考路径（navi_mode=3 的示例/桩发布器）。

用法：由 run.launch.py 在传入 reference_path_file:=<参数文件> 时拉起。
节点会先等里程计出现，再等规划器订阅上话题，然后发布一次就退出发布循环。
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)


def parse_waypoints(values):
    """把参数里拍平的 [x,y,z, x,y,z, ...] 校验并切成三元组列表。"""
    # 至少两个点，且长度必须是 3 的整数倍
    if values is None or len(values) < 6 or len(values) % 3 != 0:
        raise ValueError("waypoints must contain at least two x,y,z triples")
    numbers = [float(value) for value in values]
    # 数值必须是有限值（不允许 NaN / Inf）
    if not all(math.isfinite(value) for value in numbers):
        raise ValueError("waypoints must contain only finite values")
    return [tuple(numbers[index : index + 3]) for index in range(0, len(numbers), 3)]


def build_path_message(waypoints, frame_id, stamp):
    """把航点列表组装成 nav_msgs/Path 消息（朝向统一给“无旋转”）。"""
    message = Path()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    for x, y, z in waypoints:
        pose = PoseStamped()
        message.poses.append(pose)
        pose.header = message.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
    return message


class ReferencePathPublisher(Node):
    """等里程计就绪后，把配置里的一条参考路径发出去（只发一次）。"""

    def __init__(self):
        super().__init__("reference_path_publisher")
        # 参数：路径所在坐标系、航点表、发布前的等待时间
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("waypoints", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("publish_delay_sec", 0.5)

        self.frame_id = self.get_parameter("frame_id").value
        self.waypoints = parse_waypoints(self.get_parameter("waypoints").value)
        self.publish_delay_sec = float(self.get_parameter("publish_delay_sec").value)
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")
        if not math.isfinite(self.publish_delay_sec) or self.publish_delay_sec < 0.0:
            raise ValueError("publish_delay_sec must be finite and non-negative")

        # QoS 用“可靠 + 掉线后仍能补发”的配置：
        # 规划器可能比本节点启动得晚，瞬时/尽力而为的 QoS 会直接丢消息。
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(Path, "initial_path", path_qos)
        # 订阅里程计只是为了知道“系统已经跑起来了”，消息内容本身不用
        self.odom_subscription = self.create_subscription(
            Odometry, "body_pose", self.odom_callback, qos_profile_sensor_data
        )
        self.first_odom_time = None
        self.published = False
        # 用定时器反复尝试发布，直到满足条件（不能阻塞在 __init__ 里）
        self.timer = self.create_timer(0.05, self.try_publish)
        self.get_logger().info(
            f"Waiting to publish {len(self.waypoints)} reference-path points"
        )

    def odom_callback(self, _message):
        """记下第一条里程计到达的时刻，作为“系统已就绪”的起点。"""
        if self.first_odom_time is None:
            self.first_odom_time = self.get_clock().now()

    def try_publish(self):
        """满足“有里程计 + 已有订阅者 + 等够时间”三个条件时发布一次，然后停掉定时器。"""
        if self.published or self.first_odom_time is None:
            return
        # 还没有订阅者 → 现在发出去会被丢掉，继续等
        if self.path_publisher.get_subscription_count() == 0:
            return
        elapsed = (self.get_clock().now() - self.first_odom_time).nanoseconds / 1e9
        if elapsed < self.publish_delay_sec:
            return

        message = build_path_message(
            self.waypoints, self.frame_id, self.get_clock().now().to_msg()
        )
        self.path_publisher.publish(message)
        self.published = True
        self.timer.cancel()
        self.get_logger().info(
            f"Published reference path on initial_path with {len(message.poses)} points"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ReferencePathPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError) as error:
        # 参数不合法时打印原因；ValueError 继续抛出，让进程以失败状态退出
        if node is not None:
            node.get_logger().error(str(error))
        else:
            rclpy.logging.get_logger("reference_path_publisher").error(str(error))
        if isinstance(error, ValueError):
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
