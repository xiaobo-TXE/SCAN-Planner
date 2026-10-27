#!/usr/bin/env python3
# ============================================================================
# auto_event_pub.py —— 自动"卡住/恢复"事件标注
#
# 判据（与 closed_loop_controller 的卡住检测一致，但实测速度用"位置差分"更稳）：
#   想走：max(|cmd.vx|, |cmd.vy|) > cmd_eps
#   在动：最近 window 秒内的平均位移速度 > speed_eps
#   持续 > stuck_time 秒"想走但不动"  →  发一条 /test_event "AUTO:stuck"
#   恢复运动                          →  发一条 /test_event "AUTO:move"
#
# 用法：
#   python3 auto_event_pub.py [--cmd-topic /cmd_vel] [--odom-topic /loc/robo/odom]
#                             [--event-topic /test_event]
#                             [--cmd-eps 0.15] [--speed-eps 0.03] [--stuck-time 1.0] [--window 0.5]
# ============================================================================
import argparse
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class AutoEventPub(Node):
    def __init__(self, a):
        super().__init__("auto_event_pub")
        self.cmd_eps = a.cmd_eps
        self.speed_eps = a.speed_eps
        self.stuck_time = a.stuck_time
        self.window = a.window

        self.pub = self.create_publisher(String, a.event_topic, 10)
        self.create_subscription(Twist, a.cmd_topic, self.on_cmd, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Odometry, a.odom_topic, self.on_odom, rclpy.qos.qos_profile_sensor_data)
        self.timer = self.create_timer(0.2, self.tick)

        self.cmd = Twist()
        self.have_cmd = False
        self.hist = []  # [(t, x, y)]
        self.stuck_since = None
        self.is_stuck = False

        self.get_logger().info(
            f"auto_event_pub: cmd={a.cmd_topic} odom={a.odom_topic} "
            f"cmd_eps={self.cmd_eps} speed_eps={self.speed_eps} stuck_time={self.stuck_time}"
        )

    def on_cmd(self, m: Twist):
        self.cmd = m
        self.have_cmd = True

    def on_odom(self, m: Odometry):
        t = self.get_clock().now().nanoseconds * 1e-9
        p = m.pose.pose.position
        self.hist.append((t, p.x, p.y))
        while len(self.hist) > 2 and (t - self.hist[0][0]) > self.window:
            self.hist.pop(0)

    def emit(self, text: str):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().warn(f"auto event: {text}")

    def tick(self):
        if not self.have_cmd or len(self.hist) < 2:
            return
        intend = max(abs(self.cmd.linear.x), abs(self.cmd.linear.y)) > self.cmd_eps
        (t0, x0, y0), (t1, x1, y1) = self.hist[0], self.hist[-1]
        dt = max(t1 - t0, 1e-3)
        speed = math.hypot(x1 - x0, y1 - y0) / dt
        moving = speed > self.speed_eps
        now = self.get_clock().now()

        if intend and not moving:
            if self.stuck_since is None:
                self.stuck_since = now
            elif (not self.is_stuck) and (now - self.stuck_since).nanoseconds * 1e-9 > self.stuck_time:
                self.is_stuck = True
                self.emit(f"AUTO:stuck (cmd>|{self.cmd_eps}|, speed={speed:.3f})")
        else:
            self.stuck_since = None
            if self.is_stuck:
                self.is_stuck = False
                self.emit(f"AUTO:move (speed={speed:.3f})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--odom-topic", default="/loc/robo/odom")
    ap.add_argument("--event-topic", default="/test_event")
    ap.add_argument("--cmd-eps", type=float, default=0.15)
    ap.add_argument("--speed-eps", type=float, default=0.03)
    ap.add_argument("--stuck-time", type=float, default=1.0)
    ap.add_argument("--window", type=float, default=0.5)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = AutoEventPub(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
