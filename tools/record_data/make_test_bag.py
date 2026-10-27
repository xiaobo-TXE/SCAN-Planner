#!/usr/bin/env python3
# ============================================================================
# make_test_bag.py —— 生成一个"假"的录制包，用来在没有真机时回归测试 analyze_run.py
#
# 场景：0-6s 正常前进 → 6-12s 卡住（cmd 有速度但 odom 不动）→ 12-20s 恢复前进；
#       期间有 global_path / local_path / frozen / 事件标注。
#
# 用法：
#   source /opt/ros/humble/setup.bash
#   python3 make_test_bag.py /tmp/testbag
#   python3 analyze_run.py /tmp/testbag
# ============================================================================
import sys

import rclpy
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String, Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

try:  # 有工作空间时，顺便塞一条 Bspline，覆盖该图的代码路径
    from scan_planner_msgs.msg import Bspline
    HAVE_BSPLINE = True
except Exception:
    HAVE_BSPLINE = False


def mk_odom(x, y, yaw=0.0):
    m = Odometry()
    m.pose.pose.position.x = x
    m.pose.pose.position.y = y
    m.pose.pose.position.z = 0.0
    m.pose.pose.orientation.z = __import__("math").sin(yaw / 2)
    m.pose.pose.orientation.w = __import__("math").cos(yaw / 2)
    return m


def mk_path(pts):
    p = Path()
    for (x, y) in pts:
        ps = PoseStamped()
        ps.pose.position.x = x
        ps.pose.position.y = y
        p.poses.append(ps)
    return p


def mk_cloud(xs, y, z=0.0):
    """一条平行于 x 轴、位于 y 处的"墙"点云"""
    hdr = Header()
    hdr.frame_id = "map"
    return pc2.create_cloud_xyz32(hdr, [(x, y, z) for x in xs])


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/testbag"
    import math
    w = SequentialWriter()
    w.open(StorageOptions(uri=out, storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
    for name, typ in [("/loc/robo/odom", "nav_msgs/msg/Odometry"),
                      ("/cmd_vel", "geometry_msgs/msg/Twist"),
                      ("/local_path", "nav_msgs/msg/Path"),
                      ("/global_path", "nav_msgs/msg/Path"),
                      ("/planning/go2_execution_frozen", "std_msgs/msg/Bool"),
                      ("/test_event", "std_msgs/msg/String"),
                      ("/grid_map/occupancy", "sensor_msgs/msg/PointCloud2"),
                      ("/grid_map/occupancy_inflate", "sensor_msgs/msg/PointCloud2")]:
        w.create_topic(TopicMetadata(name=name, type=typ, serialization_format="cdr"))
    if HAVE_BSPLINE:
        w.create_topic(TopicMetadata(name="/planning/bspline",
                                     type="scan_planner_msgs/msg/Bspline", serialization_format="cdr"))

    ns = int(1e9)
    t = 0.0
    x = 0.0
    step = 0.1
    # 全局路径（带一个右转）
    gp = mk_path([(i * 0.3, 0.0) for i in range(10)] + [(3.0, i * 0.3) for i in range(1, 8)])
    w.write("/global_path", serialize_message(gp), int((t + 0.05) * ns))
    last_lp = -1.0
    last_cloud = -1.0
    while t < 20.0:
        moving = (t < 6.0) or (t >= 12.0)
        if moving:
            x += 0.3 * step
        w.write("/loc/robo/odom", serialize_message(mk_odom(x, 0.0)), int(t * ns))
        # 建图点云：左侧一堵墙；卡住时段墙更近（模拟窄处/被挡），膨胀层=墙外扩 0.32
        if t - last_cloud > 0.2:
            last_cloud = t
            y_wall = 0.25 if (6.0 <= t < 12.0) else 0.60
            xs = [x - 1.0 + i * 0.1 for i in range(30)]
            w.write("/grid_map/occupancy", serialize_message(mk_cloud(xs, y_wall)), int(t * ns))
            w.write("/grid_map/occupancy_inflate", serialize_message(mk_cloud(xs, max(y_wall - 0.32, 0.0))),
                    int(t * ns))
        c = Twist()
        c.linear.x = 1.0 if (t >= 0.5) else 0.0       # 全程几乎都在"要求前进"
        w.write("/cmd_vel", serialize_message(c), int(t * ns))
        if t - last_lp > 0.5:
            last_lp = t
            w.write("/local_path", serialize_message(mk_path([(x + i * 0.3, 0.0) for i in range(12)])),
                    int(t * ns))
        if HAVE_BSPLINE:
            b = Bspline()
            b.order = 3
            b.traj_id = int(t * 10)
            pts = [(x + i * 0.3, 0.05, 0.0) for i in range(6)]
            for (px, py, pz) in pts:
                q = __import__("geometry_msgs.msg", fromlist=["Point"]).Point()
                q.x, q.y, q.z = px, py, pz
                b.pos_pts.append(q)
            b.knots = [0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0, 3.0]
            w.write("/planning/bspline", serialize_message(b), int(t * ns))
        w.write("/planning/go2_execution_frozen", serialize_message(Bool(data=(4.0 <= t < 5.0))), int(t * ns))
        if abs(t - 0.5) < 1e-6:
            w.write("/test_event", serialize_message(String(data="点目标A")), int(t * ns))
        if abs(t - 7.5) < 1e-6:
            w.write("/test_event", serialize_message(String(data="AUTO:stuck (speed=0.000)")), int(t * ns))
        if abs(t - 12.5) < 1e-6:
            w.write("/test_event", serialize_message(String(data="AUTO:move (speed=0.300)")), int(t * ns))
        t += step
    w.close()
    print(f"[✓] 假 bag 生成：{out}   (Bspline={'有' if HAVE_BSPLINE else '无'})")


if __name__ == "__main__":
    main()
