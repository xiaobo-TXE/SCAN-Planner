#!/usr/bin/env python3
# ============================================================================
# 文件：tools/sim_check_probe.py —— 仿真自检探针（被 tools/sim_check.sh 调用）
#
# 在仿真栈已经拉起后，检查四项（对应 README 的「仿真自检清单」）：
#   ① 模型显示     ：/robot_description 存在且是一份 URDF
#   ② TF world→base：能被 tf2 查到（模型根节点必须是 base）
#   ③ 跟速度走     ：往 /quad_0/cmd_vel 发速度，base 的位移要跟着变
#   ④ 动画与碰撞   ：/joint_states 有 12 个关节；/self_inflation 收到 marker
#
# 用法：python3 sim_check_probe.py <机型名>
# 退出码：0 全部通过，1 有失败项
# ============================================================================
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener

ROBOT = sys.argv[1] if len(sys.argv) > 1 else "?"
TIMEOUT = 20.0


class Probe(Node):
    def __init__(self):
        super().__init__("sim_check_probe")
        self.desc = None
        self.joints = None
        self.marker = None
        self.cmd = self.create_publisher(Twist, "/quad_0/cmd_vel", 10)
        # robot_description / joint_states / self_inflation 都是 transient_local 发布，
        # 这里用兼容的 QoS 订阅，保证晚加入也能拿到最后一帧。
        late = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description",
                                 lambda m: setattr(self, "desc", m.data), late)
        self.create_subscription(String, "/robot_description",
                                 lambda m: setattr(self, "desc", m.data), 1)
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: setattr(self, "joints", m), 10)
        self.create_subscription(Marker, "/self_inflation",
                                 lambda m: setattr(self, "marker", m), late)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def base_xy(self):
        try:
            t = self.tf.lookup_transform("world", "base", rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None


def main():
    rclpy.init()
    p = Probe()
    results = []

    # ① 模型
    p.spin_for(6.0)
    ok1 = bool(p.desc) and "<robot" in p.desc
    links = p.desc.count("<link ") if p.desc else 0
    joints = p.desc.count("<joint ") if p.desc else 0
    results.append(("① 模型 robot_description",
                    f"OK  link={links} joint={joints}" if ok1
                    else "FAIL 没收到 /robot_description"))

    # ② TF world→base
    p.spin_for(5.0)
    p0 = p.base_xy()
    results.append(("② TF world->base",
                    f"OK  base=({p0[0]:.2f}, {p0[1]:.2f})" if p0
                    else "FAIL 查不到 world->base"))

    # ③ 跟 cmd_vel 走
    ok3 = False
    detail3 = "FAIL 无 TF，无法判断"
    if p0:
        # 朝当前朝向推 0.4 m/s，约 2 秒
        yaw = None
        try:
            t = p.tf.lookup_transform("world", "base", rclpy.time.Time())
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except Exception:
            yaw = 0.0
        tw = Twist()
        tw.linear.x = 0.4
        end = time.time() + 2.5
        while time.time() < end:
            p.cmd.publish(tw)
            rclpy.spin_once(p, timeout_sec=0.05)
        tw.linear.x = 0.0
        p.cmd.publish(tw)
        p.spin_for(1.5)
        p1 = p.base_xy()
        if p1:
            moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            ok3 = moved > 0.15
            detail3 = (f"OK  移动 {moved:.2f} m" if ok3
                       else f"FAIL 只移动 {moved:.3f} m")
    results.append(("③ 跟随 /quad_0/cmd_vel", detail3))

    # ④ 关节动画 + 碰撞双圆柱
    p.spin_for(8.0)
    names = list(p.joints.name) if p.joints else []
    leg = [n for n in names if n.endswith(("_hip_joint", "_thigh_joint", "_calf_joint"))]
    ok4a = len(leg) == 12
    results.append(("④a /joint_states 12 关节",
                    f"OK  {len(leg)} 个腿关节" if ok4a
                    else f"FAIL 只收到 {len(names)} 个关节 {names[:4]}"))
    ok4b = p.marker is not None and p.marker.type == Marker.CYLINDER
    rad = (p.marker.scale.x / 2.0) if ok4b else 0.0
    results.append(("④b /self_inflation 圆柱",
                    f"OK  半径 {rad:.3f} m" if ok4b
                    else "FAIL 没收到 marker"))

    print(f"  ---- {ROBOT} 自检 ----")
    for name, msg in results:
        print(f"    {name:26s} {msg}")
    bad = [n for n, m in results if m.startswith("FAIL")]
    print(f"    => {'全部通过' if not bad else '失败项: ' + ', '.join(bad)}")
    p.destroy_node()
    rclpy.shutdown()
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
