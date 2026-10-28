#!/usr/bin/env python3
# ============================================================================
# bag_tools.py —— 读 rosbag2 + B 样条求值的公共小工具
#   被 analyze_run.py 与 export_tum.py 共用，避免重复实现。
# ============================================================================
import glob
import os
import sys

import numpy as np


def open_reader(bag):
    """返回 (reader, deserialize_fn, backend)。优先 rosbag2_py（需 source ROS），退化到 rosbags。"""
    # rosbag2 的 file 压缩包（只剩 *.db3.zstd）rosbag2_py 直接读不了，给出解压办法
    if isinstance(bag, str) and not glob.glob(os.path.join(bag, "*.db3")):
        zs = glob.glob(os.path.join(bag, "*.db3.zstd"))
        if zs:
            print("[提示] 这是 rosbag2 file 压缩的包（*.db3.zstd），rosbag2_py 读不了。先解压：")
            for z in zs:
                print(f"    zstd -d -f '{z}' -o '{z[:-5]}' && rm '{z}'")
            print("    并把 metadata.yaml 里的 compression_mode: FILE 改成 NONE")
            sys.exit(3)
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""),
               rosbag2_py.ConverterOptions("cdr", "cdr"))
        return r, deserialize_message, "rosbag2_py"
    except Exception:
        pass
    try:
        from rosbags.highlevel import AnyReader
        return AnyReader(bag), None, "rosbags"
    except Exception as e:
        print(f"[错误] 需要 rosbag2_py（source ROS）或 rosbags（pip install rosbags）：{e}")
        sys.exit(1)


def resolve_type(type_str):
    """'nav_msgs/msg/Odometry' -> nav_msgs.msg.Odometry 类；失败返回 None"""
    try:
        pkg, _, name = type_str.split("/")
        mod = __import__(f"{pkg}.msg", fromlist=[name])
        return getattr(mod, name)
    except Exception:
        return None


def bspline_eval(knots, pts, p, t):
    """B 样条求值（deBoor）。pts: 控制点序列；knots: 节点向量；p: 阶数。"""
    m = len(pts)
    t = min(max(t, knots[p]), knots[m] - 1e-9)
    k = p
    while k < m - 1 and t >= knots[k + 1]:
        k += 1
    d = [[float(v) for v in pts[k - p + j]] for j in range(p + 1)]
    for r in range(1, p + 1):
        for j in range(p, r - 1, -1):
            i = k - p + j
            den = knots[i + p - r + 1] - knots[i]
            a = (t - knots[i]) / den if den > 1e-12 else 0.0
            d[j] = [(1 - a) * d[j - 1][c] + a * d[j][c] for c in range(3)]
    return d[p]


def validate_bspline():
    """直线上的控制点应得到同一条直线（自检用）"""
    pts = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]
    knots = [0, 0, 0, 0, 3, 3, 3, 3]
    ts = np.linspace(0, 3, 7)
    xy = np.array([bspline_eval(knots, pts, 3, t) for t in ts])
    return float(np.max(np.abs(xy[:, 1]))) < 1e-9 and abs(xy[-1, 0] - 3.0) < 1e-6
