#!/usr/bin/env python3
# ============================================================================
# export_tum.py —— 把 bag 里的轨迹导出成 TUM 格式，交给 evo 算 ATE/RPE
#
# 用法（先 source ROS；解析自定义类型需要工作空间）：
#   source /opt/ros/humble/setup.bash
#   source ~/scan_planner_github_ws/install/setup.bash
#   python3 export_tum.py ~/bags/<包>
#   python3 export_tum.py <bag> --est-topic /loc/robo/odom --ref-topic /lio/robo/odom
#
# 产物（默认 <bag>/tum/）：
#   est.tum          估计轨迹（--est-topic，默认 /loc/robo/odom，即地图定位）
#   ref.tum          参考轨迹（--ref-topic，默认 /lio/robo/odom，即纯 LIO；没有该话题就不生成）
#   global_path.tum  /global_path 的每一帧（时间戳=记录时间+点序号，非真实时间参数化）
#   local_path.tum   /local_path  同上
#   bsp_last.tum     /planning/bspline 的最后一条，按 knots 时间轴真实采样
#
# 配套 evo 命令：
#   evo_traj tum est.tum ref.tum --ref=ref.tum -p --plot_mode=xy
#   evo_ape  tum ref.tum est.tum -va --plot --plot_mode=xy
#   evo_rpe  tum ref.tum est.tum -va --delta 1 --delta_unit m --plot
#
# 【重要说明】没有真值(GT)时，这是"两个估计之间"的比较，不是绝对精度：
#   - est=/loc/robo/odom（地图定位） vs ref=/lio/robo/odom（纯 LIO）
#     → RPE 小 = 两者一致；APE 大 = 地图定位在持续往回拉（修正量/漂移）。
#   - 若将来有 RTK / 动捕 / 全站仪等真值轨迹，套同样的命令即为"绝对精度"。
#
# 【小坑】轨迹近似一条直线时，Umeyama 对齐会报 "Degenerate covariance rank"
#   → 去掉 -a，或改用 --align_origin（只对齐起点）。
# ============================================================================
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_tools import bspline_eval, open_reader, resolve_type  # noqa: E402


def tq(q):
    return (q.x, q.y, q.z, q.w)


def write_tum(path, rows):
    """rows: [(t, x, y, z, qx, qy, qz, qw)]"""
    if not rows:
        return False
    with open(path, "w") as f:
        for r in rows:
            f.write("%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % r)
    print(f"  -> {path}  ({len(rows)} 条)")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--out", default=None, help="输出目录（默认 <bag>/tum）")
    ap.add_argument("--est-topic", default="/loc/robo/odom")
    ap.add_argument("--ref-topic", default="/lio/robo/odom")
    ap.add_argument("--paths", action="store_true", default=True, help="同时导出 global/local path")
    ap.add_argument("--no-paths", dest="paths", action="store_false")
    a = ap.parse_args()

    out = a.out or os.path.join(a.bag, "tum")
    os.makedirs(out, exist_ok=True)

    reader, dm, backend = open_reader(a.bag)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()} \
        if backend == "rosbag2_py" else {}
    have = set(type_map)
    print(f"[i] 后端={backend}  可用话题 {len(have)} 个")

    want = [t for t in (a.est_topic, a.ref_topic, "/global_path", "/local_path",
                        "/planning/bspline") if (not have) or t in have]
    from rosbag2_py import StorageFilter
    reader.set_filter(StorageFilter(topics=want))

    acc = {a.est_topic: [], a.ref_topic: [], "/global_path": [], "/local_path": []}
    bsp_last = None
    cls_cache = {}
    t0 = None
    while True:
        try:
            topic, raw, ts = reader.read_next()
        except Exception:
            break
        tname = type_map.get(topic, "")
        if tname not in cls_cache:
            cls_cache[tname] = resolve_type(tname)
        cls = cls_cache[tname]
        if cls is None:
            continue
        msg = dm(raw, cls)
        t = ts * 1e-9
        if t0 is None:
            t0 = t
        if topic in acc and hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            acc[topic].append((t - t0, p.x, p.y, p.z) + tq(q))
        elif topic in ("/global_path", "/local_path") and hasattr(msg, "poses"):
            for i, ps in enumerate(msg.poses):
                p, q = ps.pose.position, ps.pose.orientation
                acc[topic].append((t - t0 + i * 1e-3, p.x, p.y, p.z) + tq(q))
        elif topic == "/planning/bspline":
            pts = [(p.x, p.y, p.z) for p in msg.pos_pts]
            kn = list(msg.knots)
            if len(pts) > 3 and len(kn) == len(pts) + msg.order + 1:
                bsp_last = (msg.order, kn, pts, t - t0)

    # 若 est/ref 话题没在 bag 里，提示可用话题
    print("[i] 导出：")
    est_ok = write_tum(os.path.join(out, "est.tum"), acc[a.est_topic])
    ref_ok = write_tum(os.path.join(out, "ref.tum"), acc[a.ref_topic])
    if a.paths:
        write_tum(os.path.join(out, "global_path.tum"), acc["/global_path"])
        write_tum(os.path.join(out, "local_path.tum"), acc["/local_path"])
    if bsp_last is not None:
        order, kn, pts, tb = bsp_last
        tt = np.linspace(kn[order], kn[len(pts)], 200)
        rows = []
        for x in tt:
            px, py, pz = bspline_eval(kn, pts, order, x)
            rows.append((tb + x, px, py, pz, 0.0, 0.0, 0.0, 1.0))
        write_tum(os.path.join(out, "bsp_last.tum"), rows)

    if not est_ok:
        print(f"[警告] 没读到 --est-topic={a.est_topic}；bag 里可用话题：")
        for k in sorted(have):
            print(f"    {k}")
    if not ref_ok:
        print(f"[提示] 没读到 --ref-topic={a.ref_topic}（没有它就无法算 APE/RPE，"
              f"可用 --ref-topic 指定别的参考轨迹）")

    print(f"\n[i] 下一步：")
    if est_ok and ref_ok:
        print(f"  evo_traj tum {out}/est.tum {out}/ref.tum --ref={out}/ref.tum -p --plot_mode=xy")
        print(f"  evo_ape  tum {out}/ref.tum {out}/est.tum -va --plot --plot_mode=xy")
        print(f"  evo_rpe  tum {out}/ref.tum {out}/est.tum -va --delta 1 --delta_unit m --plot")
    else:
        print(f"  evo_traj tum {out}/est.tum {out}/global_path.tum -p --plot_mode=xy   # 至少能对照看")
    print("  （没装 evo 时：pip install --user evo）")


if __name__ == "__main__":
    main()
