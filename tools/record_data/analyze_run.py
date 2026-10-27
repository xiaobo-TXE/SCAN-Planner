#!/usr/bin/env python3
# ============================================================================
# analyze_run.py —— 离线分析一次真机录制的 bag，产出指标 + 图 + 结论
#
# 用法（需先 source ROS，或 pip install rosbags 走纯 python 分支）：
#   source /opt/ros/humble/setup.bash
#   source ~/scan_planner_github_ws/install/setup.bash     # 为了解析 Bspline 等自定义类型
#   python3 analyze_run.py ~/bags/stair_corner_20261027_153000
#   python3 analyze_run.py <bag> --out ~/bags/report_x --with-clouds
#
# 产出（默认写在 <bag>/analysis/ 下）：
#   report.md              指标表 + 自动结论
#   figs/01_traj.png       俯视轨迹：odom / global_path / local_path / B样条
#   figs/02_speed.png      指令速度 vs 实测速度（卡住段着色）
#   figs/03_track_error.png 横向跟踪误差
#   figs/04_state.png      /loc/status 的 state（+ 数值字段）
#   figs/05_lanes.png      事件泳道：冻结 / 卡住 / 事件标注
#   figs/06_nearest_cloud.png  RAW vs INFLATE 最近障碍距离（--with-clouds）
# ============================================================================
import argparse
import bisect
import math
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def setup_font():
    """挑一个可用的中文字体，避免图上中文变方框；找不到就返回 None（用英文兜底）"""
    import matplotlib.font_manager as fm
    avail = {f.name for f in fm.fontManager.ttflist}
    for c in ("Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Zen Hei",
              "WenQuanYi Micro Hei", "Droid Sans Fallback", "AR PL UMing CN"):
        if c in avail:
            matplotlib.rcParams["font.sans-serif"] = [c] + list(matplotlib.rcParams["font.sans-serif"])
            matplotlib.rcParams["axes.unicode_minus"] = False
            return c
    return None

# ---------------------------------------------------------------------------
# bag 读取：优先 rosbag2_py（需 source ROS），退化到 rosbags（纯 python）
# ---------------------------------------------------------------------------
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_tools import bspline_eval, open_reader, resolve_type, validate_bspline  # noqa: E402


# ---------------------------------------------------------------------------
def collect(bag):
    reader, dm, backend = open_reader(bag)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()} \
        if backend == "rosbag2_py" else {}
    have = set(type_map)
    data = {k: [] for k in ("odom", "cmd", "local_path", "global_path", "bspline",
                            "frozen", "events", "status", "occ", "inf")}
    names = {
        "odom": ["/loc/robo/odom", "/lio/robo/odom", "/loc/odom"],
        "cmd": ["/cmd_vel", "/nav_cmd"],
        "local_path": ["/local_path"], "global_path": ["/global_path"],
        "bspline": ["/planning/bspline"], "frozen": ["/planning/go2_execution_frozen"],
        "events": ["/test_event"], "status": ["/loc/status"],
        "occ": ["/grid_map/occupancy"], "inf": ["/grid_map/occupancy_inflate"],
    }
    pick = {}
    for k, cands in names.items():
        for c in cands:
            if (not type_map) or c in have:
                pick[k] = c
                break
    print(f"[i] 后端={backend}  使用话题：" +
          ", ".join(f"{k}={v}" for k, v in pick.items() if v))
    want = [v for v in pick.values() if v]
    if backend == "rosbag2_py":
        from rosbag2_py import StorageFilter
        reader.set_filter(StorageFilter(topics=want))
        it = reader
        cls_cache = {}
        while True:
            try:
                topic, raw, ts = it.read_next()
            except Exception:
                break
            key = next((k for k, v in pick.items() if v == topic), None)
            if key is None:
                continue
            tname = type_map.get(topic, "")
            if tname not in cls_cache:
                cls_cache[tname] = resolve_type(tname)
            cls = cls_cache[tname]
            if cls is None:
                continue
            msg = dm(raw, cls)
            store(data, key, ts * 1e-9, msg)
    else:  # rosbags
        from rosbags.typesys import Stores, get_typestore
        ts_ = get_typestore(Stores.ROS2_HUMBLE)
        with reader as rr:
            for conn, t_ns, raw in rr.messages(connections=[c for c in rr.connections if c.topic in want]):
                key = next((k for k, v in pick.items() if v == conn.topic), None)
                if key is None:
                    continue
                try:
                    msg = ts_.deserialize_cdr(raw, conn.msgtype)
                except Exception:
                    continue
                store(data, key, t_ns * 1e-9, msg)
    return data, pick, type_map


def store(d, key, t, msg):
    if key == "odom":
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear
        d[key].append((t, p.x, p.y, p.z, yaw, math.hypot(v.x, v.y)))
    elif key == "cmd":
        d[key].append((t, msg.linear.x, msg.linear.y, msg.angular.z))
    elif key in ("local_path", "global_path"):
        pts = [(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z) for ps in msg.poses]
        if pts:
            d[key].append((t, np.array(pts)))
    elif key == "bspline":
        pts = np.array([[p.x, p.y, p.z] for p in msg.pos_pts])
        kn = np.array(msg.knots, dtype=float)
        if pts.shape[0] > 3 and kn.size == pts.shape[0] + msg.order + 1:
            d[key].append((t, pts, kn, msg.order))
    elif key == "frozen":
        d[key].append((t, bool(msg.data)))
    elif key == "events":
        d[key].append((t, str(msg.data)))
    elif key == "status":
        d[key].append((t, msg))
    elif key in ("occ", "inf"):
        d[key].append((t, msg))


# ---------------------------------------------------------------------------
def from_pc2(msg):
    import sensor_msgs_py.point_cloud2 as pc2
    try:
        a = np.array([[p[0], p[1], p[2]] for p in
                      pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])
        return a
    except Exception:
        return np.zeros((0, 3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--out", default=None)
    ap.add_argument("--stuck-cmd-eps", type=float, default=0.15)
    ap.add_argument("--stuck-speed-eps", type=float, default=0.03)
    ap.add_argument("--stuck-time", type=float, default=1.0)
    ap.add_argument("--window", type=float, default=0.5)
    ap.add_argument("--with-clouds", action="store_true", help="额外算 RAW/INFLATE 最近障碍距离（慢）")
    ap.add_argument("--max-cloud-frames", type=int, default=150)
    ap.add_argument("--cyl-offset", type=float, default=0.205, help="机身两圆柱中心偏移(m)")
    a = ap.parse_args()
    font = setup_font()
    if font is None:
        print("[警告] 没找到中文字体，图中的中文可能显示为方框")

    out = a.out or os.path.join(a.bag, "analysis")
    figs = os.path.join(out, "figs")
    os.makedirs(figs, exist_ok=True)

    print(f"[i] 读取 {a.bag}")
    if not validate_bspline():
        print("[警告] B 样条求值自检未通过")
    data, pick, type_map = collect(a.bag)
    if not data["odom"]:
        print("[错误] 没读到里程计（/loc/robo/odom 或 /lio/robo/odom）。话题清单：")
        for k, v in sorted(type_map.items()):
            print(f"    {v}  {k}")
        sys.exit(2)

    odom = np.array([r[:6] for r in data["odom"]])
    t0 = odom[0, 0]
    T = odom[:, 0] - t0
    P = odom[:, 1:4]

    # 里程 / 时长 / 速度
    dxy = np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1)
    dist = float(dxy.sum())
    dur = float(T[-1])
    # 窗口化实测速度（用位置差分）
    ws = []
    for i in range(len(odom)):
        j = np.searchsorted(odom[:, 0], odom[i, 0] - a.window)
        j = min(j, i)
        dt = max(odom[i, 0] - odom[j, 0], 1e-3)
        ws.append(float(np.linalg.norm(P[i, :2] - P[j, :2]) / dt))
    v_odom = np.array(ws)

    # 指令速度
    cmd = np.array(data["cmd"]) if data["cmd"] else np.zeros((0, 4))
    v_cmd = np.zeros_like(v_odom)
    if cmd.shape[0]:
        for i in range(len(odom)):
            k = np.searchsorted(cmd[:, 0], odom[i, 0]) - 1
            if k >= 0:
                v_cmd[i] = max(abs(cmd[k, 1]), abs(cmd[k, 2]))

    # 卡住段
    stalls = []
    i = 0
    while i < len(odom):
        if v_cmd[i] > a.stuck_cmd_eps and v_odom[i] < a.stuck_speed_eps:
            j = i
            while j + 1 < len(odom) and v_cmd[j + 1] > a.stuck_cmd_eps and v_odom[j + 1] < a.stuck_speed_eps:
                j += 1
            if odom[j, 0] - odom[i, 0] >= a.stuck_time:
                stalls.append((odom[i, 0] - t0, odom[j, 0] - t0, odom[i, 1], odom[i, 2]))
            i = j + 1
        else:
            i += 1

    # 横向跟踪误差（到"当前生效"的 local_path 最近点）
    lp = data["local_path"]
    err = np.full(len(odom), np.nan)
    if lp:
        lt = np.array([x[0] for x in lp])
        for i in range(len(odom)):
            k = np.searchsorted(lt, odom[i, 0] + 1e-9) - 1
            if k < 0:
                continue
            Q = lp[k][1][:, :2]
            err[i] = float(np.min(np.linalg.norm(Q - P[i, :2], axis=1)))

    # 冻结段
    froz = data["frozen"]
    frozen_secs = 0.0
    if froz:
        ft = np.array([x[0] for x in froz])
        fv = np.array([x[1] for x in froz])
        for i in range(len(ft) - 1):
            if fv[i]:
                frozen_secs += ft[i + 1] - ft[i]

    # 状态
    st_t, st_v, st_extra = [], [], {}
    if data["status"]:
        for (t, m) in data["status"]:
            st_t.append(t - t0)
            st_v.append(getattr(m, "state", np.nan))
            for f in ("rmse", "inlier_ratio", "correction_t", "last_match_age"):
                if hasattr(m, f):
                    st_extra.setdefault(f, []).append((t - t0, getattr(m, f)))

    # ---------------- 出图 ----------------
    # 01 轨迹
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.plot(odom[:, 1], odom[:, 2], "-", lw=3, color="#1565c0", label="odom（实际）", zorder=5)
    for (t, pts) in data["global_path"]:
        ax.plot(pts[:, 0], pts[:, 1], "-", lw=1.0, color="#9e9e9e", alpha=0.6)
    if data["global_path"]:
        ax.plot([], [], "-", color="#9e9e9e", label="global_path")
    for (t, pts) in data["local_path"][-40:]:
        ax.plot(pts[:, 0], pts[:, 1], "-", lw=1.0, color="#2e7d32", alpha=0.5)
    if data["local_path"]:
        ax.plot([], [], "-", color="#2e7d32", label="local_path(后40条)")
    for (t, pts, kn, order) in data["bspline"][-40:]:
        tt = np.linspace(kn[order], kn[len(pts)], 60)
        xy = np.array([bspline_eval(list(kn), list(pts), order, x) for x in tt])
        ax.plot(xy[:, 0], xy[:, 1], "--", lw=1.0, color="#ef6c00", alpha=0.6)
    if data["bspline"]:
        ax.plot([], [], "--", color="#ef6c00", label="bspline(后40条)")
    if stalls:
        ax.plot([s[2] for s in stalls], [s[3] for s in stalls], "x", ms=10, mew=3,
                color="#d32f2f", label="卡住点", zorder=6)
    ax.plot(P[0, 0], P[0, 1], "o", ms=10, color="#000000", label="起点")
    ax.plot(P[-1, 0], P[-1, 1], "s", ms=10, color="#000000", label="终点")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(loc="best", fontsize=8)
    ax.set_title("俯视轨迹对比 (m)"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    fig.savefig(os.path.join(figs, "01_traj.png"), dpi=110, bbox_inches="tight"); plt.close(fig)

    # 02 速度
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(T, v_cmd, lw=1.0, color="#c62828", label="cmd 指令速度(|vx|/|vy| max)")
    ax.plot(T, v_odom, lw=1.2, color="#1565c0", label="odom 实测速度")
    for (s, e, x, y) in stalls:
        ax.axvspan(s, e, color="#ffcdd2", alpha=0.6, label="卡住" if s == stalls[0][0] else None)
    ax.set_xlabel("t (s)"); ax.set_ylabel("m/s"); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.set_title("指令速度 vs 实测速度")
    fig.savefig(os.path.join(figs, "02_speed.png"), dpi=110, bbox_inches="tight"); plt.close(fig)

    # 03 跟踪误差
    if lp:
        fig, ax = plt.subplots(figsize=(12, 3.5))
        ax.plot(T, err, lw=1.0, color="#6a1b9a")
        ax.set_xlabel("t (s)"); ax.set_ylabel("m"); ax.grid(alpha=0.3)
        ax.set_title("横向跟踪误差（odom 到当帧 local_path 的最近距离）")
        fig.savefig(os.path.join(figs, "03_track_error.png"), dpi=110, bbox_inches="tight"); plt.close(fig)

    # 04 定位状态
    if st_t:
        n = 1 + len(st_extra)
        fig, axs = plt.subplots(n, 1, figsize=(12, 2.2 * n), sharex=True, squeeze=False)
        axs[0][0].step(st_t, st_v, where="post", color="#00695c")
        axs[0][0].set_ylabel("state"); axs[0][0].grid(alpha=0.3)
        axs[0][0].set_title("/loc/status 定位状态与质量")
        for i, (f, arr) in enumerate(st_extra.items(), start=1):
            axs[i][0].plot([p[0] for p in arr], [p[1] for p in arr], lw=0.8)
            axs[i][0].set_ylabel(f); axs[i][0].grid(alpha=0.3)
        axs[-1][0].set_xlabel("t (s)")
        fig.savefig(os.path.join(figs, "04_state.png"), dpi=110, bbox_inches="tight"); plt.close(fig)

    # 05 泳道
    fig, ax = plt.subplots(figsize=(12, 3.6))
    lanes = []
    lanes.append(("卡住", [(s, e) for (s, e, _, _) in stalls], "#d32f2f"))
    if froz:
        segs, cur, start = [], False, None
        for i in range(len(ft)):
            if fv[i] and not cur:
                cur, start = True, ft[i] - t0
            elif not fv[i] and cur:
                cur = False; segs.append((start, ft[i] - t0))
        if cur:
            segs.append((start, T[-1]))
        lanes.append(("冻结(frozen)", segs, "#1976d2"))
    for li, (name, segs, col) in enumerate(lanes):
        for (s, e) in segs:
            ax.barh(li, max(e - s, 0.05), left=s, height=0.5, color=col, alpha=0.8)
    n_ev = 0
    for (t, txt) in data["events"]:
        x = t - t0
        auto = txt.startswith("AUTO:")
        ax.axvline(x, color=("#00897b" if auto else "#f9a825"), lw=0.8, alpha=0.7)
        if not auto:
            ax.text(x, len(lanes) - 0.4, txt[:22], rotation=90, fontsize=6, va="top")
        n_ev += 1
    ax.set_yticks(range(len(lanes))); ax.set_yticklabels([l[0] for l in lanes])
    ax.set_ylim(-0.6, len(lanes) + 0.6); ax.set_xlabel("t (s)")
    ax.set_title(f"事件泳道（竖线=事件标注 {n_ev} 条，黄=手动、绿=自动）")
    ax.grid(alpha=0.3, axis="x")
    fig.savefig(os.path.join(figs, "05_lanes.png"), dpi=110, bbox_inches="tight"); plt.close(fig)

    # 06 RAW vs INFLATE 最近障碍距离
    nearest = None
    if a.with_clouds and data["occ"] and data["inf"]:
        import sensor_msgs_py.point_cloud2 as pc2
        step = max(1, len(odom) // a.max_cloud_frames)
        rows = []
        for i in range(0, len(odom), step):
            t = odom[i, 0]
            def nearest_in(key):
                k = min(range(len(data[key])), key=lambda q: abs(data[key][q][0] - t))
                if abs(data[key][k][0] - t) > 0.5:
                    return np.nan
                A = from_pc2(data[key][k][1])
                if A.shape[0] == 0:
                    return np.nan
                c = np.array([P[i, 0] + a.cyl_offset * math.cos(odom[i, 4]),
                              P[i, 1] + a.cyl_offset * math.sin(odom[i, 4])])
                c2 = np.array([P[i, 0] - a.cyl_offset * math.cos(odom[i, 4]),
                               P[i, 1] - a.cyl_offset * math.sin(odom[i, 4])])
                d = np.minimum(np.linalg.norm(A[:, :2] - c, axis=1),
                               np.linalg.norm(A[:, :2] - c2, axis=1))
                return float(np.min(d))
            rows.append((t - t0, nearest_in("occ"), nearest_in("inf")))
        nearest = np.array(rows)
        fig, ax = plt.subplots(figsize=(12, 3.5))
        ax.plot(nearest[:, 0], nearest[:, 1], lw=1.0, color="#455a64", label="RAW(原始占据)最近距离")
        ax.plot(nearest[:, 0], nearest[:, 2], lw=1.0, color="#d32f2f", label="INFLATE(膨胀后)最近距离")
        ax.axhline(0.32, ls="--", lw=0.8, color="#9e9e9e", label="膨胀半径 0.32")
        ax.set_xlabel("t (s)"); ax.set_ylabel("m"); ax.grid(alpha=0.3); ax.legend(fontsize=8)
        ax.set_title("机身到最近障碍：RAW vs INFLATE")
        fig.savefig(os.path.join(figs, "06_nearest_cloud.png"), dpi=110, bbox_inches="tight"); plt.close(fig)

    # ---------------- report.md ----------------
    stall_secs = sum(e - s for (s, e, _, _) in stalls)
    L = []
    L.append(f"# 录制分析报告\n")
    L.append(f"- bag：`{a.bag}`")
    L.append(f"- 时长：**{dur:.1f} s**　里程：**{dist:.1f} m**　平均速度：**{dist/max(dur,1e-6):.2f} m/s**")
    L.append(f"- 采样：odom {len(odom)} 条，cmd {cmd.shape[0]} 条，local_path {len(lp)} 条，"
             f"global_path {len(data['global_path'])} 条，bspline {len(data['bspline'])} 条\n")
    L.append("## 关键指标\n")
    L.append("| 指标 | 值 |")
    L.append("|---|---|")
    L.append(f"| 卡住段数 | {len(stalls)} |")
    L.append(f"| 卡住总时长 | {stall_secs:.1f} s（占 {100*stall_secs/max(dur,1e-6):.1f}%） |")
    if lp:
        L.append(f"| 横向跟踪误差 均值/最大 | {np.nanmean(err):.3f} / {np.nanmax(err):.3f} m |")
    L.append(f"| 冻结(frozen)总时长 | {frozen_secs:.1f} s |")
    L.append(f"| 事件标注 | {len(data['events'])} 条 |")
    if st_t:
        bad = sum(1 for v in st_v if v != 3)
        L.append(f"| 定位 state≠3 采样数 | {bad} / {len(st_v)} |")
    L.append("")
    if stalls:
        L.append("## 卡住段明细\n")
        L.append("| 起(s) | 止(s) | 时长(s) | 位置(x,y) |")
        L.append("|---|---|---|---|")
        for (s, e, x, y) in stalls:
            L.append(f"| {s:.1f} | {e:.1f} | {e-s:.1f} | ({x:.2f}, {y:.2f}) |")
        L.append("")
    if data["events"]:
        L.append("## 事件标注\n")
        for (t, txt) in data["events"]:
            L.append(f"- `{t-t0:7.1f}s`  {txt}")
        L.append("")
    L.append("## 自动结论 / 可疑点\n")
    sus = []
    if stalls:
        sus.append(f"- 共 {len(stalls)} 段卡住、合计 {stall_secs:.1f} s：多为『指令有速度但实测不动』，"
                   f"可在 02_speed.png 核对，在 01_traj.png 看卡住点位置。")
    if lp and np.nanmax(err) > 0.3:
        sus.append(f"- 横向跟踪误差最大 {np.nanmax(err):.2f} m：局部轨迹与实走偏差较大"
                   f"（看 01_traj / 03_track_error）。")
    if frozen_secs > 5:
        sus.append(f"- 冻结合计 {frozen_secs:.1f} s：转向对齐占用了不少时间。")
    if not sus:
        sus.append("- 未发现明显异常。")
    L += sus
    L.append("\n## 图\n")
    for f in sorted(os.listdir(figs)):
        L.append(f"![{f}](figs/{f})")
    open(os.path.join(out, "report.md"), "w").write("\n".join(L) + "\n")

    print(f"[✓] 报告：{os.path.join(out,'report.md')}")
    print(f"[✓] 图：  {figs}/")
    print(f"    时长 {dur:.1f}s  里程 {dist:.1f}m  卡住 {len(stalls)} 段/{stall_secs:.1f}s")


if __name__ == "__main__":
    main()
