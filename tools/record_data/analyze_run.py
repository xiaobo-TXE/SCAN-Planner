#!/usr/bin/env python3
# ============================================================================
# analyze_run.py —— 离线分析一次真机录制，产出 4 张图 + 报告
#
#   fig1_localization.png : 定位测试（MAP 系 vs WORLD 系，用 /tf 的 map→world 对齐后比偏差）
#   fig2_planning.png     : 全局轨迹 vs 局部轨迹 vs 实际轨迹（全局已按 TF 变到 WORLD 系）
#   fig3_speed.png        : 计划速度 vs 指令速度 vs 实测速度
#   fig4_lanes.png        : 事件泳道（冻结 / 卡住 / 事件标注）
#
# 用法（先 source ROS；解析 Bspline 等自定义类型需要工作空间）：
#   source /opt/ros/humble/setup.bash
#   source ~/scan_planner_github_ws/install/setup.bash
#   python3 analyze_run.py ~/bags/<包>
#   python3 analyze_run.py <包> --odom-topic /loc/robo/odom     # 换"实际轨迹"来源
#   python3 analyze_run.py <包> --map-frame map --world-frame world
#
# 坐标系（真机实测）：
#   WORLD : /lio/robo/odom、/local_path、/planning/bspline   （规划器/控制器实际用）
#   MAP   : /loc/robo/odom、/global_path
#   /tf 里有 map → world 的变换，用来把 MAP 系的东西对齐到 WORLD 系
# ============================================================================
import argparse
import os
import sys
from collections import deque

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_tools import bspline_eval, open_reader, resolve_type, validate_bspline  # noqa: E402


def setup_font():
    """挑一个可用的中文字体，避免图上中文变方框"""
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
# TF 工具
# ---------------------------------------------------------------------------
def quat_to_R(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def make_T(tr, q):
    M = np.eye(4)
    M[:3, :3] = quat_to_R(q)
    M[:3, 3] = tr
    return M


def apply_T(Tm, pts):
    return pts @ Tm[:3, :3].T + Tm[:3, 3]


class TfIndex:
    """按 (parent,child) 存边、按时间最近取；支持多跳查找 src→dst"""

    def __init__(self, entries):
        self.edges = {}
        for (t, parent, child, tr, q) in entries:
            self.edges.setdefault((parent, child), []).append((t, make_T(tr, q)))
        for k in self.edges:
            self.edges[k].sort(key=lambda x: x[0])
        # 预先算好每条边的时间数组（否则每次查找都要重建，几十万条会很慢）
        self.times = {k: np.array([x[0] for x in v]) for k, v in self.edges.items()}
        self.adj = {}
        for (p, c) in self.edges:
            self.adj.setdefault(p, set()).add(c)
            self.adj.setdefault(c, set()).add(p)

    def _edge(self, a, b, t):
        """返回 T_b_a：把 a 系的点变到 b 系（a、b 直接相邻）"""
        arr = self.edges.get((a, b))
        if arr is not None:                     # 消息里存的是 T_a_b → 取逆
            i = max(0, min(int(np.searchsorted(self.times[(a, b)], t)), len(arr) - 1))
            return np.linalg.inv(arr[i][1])
        arr = self.edges.get((b, a))
        if arr is not None:                     # 消息里存的就是 T_b_a
            i = max(0, min(int(np.searchsorted(self.times[(b, a)], t)), len(arr) - 1))
            return arr[i][1]
        return None

    def lookup(self, src, dst, t):
        """返回 T_dst_src（把 src 系点变到 dst 系）；找不到返回 None"""
        if src == dst:
            return np.eye(4)
        prev = {src: None}
        dq = deque([src])
        while dq:
            u = dq.popleft()
            if u == dst:
                break
            for v in self.adj.get(u, ()):
                if v not in prev:
                    prev[v] = u
                    dq.append(v)
        if dst not in prev:
            return None
        chain, u = [], dst
        while prev[u] is not None:
            chain.append((prev[u], u))
            u = prev[u]
        chain.reverse()
        T = np.eye(4)
        for (x, y) in chain:
            M = self._edge(x, y, t)
            if M is None:
                return None
            T = M @ T
        return T


# ---------------------------------------------------------------------------
# 读包
# ---------------------------------------------------------------------------
def collect(bag, odom_topic=None):
    reader, dm, backend = open_reader(bag)
    if backend != "rosbag2_py":
        print("[错误] 目前只支持 rosbag2_py 后端（请 source ROS）")
        sys.exit(1)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    have = set(type_map)
    data = {k: [] for k in ("odom", "odom_map", "cmd", "local_path", "global_path",
                            "bspline", "frozen", "events", "status", "tf")}
    names = {
        "odom": [odom_topic] if odom_topic else ["/lio/robo/odom", "/loc/robo/odom", "/loc/odom"],
        "odom_map": ["/loc/robo/odom", "/loc/odom"],
        "cmd": ["/cmd_vel", "/nav_cmd"],
        "local_path": ["/local_path"],
        "global_path": ["/global_path"],
        "bspline": ["/planning/bspline"],
        "frozen": ["/planning/go2_execution_frozen"],
        "events": ["/test_event"],
        "status": ["/loc/status"],
        "tf": ["/tf"],
    }
    pick = {}
    for k, cands in names.items():
        for c in cands:
            if c in have:
                pick[k] = c
                break
    print("[i] 使用话题：" + ", ".join(f"{k}={v}" for k, v in pick.items() if v))
    want = [v for v in pick.values() if v]
    from rosbag2_py import StorageFilter
    reader.set_filter(StorageFilter(topics=want))
    cls_cache = {}
    while True:
        try:
            topic, raw, ts = reader.read_next()
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
        store(data, key, ts * 1e-9, dm(raw, cls))
    return data, pick, type_map


def store(d, key, t, msg):
    if key in ("odom", "odom_map"):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        d[key].append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    elif key == "cmd":
        d[key].append((t, msg.linear.x, msg.linear.y, msg.angular.z))
    elif key in ("local_path", "global_path"):
        pts = np.array([[ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
                        for ps in msg.poses])
        if pts.size:
            d[key].append((t, pts, msg.header.frame_id or "map"))
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
    elif key == "tf":
        for tr in msg.transforms:
            q = tr.transform.rotation
            v = tr.transform.translation
            d[key].append((t, tr.header.frame_id, tr.child_frame_id,
                           (v.x, v.y, v.z), (q.x, q.y, q.z, q.w)))


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--out", default=None)
    ap.add_argument("--stuck-cmd-eps", type=float, default=0.15)
    ap.add_argument("--stuck-speed-eps", type=float, default=0.03)
    ap.add_argument("--stuck-time", type=float, default=1.0)
    ap.add_argument("--window", type=float, default=0.5)
    ap.add_argument("--odom-topic", default=None, help="实际轨迹来源（默认 /lio/robo/odom）")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--world-frame", default="world")
    a = ap.parse_args()
    if setup_font() is None:
        print("[警告] 没找到中文字体，图中的中文可能显示为方框")

    out = a.out or os.path.join(a.bag, "analysis")
    figs = os.path.join(out, "figs")
    os.makedirs(figs, exist_ok=True)

    print(f"[i] 读取 {a.bag}")
    if not validate_bspline():
        print("[警告] B 样条求值自检未通过")
    data, pick, type_map = collect(a.bag, a.odom_topic)
    if not data["odom"]:
        print("[错误] 没读到里程计。可用话题：")
        for k, v in sorted(type_map.items()):
            print(f"    {v}  {k}")
        sys.exit(2)

    od = np.array(data["odom"])
    t0 = od[0, 0]
    T = od[:, 0] - t0
    P = od[:, 1:4]
    dur = float(T[-1])
    dist = float(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1).sum())
    dt_med = float(np.median(np.diff(od[:, 0]))) if len(od) > 2 else 0.01

    # 实测速度（位置差分窗口）
    v_odom = np.zeros(len(od))
    for i in range(len(od)):
        j = min(np.searchsorted(od[:, 0], od[i, 0] - a.window), i)
        dt = max(od[i, 0] - od[j, 0], 1e-3)
        v_odom[i] = float(np.linalg.norm(P[i, :2] - P[j, :2]) / dt)

    # 指令速度
    cmd = data["cmd"]
    v_cmd = np.zeros(len(od))
    if cmd:
        ca = np.array(cmd)
        for i in range(len(od)):
            k = np.searchsorted(ca[:, 0], od[i, 0]) - 1
            if k >= 0:
                v_cmd[i] = max(abs(ca[k, 1]), abs(ca[k, 2]))

    # 卡住段
    stalls, i = [], 0
    while i < len(od):
        if v_cmd[i] > a.stuck_cmd_eps and v_odom[i] < a.stuck_speed_eps:
            j = i
            while j + 1 < len(od) and v_cmd[j + 1] > a.stuck_cmd_eps and v_odom[j + 1] < a.stuck_speed_eps:
                j += 1
            if od[j, 0] - od[i, 0] >= a.stuck_time:
                stalls.append((od[i, 0] - t0, od[j, 0] - t0, od[i, 1], od[i, 2]))
            i = j + 1
        else:
            i += 1

    # 冻结
    frozen_secs = 0.0
    ft = fv = None
    fz = data["frozen"]
    if fz:
        ft = np.array([x[0] for x in fz])
        fv = np.array([x[1] for x in fz])
        for k in range(len(ft) - 1):
            if fv[k]:
                frozen_secs += ft[k + 1] - ft[k]

    # 横向跟踪误差（降采样后插值）
    lp = data["local_path"]
    err = np.full(len(od), np.nan)
    if lp:
        lt = np.array([x[0] for x in lp])
        step = max(1, int(0.1 / dt_med))
        for i in range(0, len(od), step):
            k = np.searchsorted(lt, od[i, 0] + 1e-9) - 1
            if k < 0:
                continue
            err[i] = float(np.min(np.linalg.norm(lp[k][1][:, :2] - P[i, :2], axis=1)))
        good = ~np.isnan(err)
        if good.sum() > 2:
            err = np.interp(np.arange(len(err)), np.where(good)[0], err[good])

    # 计划速度（在当帧 bspline 上找离机器人最近点，取那里的速度）
    bsp = data["bspline"]
    v_plan = np.full(len(od), np.nan)
    if bsp:
        bt = np.array([x[0] for x in bsp])
        step = max(1, int(0.2 / dt_med))
        for i in range(0, len(od), step):
            k = np.searchsorted(bt, od[i, 0] + 1e-9) - 1
            if k < 0 or od[i, 0] - bt[k] > 3.0:
                continue
            _, pts, kn, order = bsp[k]
            ts = np.linspace(kn[order], kn[len(pts)], 40)
            xy = np.array([bspline_eval(list(kn), list(pts), order, x) for x in ts])
            j = int(np.argmin(np.linalg.norm(xy[:, :2] - P[i, :2], axis=1)))
            j1, j2 = max(j - 1, 0), min(j + 1, len(ts) - 1)
            v_plan[i] = float(np.linalg.norm(xy[j2] - xy[j1]) / max(ts[j2] - ts[j1], 1e-6))
        good = ~np.isnan(v_plan)
        if good.sum() > 2:
            v_plan = np.interp(np.arange(len(v_plan)), np.where(good)[0], v_plan[good])

    # TF：map → world
    tf = TfIndex(data["tf"]) if data["tf"] else None
    od_map = np.array(data["odom_map"]) if data["odom_map"] else None
    dev_t, dev_v, corr_v = [], [], []
    map_in_world = []
    if tf is not None and od_map is not None and len(od_map):
        for row in od_map:
            Tm = tf.lookup(a.map_frame, a.world_frame, row[0])
            if Tm is None:
                continue
            pw = apply_T(Tm, np.array([[row[1], row[2], row[3]]]))[0]
            map_in_world.append(pw)
            j = max(0, min(int(np.searchsorted(od[:, 0], row[0])), len(od) - 1))
            dev_t.append(row[0] - t0)
            dev_v.append(float(np.linalg.norm(pw[:2] - P[j, :2])))
            corr_v.append(float(np.linalg.norm(Tm[:3, 3][:2])))
    map_in_world = np.array(map_in_world) if map_in_world else None

    # 状态
    st_t, st_v, st_extra = [], [], {}
    for (t, m) in data["status"]:
        st_t.append(t - t0)
        st_v.append(getattr(m, "state", np.nan))
        for f in ("rmse", "inlier_ratio"):
            if hasattr(m, f):
                st_extra.setdefault(f, []).append((t - t0, getattr(m, f)))

    # =============================== 图1：定位 ===============================
    nrow = 2 + (1 if st_t else 0) + (1 if "rmse" in st_extra else 0)
    fig, axs = plt.subplots(nrow, 1, figsize=(12, 4.0 * nrow), squeeze=False)
    ax = axs[0][0]
    ax.plot(P[:, 0], P[:, 1], lw=2.5, color="#1565c0", label="WORLD 系 /lio/robo/odom")
    if map_in_world is not None:
        ax.plot(map_in_world[:, 0], map_in_world[:, 1], lw=1.5, color="#e65100",
                label="MAP 系 /loc/robo/odom（经 TF 变到 world）")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=9)
    ax.set_title("图1a  定位轨迹对比（统一到 WORLD 系）")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")

    ax = axs[1][0]
    if dev_t:
        ax.plot(dev_t, dev_v, lw=1.0, color="#6a1b9a", label="两定位源偏差（TF 对齐后）")
        ax.plot(dev_t, corr_v, lw=1.0, color="#00897b", label="TF 修正量（map→world 平移）")
        ax.legend(fontsize=9)
    else:
        ax.text(0.5, 0.5, "缺少 /tf 或 /loc/robo/odom，无法做对齐对比",
                ha="center", va="center", transform=ax.transAxes)
    ax.set_xlabel("t (s)"); ax.set_ylabel("m"); ax.grid(alpha=0.3)
    ax.set_title("图1b  定位偏差与 TF 修正量随时间")

    r = 2
    if st_t:
        ax = axs[r][0]
        ax.step(st_t, st_v, where="post", color="#00695c")
        ax.set_ylabel("state"); ax.grid(alpha=0.3)
        ax.set_title("图1c  /loc/status 定位状态（3=跟踪中）"); ax.set_xlabel("t (s)")
        r += 1
    if "rmse" in st_extra:
        arr = st_extra["rmse"]
        ax = axs[r][0]
        ax.plot([p[0] for p in arr], [p[1] for p in arr], lw=0.9, color="#8d6e63")
        ax.set_ylabel("rmse"); ax.grid(alpha=0.3)
        ax.set_title("图1d  定位 rmse"); ax.set_xlabel("t (s)")
    fig.savefig(os.path.join(figs, "fig1_localization.png"), dpi=110, bbox_inches="tight")
    plt.close(fig)

    # =============================== 图2：规划 ===============================
    fig, ax = plt.subplots(figsize=(10, 10))
    for (t, pts, frame) in data["global_path"]:
        Tm = tf.lookup(frame, a.world_frame, t) if tf is not None else None
        q = apply_T(Tm, pts) if Tm is not None else pts
        ax.plot(q[:, 0], q[:, 1], "-", lw=1.4, color="#9e9e9e", alpha=0.8)
    if data["global_path"]:
        ax.plot([], [], "-", color="#9e9e9e", label="全局轨迹（已变到 world）")
    for (t, pts, _f) in data["local_path"][-40:]:
        ax.plot(pts[:, 0], pts[:, 1], "-", lw=1.0, color="#2e7d32", alpha=0.5)
    if data["local_path"]:
        ax.plot([], [], "-", color="#2e7d32", label="局部轨迹（后 40 条）")
    ax.plot(P[:, 0], P[:, 1], "-", lw=3, color="#1565c0", label="实际轨迹（odom）", zorder=5)
    if stalls:
        ax.plot([s[2] for s in stalls], [s[3] for s in stalls], "x", ms=9, mew=3,
                color="#d32f2f", label="卡住点", zorder=6)
    ax.plot(P[0, 0], P[0, 1], "o", ms=10, color="#000", label="起点")
    ax.plot(P[-1, 0], P[-1, 1], "s", ms=10, color="#000", label="终点")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=9)
    ax.set_title("图2  全局 / 局部 / 实际 轨迹对比（统一到 WORLD 系）")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    fig.savefig(os.path.join(figs, "fig2_planning.png"), dpi=110, bbox_inches="tight")
    plt.close(fig)

    # =============================== 图3：速度 ===============================
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(T, v_plan, lw=1.2, color="#2e7d32", label="计划速度（bspline）")
    ax.plot(T, v_cmd, lw=1.0, color="#c62828", label="指令速度（cmd_vel）")
    ax.plot(T, v_odom, lw=1.2, color="#1565c0", label="实测速度（odom）")
    for (s, e, x, y) in stalls:
        ax.axvspan(s, e, color="#ffcdd2", alpha=0.5,
                   label="卡住" if s == stalls[0][0] else None)
    ax.set_xlabel("t (s)"); ax.set_ylabel("m/s"); ax.grid(alpha=0.3); ax.legend(fontsize=9)
    ax.set_title("图3  速度跟踪：计划 / 指令 / 实测")
    fig.savefig(os.path.join(figs, "fig3_speed.png"), dpi=110, bbox_inches="tight")
    plt.close(fig)

    # =============================== 图4：泳道 ===============================
    fig, ax = plt.subplots(figsize=(12, 3.6))
    lanes = [("卡住", [(s, e) for (s, e, _, _) in stalls], "#d32f2f")]
    if fz:
        segs, cur, start = [], False, None
        for k in range(len(ft)):
            if fv[k] and not cur:
                cur, start = True, ft[k] - t0
            elif not fv[k] and cur:
                cur = False
                segs.append((start, ft[k] - t0))
        if cur:
            segs.append((start, T[-1]))
        lanes.append(("冻结(frozen)", segs, "#1976d2"))
    for li, (_nm, segs, col) in enumerate(lanes):
        for (s, e) in segs:
            ax.barh(li, max(e - s, 0.05), left=s, height=0.5, color=col, alpha=0.8)
    n_ev = 0
    for (t, txt) in data["events"]:
        x = t - t0
        auto = txt.startswith("AUTO:")
        ax.axvline(x, color=("#00897b" if auto else "#f9a825"), lw=0.8, alpha=0.7)
        if not auto:
            ax.text(x, len(lanes) - 0.4, txt[:20], rotation=90, fontsize=6, va="top")
        n_ev += 1
    ax.set_yticks(range(len(lanes)))
    ax.set_yticklabels([l[0] for l in lanes])
    ax.set_ylim(-0.6, len(lanes) + 0.6); ax.set_xlabel("t (s)")
    ax.set_title(f"图4  事件泳道（竖线=事件标注 {n_ev} 条，黄=手动、绿=自动）")
    ax.grid(alpha=0.3, axis="x")
    fig.savefig(os.path.join(figs, "fig4_lanes.png"), dpi=110, bbox_inches="tight")
    plt.close(fig)

    # =============================== 报告 ===============================
    stall_secs = sum(e - s for (s, e, _, _) in stalls)
    bad_state = sum(1 for v in st_v if v != 3) if st_t else 0
    L = ["# 录制分析报告", "",
         f"- bag：`{a.bag}`",
         f"- 时长：**{dur:.1f} s**　里程：**{dist:.1f} m**　平均速度：**{dist/max(dur,1e-6):.2f} m/s**",
         f"- 采样：odom {len(od)}，cmd {len(cmd)}，local_path {len(lp)}，"
         f"global_path {len(data['global_path'])}，bspline {len(bsp)}，tf {len(data['tf'])}", "",
         "## 关键指标", "", "| 指标 | 值 |", "|---|---|",
         f"| 卡住段数 | {len(stalls)} |",
         f"| 卡住总时长 | {stall_secs:.1f} s（占 {100*stall_secs/max(dur,1e-6):.1f}%） |",
         f"| 冻结(frozen)总时长 | {frozen_secs:.1f} s |",
         f"| 计划 vs 实测速度 平均差 | {np.nanmean(np.abs(v_plan-v_odom)):.3f} m/s |",
         f"| 指令 vs 实测速度 平均差 | {float(np.mean(np.abs(v_cmd-v_odom))):.3f} m/s |",
         f"| 横向跟踪误差 均值/最大 | {np.nanmean(err):.3f} / {np.nanmax(err):.3f} m |",
         f"| 定位 state≠3 采样数 | {bad_state} / {len(st_v)} |"]
    if dev_v:
        L += [f"| 定位偏差(MAP 经 TF 对齐) 均值/最大 | {np.mean(dev_v):.3f} / {np.max(dev_v):.3f} m |",
              f"| TF 修正量 均值/最大 | {np.mean(corr_v):.3f} / {np.max(corr_v):.3f} m |"]
    L.append("")
    if stalls:
        L += ["## 卡住段（前 15 段）", "", "| 起(s) | 止(s) | 时长(s) | 位置(x,y) |", "|---|---|---|---|"]
        for (s, e, x, y) in stalls[:15]:
            L.append(f"| {s:.1f} | {e:.1f} | {e-s:.1f} | ({x:.2f}, {y:.2f}) |")
        if len(stalls) > 15:
            L.append(f"| … | … | 共 {len(stalls)} 段 | |")
        L.append("")
    sus = []
    if stalls:
        sus.append(f"- 卡住 {len(stalls)} 段 / {stall_secs:.1f} s：见图3（指令高、实测低）与图4（红色段）。")
    if not np.isnan(np.nanmax(err)) and np.nanmax(err) > 0.3:
        sus.append(f"- 横向跟踪误差最大 {np.nanmax(err):.2f} m：见图2。")
    if frozen_secs > 5:
        sus.append(f"- 冻结合计 {frozen_secs:.1f} s：原地转向对齐占用较多。")
    if dev_v and np.max(dev_v) > 0.5:
        sus.append(f"- 两定位源偏差最大 {np.max(dev_v):.2f} m：定位一致性/时延需要关注（图1b）。")
    if bad_state:
        sus.append(f"- 定位 state≠3 有 {bad_state} 次采样：定位短时不可信。")
    if not sus:
        sus.append("- 未发现明显异常。")
    L += ["## 自动结论 / 可疑点", ""] + sus + ["",
          "## 图", "",
          "![图1 定位](figs/fig1_localization.png)",
          "![图2 规划](figs/fig2_planning.png)",
          "![图3 速度](figs/fig3_speed.png)",
          "![图4 事件](figs/fig4_lanes.png)", ""]
    open(os.path.join(out, "report.md"), "w").write("\n".join(L) + "\n")

    print(f"[✓] 报告：{os.path.join(out, 'report.md')}")
    print(f"[✓] 图：  {figs}/  (fig1_localization / fig2_planning / fig3_speed / fig4_lanes)")
    print(f"    时长 {dur:.1f}s  里程 {dist:.1f}m  卡住 {len(stalls)} 段/{stall_secs:.1f}s")


if __name__ == "__main__":
    main()
