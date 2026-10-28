#!/usr/bin/env python3
# ============================================================================
# analyze_run.py —— 离线分析一次真机录制，产出 4 张图 + 报告
#
#   1_定位轨迹.png    : 定位轨迹（WORLD vs MAP，经 TF 对齐）
#   2_定位偏差.png    : 两定位源偏差 + TF 修正量随时间
#   3_定位状态.png    : /loc/status 定位状态
#   4_定位rmse.png    : 定位 rmse（匹配残差）
#   5_轨迹对比.png    : 全局 / 局部 / 实际 轨迹（全局已按 TF 变到 WORLD）
#   6_速度对比.png    : 计划 / 指令 / 实测 速度
#   7_事件时间轴.png  : 冻结 / 卡住 / 事件标注
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
import matplotlib.patheffects as pe

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
# 配色 / 线型（高对比：不同色系 + 实线/虚线，重叠时也能分辨）
# ---------------------------------------------------------------------------
C_ACTUAL = "#0b3d91"   # 实际（odom / 实测速度）：深蓝
C_MAP = "#e65100"      # 地图定位：橙（虚线，压在实际轨迹上仍可见）
C_GLOBAL = "#7b1fa2"   # 全局轨迹：紫（虚线）
C_LOCAL = "#2e7d32"    # 局部轨迹 / 计划速度：绿
C_CMD = "#c62828"      # 指令速度：红
C_DEV = "#6a1b9a"      # 定位偏差：紫
C_CORR = "#00897b"     # TF 修正量 / 定位状态：青
C_STALL = "#b71c1c"    # 卡住
C_FROZEN = "#0d47a1"   # 冻结
LW_THICK, LW_MID, LW_THIN = 2.8, 2.2, 1.4
DPI = 140


def style_ax(ax, title=None, xlabel=None, ylabel=None):
    ax.grid(True, ls="--", alpha=0.35)
    ax.set_axisbelow(True)
    if title:
        ax.set_title(title, fontsize=14, fontweight="bold")
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=11)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=11)
    ax.tick_params(labelsize=10)


def halo(lw):
    """白色描边：让叠在别的线上的曲线仍然清晰"""
    return [pe.Stroke(linewidth=lw + 2.5, foreground="white", alpha=0.9), pe.Normal()]


def corner_legend(ax, fontsize=8.0):
    """把图例缩小并放到四个角里"最空"的那个，尽量不压曲线"""
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    W, H = (x1 - x0), (y1 - y0)
    zones = {
        "upper left":  (x0, x0 + 0.42 * W, y0 + 0.66 * H, y1),
        "upper right": (x0 + 0.58 * W, x1, y0 + 0.66 * H, y1),
        "lower left":  (x0, x0 + 0.42 * W, y0, y0 + 0.34 * H),
        "lower right": (x0 + 0.58 * W, x1, y0, y0 + 0.34 * H),
    }
    counts = {}
    for name, (a, b, c, d) in zones.items():
        cnt = 0
        for ln in ax.get_lines():
            xd, yd = ln.get_data()
            if len(xd) == 0:
                continue
            cnt += int(np.sum((xd >= a) & (xd <= b) & (yd >= c) & (yd <= d)))
        counts[name] = cnt
    best = min(counts, key=counts.get)
    # markerscale 把"图标"也按比例缩小（默认和曲线上的标记一样大）；handle* 缩线段/色块
    ax.legend(loc=best, fontsize=fontsize, framealpha=0.92, edgecolor="#bbbbbb",
              borderpad=0.22, labelspacing=0.2, borderaxespad=0.35,
              handlelength=1.1, handleheight=0.55, handletextpad=0.35,
              markerscale=0.42)


def save(fig, path):
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    print("  -> " + os.path.basename(path))


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

    # 机体速度：实测(vx,vy,wz) —— 世界系位移差旋转到机体系；yaw 差求 wz
    yaw = np.unwrap(np.arctan2(2 * (od[:, 7] * od[:, 6] + od[:, 4] * od[:, 5]),
                               1 - 2 * (od[:, 5] ** 2 + od[:, 6] ** 2)))
    vx_a = np.zeros(len(od)); vy_a = np.zeros(len(od)); wz_a = np.zeros(len(od))
    for i in range(len(od)):
        j = min(np.searchsorted(od[:, 0], od[i, 0] - a.window), i)
        dt = max(od[i, 0] - od[j, 0], 1e-3)
        dx = P[i, 0] - P[j, 0]; dy = P[i, 1] - P[j, 1]
        c = np.cos(yaw[i]); sn = np.sin(yaw[i])
        vx_a[i] = (c * dx + sn * dy) / dt
        vy_a[i] = (-sn * dx + c * dy) / dt
        dyaw = (yaw[i] - yaw[j] + np.pi) % (2 * np.pi) - np.pi
        wz_a[i] = dyaw / dt

    # 下发速度（cmd_vel，机体系）
    cmd = data["cmd"]
    v_cmd = np.zeros(len(od))
    vx_c = np.zeros(len(od)); vy_c = np.zeros(len(od)); wz_c = np.zeros(len(od))
    if cmd:
        ca = np.array(cmd)
        for i in range(len(od)):
            k = np.searchsorted(ca[:, 0], od[i, 0]) - 1
            if k >= 0:
                vx_c[i], vy_c[i], wz_c[i] = ca[k, 1], ca[k, 2], ca[k, 3]
                v_cmd[i] = float(np.hypot(ca[k, 1], ca[k, 2]))

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

    # =============================== 图1a：定位轨迹 ===============================
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(P[:, 0], P[:, 1], "-", lw=LW_THICK + 1.2, color=C_ACTUAL, alpha=0.45,
            label="WORLD 轨迹(odom)", zorder=2)
    if map_in_world is not None:
        ax.plot(map_in_world[:, 0], map_in_world[:, 1], "--", lw=LW_MID, color=C_MAP,
                label="MAP 轨迹(TF对齐)", zorder=5,
                path_effects=halo(LW_MID))
    ax.plot(P[0, 0], P[0, 1], "o", ms=12, color="k", label="起点", zorder=6)
    ax.plot(P[-1, 0], P[-1, 1], "s", ms=12, color="k", label="终点", zorder=6)
    ax.set_aspect("equal")
    style_ax(ax, "图1  定位轨迹对比（统一到 WORLD 系）", "x (m)", "y (m)")
    corner_legend(ax)
    save(fig, os.path.join(figs, "1_定位轨迹.png"))

    # =============================== 图1b：偏差 / TF 修正量 ===============================
    fig, ax = plt.subplots(figsize=(12, 4.5))
    if dev_t:
        ax.plot(dev_t, dev_v, "-", lw=LW_MID, color=C_DEV, label="两源偏差(TF对齐后)")
        ax.plot(dev_t, corr_v, "--", lw=LW_MID, color=C_CORR, label="TF 修正量")
        corner_legend(ax)
    else:
        ax.text(0.5, 0.5, "缺少 /tf 或 /loc/robo/odom，无法做对齐对比",
                ha="center", va="center", transform=ax.transAxes, fontsize=12)
    style_ax(ax, "图2  定位偏差与 TF 修正量随时间", "t (s)", "m")
    save(fig, os.path.join(figs, "2_定位偏差.png"))

    # =============================== 图1c：定位状态 ===============================
    if st_t:
        fig, ax = plt.subplots(figsize=(12, 3.8))
        ax.step(st_t, st_v, where="post", lw=LW_MID, color=C_CORR)
        ax.set_ylim(min(1.5, min(st_v) - 0.3), 3.4)
        style_ax(ax, "图3  定位状态 /loc/status（3 = 正在跟踪）", "t (s)", "state")
        save(fig, os.path.join(figs, "3_定位状态.png"))

    # =============================== 图1d：定位 rmse ===============================
    if "rmse" in st_extra:
        arr = st_extra["rmse"]
        fig, ax = plt.subplots(figsize=(12, 3.8))
        ax.plot([p[0] for p in arr], [p[1] for p in arr], "-", lw=LW_THIN, color=C_DEV)
        style_ax(ax, "图4  定位 rmse（匹配残差）", "t (s)", "rmse")
        save(fig, os.path.join(figs, "4_定位rmse.png"))

    # =============================== 图5：规划（俯视 + 高度） ===============================
    # 全局轨迹先按 TF 变到 WORLD（画图和取高度都用它）
    gp_world = []
    for (t, pts, frame) in data["global_path"]:
        Tm = tf.lookup(frame, a.world_frame, t) if tf is not None else None
        gp_world.append((t, apply_T(Tm, pts) if Tm is not None else pts, frame))

    def nearest_z(paths, step_s=0.2, max_age=3.0):
        """每一时刻，取"当帧路径上离机器人最近点"的 z"""
        out = np.full(len(od), np.nan)
        if not paths:
            return out
        pt = np.array([x[0] for x in paths])
        step = max(1, int(step_s / dt_med))
        for i in range(0, len(od), step):
            k = np.searchsorted(pt, od[i, 0] + 1e-9) - 1
            if k < 0 or od[i, 0] - pt[k] > max_age:
                continue
            pts = paths[k][1]
            j = int(np.argmin(np.linalg.norm(pts[:, :2] - P[i, :2], axis=1)))
            out[i] = pts[j, 2]
        good = ~np.isnan(out)
        if good.sum() > 2:
            out = np.interp(np.arange(len(out)), np.where(good)[0], out[good])
        return out

    z_loc = nearest_z(data["local_path"])
    z_glo = nearest_z(gp_world)

    fig, (ax, axz) = plt.subplots(2, 1, figsize=(12, 12),
                                  gridspec_kw={"height_ratios": [2.1, 1.0]})
    # ---------- 上：俯视 x-y ----------
    # 实际轨迹：半透明宽带打底（最底层）
    ax.plot(P[:, 0], P[:, 1], "-", lw=LW_THICK + 1.6, color=C_ACTUAL, alpha=0.40,
            label="实际轨迹(odom带)", zorder=1)
    # 局部轨迹：全部亮绿显示（不抽样），画在全局下面
    nlp = len(data["local_path"])
    for (_t, pts, _f) in data["local_path"]:
        ax.plot(pts[:, 0], pts[:, 1], "-", lw=1.2, color=C_LOCAL, alpha=0.70, zorder=4)
    if nlp:
        ax.plot([], [], "-", lw=1.2, color=C_LOCAL, alpha=0.85, label=f"局部轨迹(全部 {nlp} 条)")
    # 全局轨迹：全部画出来（都可见），最后一条稍加粗、带白描边
    ng = len(gp_world)
    for k, (_t, q, _f) in enumerate(gp_world):
        is_last = (k == ng - 1)
        ax.plot(q[:, 0], q[:, 1], "--",
                lw=(LW_MID + 0.6) if is_last else LW_MID,
                color=C_GLOBAL,
                alpha=0.95 if is_last else 0.70,
                zorder=7 if is_last else 5,
                path_effects=halo(LW_MID) if is_last else None)
    if ng:
        ax.plot([], [], "--", lw=LW_MID + 0.6, color=C_GLOBAL, label=f"全局轨迹(全部 {ng} 条，末条加粗)")
    if stalls:
        ax.plot([q[2] for q in stalls], [q[3] for q in stalls], "X", ms=11, mew=2.5,
                color=C_STALL, label="卡住点", zorder=7)
    ax.plot(P[0, 0], P[0, 1], "o", ms=13, color="k", label="起点", zorder=8)
    ax.plot(P[-1, 0], P[-1, 1], "s", ms=13, color="k", label="终点", zorder=8)
    ax.set_aspect("equal")
    style_ax(ax, "图5a  三条轨迹对比（俯视，统一到 WORLD 系）", "x (m)", "y (m)")
    corner_legend(ax)

    # ---------- 下：高度 z 随时间 ----------
    for (qs, qe, _x, _y) in stalls:
        axz.axvspan(qs, qe, color="#ffcdd2", alpha=0.45, lw=0)
    axz.plot(T, P[:, 2], "-", lw=LW_THICK, color=C_ACTUAL, label="实际高度 z(odom)", zorder=3)
    if not np.all(np.isnan(z_glo)):
        axz.plot(T, z_glo, "--", lw=LW_MID, color=C_GLOBAL, label="全局规划高度 z", zorder=4,
                 path_effects=halo(LW_MID))
    if not np.all(np.isnan(z_loc)):
        axz.plot(T, z_loc, "-", lw=LW_THIN + 0.4, color=C_LOCAL, label="局部规划高度 z", zorder=5,
                 path_effects=halo(LW_THIN))
    style_ax(axz, "图5b  高度 z 随时间（实际 / 局部规划 / 全局规划）", "t (s)", "z (m)")
    corner_legend(axz)
    save(fig, os.path.join(figs, "5_轨迹对比.png"))

    # =============================== 图6：机体速度 3 分量对比 ===============================
    fig, axs = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    spec = [
        (vx_a, vx_c, "vx (m/s)", "图6a  前进速度 vx：下发 vs 实际"),
        (vy_a, vy_c, "vy (m/s)", "图6b  横移速度 vy：下发 vs 实际"),
        (wz_a, wz_c, "wz (rad/s)", "图6c  转向角速度 wz：下发 vs 实际"),
    ]
    for k, (ya, yc, ylab, ttl) in enumerate(spec):
        ax2 = axs[k]
        for (qs, qe, _x, _y) in stalls:
            ax2.axvspan(qs, qe, color="#ffcdd2", alpha=0.45, lw=0)
        ax2.axhline(0.0, color="#9e9e9e", lw=0.8, ls="-")
        ax2.plot(T, ya, "-", lw=LW_THICK + 1.0, color=C_ACTUAL, alpha=0.55,
                 label="实际（odom）", zorder=3)
        ax2.plot(T, yc, "-", lw=LW_MID, color=C_CMD, label="下发（cmd_vel）",
                 zorder=4, path_effects=halo(LW_MID))
        style_ax(ax2, ttl, "t (s)" if k == len(spec) - 1 else None, ylab)
        if k == 0:
            corner_legend(ax2)
    save(fig, os.path.join(figs, "6_速度对比.png"))

    # =============================== 图4：泳道 ===============================
    fig, ax = plt.subplots(figsize=(12, 3.6))
    lanes = [("卡住", [(s, e) for (s, e, _, _) in stalls], C_STALL)]
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
        lanes.append(("冻结(frozen)", segs, C_FROZEN))
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
    ax.set_title(f"图7  事件泳道（竖线=事件标注 {n_ev} 条，黄=手动、绿=自动）", fontsize=14, fontweight="bold")
    ax.grid(True, axis="x", ls="--", alpha=0.35)
    save(fig, os.path.join(figs, "7_事件时间轴.png"))

    # =============================== 报告 ===============================
    stall_secs = sum(e - s for (s, e, _, _) in stalls)
    bad_state = sum(1 for v in st_v if v != 3) if st_t else 0
    L = ["# 录制分析报告", "",
         f"- bag：`{a.bag}`",
         f"- 时长：**{dur:.1f} s**　里程：**{dist:.1f} m**　平均速度：**{dist/max(dur,1e-6):.2f} m/s**",
         f"- 采样：odom {len(od)}，cmd {len(cmd)}，local_path {len(lp)}，"
         f"global_path {len(data['global_path'])}，bspline {len(data['bspline'])}，tf {len(data['tf'])}", "",
         "## 关键指标", "", "| 指标 | 值 |", "|---|---|",
         f"| 卡住段数 | {len(stalls)} |",
         f"| 卡住总时长 | {stall_secs:.1f} s（占 {100*stall_secs/max(dur,1e-6):.1f}%） |",
         f"| 冻结(frozen)总时长 | {frozen_secs:.1f} s |",
         f"| vx 下发 vs 实际 平均差 | {float(np.mean(np.abs(vx_c-vx_a))):.3f} m/s |",
         f"| vy 下发 vs 实际 平均差 | {float(np.mean(np.abs(vy_c-vy_a))):.3f} m/s |",
         f"| wz 下发 vs 实际 平均差 | {float(np.mean(np.abs(wz_c-wz_a))):.3f} rad/s |",
         f"| 合速度 下发 vs 实际 平均差 | {float(np.mean(np.abs(v_cmd-v_odom))):.3f} m/s |",
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
          "![1 定位轨迹](figs/1_定位轨迹.png)",
          "![2 定位偏差](figs/2_定位偏差.png)",
          "![3 定位状态](figs/3_定位状态.png)",
          "![4 定位rmse](figs/4_定位rmse.png)",
          "![5 轨迹对比](figs/5_轨迹对比.png)",
          "![6 速度对比](figs/6_速度对比.png)",
          "![7 事件时间轴](figs/7_事件时间轴.png)", ""]
    open(os.path.join(out, "report.md"), "w").write("\n".join(L) + "\n")

    print(f"[✓] 报告：{os.path.join(out, 'report.md')}")
    print(f"[✓] 图：  {figs}/  (fig1_localization / fig2_planning / fig3_speed / fig4_lanes)")
    print(f"    时长 {dur:.1f}s  里程 {dist:.1f}m  卡住 {len(stalls)} 段/{stall_secs:.1f}s")


if __name__ == "__main__":
    main()
