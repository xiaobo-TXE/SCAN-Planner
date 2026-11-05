#!/usr/bin/env bash
# ============================================================================
# 文件：tools/diag/collect_astar_log.sh
#
# 一条命令搞定「跑一次 → 收集 → 自动统计 A* 失败原因」：
#   1) 检查分支/编译（可跳过）
#   2) 起你自己的 launch（默认 run.launch.py robot:=a2），同时把终端输出写进日志
#   3) 你正常操作机器狗、复现 A* 报错
#   4) 按一次 Ctrl-C → 自动统计并生成 summary.md（把这个文件内容发出去即可）
#
# 用法（真机）：
#   bash collect_astar_log.sh
#   bash collect_astar_log.sh --cmd "ros2 launch scan_planner run.launch.py robot:=a2 navi_mode:=3"
#   bash collect_astar_log.sh --no-build              # 已经编译过、跳过编译
#   bash collect_astar_log.sh --log ~/my_run.log      # 指定日志文件
#   bash collect_astar_log.sh --ws ~/scan_planner_github_ws
#   bash collect_astar_log.sh --analyze-only --log ~/run_old.log   # 只分析已有日志
#
# 环境变量：WS / CMD / LOGFILE 同样可以覆盖。
# ============================================================================
# 注意：不要加 set -u（ROS 的 setup.bash 会引用未定义变量）

WS="${WS:-$HOME/scan_planner_github_ws}"
CMD="${CMD:-ros2 launch scan_planner run.launch.py robot:=a2}"
LOGFILE=""
DO_BUILD=1
ANALYZE_ONLY=0

while [ $# -gt 0 ]; do
  case "$1" in
    --cmd) CMD="$2"; shift ;;
    --ws) WS="$2"; shift ;;
    --log) LOGFILE="$2"; shift ;;
    --no-build) DO_BUILD=0 ;;
    --analyze-only) ANALYZE_ONLY=1 ;;
    -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
    *) echo "未知参数：$1"; exit 1 ;;
  esac
  shift
done

[ -n "$LOGFILE" ] || LOGFILE="$HOME/astar_run_$(date +%Y%m%d_%H%M%S).log"
OUTDIR="${LOGFILE%.log}_summary"
mkdir -p "$OUTDIR"

log() { echo "[$(date +%H:%M:%S)] $*"; }

# ---------------------------------------------------------------------------
# 统计与判定
# ---------------------------------------------------------------------------
analyze() {
  local logf="$1" out="$2"
  local total i1 o1 im om ie np lenbig
  total=$(grep -c "A-star FAIL" "$logf" 2>/dev/null || true); total=${total:-0}
  i1=$(grep -o "occ_in=1\b"   "$logf" 2>/dev/null | wc -l)
  o1=$(grep -o "occ_out=1\b"  "$logf" 2>/dev/null | wc -l)
  im=$(grep -o "occ_in=-1\b"  "$logf" 2>/dev/null | wc -l)
  om=$(grep -o "occ_out=-1\b" "$logf" 2>/dev/null | wc -l)
  ie=$(grep -o "type=astar_init_err" "$logf" 2>/dev/null | wc -l)
  np=$(grep -o "type=astar_no_path"  "$logf" 2>/dev/null | wc -l)
  lenbig=$(grep -o "len=[0-9.]*m" "$logf" 2>/dev/null | sed 's/len=//;s/m//' \
           | awk '$1>=5.0' | wc -l)

  {
    echo "# A* 失败诊断报告"
    echo
    echo "- 日志文件：\`$logf\`"
    echo "- 生成时间：$(date '+%Y-%m-%d %H:%M:%S')"
    echo "- 主机：$(hostname)"
    echo
    echo "## 统计"
    echo
    echo "| 项 | 数量 |"
    echo "|---|---|"
    echo "| \`[A-star FAIL]\` 总条数 | $total |"
    echo "| \`type=astar_init_err\`（起点/终点问题） | $ie |"
    echo "| \`type=astar_no_path\`（搜空/超时） | $np |"
    echo "| \`occ_in=1\`（起点侧在膨胀区） | $i1 |"
    echo "| \`occ_out=1\`（终点侧在膨胀区） | $o1 |"
    echo "| \`occ_in=-1\`（起点出图） | $im |"
    echo "| \`occ_out=-1\`（终点出图） | $om |"
    echo "| 段长 ≥5 m（超搜索池） | $lenbig |"
    echo
    echo "## 自动判定"
    echo
    if [ "$total" -eq 0 ]; then
      echo "**本次日志里没有 \`[A-star FAIL]\`。**"
      echo
      echo "可能原因："
      echo "1. 这次没复现 A* 失败；"
      echo "2. 诊断日志没生效（没编译 \`bspline_opt\`，或跑的不是 \`diag/astar-logging\` 分支）。"
      echo
      echo "顺带看老日志有没有："
      for k in "Start point outside" "End point outside" "initial or end point" \
               "0.2 second" "search took" "A-star failed; aborting" "A-star error"; do
        printf -- "- \`%s\`: %s\n" "$k" "$(grep -c "$k" "$logf" 2>/dev/null || true)"
      done
    else
      if [ "$i1" -gt "$o1" ] && [ "$i1" -gt 0 ]; then
        echo "**→ 起点侧为主**：机器人自己的控制点落在膨胀区（贴墙/偏离走廊中线）。"
        echo
        echo "建议顺序：① 滑动地图 \`sliding_map_size_x/y\` 6→8 m；② 走线尽量居中；③ 最后才考虑缩 \`double_cylinder_radius\`。"
      elif [ "$o1" -gt "$i1" ]; then
        echo "**→ 终点侧为主**：局部目标（沿全局路径前视 \`planning_horizon\` 处）落在膨胀区。"
        echo
        echo "建议：\`fsm.planning_horizon\` 2.5→2.0；并检查 \`getLocalTarget\` 选出的点。"
      elif [ "$np" -gt 0 ] && [ "$i1" -eq 0 ] && [ "$o1" -eq 0 ]; then
        echo "**→ 拓扑被堵 / 2.5D 平面限制**：两端都不在膨胀区，但 A* 找不到路（或超时）。"
        echo
        echo "建议：看现场是否死角/门框/低矮结构；考虑恢复\"连续失败→清局部栅格+随机初值\"。"
      elif [ "$lenbig" -gt 0 ]; then
        echo "**→ 段长超出搜索池**：出现 ≥5 m 的碰撞段（A* 池只有 5 m）。"
        echo
        echo "建议：查上游是否有大幅度的重规划失败（初始猜测轨迹过长）。"
      elif [ "$im" -gt 0 ] || [ "$om" -gt 0 ]; then
        echo "**→ 控制点出图**：起/终点跑到了地图外。"
        echo
        echo "建议：检查滑动地图尺寸与地图边界。"
      else
        echo "**→ 混合原因**，请看下面的原始样本逐条分析。"
      fi
    fi
    echo
    echo "## 原始样本（前 20 条）"
    echo
    echo '```'
    grep "A-star FAIL" "$logf" 2>/dev/null | head -20
    echo '```'
    echo
    echo "## 前后文（与规划结果相关的 cout）"
    echo
    echo '```'
    grep -E "final_plan_success=|first_optimize_step_success=" "$logf" 2>/dev/null | tail -10
    echo '```'
  } > "$out/summary.md"

  echo
  echo "================ 统计 ================"
  printf "总失败条数            : %s\n" "$total"
  printf "astar_init_err        : %s\n" "$ie"
  printf "astar_no_path         : %s\n" "$np"
  printf "occ_in=1（起点侧）    : %s\n" "$i1"
  printf "occ_out=1（终点侧）   : %s\n" "$o1"
  printf "occ_*= -1（出图）     : %s / %s\n" "$im" "$om"
  printf "段长 ≥5 m             : %s\n" "$lenbig"
  echo "======================================"
  echo "报告：$out/summary.md"
  echo
  echo "前 5 条样本："
  grep "A-star FAIL" "$logf" 2>/dev/null | head -5
  echo
  echo ">>> 把上面这块统计 + 前 5~20 条样本，或直接发 $out/summary.md 的内容，发给助手即可。"
}

if [ "$ANALYZE_ONLY" = 1 ]; then
  [ -f "$LOGFILE" ] || { echo "日志不存在：$LOGFILE"; exit 1; }
  analyze "$LOGFILE" "$OUTDIR"
  exit 0
fi

# ---------------------------------------------------------------------------
# 0) 环境 / 分支 / 编译
# ---------------------------------------------------------------------------
log "工作空间：$WS"
[ -d "$WS" ] || { echo "工作空间不存在：$WS（用 --ws 指定）"; exit 1; }
cd "$WS" || exit 1

BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo '?')"
log "当前分支：$BRANCH"
case "$BRANCH" in
  diag/astar-logging*) : ;;
  *) log "⚠ 当前不是 diag/astar-logging* 分支——如果没看到 [A-star FAIL]，先确认分支/编译" ;;
esac

source /opt/ros/humble/setup.bash 2>/dev/null
[ -f install/setup.bash ] && source install/setup.bash

if [ "$DO_BUILD" = 1 ]; then
  log "编译 bspline_opt + scan_planner（可用 --no-build 跳过）…"
  colcon build --packages-select bspline_opt scan_planner \
    --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release > "$OUTDIR/build.log" 2>&1
  if [ $? -ne 0 ]; then
    echo "编译失败，日志：$OUTDIR/build.log"; tail -15 "$OUTDIR/build.log"; exit 1
  fi
  source install/setup.bash
  log "编译完成"
fi

# ---------------------------------------------------------------------------
# 1) 起栈（输出写日志），2) 等 Ctrl-C
# ---------------------------------------------------------------------------
log "启动：$CMD"
log "日志：$LOGFILE"
log "---------------------------------------------------------"
bash -c "$CMD" > "$LOGFILE" 2>&1 &
LPID=$!
# 实时显示（过滤掉点云类刷屏，只保留日志/规划相关内容）
tail -n +1 -f "$LOGFILE" 2>/dev/null &
TPID=$!

finish() {
  kill "$TPID" 2>/dev/null
  sleep 1
  log "已停止，开始统计…"
  analyze "$LOGFILE" "$OUTDIR"
  exit 0
}
trap finish INT TERM

# 前台等待：Ctrl-C 会触发 trap
wait "$LPID" 2>/dev/null
# 如果 launch 自己退出了（不是 Ctrl-C），也做一次统计
kill "$TPID" 2>/dev/null
log "launch 已退出，开始统计…"
analyze "$LOGFILE" "$OUTDIR"
