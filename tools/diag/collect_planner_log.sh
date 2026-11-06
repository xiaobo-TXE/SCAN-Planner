#!/usr/bin/env bash
# ============================================================================
# 文件：tools/diag/collect_planner_log.sh
#
# 一条命令搞定「跑一次 → 收集 → 自动统计局部规划失败原因」：
#   1) 检查分支/编译（可跳过）
#   2) 起你自己的 launch（默认 run.launch.py robot:=a2），同时把终端输出写进日志
#   3) 你正常操作机器狗、复现规划报错
#   4) 按一次 Ctrl-C → 自动统计并生成 summary.md（把这个文件内容发出去即可）
#
# 统计对象：所有形如
#     [规划失败] 原因=<中文原因> <上下文…>
# 的诊断行（由本分支的临时诊断打点产生；覆盖 A* / 优化 / 细化 / 动力学 / TF /
# 局部目标 / 全局轨迹 / 急停 / 连续失败等全部失败原因）。
#
# 用法（真机）：
#   bash collect_planner_log.sh
#   bash collect_planner_log.sh --cmd "ros2 launch scan_planner run.launch.py robot:=a2 navi_mode:=3"
#   bash collect_planner_log.sh --no-build              # 已经编译过、跳过编译
#   bash collect_planner_log.sh --log ~/my_run.log      # 指定日志文件
#   bash collect_planner_log.sh --ws ~/scan_planner_github_ws
#   bash collect_planner_log.sh --analyze-only --log ~/run_old.log   # 只分析已有日志
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
    -h|--help) sed -n '2,28p' "$0"; exit 0 ;;
    *) echo "未知参数：$1"; exit 1 ;;
  esac
  shift
done

[ -n "$LOGFILE" ] || LOGFILE="$HOME/planner_run_$(date +%Y%m%d_%H%M%S).log"
OUTDIR="${LOGFILE%.log}_summary"
mkdir -p "$OUTDIR"

log() { echo "[$(date +%H:%M:%S)] $*"; }

# ---------------------------------------------------------------------------
# 统计与判定
# ---------------------------------------------------------------------------
MARK="规划失败] 原因="        # 诊断行标记（前面有 "["，这里不写以免转义）

analyze() {
  local logf="$1" out="$2"
  local total reasons_tsv
  total=$(grep -c "$MARK" "$logf" 2>/dev/null || true); total=${total:-0}

  {
    echo "# 局部规划失败原因诊断报告"
    echo
    echo "- 日志文件：\`$logf\`"
    echo "- 生成时间：$(date '+%Y-%m-%d %H:%M:%S')"
    echo "- 主机：$(hostname)"
    echo
    echo "## 失败原因统计（共 $total 条诊断）"
    echo
    if [ "$total" -eq 0 ]; then
      echo "**本次日志里没有 \`[规划失败]\` 诊断行。**"
      echo
      echo "可能原因："
      echo "1. 这次没复现失败；"
      echo "2. 诊断没生效（没编译，或跑的不是 \`diag/planner-fail-logging\` 分支）。"
    else
      echo "| 原因 | 次数 |"
      echo "|---|---|"
      grep -o "$MARK[^ ]*" "$logf" | sed "s/.*原因=//" | sort | uniq -c | sort -rn \
        | awk '{n=$1; $1=""; sub(/^ /,""); printf "| `%s` | %s |\n", $0, n}'
      echo
      echo "## 时间线（前 40 条，含时间戳的原始行）"
      echo
      echo '```'
      grep -n "$MARK" "$logf" | head -40
      echo '```'
      echo
      echo "## 原始样本（最后 10 条）"
      echo
      echo '```'
      grep "$MARK" "$logf" | tail -10
      echo '```'
    fi
  } > "$out/summary.md"

  echo
  echo "==================== 汇总 ===================="
  cat "$out/summary.md"
  echo "=============================================="
  echo "报告：$out/summary.md"
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
  diag/planner-fail-logging*) : ;;
  *) log "⚠ 当前不是 diag/planner-fail-logging* 分支——如果没看到 [规划失败]，先确认分支/编译" ;;
esac

source /opt/ros/humble/setup.bash 2>/dev/null
[ -f install/setup.bash ] && source install/setup.bash

if [ "$DO_BUILD" = 1 ]; then
  log "编译 bspline_opt + scan_planner（可用 --no-build 跳过）…"
  colcon build --packages-select bspline_opt scan_planner path_searching \
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

wait "$LPID" 2>/dev/null
kill "$TPID" 2>/dev/null
log "launch 已退出，开始统计…"
analyze "$LOGFILE" "$OUTDIR"
