#!/usr/bin/env bash
# ============================================================================
# record_run.sh —— A2 真机全量数据录制（定位 / 建图 / 全局 / 局部 / 控制）
#
# 目的：每次真机运行，一条命令就把"定位 + 建图 + 全局规划 + 局部规划 + 控制"
#       的全部话题录下来，并自动归档、校验、写索引。
#
# 用法（在真机上运行）：
#   bash record_run.sh <标签>                     # 前台录制，Ctrl-C 按一次停止
#   bash record_run.sh <标签> --duration 120      # 录 120s 后自动停
#   bash record_run.sh <标签> --points            # 只录"点云短包"（默认 30s）
#   bash record_run.sh <标签> --points --duration 20
#   bash record_run.sh <标签> --compress          # 开启 zstd file 压缩（省空间；分析前需先解压）
#   bash record_run.sh <标签> --light             # 精简档：去掉两朵大点云（约省 37% 带宽）
#   bash record_run.sh <标签> --slim              # 只录 定位/全局/局部/速度（约 0.1-0.2 MB/s）
#   bash record_run.sh <标签> --tracks            # 轨迹+滑动地图专用档（默认 300s 自动停，≈47 MB/s）
#   bash record_run.sh <标签> --manual            # 不限时，靠 Ctrl-C 停（任何档位都可加）
#   bash record_run.sh <标签> --auto-events       # 自动标注"卡住/恢复"（无需手按）
#   bash record_run.sh --check                    # 只做"录前自检"，不录
#   bash record_run.sh --event "goal A"           # 往 /test_event 发一条事件标注
#   bash record_run.sh --stop                     # 优雅停止正在进行的录制
#
# 可用环境变量覆盖默认路径：
#   NAV_WS=$HOME/nav_ws
#   LOCAL_WS=$HOME/scan_planner_github_ws
#   BAG_ROOT=$HOME/bags
#   ROS_DOMAIN_ID=0
# ============================================================================
set -o pipefail   # 注意：不要加 set -u/-e，ROS 的 setup.bash 会因未定义变量报错

NAV_WS="${NAV_WS:-$HOME/nav_ws}"
LOCAL_WS="${LOCAL_WS:-$HOME/scan_planner_github_ws}"
BAG_ROOT="${BAG_ROOT:-$HOME/bags}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
QOS_FILE="$BAG_ROOT/qos_overrides.yaml"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# --auto-events 用的话题（真机不同就覆盖）
AUTO_CMD_TOPIC="${AUTO_CMD_TOPIC:-/cmd_vel}"
AUTO_ODOM_TOPIC="${AUTO_ODOM_TOPIC:-/loc/robo/odom}"

# ---------------------------------------------------------------------------
# 话题清单（按 A2 真机话题名；真机恢复后请用 `ros2 topic list` 核一遍再定稿）
# ---------------------------------------------------------------------------
DIAG_TOPICS=(
  # --- 基础 ---
  /tf /tf_static /rosout
  # --- 定位 ---
  /loc/robo/odom /loc/odom /loc/odom_raw /loc/status /loc/map /loc/keyframes /loc/submap
  /loc/imu/odom /loc/imu/robo/odom
  /lio/robo/odom /lio/imu/odom /lio/odom /lio/path
  # --- 建图 ---
  /lio/cloud_world
  /grid_map/occupancy /grid_map/occupancy_inflate
  /grid_map/sliding_map_bbox /grid_map/depth_cloud
  # --- 全局规划 ---
  /global_path /global_planner/raw_map /goal_3d /goal_point
  # --- 局部规划 ---
  /planning/bspline /local_path /planning/data_display /self_inflation /planning/go2_execution_frozen
  # --- 控制 ---
  /cmd_vel /motion_enabled
  # --- 事件标注 ---
  /test_event
)

POINTS_TOPICS=(
  /tf /tf_static
  /loc/robo/odom /loc/status /loc/map /loc/keyframes /loc/submap
  /lio/cloud_world
  /grid_map/occupancy /grid_map/occupancy_inflate
  /self_inflation
  /test_event
)

# --light：去掉两朵"大而不关键"的云（/lio/cloud_world 14.7 MB/s、/grid_map/depth_cloud 13 MB/s，约省 37% 带宽）
LIGHT_TOPICS=(
  /tf /tf_static /rosout
  /loc/robo/odom /loc/status /loc/map /loc/keyframes /loc/submap /lio/robo/odom
  /grid_map/occupancy /grid_map/occupancy_inflate /grid_map/sliding_map_bbox
  /global_path /global_planner/raw_map /goal_3d /goal_point
  /planning/bspline /local_path /planning/data_display /self_inflation
  /planning/go2_execution_frozen /cmd_vel /motion_enabled /test_event
)

# --slim：只录"定位 / 全局 / 局部 / 速度"四类（约 0.1-0.2 MB/s，15 分钟约 100-200 MB）
#   注意 /tf /tf_static 必须有 —— 分析时要把 MAP 系对齐到 WORLD 系
SLIM_TOPICS=(
  # 定位
  /loc/robo/odom /loc/status /lio/robo/odom
  # 全局
  /global_path /goal_3d
  # 局部
  /planning/bspline /local_path
  # 速度
  /cmd_vel /motion_enabled /dog_odom
  # 坐标系（分析对齐必需）
  /tf /tf_static
  # 诊断 / 事件
  /planning/go2_execution_frozen /test_event
)

# --tracks：只看"轨迹 + 滑动地图"的专用档。用途=回放时一眼看清
#   "机器人从起点到拐角怎么走的、拐角处滑动地图判成了什么"。
#   对应关系（详见同目录《轨迹与滑动地图回放手册.md》）：
#     全局轨迹   /global_path          （MAP 系）
#     局部轨迹   /local_path           （WORLD 系）
#     实时轨迹   /lio/robo/odom        （WORLD 系，实际走过的）
#     滑动地图   /grid_map/occupancy + /grid_map/occupancy_inflate + /grid_map/sliding_map_bbox
#   注意带宽：滑动地图是本系统里最贵的两朵 —— 实测 occupancy_inflate 40.9 MB/s、
#   occupancy 5.7 MB/s，合计 ≈47 MB/s（≈2.8 GB/min）。所以本档**默认 300s 自动停**
#   （300s ≈ 14 GB），避免忘记 Ctrl-C 把盘写满；想不限时用 `--manual`（或 `--duration 0`）。
#   另外注意：Ctrl-C 任何时候都能停（自动停只是上限），不用等它到点。
TRACKS_TOPICS=(
  # 坐标系（缺了什么都对不上；MAP 系靠它对齐到 WORLD 系）
  /tf /tf_static
  # 全局轨迹（MAP 系，保留型）
  /global_path
  # 局部轨迹（WORLD 系）+ 原始 B 样条消息（RViz 画不了，留给离线分析）
  /local_path /planning/bspline
  # 实时轨迹 / 位姿（两个定位源都录，回放里可对比）
  /lio/robo/odom /loc/robo/odom /loc/status /lio/path
  # 滑动地图（贵！见上面的带宽说明）
  /grid_map/occupancy /grid_map/occupancy_inflate /grid_map/sliding_map_bbox
  # 机身两圆柱（碰撞模型，保留型）
  /self_inflation
  # 目标点 / 速度 / 事件（都很小）
  /goal_3d /goal_point /cmd_vel /motion_enabled /test_event
)

# ---------------------------------------------------------------------------
log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

source_env() {
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash 2>/dev/null || { log "错误：找不到 /opt/ros/humble/setup.bash"; exit 1; }
  for ws in "$NAV_WS" "$LOCAL_WS"; do
    if [ -f "$ws/install/setup.bash" ]; then
      # shellcheck disable=SC1090
      source "$ws/install/setup.bash"
      log "已 source：$ws"
    else
      log "警告：找不到 $ws/install/setup.bash（跳过）"
    fi
  done
  log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
}

write_qos() {
  mkdir -p "$BAG_ROOT"
  if [ -s "$QOS_FILE" ]; then
    log "沿用已有 QoS：$QOS_FILE"
  elif [ -f "$SCRIPT_DIR/qos_overrides.yaml" ]; then
    cp "$SCRIPT_DIR/qos_overrides.yaml" "$QOS_FILE"
    log "已从脚本目录复制 QoS -> $QOS_FILE"
  else
    cat > "$QOS_FILE" <<'YAML'
/loc/map: {reliability: reliable, durability: transient_local, history: keep_last, depth: 1}
/loc/keyframes: {reliability: reliable, durability: transient_local, history: keep_last, depth: 1}
/global_path: {reliability: reliable, durability: transient_local, history: keep_last, depth: 1}
/global_planner/raw_map: {reliability: reliable, durability: transient_local, history: keep_last, depth: 1}
/global_planner/map: {reliability: reliable, durability: transient_local, history: keep_last, depth: 1}
/motion_enabled: {reliability: reliable, durability: transient_local, history: keep_last, depth: 1}
/self_inflation: {reliability: reliable, durability: transient_local, history: keep_last, depth: 1}
/grid_map/occupancy: {reliability: best_effort, durability: volatile, history: keep_last, depth: 5}
/grid_map/occupancy_inflate: {reliability: best_effort, durability: volatile, history: keep_last, depth: 5}
/grid_map/sliding_map_bbox: {reliability: best_effort, durability: volatile, history: keep_last, depth: 5}
/grid_map/depth_cloud: {reliability: best_effort, durability: volatile, history: keep_last, depth: 5}
YAML
    log "已生成 QoS：$QOS_FILE"
  fi
}

# 录前自检：话题存在性 + 关键话题频率 + 定位状态
precheck() {
  log "==================== 录前自检 ===================="
  local all_topics; all_topics="$(ros2 topic list 2>/dev/null)"
  if [ -z "$all_topics" ]; then
    log "错误：ros2 topic list 为空 —— 环境/网络没通，先别录。"; return 1
  fi
  local missing=()
  for t in "${DIAG_TOPICS[@]}"; do
    printf '%s\n' "$all_topics" | grep -qx "$t" || missing+=("$t")
  done
  if [ ${#missing[@]} -eq 0 ]; then
    log "清单话题全部存在 ✓（${#DIAG_TOPICS[@]} 个）"
  else
    log "以下话题当前不存在（可能尚未发布 / 名字不同；录包时会 WARN，不影响其它话题）："
    printf '   %s\n' "${missing[@]}"
  fi
  log "---- 关键话题频率 ----"
  for t in /loc/status /loc/robo/odom /lio/cloud_world /planning/bspline /cmd_vel; do
    printf '%s\n' "$all_topics" | grep -qx "$t" || continue
    local rate
    rate="$(timeout 3 ros2 topic hz "$t" 2>&1 | grep -m1 'average rate')"
    printf '  %-24s %s\n' "$t" "${rate:-（3s 内无数据）}"
  done
  if printf '%s\n' "$all_topics" | grep -qx /loc/status; then
    printf '  /loc/status state  '
    timeout 3 ros2 topic echo /loc/status --once 2>/dev/null | grep -m1 -E 'state' || echo '(读不到)'
    log "  → 期望 state: 3（正在跟踪）；不是 3 说明定位还没进入跟踪"
  fi
  log "=================================================="
}

# 录后：解析 bag，列出缺失/为 0 的话题，写 meta.yaml 与 INDEX.md，收拢附加文件
postprocess() {
  local bag="$1"; shift
  local expected=("$@")
  ros2 bag info "$bag" > "$bag/bag_info.txt" 2>&1 || true

  # 解析 "Topic: /x | Type: ... | Count: N"
  local pairs; pairs="$(grep -oP 'Topic: \S+ \| Type: \S+ \| Count: \d+' "$bag/bag_info.txt" \
      | sed -E 's/Topic: (\S+) \| Type: \S+ \| Count: ([0-9]+).*/\1 \2/')"

  local recorded=0; local zero=()
  log "==================== 录制结果 ===================="
  for t in "${expected[@]}"; do
    local c; c="$(printf '%s\n' "$pairs" | awk -v k="$t" '$1==k{print $2}')"
    if [ -n "$c" ] && [ "$c" -gt 0 ]; then
      recorded=$((recorded+1)); printf '  %-34s %s\n' "$t" "$c"
    else
      zero+=("$t"); printf '  %-34s %s\n' "$t" "!! 0/缺失"
    fi
  done
  log "记录到 $recorded / ${#expected[@]} 个话题"
  [ ${#zero[@]} -gt 0 ] && { log "缺失/为 0 的话题："; printf '   %s\n' "${zero[@]}"; }

  # meta.yaml
  cat > "$bag/meta.yaml" <<META
label: $(basename "$bag")
date: $(date -Is)
host: $(hostname)
ros_domain_id: $ROS_DOMAIN_ID
topics_expected: ${#expected[@]}
topics_recorded: $recorded
missing_topics: [$(printf '%s, ' "${zero[@]}" | sed 's/, $//')]
META
  log "已写 $bag/meta.yaml"

  # INDEX.md 追加
  local idx="$BAG_ROOT/INDEX.md"
  [ -f "$idx" ] || printf '# 录制索引\n\n| 时间 | 名称 | 话题(录到/期望) | 大小 | 备注 |\n|---|---|---|---|---|\n' > "$idx"
  printf '| %s | %s | %s/%s | %s | %s |\n' \
    "$(date '+%F %T')" "$(basename "$bag")" "$recorded" "${#expected[@]}" \
    "$(du -sh "$bag" 2>/dev/null | cut -f1)" "${LABEL_NOTE:-}" >> "$idx"
  log "已追加 $idx"

  # 附加文件一起归档（有就收，没有就跳过）
  for f in "$BAG_ROOT/loc_tum.txt" "$BAG_ROOT/match.csv"; do
    [ -f "$f" ] && cp "$f" "$bag/" && log "  收拢 $(basename "$f")"
  done
  if [ -d "$HOME/.ros/log/maploc" ]; then
    cp -r "$HOME/.ros/log/maploc" "$bag/logs_maploc" && log "  收拢 ~/.ros/log/maploc"
  fi
  log "完成：$bag"
}

# ---------------------------------------------------------------------------
# 参数解析
# ---------------------------------------------------------------------------
MODE="record"; LABEL=""; DURATION=""; STORAGE="sqlite3"; COMPRESS=0; POINTS=0; LIGHT=0; SLIM=0; TRACKS=0; MANUAL=0; EVENT_MSG=""; AUTO_EVENTS=0
while [ $# -gt 0 ]; do
  case "$1" in
    --check)       MODE="check" ;;
    --event)       MODE="event"; EVENT_MSG="${2:-}"; shift ;;
    --points)      POINTS=1; MODE="record" ;;
    --duration)    DURATION="${2:-}"; shift ;;
    --storage)     STORAGE="${2:-sqlite3}"; shift ;;
    --no-compress) COMPRESS=0 ;;
    --compress)    COMPRESS=1 ;;
    --light)       LIGHT=1 ;;
    --slim)        SLIM=1 ;;
    --tracks)      TRACKS=1 ;;
    --manual)      MANUAL=1 ;;
    --auto-events) AUTO_EVENTS=1 ;;
    --stop)        MODE="stop" ;;
    -h|--help)     sed -n '2,28p' "$0"; exit 0 ;;
    -*)            echo "未知参数：$1"; exit 1 ;;
    *)             LABEL="$1" ;;
  esac
  shift
done

case "$MODE" in
  check)
    source_env; write_qos; precheck; exit 0 ;;
  stop)
    # 优雅停止：发 SIGINT 让脚本走完"finalize + 后处理"；排除自己
    n=0
    for p in $(pgrep -f "record_run.sh"); do
      [ "$p" = "$$" ] && continue
      kill -INT "$p" 2>/dev/null && n=$((n + 1))
    done
    log "已向 $n 个录制进程发送停止信号（等它 finalize 完再归档）"
    exit 0 ;;
  event)
    if [ -z "$EVENT_MSG" ]; then echo "用法：bash record_run.sh --event \"事件描述\""; exit 1; fi
    source_env
    ros2 topic pub --once /test_event std_msgs/msg/String "{data: '$EVENT_MSG'}" && log "事件已发出：$EVENT_MSG"
    exit 0 ;;
esac

# 录制模式：先校验必须参数，再动目录/环境（避免报用法时就把 ~/bags 建出来）
[ -n "$LABEL" ] || {
  echo "用法：bash record_run.sh <标签> [--duration N] [--manual] [--tracks] [--points] [--slim] [--light] [--no-compress] [--storage mcap]"
  echo "      bash record_run.sh --check | --event \"事件描述\""
  exit 1
}

source_env
write_qos

if [ "$POINTS" = 1 ]; then
  TOPICS=("${POINTS_TOPICS[@]}")
  [ -n "$DURATION" ] || DURATION=30
  LABEL_NOTE="点云短包"
elif [ "$TRACKS" = 1 ]; then
  TOPICS=("${TRACKS_TOPICS[@]}")
  # 这个档只有滑动地图贵（≈47 MB/s）：默认 300s（≈14 GB）自动停，防忘记停写满盘。
  # 不限时就加 --manual。注意 Ctrl-C 任何时候都能停，自动停只是上限。
  [ -n "$DURATION" ] || DURATION=300
  LABEL_NOTE="轨迹+滑动地图档"
elif [ "$SLIM" = 1 ]; then
  TOPICS=("${SLIM_TOPICS[@]}")
  LABEL_NOTE="精简档(定位/规划/速度)"
elif [ "$LIGHT" = 1 ]; then
  TOPICS=("${LIGHT_TOPICS[@]}")
  LABEL_NOTE="精简包"
else
  TOPICS=("${DIAG_TOPICS[@]}")
  LABEL_NOTE="诊断包"
fi

# --manual（与 --duration 0 等价）：不限时，靠 Ctrl-C 停。
# 放在档位选择之后 → 覆盖档位给的默认时长。
if [ "$MANUAL" = 1 ] || [ "$DURATION" = "0" ]; then DURATION=""; fi

precheck || exit 1

BAG_PATH="$BAG_ROOT/${LABEL}_$(date +%Y%m%d_%H%M%S)"
[ -e "$BAG_PATH" ] && BAG_PATH="${BAG_PATH}_$$"

REC_ARGS=(--qos-profile-overrides-path "$QOS_FILE" --storage "$STORAGE")
[ "$COMPRESS" = 1 ] && REC_ARGS+=(--compression-mode file --compression-format zstd)

log "开始录制 → $BAG_PATH"
log "话题数：${#TOPICS[@]}  storage=$STORAGE  zstd压缩=$COMPRESS  时长=${DURATION:-手动(Ctrl-C)}"
if [ "$COMPRESS" = 1 ]; then
  log "提示：已开 file 压缩 —— rosbag2_py 读不了，分析前需先解压（zstd -d + 改 metadata 的 compression_mode）"
fi
if [ "$AUTO_EVENTS" = 1 ]; then
  log "自动事件标注：开（cmd=$AUTO_CMD_TOPIC odom=$AUTO_ODOM_TOPIC）"
else
  log "事件标注：另开终端执行  bash $0 --event \"goal A\"（或加 --auto-events 自动标注）"
fi
log "---------------------------------------------------------"

# 启动"自动卡住/恢复"标注（订阅 cmd_vel + odom，命中判据就往 /test_event 发标注）
AUTOPID=""
if [ "$AUTO_EVENTS" = 1 ]; then
  if [ -f "$SCRIPT_DIR/auto_event_pub.py" ]; then
    python3 "$SCRIPT_DIR/auto_event_pub.py" --cmd-topic "$AUTO_CMD_TOPIC" --odom-topic "$AUTO_ODOM_TOPIC" \
      > "$BAG_ROOT/auto_events.log" 2>&1 &
    AUTOPID=$!
  else
    log "警告：找不到 $SCRIPT_DIR/auto_event_pub.py，--auto-events 忽略"
  fi
fi

ros2 bag record "${TOPICS[@]}" -o "$BAG_PATH" "${REC_ARGS[@]}" &
BAGPID=$!

# Ctrl-C 转发给 record：只停一次，让它自己 finalize
cleanup() { log "收到停止信号，正在结束录制（等待 Bag finalized）..."; kill -INT "$BAGPID" 2>/dev/null; }
trap cleanup INT TERM

if [ -n "$DURATION" ]; then
  ( sleep "$DURATION"; log "到达 ${DURATION}s，自动停止"; kill -INT "$BAGPID" 2>/dev/null ) &
  TIMER=$!
fi

# 注意：收到信号时 bash 的 wait 会被打断并返回（130），而 rosbag2 可能还在给
# 几十 GB 的包做 zstd 收尾 —— 必须循环等到它真的退出，否则后处理会读到没写完的包。
RC=0
while kill -0 "$BAGPID" 2>/dev/null; do wait "$BAGPID"; RC=$?; done
[ -n "${TIMER:-}" ] && kill "$TIMER" 2>/dev/null
if [ -n "${AUTOPID:-}" ]; then
  kill -INT "$AUTOPID" 2>/dev/null; wait "$AUTOPID" 2>/dev/null
  log "自动事件标注已停止（日志：$BAG_ROOT/auto_events.log）"
fi
if [ "$RC" -ne 0 ]; then
  log "ros2 bag record 退出码 $RC。若提示 zstd/压缩不支持，请加 --no-compress 重试。"
fi

postprocess "$BAG_PATH" "${TOPICS[@]}"
