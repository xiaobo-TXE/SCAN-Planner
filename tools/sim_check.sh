#!/usr/bin/env bash
# ============================================================================
# 文件：tools/sim_check.sh —— 三机型仿真自检
#
# 逐个机型拉起仿真栈（is_real_world:=false navi_mode:=1 robot:=<机型>），
# 然后用 tools/sim_check_probe.py 检查 README「仿真自检清单」的四项：
#   ① 模型显示（/robot_description）   ② TF world->base
#   ③ 跟随 /quad_0/cmd_vel             ④ 12 个腿关节 + /self_inflation 双圆柱
#
# 用法：
#   tools/sim_check.sh              # 三个机型都跑（a2 go2 go2w）
#   tools/sim_check.sh a2           # 只跑一个
# 日志：log/sim_check/<机型>.log
# ============================================================================
# 注意：不要开 set -u —— ROS 的 setup.bash 会引用未定义变量而中断

ROBOTS=("$@")
[ ${#ROBOTS[@]} -eq 0 ] && ROBOTS=(a2 go2 go2w)

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${TOOLS_DIR}/.." && pwd)"
LOGDIR="${WS}/log/sim_check"
mkdir -p "${LOGDIR}"

source /opt/ros/humble/setup.bash
# 本工作区 + 依赖工作区（local_sensing / mockamap 等未必在本工作区里）
for s in "${WS}/install/setup.bash" "$HOME/nav_ws/install/setup.bash"; do
  [ -f "$s" ] && source "$s"
done

LAUNCH_PAT="[r]os2 launch scan_planner run.launch.py"
stop_stack() {
  pkill -INT -f "${LAUNCH_PAT}" 2>/dev/null
  sleep 4
  pkill -9 -f "${LAUNCH_PAT}" 2>/dev/null
  sleep 2
}
trap 'stop_stack; exit 130' INT TERM

fail_total=0
for robot in "${ROBOTS[@]}"; do
  echo "=================================================================="
  echo "机型 ${robot}：拉起仿真栈…"
  LOG="${LOGDIR}/${robot}.log"
  ros2 launch scan_planner run.launch.py \
      is_real_world:=false navi_mode:=1 robot:="${robot}" >"${LOG}" 2>&1 &

  ready=0
  for _ in $(seq 1 45); do
    if ros2 topic list 2>/dev/null | grep -q '^/quad_0/body_pose$'; then ready=1; break; fi
    sleep 1
  done
  if [ "${ready}" -eq 0 ]; then
    echo "  !! 45 秒内仿真栈未就绪，日志尾部："
    tail -6 "${LOG}" | sed 's/^/     /'
    stop_stack
    fail_total=$((fail_total + 1))
    continue
  fi
  echo "  仿真栈就绪，开始检查"
  if ! python3 "${TOOLS_DIR}/sim_check_probe.py" "${robot}"; then
    fail_total=$((fail_total + 1))
  fi
  stop_stack
done

echo "=================================================================="
if [ "${fail_total}" -eq 0 ]; then
  echo "全部机型自检通过 ✅"
else
  echo "有 ${fail_total} 个机型存在失败项 ❌（日志见 ${LOGDIR}/）"
fi
exit "${fail_total}"
