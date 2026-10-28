# =============================================================================
# env.sh —— 在真机的每个终端里 source 一次即可
#   source ~/record_tools/env.sh
# 作用：source ROS + 两个工作空间，固定 ROS_DOMAIN_ID
# =============================================================================
source /opt/ros/humble/setup.bash
source "$HOME/nav_ws/install/setup.bash" 2>/dev/null
source "$HOME/scan_planner_github_ws/install/setup.bash" 2>/dev/null
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
echo "[env] 已就绪  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  工作空间已 source"
