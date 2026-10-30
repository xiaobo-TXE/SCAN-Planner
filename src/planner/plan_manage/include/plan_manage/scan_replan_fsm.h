// ============================================================================
// 文件：include/plan_manage/scan_replan_fsm.h
//
// 【这个文件是干什么的】
//   定义“状态机”类 SCANReplanFSM。状态机 = 程序的大脑：
//   它不停判断“我现在处于哪个阶段”，然后做那个阶段该做的事。
//
// 【六个阶段（状态）怎么流转】
//   INIT          启动后等待，直到收到“里程计”和“目标点”
//        ↓
//   WAIT_TARGET   等着用户/程序给一个目标点
//        ↓
//   GEN_NEW_TRAJ  第一次生成一条新轨迹
//        ↓
//   EXEC_TRAJ     正在沿轨迹走；每走一小段就检查要不要重新规划
//        ↓ ↑
//   REPLAN_TRAJ   重新规划（比如偏离了、前面有障碍了）
//        ↓
//   EMERGENCY_STOP 紧急停止（前面有障碍、来不及重规划时）
//
// 【两个定时器（心脏跳动）】
//   exec_timer_   每 10 毫秒跳一次 → execFSMCallback()，驱动状态机
//   safety_timer_ 每 50 毫秒跳一次 → checkCollisionCallback()，检查碰撞
//
// 【三种“目标怎么来”的模式 NAVI_MODE】
//   1 在 RViz 里用鼠标点一个目标点
//   2 用参数文件里预先写好的航点序列
//   3 订阅外部给的一条全局路径
//
// 【阅读提醒】
//   本文件只是“声明”，所有函数实现都在 src/scan_replan_fsm.cpp，
//   两边函数名一一对应。
// ============================================================================
#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  class SCANReplanFSM
  {

  private:
    // 状态机的六个状态（下面各自的中文含义见上）
    // enum = 枚举：把一组有名字的整数常量放在一起。
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      // 刚启动：等里程计和触发信号
      INIT,
      // 等待目标点
      WAIT_TARGET,
      // 生成一条全新的轨迹
      GEN_NEW_TRAJ,
      // 重新规划（在已有轨迹基础上重算）
      REPLAN_TRAJ,
      // 正在沿轨迹执行
      EXEC_TRAJ,
      // 紧急停止
      EMERGENCY_STOP
    };
    // 三种“目标来源”模式。= 1/2/3 与参数 fsm.navi_mode 对应。
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFERENCE_PATH = 3,
    };

    // ---------- 规划用的工具对象 ----------
    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    scan_planner_msgs::msg::DataDisp data_disp_;

    // ---------- 参数（从 YAML 文件读进来） ----------
    /* parameters */
    int navi_mode_; // 1 manual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    std::vector<Eigen::Vector3d> preset_waypoints_;
    int waypoint_num_;
    double planning_horizon_;
    double emergency_time_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    std::string self_inflation_frame_id_;
    std::string reference_frame_id_;
    double reference_tf_max_age_{0.3}, reference_tf_grace_{0.5};
    double reference_tf_jump_translation_{0.2}, reference_tf_jump_yaw_{5.0 * 3.14159265358979323846 / 180.0};

    // ---------- 运行状态(running/goal_reached)判定参数 ----------
    // 到达目标的 2D 水平距离容差(米)
    double status_goal_tol_{0.15};
    // "速度为零"阈值(米/秒)
    double status_speed_tol_{0.05};
    // 到达且停稳持续多久才判为 goal_reached(秒)
    double status_stop_debounce_{1.0};
    // 心跳频率(Hz)：周期性重发当前状态；0 表示关闭
    double status_heartbeat_hz_{1.0};
    // 判定"目标变了"的位置阈值(米)：新目标点与上一次相差超过它才算新目标
    double status_goal_change_eps_{0.05};

    // ---------- 运行时数据（程序自己维护的状态） ----------
    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool preset_started_{false};
    bool rviz_height_ready_;
    bool go2_execution_frozen_;
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int max_replan_fail_count_{1000};
    rclcpp::Time last_freeze_update_time_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    Eigen::Vector3d reference_end_pt_{Eigen::Vector3d::Zero()};
    Eigen::Isometry3d planning_reference_to_world_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d last_reference_to_world_{Eigen::Isometry3d::Identity()};
    bool have_last_reference_transform_{false};
    bool reference_tf_stop_{false};
    bool reference_tf_jump_failed_{false};
    bool reference_jump_replan_{false};
    rclcpp::Time reference_tf_missing_since_{0, 0, RCL_ROS_TIME};
    std::vector<Eigen::Vector3d> active_waypoints_;
    int current_wp_;

    // ---------- 运行状态发布(running/goal_reached) ----------
    // 是否收到过目标：没收到过属于 idle，不上话题
    bool status_have_goal_{false};
    // 当前是否已判为 goal_reached（锁存，直到收到新目标）
    bool status_is_stop_{false};
    // 是否已有"上一次目标点"，用于重复路径去重
    bool status_goal_valid_{false};
    // 上一次用于判定的目标点（参考系/map）
    Eigen::Vector3d status_goal_ref_{Eigen::Vector3d::Zero()};
    // "到达且停稳"起始时刻，用于去抖
    rclcpp::Time status_hold_since_{0, 0, RCL_ROS_TIME};
    // 上一次发布状态的时刻，用于心跳
    rclcpp::Time status_last_pub_time_{0, 0, RCL_ROS_TIME};

    // 外部任务控制：/mission/control 是否处于 pause（停止）
    bool externally_stopped_{false};
    // 最近一条可用的全局路径（/global_path）；resume 时用它重建
    nav_msgs::msg::Path::ConstSharedPtr last_global_path_;

    bool flag_escape_emergency_;

    /* ROS utils */
    rclcpp::Node *node_{nullptr};
    // TF 缓冲与监听器：用来查询“参考坐标系 → world”的变换（navi_mode=3 用）
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // 两个定时器：exec_timer_ 每 10ms 驱动状态机；safety_timer_ 每 50ms 查碰撞
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;
    // 任务控制：/mission/control（pause 停 / resume 继续）
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_control_sub_;
    rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<scan_planner_msgs::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;
    // 运行状态话题：只发 running / goal_reached（idle 不发）
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    // ---------- 辅助函数（内部使用） ----------
    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    void setStartStateFromOdomOrCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    // 切换状态（例如从 GEN_NEW_TRAJ 切到 EXEC_TRAJ），并打印日志
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool planNextWaypoint();
    bool isWaypointSequenceMode() const;
    bool adjustGlobalTargetIfOccupied();
    // 从全局轨迹上取“前方 planning_horizon 米处”的点，作为局部目标
    bool getLocalTarget();
    // 查询“参考坐标系 → world”的变换；查不到（超时等）时返回 false
    bool lookupReferenceTransform(Eigen::Isometry3d &reference_to_world);
    // 监控参考 TF 的新鲜度与跳变；返回 true 表示当前应停止推进
    bool monitorReferenceTransform();
    void finishProcess();
    // 发布机器人自身外形（前后两个圆柱），在 RViz 里看得到
    void publishSelfInflationMarker();
    void publishLocalPath(const UniformBspline &trajectory);
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();
    // 收到新目标时调用：进入 running 并清掉 goal_reached/去抖状态
    void armPlannerStatus();
    // 每个 tick 评估是否"到达目标且停稳"，并用心跳周期重发
    void updatePlannerStatus();
    // 向 /local_planner/status 发布状态（空串不发布）
    void publishPlannerStatus(const std::string &state);
    // 任务控制回调：pause / resume
    void missionControlCallback(const std_msgs::msg::String::ConstSharedPtr &msg);
    // 外部停止：清全局+局部轨迹，状态置 stopped
    void applyExternalStop();
    // 外部继续：用缓存路径重建，或等待新的 /global_path
    void applyExternalResume();

    // ---------- ROS 回调函数（收到消息/定时器到点时被自动调用） ----------
    /* ROS functions */
    // 【核心】10ms 定时器回调：状态机的主循环
    void execFSMCallback();
    // 50ms 定时器回调：检查轨迹前段是否会撞到障碍
    void checkCollisionCallback();
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);

    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM()
    {
    }

    // 唯一对外入口：做全部初始化（读参数、订阅话题、建定时器）
    void init(rclcpp::Node *node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
