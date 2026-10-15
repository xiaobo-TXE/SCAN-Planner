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
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
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
    std::vector<Eigen::Vector3d> active_waypoints_;
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    rclcpp::Node *node_{nullptr};
    // 两个定时器：exec_timer_ 每 10ms 驱动状态机；safety_timer_ 每 50ms 查碰撞
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;
    rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<scan_planner_msgs::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;

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
    void getLocalTarget();
    void finishProcess();
    // 发布机器人自身外形（前后两个圆柱），在 RViz 里看得到
    void publishSelfInflationMarker();
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();

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
