
// ============================================================================
// 文件：src/planner/plan_manage/src/scan_replan_fsm.cpp
//
// 【这个文件是干什么的】
//   状态机 SCANReplanFSM 的全部实现代码（头文件里只是声明）。
//   状态机就是程序的大脑：它每 10 毫秒醒一次，看看当前处于哪个阶段，
//   然后执行那个阶段该做的事。
//
// 【最重要的一句话】
//   想看懂整个规划程序，看懂 execFSMCallback() 和 checkCollisionCallback()
//   这两个函数就够了，其余都是它们的辅助。
//
// 【建议阅读顺序】
//   1) init()                  —— 启动时做了什么准备
//   2) execFSMCallback()       —— 主循环：六个状态各自干什么
//   3) checkCollisionCallback()—— 怎么发现快要撞上
//   4) 其余辅助函数            —— 用到时再查
// ============================================================================
#include <plan_manage/scan_replan_fsm.h>
#include <plan_manage/reference_path_utils.h>
#include <plan_manage/reference_frame_utils.h>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tf2/time.hpp>
#include <tf2/exceptions.hpp>

namespace
{
  // 小工具函数：读取一个 ROS 参数。
  // 如果参数还没声明过就先声明（用给定的默认值），然后返回它的值。
  // 这样可以避免“参数不存在”导致的报错。
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  // ==========================================================================
  // init()：程序启动时执行一次的全部准备工作。
  //   1. 把内部的标志位/计数器清零
  //   2. 从 YAML 参数文件读取所有配置（navi_mode、阈值、膨胀尺寸……）
  //   3. 创建可视化对象和规划总管，并让规划总管去建地图、建优化器
  //   4. 创建两个定时器（10ms 主循环 / 50ms 安全检查）
  //   5. 订阅话题：里程计、底盘冻结；再按 navi_mode 订阅目标点或外部路径
  //   6. 创建发布器：规划出的轨迹、调试数据、机器人外形
  // ==========================================================================
  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    // ---------- 第1步：把所有内部状态重置为初始值 ----------
    node_ = node;
    // TF 缓冲与监听器：navi_mode=3 需要把参考坐标系下的路径换算到 world 系
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // 航点下标从 0 开始；状态机初始状态是 INIT
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    // 这几个布尔量表示“收到过/发生过没有”：触发信号、目标、里程计、新目标
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    // 是否已从首帧里程计记录下目标高度
    rviz_height_ready_ = false;
    // 底盘是否正在原地转向（会暂停轨迹时钟）
    go2_execution_frozen_ = false;
    // 自动恢复开关 / 是否需要悬停停下 / 是否正在从急停恢复
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    // 连续重规划失败计数器清零
    replan_fail_count_ = 0;
    // 记录当前时间，供“冻结”逻辑计算时间差
    last_freeze_update_time_ = node_->now();

    /*  fsm param  */
    // ---------- 第2步：读取全部参数（名字对应 planner.yaml） ----------
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    // 起点与当前位置相差超过这个距离 → 触发重规划
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    // 离终点小于这个距离 → 不必再重规划
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    // 规划视野：每次只规划前方这么远（米）
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    // 预计在这个时间(秒)内会撞上障碍 → 判为紧急
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    // 是否允许自动从急停恢复
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    // 连续规划失败超过这个次数 → 强制急停
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000);
    // 下面几个是机器人外形尺寸（用于画自身膨胀 + 碰撞检测）
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");
    reference_tf_max_age_ = load_parameter<double>(node_, "fsm.reference_tf_max_age", 0.3);
    reference_tf_grace_ = load_parameter<double>(node_, "fsm.reference_tf_grace", 0.5);
    reference_tf_jump_translation_ = load_parameter<double>(node_, "fsm.reference_tf_jump_translation", 0.2);
    reference_tf_jump_yaw_ = load_parameter<double>(node_, "fsm.reference_tf_jump_yaw_deg", 5.0) * M_PI / 180.0;
    if (reference_tf_max_age_ <= 0.0 || reference_tf_grace_ <= 0.0 ||
        reference_tf_jump_translation_ <= 0.0 || reference_tf_jump_yaw_ <= 0.0)
      throw std::runtime_error("reference TF limits must be positive");

    // ---------- 运行状态(running/goal_reached)判定参数 ----------
    status_goal_tol_ = load_parameter<double>(node_, "fsm.status_goal_tol", 0.15);
    status_speed_tol_ = load_parameter<double>(node_, "fsm.status_speed_tol", 0.05);
    status_stop_debounce_ = load_parameter<double>(node_, "fsm.status_stop_debounce", 1.0);
    status_heartbeat_hz_ = load_parameter<double>(node_, "fsm.status_heartbeat_hz", 1.0);
    status_goal_change_eps_ = load_parameter<double>(node_, "fsm.status_goal_change_eps", 0.05);

    // ---------- 第3步：预设航点模式下，解析航点数组 ----------
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      // fsm.waypoints 是扁平数组，每 3 个数字组成一个航点 (x,y,z)
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      // 校验：不能为空，且个数必须是 3 的倍数，否则启动直接报错
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      // 航点个数 = 数字总数 / 3
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);
      preset_waypoints_.resize(waypoint_num_);
      // 每 3 个数字拼成一个三维航点
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    // ---------- 第4步：创建可视化对象与规划总管 ----------
    /* initialize main modules */
    // 可视化对象：在 RViz 里画目标点/轨迹/A* 路径
    visualization_.reset(new PlanningVisualization(node_));
    // 规划总管：真正算轨迹的对象（内部会建地图、建优化器）
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* callback */
    // ---------- 第5步：创建两个定时器 ----------
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           // 10ms 定时器 → 状态机主循环 execFSMCallback
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             // 50ms 定时器 → 碰撞安全检查 checkCollisionCallback
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    // ---------- 第6步：订阅话题 ----------
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        // 订阅里程计：机器人真实位置/速度/姿态
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    // 订阅“底盘是否冻结”：原地转向时让规划器暂停轨迹时钟
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));
    // 任务控制：/mission/control（"pause" 清轨迹停住 / "resume" 重建并继续）；锁存
    mission_control_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/mission/control", rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&SCANReplanFSM::missionControlCallback, this, std::placeholders::_1));

    // ---------- 第7步：创建发布器 ----------
    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    // 把当前局部轨迹也以 Path 形式发出来（navi_mode=3 用来观察参考路径跟随效果）
    local_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("local_path", 10);
    // 发布规划出的轨迹（控制器订阅它来跟踪）
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    // 发布调试数据（给 RViz 显示）
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        // 发布机器人外形（前后两个圆柱的 Marker）
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());
    // 运行状态话题：只发 running / goal_reached（idle 不发），用 transient_local 让后订阅者也能拿到当前状态
    status_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/local_planner/status", rclcpp::QoS(1).reliable().transient_local());

    // ---------- 第8步：按导航模式订阅对应的“目标来源” ----------
    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      // 模式 1：订阅 RViz 的 2D 目标点
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    // 模式 3：订阅外部发布的全局路径
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", rclcpp::QoS(1).reliable().transient_local(), std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    // 模式 2：预设航点，不需要订阅；收到第一帧里程计后自动开始
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    // 其它值 → 参数非法，直接抛异常退出
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  // 预设航点模式的入口：收到第一帧里程计后，从第一个预设航点开始规划。
  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    // 复制一份预设航点，作为“本轮要依次前往的航点列表”
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    // 先把每个航点画到 RViz（显示成目标球）
    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    // 激活航点列表，从第 0 个航点开始
    active_waypoints_ = wps;
    current_wp_ = 0;
    // 打开触发开关；把起点记为当前位置
    trigger_ = true;
    init_pt_ = odom_pos_;

    // 规划到第一个航点
    if (planNextWaypoint())
    {
      // 成功 → 切换到“生成新轨迹”状态
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      // 失败 → 打错误日志（状态保持不变）
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  // 回调：在 RViz 里点了 2D 目标点后，这里被触发（navi_mode = 1）。
  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    // 空消息直接忽略
    if (!msg)
      return;

    // 还没收到过里程计 → 不知道目标该放在什么高度，先忽略
    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    // 把单个目标点包装成“只有 1 个点的 Path”，复用统一处理函数
    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    // 交给统一入口 waypointCallback 处理
    waypointCallback(path);
  }

  // 目标点处理的“主入口”：把目标点转成全局轨迹，并决定进入哪个状态。
  // （RViz 点选的目标会先被包装成只有 1 个点的 Path，再交给这里）
  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    // 空消息或没有点 → 忽略
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    // 高度小于 -0.1 视为无效目标（RViz 点选时可能点到地面以下）
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    // 正式触发：记录触发标志和起点
    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    // 目标点：x、y 用点选的坐标，高度用之前记录的目标高度
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    // 让规划总管生成一条“从当前位置到目标”的全局轨迹
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 全局轨迹成功后再检查：目标点是否落在障碍里
    if (success)
      success = adjustGlobalTargetIfOccupied();

    // 把目标点画到 RViz
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    // ---- 成功分支：更新标志，并在 RViz 显示全局路径 ----
    if (success)
    {

      /*** display ***/
      // 每 0.1 秒采样一个点，用来把全局路径画成一条线
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      // 逐点采样
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      // 终点速度设为 0（到终点要停住）
      end_vel_.setZero();
      // 标记“已有目标”“这是一个新目标”
      have_target_ = true;
      have_new_target_ = true;
      // 新目标 → 运行状态置为 running
      armPlannerStatus();

      /*** FSM ***/
      // 按当前状态切换：等待中→生成新轨迹；正在执行→重新规划
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      // 显示全局路径
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      // ---- 失败分支：打错误日志 ----
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  // 根据一串航点，让规划总管生成一条全局参考轨迹。
  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    // 参考路径至少要有两个点（第一个点当起点，最后一个点当终点）
    if (waypoints.size() < 2)
    {
      RCLCPP_WARN(node_->get_logger(), "Reference path requires at least two points");
      return false;
    }

    // 终点记在参考坐标系下，真正规划时再乘上“参考系 → world”的变换
    reference_end_pt_ = waypoints.back();
    // 第一个点作为起点，其余点作为中间航点
    std::vector<Eigen::Vector3d> reference_waypoints(waypoints.begin() + 1, waypoints.end());

    // 把每个航点都画出来
    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i,
                                       reference_frame_id_);
    }

    // 让规划总管用这些航点生成全局参考轨迹
    bool success = planner_manager_->planGlobalTrajWaypoints(
        waypoints.front(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        reference_waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    // 失败 → 打日志并返回
    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    // 采样全局轨迹（每 0.1 秒一个点）用于显示
    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    // 终点速度设为 0；标记有目标、有新目标
    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    // 显示全局路径与终点（都要指定参考坐标系，否则 RViz 里会画错位置）
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0, reference_frame_id_);
    visualization_->displayGoalPoint(reference_end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3,
                                     static_cast<int>(waypoints.size()) - 1, reference_frame_id_);

    // 成功
    return true;
  }

  // 前往“下一个”航点：把起点设成当前状态，终点设成该航点，然后规划全局轨迹。
  bool SCANReplanFSM::planNextWaypoint()
  {
    // 航点下标越界 → 报错并返回失败
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    // 终点 = 当前要前往的航点
    end_pt_ = active_waypoints_[current_wp_];
    // 起点：默认用里程计；已有轨迹时用轨迹上的速度/加速度（衔接更平滑）
    setStartStateFromOdomOrCurrentTraj();

    // 规划“从起点到该航点”的全局轨迹
    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    // 失败 → 报错返回
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    // 检查终点是否被障碍占据
    if (!adjustGlobalTargetIfOccupied())
      return false;

    // 采样轨迹用于显示
    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    // 终点速度 0；标记有目标、有新目标
    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    // 显示全局路径与当前航点
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    // 打日志：正在前往第几个航点
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  // 判断当前是不是“按预设航点依次走”的模式（只有 navi_mode = 2 才是）。
  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    // 只有 navi_mode == 2 才是“依次走航点”模式
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  // 如果终点落在了障碍物里，就沿着全局轨迹往回找最后一个“能站人”的点，
  // 把它当作新的终点。返回 false 表示整条轨迹都被占了，无解。
  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    // 取出地图和全局轨迹数据
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    // 没有地图、或轨迹太短 → 无需调整，直接当成功
    if (!map || duration < 1e-3)
      return true;

    // 从轨迹末端开始，按 0.05 秒间隔采样
    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    // 轨迹最后一个点（原始终点）
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    // 终点的前一个采样点（用来估计朝向）
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    // 检查原始终点是否被占据
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    // 终点是空的 → 不需要调整
    if (final_occ <= 0)
      return true;

    // 终点被占：从末端往回逐个采样点找“能站人”的位置
    for (int i = sample_num; i >= 0; --i)
    {
      // 当前采样时刻
      const double t = duration * i / sample_num;
      // 前一个采样时刻（用于估计朝向）
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      // 找到第一个空点 → 把它当作新终点
      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        // 记下原始终点，用于日志
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        // 把全局轨迹时长缩短到这一点（相当于提前结束）
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        // 打警告：原目标被占，改用新终点
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    // 整条轨迹都找不到可用点 → 报错返回失败
    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }

  // 回调：收到外部给的一条全局路径时触发（navi_mode = 3）。
  // 会把路径上每个点抬高 body_height_，再转成全局轨迹。
  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    // 空路径 → 忽略
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }

    // 收到参考路径先做三项校验：必须有里程计、必须带坐标系、各点坐标系要一致
    if (!have_odom_)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "No odometry yet; ignoring initial_path");
      return;
    }

    if (msg->header.frame_id.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "Ignoring initial_path with an empty frame_id");
      return;
    }
    for (const auto &pose : msg->poses)
    {
      if (!pose.header.frame_id.empty() && pose.header.frame_id != msg->header.frame_id)
      {
        RCLCPP_WARN(node_->get_logger(), "Ignoring initial_path with mixed pose frames");
        return;
      }
    }

    // 把路径上每个点转成三维航点
    std::vector<Eigen::Vector3d> waypoints;
    // 把路径上的点统一换算成 world 系下的三维航点（内部会补上 body_height_，因为外部给的是地面高度）
    std::string path_error;
    if (!prepareReferenceWaypoints(*msg, body_height_, 0.5, waypoints, &path_error))
    {
      RCLCPP_WARN(node_->get_logger(), "Ignoring initial_path: %s", path_error.c_str());
      return;
    }

    // 缓存最近一条可用的全局路径：外部 resume 时用它重建
    last_global_path_ = msg;
    // 外部停止期间只缓存、不规划（恢复时再重建）
    if (externally_stopped_)
    {
      RCLCPP_INFO(node_->get_logger(), "External STOP active: /global_path cached, not planning");
      return;
    }

    // 运行状态：只有"目标点变了"才算新目标；重复的同一条路径不重置状态
    const bool status_new_goal =
        !status_goal_valid_ || (waypoints.back() - status_goal_ref_).norm() > status_goal_change_eps_;
    if (status_new_goal)
    {
      status_goal_ref_ = waypoints.back();
      status_goal_valid_ = true;
    }

    // 记住这条参考路径的坐标系，并清掉上一轮的 TF 状态
    reference_frame_id_ = msg->header.frame_id;
    have_last_reference_transform_ = false;
    reference_jump_replan_ = false;
    // 收到参考路径即视为触发
    trigger_ = true;
    // 用这些航点生成全局轨迹
    bool success = planGlobalTrajByWaypoints(waypoints);

    // 成功 → 按当前状态切到“生成新轨迹”或“重新规划”
    if (success)
    {
      // 目标变化 → 进入 running（重复路径不重置）
      if (status_new_goal)
        armPlannerStatus();

      if (reference_tf_jump_failed_)
      {
        reference_tf_jump_failed_ = false;
        reference_tf_stop_ = false;
        changeFSMExecState(GEN_NEW_TRAJ, "NEW_PATH");
      }
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      // 打日志确认已接受参考路径
      RCLCPP_INFO(node_->get_logger(), "Reference path accepted");
    }
    // 失败 → 报错
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
    }
  }

  // 回调：每收到一帧里程计就执行。
  // 作用：更新“我现在在哪、速度多少”，记录首帧高度，并画出机器人外形。
  // 这是整个程序的数据来源——没有里程计就寸步难行。
  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    // 更新当前位置 (x, y, z)
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    // 模式 1：用首帧里程计的高度作为 RViz 目标点的默认高度（只做一次）
    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    // 更新当前速度
    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    // 更新当前姿态（四元数）
    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    // 标记“已收到里程计”——这是状态机从 INIT 走出来的前提条件
    have_odom_ = true;
    // 画出机器人外形（前后两个圆柱）
    publishSelfInflationMarker();
    // 预设航点模式：第一帧里程计到达后启动航点导航（只做一次）
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
  }

  // 回调：底盘告诉规划器“我正在原地转向，先别推进轨迹”（消息是 true/false）。
  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    // 记录底盘是否“冻结”（正在原地转向）
    go2_execution_frozen_ = msg->data;
  }

  // 处理上面的“冻结”信号：
  // 如果底盘正在原地转向，就把局部轨迹的开始时间往后推，等于让轨迹时钟暂停，
  // 这样轨迹不会在机器人还没转过来时就白白走完。
  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    // 距上次处理过了多少秒
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    // 更新“上次处理时间”
    last_freeze_update_time_ = now;

    // 时间差异常（<=0 或 >0.2 秒）→ 忽略本次，防止时钟跳变
    if (dt <= 0.0 || dt > 0.2)
      return;

    // 取出当前局部轨迹数据
    LocalTrajData *info = &planner_manager_->local_data_;
    // 若底盘冻结且轨迹已经开始 → 把轨迹的“开始时间”往后推 dt
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      // 效果：轨迹时钟暂停，机器人原地转向的时间不会“消耗”轨迹
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  // 收到新目标时进入 running；若之前是 goal_reached，这里把它重置。
  void SCANReplanFSM::armPlannerStatus()
  {
    status_have_goal_ = true;
    status_is_stop_ = false;
    status_hold_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    // 外部停止期间不发 running（状态话题保持 stopped）
    if (!externally_stopped_)
      publishPlannerStatus("running");
  }

  // 每个 tick 判断"到达目标且停稳"：到达(2D 距离 < 容差)且速度≈0，持续去抖后发一次 goal_reached；
  // running 期间按心跳周期重发。idle（从未收到目标）不发消息。
  void SCANReplanFSM::updatePlannerStatus()
  {
    const rclcpp::Time now = node_->now();

    // 外部停止（/mission/control = pause）：状态话题为 stopped，1Hz 重发
    if (externally_stopped_)
    {
      if (status_heartbeat_hz_ > 1e-6 &&
          (now - status_last_pub_time_).seconds() >= 1.0 / status_heartbeat_hz_)
      {
        publishPlannerStatus("stopped");
      }
      return;
    }

    if (!status_have_goal_)
      return;

    if (!status_is_stop_)
    {
      // 2D 水平距离（end_pt_ 与 odom_pos_ 都在 world 系），口径与控制器 finish_dist 一致
      const bool arrived =
          (end_pt_.head<2>() - odom_pos_.head<2>()).norm() < status_goal_tol_;
      // 实际速度范数（与坐标系无关）
      const bool stopped = odom_vel_.norm() < status_speed_tol_;
      if (arrived && stopped)
      {
        if (status_hold_since_.nanoseconds() == 0)
        {
          status_hold_since_ = now;
        }
        else if ((now - status_hold_since_).seconds() >= status_stop_debounce_)
        {
          status_is_stop_ = true;
          publishPlannerStatus("goal_reached");
        }
      }
      else
      {
        status_hold_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
      }
    }

    // 低频心跳：只在 running 期间周期重发；goal_reached 只在到达时发一次
    if (!status_is_stop_ && status_heartbeat_hz_ > 1e-6 &&
        (now - status_last_pub_time_).seconds() >= 1.0 / status_heartbeat_hz_)
    {
      publishPlannerStatus("running");
    }
  }

  // 把状态发到 /local_planner/status（std_msgs/String）；空串不发布。
  void SCANReplanFSM::publishPlannerStatus(const std::string &state)
  {
    if (state.empty())
      return;
    std_msgs::msg::String msg;
    msg.data = state;
    status_pub_->publish(msg);
    status_last_pub_time_ = node_->now();
  }

  // 任务控制回调：/mission/control → "pause"（清轨迹停住）/ "resume"（重建并继续）
  void SCANReplanFSM::missionControlCallback(const std_msgs::msg::String::ConstSharedPtr &msg)
  {
    if (!msg)
      return;
    if (msg->data == "pause")
    {
      if (!externally_stopped_)
        applyExternalStop();
    }
    else if (msg->data == "resume")
    {
      if (externally_stopped_)
        applyExternalResume();
    }
  }

  // 外部停止：清全局参考轨迹 + 清局部轨迹（机器人由控制器停住）；局部地图照常更新。
  void SCANReplanFSM::applyExternalStop()
  {
    externally_stopped_ = true;
    // 清全局参考轨迹
    planner_manager_->global_data_.global_duration_ = 0.0;
    planner_manager_->global_data_.last_progress_time_ = 0.0;
    reference_end_pt_.setZero();
    end_pt_.setZero();
    end_vel_.setZero();
    have_target_ = false;
    have_new_target_ = false;
    trigger_ = false;
    have_last_reference_transform_ = false;
    // 清局部轨迹
    planner_manager_->local_data_.duration_ = 0.0;
    planner_manager_->local_data_.start_time_ =
        rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    // 状态 → stopped（立即发一次，之后 1Hz）
    status_hold_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    publishPlannerStatus("stopped");
    RCLCPP_WARN(node_->get_logger(),
                "External STOP: global/local trajectories cleared; local map kept");
  }

  // 外部继续：(a) 有缓存的全局路径 → 重建并规划；(b) 否则等新的 /global_path。
  void SCANReplanFSM::applyExternalResume()
  {
    externally_stopped_ = false;
    status_is_stop_ = false;  // 恢复后重新判定是否到达
    if (last_global_path_ && !last_global_path_->poses.empty())
    {
      RCLCPP_INFO(node_->get_logger(), "External RESUME: rebuilding from cached /global_path");
      pathCallback(last_global_path_);
      changeFSMExecState(GEN_NEW_TRAJ, "RESUME");
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "External RESUME: waiting for a new /global_path");
      changeFSMExecState(WAIT_TARGET, "RESUME");
    }
    // 状态话题回到 running（下一次心跳也会重发）
    publishPlannerStatus("running");
  }

  // 从里程计的姿态里取出“偏航角 yaw”（即机器人朝哪个方向）。
  double SCANReplanFSM::getOdomYaw() const
  {
    // 取旋转矩阵第一列，即机器人朝向在水平面的投影
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    // 投影太小（数值退化）→ 无法判断，返回 0
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    // 用 atan2 求出偏航角
    return std::atan2(heading(1), heading(0));
  }

  // 用“从 from 指向 to 的这段线段”来估计朝向。
  // 因为碰撞检测要知道机器人朝哪个方向（机器人是前后两个圆柱），
  // 所以凡是查障碍都要带上一个 yaw，这个函数就是用来算 yaw 的。
  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    // 用 from→to 的水平方向作为朝向
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    // 两点几乎重合 → 退回用里程计的朝向
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  // 发布机器人自身外形：在 RViz 里画前后两个半透明圆柱，
  // 它俩就是碰撞检测所用的“机器人形状”。
  void SCANReplanFSM::publishSelfInflationMarker()
  {
    // 读取半径和上下膨胀高度（都保证非负）
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    // 圆柱总高度（至少 1 毫米，避免退化成 0）
    const double height = std::max(1e-3, z_up + z_down);

    // 新建一个 Marker（RViz 里的可视化图元）
    visualization_msgs::msg::Marker marker;
    // 坐标框架：参数里没配就用 world
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    // 时间戳
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    // 形状选圆柱；动作是“添加/更新”
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // 尺寸：直径 × 直径 × 高度
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    // 颜色：浅蓝、半透明
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    // 生命周期 2 秒：既要“过期自动消失，避免残影”，又不能短到在重规划期间
    // 被 RViz 判为过期（marker 由里程计回调发出，若规划占住执行器，回调会短暂停顿，
    // 0.2 秒会因此整块闪掉）。2 秒足以跨过规划停顿，规划器退出后也会自动消失。
    marker.lifetime = rclcpp::Duration::from_seconds(2.0);

    // 圆柱中心：机器人位置，高度方向按上下膨胀不对称做补偿
    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    // 沿朝向方向，算出“前圆柱”和“后圆柱”的圆心
    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    // 先发布前圆柱
    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    // 再发布后圆柱
    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  // 切换状态（例如 GEN_NEW_TRAJ → EXEC_TRAJ），同时打印一行日志。
  // pos_call 只是日志里说明“是谁触发的切换”。
  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    // 切到的是同一个状态 → 连续调用次数 +1
    if (new_state == exec_state_)
      continuously_called_times_++;
    // 切到了新状态 → 计数重置为 1
    else
      continuously_called_times_ = 1;

    // 状态名表（下标对应枚举值，仅用于打印）
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    // 记住切换前的状态
    int pre_s = int(exec_state_);
    // 更新当前状态
    exec_state_ = new_state;
    // 打印“从 X 到 Y”
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  // 返回 <当前状态已经连续被调用了多少次, 当前状态>，用来做重试策略。
  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    // 返回 <同一状态连续被调用的次数, 当前状态>
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  // 打印当前状态名（调试用）。
  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    // 打印当前状态名（调试用）
    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  // ==========================================================================
  // 【核心】execFSMCallback()：每 10 毫秒被调用一次，是状态机的主循环。
  //   1. 先处理“冻结”：如果底盘原地转向，就暂停轨迹时钟
  //   2. 每隔约 1 秒打印一次当前状态（方便观察）
  //   3. 用 switch 分派到六个状态中的一个，执行那个状态要做的事
  //   4. 收尾：finishProcess()，并发布调试数据
  // 下面每个 case 就是“某个状态下要做什么”。
  // ==========================================================================
  void SCANReplanFSM::execFSMCallback()
  {
    // 第一步：处理“冻结”（底盘原地转向时暂停轨迹时钟）
    updateLocalTrajTimeFreeze();
    // 同步运行状态（running/goal_reached/stopped）；放在最前，避免被后续的提前 return 跳过
    updatePlannerStatus();
    // 外部停止（/mission/control = pause）：不重规划、不推进，直到 resume
    if (externally_stopped_)
      return;
    if (!monitorReferenceTransform())
      return;

    // 下面几行：每 100 次（约 1 秒）打印一次当前状态
    static int fsm_num = 0;
    fsm_num++;
    // 每 100 次才打印一次
    if (fsm_num == 100)
    {
      // 打印状态；顺便提示是否还缺里程计 / 目标
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      // 计数清零
      fsm_num = 0;
    }

    // 第二步：按当前状态分派到不同分支
    switch (exec_state_)
    {
    // 状态 INIT：刚启动。等“有里程计”且“有目标触发”后，进入 WAIT_TARGET。
    // ---- 状态 INIT：等待“有里程计”且“有触发” ----
    case INIT:
    {
      // 还没收到里程计 → 继续等待
      if (!have_odom_)
      {
        return;
      }
      // 还没被触发 → 继续等待
      if (!trigger_)
      {
        return;
      }
      // 两个条件都满足 → 进入 WAIT_TARGET
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    // 状态 WAIT_TARGET：等待目标点。一旦有目标，就进入 GEN_NEW_TRAJ。
    // ---- 状态 WAIT_TARGET：等待目标点 ----
    case WAIT_TARGET:
    {
      // 还没有目标 → 什么都不做
      if (!have_target_)
        return;
      else
      {
        // 有目标了 → 去生成新轨迹
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    // 状态 GEN_NEW_TRAJ：生成一条全新轨迹。
    // 先决定起点用哪个数据，再调用 callReboundReplan() 让总管规划。
    // 成功 → EXEC_TRAJ；失败 → 记录失败次数，等待重试。
    // ---- 状态 GEN_NEW_TRAJ：生成一条全新轨迹 ----
    case GEN_NEW_TRAJ:
    {
      // 先确定起点（默认里程计；有轨迹时用轨迹上的速度/加速度）
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      // 第一次调用用普通多项式；反复失败后改用“插随机点”的方式
      bool flag_random_poly_init;
      // 判断是不是本状态的第一次调用
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      // 交给规划总管规划，返回是否成功
      bool success = callReboundReplan(true, flag_random_poly_init);
      // 成功 → 失败计数清零，进入“执行轨迹”状态
      if (success)
      {

        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        // 允许后续从急停中恢复
        flag_escape_emergency_ = true;
      }
      else
      // 失败 → 失败计数 +1，留在本状态下轮重试
      {
        if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && reference_jump_replan_)
        {
          reference_tf_stop_ = true;
          reference_tf_jump_failed_ = true;
          callEmergencyStop(odom_pos_);
          flag_escape_emergency_ = false;
          changeFSMExecState(EMERGENCY_STOP, "TF_JUMP_FAIL");
        }
        else
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      }
      break;
    }

    // 状态 REPLAN_TRAJ：在已有轨迹基础上重新规划（例如偏离了、有障碍了）。
    // ---- 状态 REPLAN_TRAJ：重新规划 ----
    case REPLAN_TRAJ:
    {
      // 从当前轨迹出发重新规划；若是参考系跳变触发的重规划，
      // 则以当前实测状态为起点（旧轨迹的预测状态已失效）
      bool success;
      if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && reference_jump_replan_)
      {
        // 定位修正后旧轨迹的预测状态已失效，改从实测状态起算
        start_pt_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();
        success = callReboundReplan(true, false);
      }
      else
        success = planFromCurrentTraj();
      if (success)
      {
        reference_jump_replan_ = false;
        // 成功 → 进入执行状态
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        // 参考系跳变导致的重规划失败 → 直接急停，避免继续跑在失效轨迹上
        if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && reference_jump_replan_)
        {
          reference_tf_stop_ = true;
          reference_tf_jump_failed_ = true;
          callEmergencyStop(odom_pos_);
          flag_escape_emergency_ = false;
          changeFSMExecState(EMERGENCY_STOP, "TF_JUMP_FAIL");
        }
        else
      {
        // 失败 → 计数 +1，保持重规划状态继续尝试
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      }

      break;
    }

    // 状态 EXEC_TRAJ：正在沿轨迹执行。
    // 计算“已经走了多久 t_cur”，取出当前位置；
    // 判断是否快到终点、是否该前往下一个航点、是否需要重新规划。
    // ---- 状态 EXEC_TRAJ：正在沿轨迹执行 ----
    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      // 取出当前局部轨迹数据
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      // 已经执行了多少秒
      double t_cur = (time_now - info->start_time_).seconds();
      // 把时间夹在 [0, 总时长] 之内
      t_cur = min(info->duration_, t_cur);

      // 当前时刻应该到达的位置
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      // 【航点模式】已到当前航点附近（0.5 米内）→ 前往下一个航点
      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        // 航点下标前进一位
        current_wp_++;
        // 规划到下一个航点，并切到“生成新轨迹”
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        // 规划失败 → 计数 +1，仍然要求重新生成新轨迹
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      // 【轨迹已经走完】
      if (t_cur > info->duration_ - 1e-2)
      {
        // 还有下一个航点 → 继续前往
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        // 没有下一个航点了 → 清空航点列表
        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        // 标记“目标已完成”
        have_target_ = false;

        // 回到 WAIT_TARGET 等新目标
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      // 离终点已经很近 → 不必重规划，继续走完
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      // 离这条轨迹的起点很近 → 说明刚重规划过，也先不重规划
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      // 其它情况 → 该重新规划了
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    // 状态 EMERGENCY_STOP：紧急停止。
    // 先把轨迹收缩成原地停住，然后等速度降下来再尝试恢复（如果开了 fail_safe）。
    // ---- 状态 EMERGENCY_STOP：紧急停止 ----
    case EMERGENCY_STOP:
    {
      if (reference_tf_stop_)
      {
        if (!reference_tf_jump_failed_ && odom_vel_.norm() < 0.1)
        {
          reference_tf_stop_ = false;
          reference_jump_replan_ = false;
          have_new_target_ = true;
          changeFSMExecState(GEN_NEW_TRAJ, "TF_RECOVERED");
        }
        break;
      }

      // 刚进入急停：先把轨迹收缩成“停在原地”（只做一次）
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      // 已经停住了：如果开了自动恢复，且速度已经很低
      else
      {
        // 普通情况 → 回到 GEN_NEW_TRAJ 重新尝试规划
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        // 是“失败太多”导致的急停 → 清空目标，回到等待新目标
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      // 本轮处理完成，清掉“刚进入急停”的标志
      flag_escape_emergency_ = false;
      break;
    }
    }

    // 第三步：收尾（失败太多则强制急停）
    finishProcess();

    // 第四步：发布调试数据（带当前时间戳）
    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  // 每轮主循环结束后的收尾：如果连续规划失败次数太多，就强制进入紧急停止。
  // 收尾检查：如果连续失败次数太多，就强制急停并等待新目标
  void SCANReplanFSM::finishProcess()
  {
    // 失败次数达到上限 → 强制进入急停
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed %d times; emergency stop and wait for a new target", replan_fail_count_);
      // 计数清零；标记“需要悬停停下”；允许从急停恢复
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  // 从“当前轨迹”出发重新规划：
  // 起点用当前位置，速度/加速度从当前轨迹上取，然后重新规划到终点。
  // 这是最常见的重规划方式（比如检测到障碍时）。
  bool SCANReplanFSM::planFromCurrentTraj()
  {
    // 取出当前局部轨迹数据
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    // 这条轨迹已经执行了多少秒
    double t_cur = (time_now - info->start_time_).seconds();
    // 把时间夹在 [0, 总时长] 之内（防止负值或超界）
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    // 参考路径模式下，重规划起点取“当前轨迹上此刻的位置”，
    // 这样参考系变动时不会因为里程计跳变而抖
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      bool success = callReboundReplan(false, false);
      if (!success)
      {
        success = callReboundReplan(true, false);
        if (!success)
        {
          success = callReboundReplan(true, true);
          if (!success)
            return false;
        }
      }

      return true;
    }

    // 起点 = 机器人当前位置
    start_pt_ = odom_pos_;
    // 速度、加速度都从“当前轨迹”上取，保证重规划后轨迹平滑衔接
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    // 计算“从当前位置指向目标”的水平方向
    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    // 如果当前速度是朝反方向走的（背对目标）
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      // 就把速度、加速度清零，避免越走越远
      start_vel_.setZero();
      start_acc_.setZero();
    }

    // 重新生成一条“从当前位置到终点”的全局参考轨迹
    if (!planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      // 失败 → 打错误日志并返回
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      return false;
    }

    // 检查终点是否落在障碍里（若在，会沿轨迹回退找一个能站的点）
    if (!adjustGlobalTargetIfOccupied())
      return false;

    // 让规划总管做一次局部重规划（普通方式）
    bool success = callReboundReplan(true, false);
    // 第一次失败 → 再用“带随机点”的方式试一次（有助于跳出卡死）
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        // 两次都失败 → 放弃本轮
        return false;
    }

    // 规划成功
    return true;
  }

  // 决定“起点”用哪套数据：默认用里程计；
  // 如果当前已有轨迹且在时间范围内，就用轨迹上的速度/加速度，让衔接更平滑。
  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    // 默认方案：起点用里程计的位置和速度，加速度设为 0
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    LocalTrajData *info = &planner_manager_->local_data_;
    // 如果当前没有正在执行的轨迹（还没开始过）→ 就用默认方案
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    // 当前时刻距离轨迹开始过了多久
    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    // 时间明显超出轨迹范围（或为负）→ 也用默认方案
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    // 把时间夹进有效范围
    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    // 用轨迹上的速度/加速度作为起点状态（比用里程计更平滑）
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    // 如果速度背对目标 → 清零，避免“往反方向起步”
    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  // ==========================================================================
  // checkCollisionCallback()：每 50 毫秒被调用一次，负责“安全检查”。
  // 做法：沿着当前局部轨迹往前走，每隔 0.01 秒取一个点，问地图“这里能站人吗”。
  //   只检查轨迹的前 2/3（因为后段本来就会重规划）。
  //   发现会撞：先尝试重规划；不行且撞上时间很近 → 紧急停止。
  // ==========================================================================
  void SCANReplanFSM::checkCollisionCallback()
  {
    // 同样先处理“冻结”
    updateLocalTrajTimeFreeze();
    // 外部停止期间不做碰撞检查
    if (externally_stopped_)
      return;

    // 取出当前局部轨迹和地图
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    // 参考 TF 已判定异常并急停过 → 不再做碰撞检查
    if (reference_tf_stop_)
      return;
    // 正在等待目标、或轨迹还没开始 → 不需要检查
    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    // 采样步长：每 0.01 秒取一个点
    constexpr double time_step = 0.01;
    // 这条轨迹已经执行了多久
    double t_cur = (node_->now() - info->start_time_).seconds();
    // 只检查轨迹的前 2/3（后段本来就会重规划，不必查）
    double t_2_3 = info->duration_ * 2 / 3;
    // 从当前时刻往后，逐步前进检查
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      // 一旦越过 2/3 分界点，就停止检查
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      // 该时刻的位置
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      // 再取后面一点点位置，用两点连线方向估计机器人的朝向
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      // 问地图：机器人以这个朝向待在这里，会不会撞？
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        // 参考 TF 缺失期间又检测到碰撞 → 直接急停（此时无法确认参考路径是否仍安全）
        if (navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
            reference_tf_missing_since_.nanoseconds() != 0)
        {
          reference_tf_stop_ = true;
          callEmergencyStop(odom_pos_);
          flag_escape_emergency_ = false;
          changeFSMExecState(EMERGENCY_STOP, "TF_MISSING_COLLISION");
          return;
        }
        // 撞了！先尝试“从当前轨迹出发重新规划”（也许还来得及绕开）
        if (planFromCurrentTraj()) // Make a chance
        {
          // 重规划成功 → 回到执行状态，本轮结束
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          // 重规划失败，而且离碰撞时刻很近 → 只能紧急停止
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            // 打警告：多少秒后就要撞上了
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          // 还比较远 → 只是要求重新规划
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          // 无论哪种情况，本轮检查到此结束
          return;
        }
        break;
      }
    }
  }

  // 调用规划总管做一次规划（先取局部目标点，再交给总管）。
  // 成功后把算出的轨迹打包成 Bspline 消息发布出去。
  // flag_use_poly_init：是否用多项式作为初始猜测；
  // flag_randomPolyTraj：是否在初始猜测里插一个随机点（用来跳出卡死）。
  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    // 第一步：从全局轨迹上取“前方 planning_horizon 处”的点作为局部目标；
    // 参考路径模式下若拿不到参考系变换会返回 false，本轮规划直接放弃
    if (!getLocalTarget())
      return false;

    // 第二步：让规划总管真正规划一条局部轨迹
    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    // “新目标”标志用完即清（它只影响第一次规划）
    have_new_target_ = false;

    // 打印规划结果
    cout << "final_plan_success=" << plan_success << endl;

    // ---- 成功：把轨迹打包成消息发布出去 ----
    if (plan_success)
    {
      if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      {
        last_reference_to_world_ = planning_reference_to_world_;
        have_last_reference_transform_ = true;
      }

      // 取出刚算好的局部轨迹
      auto info = &planner_manager_->local_data_;

      /* publish traj */
      // 构造要发送的 B 样条消息
      scan_planner_msgs::msg::Bspline bspline;
      // 阶数固定为 3（三次 B 样条，与规划器内部一致）
      bspline.order = 3;
      // 起始时刻 + 轨迹编号（控制器靠它们判断是不是新轨迹）
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      // 把控制点从矩阵搬进消息
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      // 逐个控制点复制坐标
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      // 再把节点向量（knots）搬进消息
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      // 发布轨迹 —— 控制器订阅 planning/bspline，会开始跟踪它
      bspline_pub_->publish(bspline);
      publishLocalPath(info->position_traj_);

      // 顺便在 RViz 里画出这条最优轨迹
      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  // 把当前局部轨迹按固定间隔采样成 Path 发到 local_path 话题（供外部观察 / 录制）
  void SCANReplanFSM::publishLocalPath(const UniformBspline &trajectory)
  {
    UniformBspline traj = trajectory;
    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;

    const double duration = traj.getTimeSum();
    const double sample_dt = duration > 0.0 ? std::max(0.01, std::min(0.05, duration / 200.0)) : 0.01;

    for (double t = 0.0; t <= duration + 1e-6; t += sample_dt)
    {
      const auto pos = traj.evaluateDeBoorT(std::min(t, duration));
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = path.header.frame_id;
      pose.header.stamp = path.header.stamp;
      pose.pose.position.x = pos.x();
      pose.pose.position.y = pos.y();
      pose.pose.position.z = pos.z();
      pose.pose.orientation.w = 1.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      path.poses.push_back(pose);
    }

    local_path_pub_->publish(path);
  }

  // 紧急停止：让总管生成一条“停在 stop_pos”的轨迹，并发布出去。
  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    // 让规划总管生成一条“停在原地”的轨迹
    planner_manager_->EmergencyStop(stop_pos);

    // 下面把这条静止轨迹打包成消息（流程与 callReboundReplan 完全一样）
    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    // 发布 —— 控制器收到后会立刻刹停
    bspline_pub_->publish(bspline);
    publishLocalPath(info->position_traj_);

    return true;
  }

  // 查询“参考坐标系 → world 系”的变换。
  // 参考系与 world 系相同时直接返回单位变换；TF 查不到、或数据太旧，都算失败。
  bool SCANReplanFSM::lookupReferenceTransform(Eigen::Isometry3d &reference_to_world)
  {
    reference_to_world.setIdentity();
    if (reference_frame_id_ == self_inflation_frame_id_)
      return true;
    if (reference_frame_id_.empty())
      return false;
    try
    {
      const auto transform = tf_buffer_->lookupTransform(
          self_inflation_frame_id_, reference_frame_id_, tf2::TimePointZero);
      // 变换的“新鲜度”：太旧的 TF 不可信，当成拿不到
      const double age = (node_->now() - rclcpp::Time(
          transform.header.stamp, node_->get_clock()->get_clock_type())).seconds();
      if (!referenceTransformFresh(age, reference_tf_max_age_))
        return false;
      reference_to_world = referenceToWorld(transform);
      return true;
    }
    catch (const tf2::TransformException &error)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Reference TF unavailable: %s", error.what());
      return false;
    }
  }

  // 监控参考坐标系：长时间拿不到就急停；若发生明显跳变（定位修正），
  // 就把当前轨迹作废并触发重规划。返回 true 表示参考系正常。
  bool SCANReplanFSM::monitorReferenceTransform()
  {
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH || !have_target_)
      return true;

    Eigen::Isometry3d current;
    if (!lookupReferenceTransform(current))
    {
      // 记下“从什么时候开始拿不到”；超过宽限时间就急停
      const rclcpp::Time now = node_->now();
      if (reference_tf_missing_since_.nanoseconds() == 0)
        reference_tf_missing_since_ = now;
      if (referenceTfGraceExpired((now - reference_tf_missing_since_).seconds(),
                                  reference_tf_grace_) &&
          !reference_tf_stop_)
      {
        RCLCPP_ERROR(node_->get_logger(), "Reference TF missing for %.2f s; stopping",
                     reference_tf_grace_);
        reference_tf_stop_ = true;
        callEmergencyStop(odom_pos_);
        flag_escape_emergency_ = false;
        changeFSMExecState(EMERGENCY_STOP, "TF_TIMEOUT");
      }
      return false;
    }

    // 拿到了 → 清空“缺失起始时刻”，并按最新变换刷新终点
    reference_tf_missing_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    end_pt_ = current * reference_end_pt_;
    // 执行轨迹期间，参考系若突然跳了一大步，旧轨迹的预测状态已不可信 → 重规划
    if (exec_state_ == EXEC_TRAJ && have_last_reference_transform_ &&
        referenceTransformJumped(last_reference_to_world_, current, odom_pos_,
                                 reference_tf_jump_translation_, reference_tf_jump_yaw_))
    {
      reference_jump_replan_ = true;
      changeFSMExecState(REPLAN_TRAJ, "TF_CHANGED");
    }
    return true;
  }

  // 取“局部目标点”：沿全局轨迹往前走大约 planning_horizon_ 米，
  // 把那个点作为这一轮规划的局部目标（而不是一路直奔最终终点）。
  // 若该点正好在障碍里，就前后找一个能站人的点代替。
  // navi_mode=3 时全局轨迹给在“参考坐标系”下，这里会先换算到 world 系；
  // 换算不成功（TF 不可用或过旧）则返回 false，本轮规划直接放弃。
  bool SCANReplanFSM::getLocalTarget()
  {
    Eigen::Isometry3d reference_to_world = Eigen::Isometry3d::Identity();
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      if (!lookupReferenceTransform(reference_to_world))
        return false;
      planning_reference_to_world_ = reference_to_world;
      end_pt_ = reference_to_world * reference_end_pt_;
    }
    // 起点也换算到参考系下，后面的距离都在同一个坐标系里算
    const Eigen::Vector3d reference_start = reference_to_world.inverse() * start_pt_;
    auto toWorld = [&](const Eigen::Vector3d &reference_point) {
      return reference_to_world * reference_point;
    };
    const double max_vel = planner_manager_->pp_.max_vel_;
    const double max_acc = planner_manager_->pp_.max_acc_;
    const double duration = planner_manager_->global_data_.global_duration_;
    // 采样步长：把规划视野长度分成 20 份
    double t_step = max_vel > 1e-6 ? planning_horizon_ / 20.0 / max_vel : 0.01;
    t_step = std::max(t_step, 0.01);

    // 先在全局轨迹上找“离起点最近”的时刻 t_proj，避免轨迹回绕时取错段
    double t_proj = 0.0;
    double min_dist_to_start = 9999.0;
    // t：沿全局轨迹遍历用的时间变量
    for (double t = 0.0; t < duration; t += t_step)
    {
      // 该时刻的位置
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      // 它离当前起点有多远
      double dist_to_start = (pos_t - reference_start).norm();
      // 记录“离起点最近的点”，后面回退搜索时不能越过它
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        t_proj = t;
      }
    }

    // target_t：局部目标在全局轨迹上对应的时刻（默认取全局终点）
    double target_t = duration;
    double total_dist = 0.0;
    bool target_found = false;
    Eigen::Vector3d prev_pos = planner_manager_->global_data_.getPosition(t_proj);
    // 局部目标 = 最终终点
    local_target_pt_ = end_pt_;

    // 从 t_proj 往后走，累计走过的弧长；达到规划视野长度就以那里为局部目标
    for (double t = t_proj; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      total_dist += (pos_t - prev_pos).norm();
      // 一旦距离达到规划视野长度 → 就把这个点作为局部目标
      if (total_dist >= planning_horizon_)
      {
        local_target_pt_ = toWorld(pos_t);
        target_t = t;
        target_found = true;
        break;
      }
      prev_pos = pos_t;
    }
    // 把“进度时间”记成局部目标时刻，保证下一轮从这里继续（不会回头看）
    planner_manager_->global_data_.last_progress_time_ = target_found ? target_t : duration;

    // 小工具：查询某个点是否被障碍占据（查询时带上朝向）
    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    // 如果局部目标恰好落在障碍里 → 前后各找一个“能站人”的点代替
    if (targetOccupancy(local_target_pt_) != 0)
    {
      // 是否已找到可用的替代点
      bool found_free_target = false;
      // adjusted_t：替代点对应的时刻
      double adjusted_t = target_t;

      // 从目标时刻出发，向两侧逐步扩大搜索范围
      for (double dt = 0.0; dt <= duration; dt += t_step)
      {
        // 先往“前”找（更靠近终点的那一侧）
        double t_forward = target_t + dt;
        // 不能超出全局轨迹的范围
        if (t_forward <= duration)
        {
          Eigen::Vector3d pt = toWorld(planner_manager_->global_data_.getPosition(t_forward));
          // 找到一个空点 → 采用它并结束搜索
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        // 再往“后”找，但不能退到“离起点最近的点”之前
        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, t_proj))
        {
          Eigen::Vector3d pt = toWorld(planner_manager_->global_data_.getPosition(t_backward));
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      // 找到了替代点 → 打提示日志并更新目标时刻
      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      // 没找到 → 打警告（保持原目标，稍后会因碰撞触发重规划）
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
        return false;
      }
      if (!target_found && target_t < duration)
      {
        reference_end_pt_ = planner_manager_->global_data_.getPosition(target_t);
        planner_manager_->global_data_.global_duration_ = target_t;
        end_pt_ = local_target_pt_;
      }
    }

    // 如果局部目标已经非常接近最终终点（进入刹车距离之内）
    const double braking_distance = max_acc > 1e-6
        ? (max_vel * max_vel) / (2.0 * max_acc)
        : std::numeric_limits<double>::infinity();
    if ((end_pt_ - local_target_pt_).norm() < braking_distance)
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      // 局部目标速度设为 0 —— 表示“到达后要停下”
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    // 否则
    else
    {
      // 局部目标速度 = 全局轨迹在该时刻的速度（保持匀速通过）
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      local_target_vel_ = reference_to_world.linear() * local_target_vel_;
      if (local_target_vel_.norm() > max_vel)
        local_target_vel_ = local_target_vel_.normalized() * max_vel;
      // cout << "AA" << endl;
    }
    return true;
  }

} // namespace scan_planner
