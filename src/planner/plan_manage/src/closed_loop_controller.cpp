// ============================================================================
// 文件：src/planner/plan_manage/src/closed_loop_controller.cpp
//
// 【这个文件是干什么的】
//   “闭环控制器”。它是规划器和机器人之间的“司机”：
//   订阅规划器算出的轨迹，每 10 毫秒算一次速度指令 cmd_vel 发给底盘。
//   之所以叫“闭环”，是因为它一边看轨迹、一边用真实位置纠偏。
//
// 【它订阅/发布什么】
//   订阅 planning/bspline  —— 规划器给的轨迹
//   订阅 body_pose         —— 我现在真实的位置（用于纠偏）
//   发布 cmd_vel           —— 速度指令（前进/横移/转向）
//   发布 planning/go2_execution_frozen —— 告诉规划器“我在原地转向，先暂停轨迹”
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
// 控制器类。继承 rclcpp::Node 表示它本身就是一个 ROS 节点。
class ClosedLoopController : public rclcpp::Node
{
public:
  // 构造函数：节点取名 closed_loop_controller，读取参数（前瞻时间、增益、限速等），
  // 然后订阅轨迹和里程计，并创建一个 10ms 的定时器去执行 cmdCallback()。
  ClosedLoopController() : Node("closed_loop_controller")
  {
    // 前瞻时间：看点时看 0.8 秒之后的轨迹点，避免转向滞后
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    // 朝向误差超过这个角度(弧度) → 先原地转向、暂停前进
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    // 位置纠偏增益：位置误差 → 附加速度
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    // 转向增益：朝向误差 → 转向角速度
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    // 前进/横移/转向的速度上限（转向另有硬上限）
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    // 取参数值与硬上限中的较小者，保证不超过底盘能力
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    // 到终点多近就算“到达”，可以停下
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    // ---- 卡住检测 + 刹停 + 脱困 ----
    // “想动(指令速度 > stuck_cmd_eps) 但实测速度 < stuck_speed_eps” 判为卡住
    stuck_cmd_eps_ = declare_parameter<double>("stuck_cmd_eps", 0.15);
    stuck_speed_eps_ = declare_parameter<double>("stuck_speed_eps", 0.03);
    // 连续卡住超过这个时间(秒) → 触发一次脱困动作（卡住期间先刹停）
    stuck_time_ = declare_parameter<double>("stuck_time", 1.0);
    // 脱困开关 / 每次动作持续时间(秒) / 后退或横移速度 / 原地转向角速度
    escape_enabled_ = declare_parameter<bool>("escape_enabled", true);
    escape_duration_ = declare_parameter<double>("escape_duration", 0.6);
    escape_speed_ = declare_parameter<double>("escape_speed", 0.2);
    escape_yaw_rate_ = declare_parameter<double>("escape_yaw_rate", 0.5);

    // 订阅规划器发布的轨迹
    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    // 订阅里程计（真实位置，用于纠偏）
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    // 发布速度指令给底盘
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    // 发布“冻结”信号：告诉规划器先别推进轨迹
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    // 每 10ms 执行一次控制计算
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    // 打印一行日志表示启动完成
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  // 把角度规整到 [-π, π]，避免“359 度和 1 度其实很接近”这类计算错误。
  // 把角度规整到 [-π, π]（359° 和 1° 其实是同一个方向）
  static double normalizeAngle(double angle)
  {
    // 角度太大 → 减一圈
    while (angle > M_PI) angle -= 2.0 * M_PI;
    // 角度太小 → 加一圈
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  // 限制二维向量长度不超过 max_norm（防止速度指令超过底盘上限）。
  // 限制二维速度向量长度不超过上限
  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    // 先算向量长度
    const double norm = value.norm();
    // 没超就原样返回；超了就按比例缩小到上限
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  // 估算“当前应该朝哪个方向”：看向轨迹前方 time_forward_ 秒处的点
  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    // 前瞻时刻（不能超过轨迹末尾）
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    // 前瞻点相对当前位置的向量，就是期望的前进方向
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    // 前瞻点离当前位置太近（几乎重合）→ 改用速度方向
    if (direction.head<2>().squaredNorm() < 1e-4)
      // 用当前时刻的速度方向作为朝向
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        // 还是退化的话就用里程计朝向；否则用 atan2 算出方向角
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  // 发送一个“停下”的速度指令（可选同时原地转向）。
  // 发送一个“停下”指令（可选同时原地转向）
  void publishStop(double yaw_rate = 0.0)
  {
    // 默认所有速度都是 0
    geometry_msgs::msg::Twist cmd;
    // 需要的话填一个转向角速度
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    // 发布出去
    cmd_vel_pub_->publish(cmd);
  }

  // 发布脱困速度指令（机体系：vx 前进/后退、vy 横移、wz 转向），各分量夹到限幅内
  void publishEscape(double vx, double vy, double wz)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(vx, -max_vx_, max_vx_);
    cmd.linear.y = std::clamp(vy, -max_vy_, max_vy_);
    cmd.angular.z = std::clamp(wz, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  // 把“我正在原地转向、请规划器暂停轨迹”这个信号发出去（true/false）。
  // 发布“冻结”信号（true=正在原地转向，请规划器暂停轨迹时钟）
  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    // 填入 true/false
    msg.data = frozen;
    // 发布
    execution_frozen_pub_->publish(msg);
  }

  // 回调：收到规划器发来的新轨迹后，把它解析成 B 样条并保存，
  // 同时把“执行计时器”清零，表示要从这条新轨迹的起点重新开始跟踪。
  // 回调：收到规划器发来的新轨迹 → 解析并保存
  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    // 基本合法性检查：控制点/节点/阶数都不能缺
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      // 不合法就忽略这条消息
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    // 把消息里的控制点搬到 3×N 矩阵
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    // 逐个控制点复制 xyz
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    // 把节点向量也搬过来
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    // 用控制点+阶数构造 B 样条（时间间隔先随便给 0.1，下面用真实节点覆盖）
    UniformBspline position(points, msg->order, 0.1);
    // 用消息里的真实节点向量替换
    position.setKnot(knots);
    // 保存位置曲线和速度曲线
    traj_ = {position, position.getDerivative()};
    // 再求一次导得到加速度曲线，一起存起来
    traj_.push_back(traj_[1].getDerivative());
    // 记录这条轨迹的总时长
    traj_duration_ = traj_[0].getTimeSum();
    // 记录轨迹编号（用来区分是不是新轨迹）
    traj_id_ = msg->traj_id;
    // 把“已执行时间”清零 —— 从新轨迹的起点重新开始跟踪
    exec_time_ = 0.0;
    last_update_time_ = now();
    // 标记“已经有轨迹可跟踪了”
    receive_traj_ = true;
    // 打印日志
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  // 回调：收到里程计，记录我当前的真实位置和朝向（偏航角）。
  // 回调：收到里程计 → 记录真实位置和朝向
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    // 当前位置
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    // 当前朝向（把四元数转成偏航角）
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    // 实测速度（机体系线速度的模）：用于“卡住检测”
    odom_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    // 标记“已收到里程计”
    have_odom_ = true;
  }

  // 【核心】每 10 毫秒执行一次：算出速度指令并发布。
  //   1. 从轨迹上取出“现在应该在哪”（pos_des）
  //   2. 用前瞻点算出“应该朝哪”（期望偏航角）
  //   3. 如果朝向差太多 → 先原地转向（并让规划器暂停轨迹）
  //   4. 否则 → 前馈速度 + 位置纠偏，转成底盘坐标系后发给 cmd_vel
  // 【核心】每 10ms 一次：算出并发布速度指令
  void cmdCallback()
  {
    // 还没轨迹、或还没里程计 → 什么都不做，只发一个“停”和“未冻结”
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      // 发停车指令
      publishStop();
      return;
    }

    const auto now_t = now();
    // ---- 脱困动作进行中：直接执行脱困指令，覆盖轨迹跟踪 ----
    if (now_t < escape_until_)
    {
      publishExecutionFrozen(false);
      publishEscape(escape_cmd_.linear.x, escape_cmd_.linear.y, escape_cmd_.angular.z);
      last_update_time_ = now_t;
      return;
    }
    // 当前时间
    const auto current_time = now();
    // 距上次计算过了多久
    double dt = (current_time - last_update_time_).seconds();
    // 时间差异常（时钟跳变/卡顿）→ 当作 0，不推进
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    // 本次要看轨迹上的哪一时刻
    const double t_eval = std::min(exec_time_, traj_duration_);
    // 该时刻轨迹上应该在的位置
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    // 朝向误差 = 期望朝向 - 当前朝向
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    // 用比例控制算出转向角速度，并夹在允许范围内
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    // 朝向差太多 → 先原地转向，不要往前走
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      // 告诉规划器“我在转向，先暂停轨迹时钟”
      publishExecutionFrozen(true);
      // 只发转向、不前进
      publishStop(yaw_command);
      // 记住这次的时间（注意：exec_time_ 不推进）
      last_update_time_ = current_time;
      return;
    }

    // 朝向已对齐 → 解除冻结，允许规划器继续推进轨迹
    publishExecutionFrozen(false);
    // 轨迹执行时间往前推进 dt
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    // 记住这次时间，供下一轮计算 dt（此时 exec_time_ 已经推进）
    last_update_time_ = current_time;
    // 用推进后的时刻重新取目标位置
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    // 轨迹在前馈速度（直接取轨迹的一阶导）
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    // 位置误差 = 应该在的位置 - 实际位置
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    // 期望世界系速度 = 前馈速度 + 位置误差的比例修正
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        // 整体限速，避免修正量过大
        std::max(max_vx_, max_vy_));
    // 下面把世界系速度旋转到机器人自身坐标系（底盘指令用的是机体系）
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    // 组装最终的速度指令
    geometry_msgs::msg::Twist command;
    // 机体系前进速度
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    // 机体系横移速度
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    // 转向角速度
    command.angular.z = yaw_command;
    // 轨迹已走完，且已经很靠近终点
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      // 发全 0 指令 → 停住
      command = geometry_msgs::msg::Twist();

    // ---- 卡住检测：想动(指令速度够大)但实测几乎不动 → 先刹停，久则脱困 ----
    const bool intend_move = std::abs(command.linear.x) > stuck_cmd_eps_ ||
                             std::abs(command.linear.y) > stuck_cmd_eps_;
    const bool moving = odom_speed_ > stuck_speed_eps_;
    if (escape_enabled_ && intend_move && !moving)
    {
      if (stuck_since_.nanoseconds() == 0) stuck_since_ = current_time;
      if ((current_time - stuck_since_).seconds() > stuck_time_)
      {
        // 轮换脱困动作：后退 / 原地左转 / 横移左 / 原地右转 / 横移右
        escape_idx_ = (escape_idx_ + 1) % 5;
        escape_cmd_ = geometry_msgs::msg::Twist();
        switch (escape_idx_)
        {
          case 0: escape_cmd_.linear.x = -escape_speed_; break;    // 后退
          case 1: escape_cmd_.angular.z = escape_yaw_rate_; break; // 原地左转
          case 2: escape_cmd_.linear.y = escape_speed_; break;     // 横移左
          case 3: escape_cmd_.angular.z = -escape_yaw_rate_; break;// 原地右转
          default: escape_cmd_.linear.y = -escape_speed_; break;   // 横移右
        }
        escape_until_ = current_time + rclcpp::Duration::from_seconds(escape_duration_);
        stuck_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        RCLCPP_WARN(get_logger(), "Robot stuck (intend move but odom_speed=%.3f m/s); escape action #%d",
                    odom_speed_, escape_idx_);
      }
      // 卡住期间先刹停，别继续满速顶
      publishStop();
      return;
    }
    stuck_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    // 发布速度指令
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  // ---- 卡住检测 + 脱困 ----
  rclcpp::Time stuck_since_{0, 0, RCL_ROS_TIME};      // 开始卡住的时刻(0=未卡住)
  rclcpp::Time escape_until_{0, 0, RCL_ROS_TIME};     // 脱困动作结束时刻
  geometry_msgs::msg::Twist escape_cmd_;              // 当前脱困指令
  int escape_idx_{0};                                 // 脱困动作轮换索引
  bool escape_enabled_{true};
  double odom_speed_{0.0};                            // 实测速度(来自 odom twist)
  double stuck_cmd_eps_{0.15}, stuck_speed_eps_{0.03};// 卡住判据阈值
  double stuck_time_{1.0}, escape_duration_{0.6},
      escape_speed_{0.2}, escape_yaw_rate_{0.5};
};
}  // namespace scan_planner

// 程序入口：初始化 ROS，创建控制器节点并一直运行。
// 程序入口：初始化 ROS 并一直运行
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
