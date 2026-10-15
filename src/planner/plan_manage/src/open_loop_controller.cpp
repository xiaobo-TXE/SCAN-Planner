// ============================================================================
// 文件：src/planner/plan_manage/src/open_loop_controller.cpp
//
// 【这个文件是干什么的】
//   “开环控制器”。它不做任何控制计算，只是把规划好的轨迹“照抄”成里程计。
//   也就是假装机器人完美地沿着轨迹走（理想执行，不引入任何误差）。
//
// 【它用来干什么】
//   调试/演示用：想单独看规划结果时，用它代替真实底盘。
//   它发布 body_pose（假装的位置），规划器拿到后就以为机器人在动。
//
// 【和闭环控制器的区别】
//   闭环：看真实位置 → 算速度指令 cmd_vel（会纠偏，更真实）；
//   开环：直接输出轨迹上的点当作真实位置（无控制、无误差）。
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
// 开环控制器类（本身是一个 ROS 节点）。
class OpenLoopController : public rclcpp::Node
{
public:
  // 构造函数：节点取名 open_loop_controller，读参数，
  // 订阅 planning/bspline，并创建一个定时器按 publish_rate 发布 body_pose。
  OpenLoopController() : Node("open_loop_controller")
  {
    // 里程计消息里的坐标框架名
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    // 子框架名（机器人本体）
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "quadruped");
    // 每秒发布多少帧假里程计
    const double publish_rate = declare_parameter<double>("publish_rate", 100.0);
    // 速度大于这个值才更新朝向（速度太小时方向没意义）
    yaw_min_speed_ = declare_parameter<double>("yaw_min_speed", 0.05);
    // 轨迹走完后是否“停在终点不动”（否则就不再发布）
    hold_final_position_ = declare_parameter<bool>("hold_final_position", true);
    // 初始朝向
    last_yaw_ = declare_parameter<double>("init_yaw", 0.0);
    // 初始位置（必须与地图里机器人的起始位置一致）
    current_pos_ = Eigen::Vector3d(
        declare_parameter<double>("init_x", 0.0),
        declare_parameter<double>("init_y", 0.0),
        declare_parameter<double>("init_z", 0.3));

    // 发布假里程计 —— 规划器就靠它认为“机器人在动”
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_pose", 20);
    // 订阅规划器发布的轨迹
    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&OpenLoopController::bsplineCallback, this, std::placeholders::_1));
    // 按 publish_rate 周期性地把轨迹上的点当成里程计发出去
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate)),
        std::bind(&OpenLoopController::publishOdom, this));
    // 启动完成日志
    RCLCPP_INFO(get_logger(), "Open-loop controller ready");
  }

private:
  // 把 Bspline 消息解析成 B 样条对象（解析失败返回 false）
  bool parseBspline(const scan_planner_msgs::msg::Bspline::ConstSharedPtr &msg,
                    UniformBspline &pos_traj)
  {
    // 合法性检查
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return false;
    }
    // 控制点从消息搬进矩阵
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    // 逐点复制
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      pos_pts.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    // 节点向量搬进向量
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    // 构造 B 样条，再用真实节点覆盖
    pos_traj = UniformBspline(pos_pts, msg->order, 0.1);
    pos_traj.setKnot(knots);
    return true;
  }

  // 回调：收到轨迹后解析成 B 样条，并记下这条轨迹的起始时刻。
  // 回调：收到新轨迹 → 保存，并记录它的起始时刻
  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    UniformBspline pos_traj;
    // 解析失败就直接忽略
    if (!parseBspline(msg, pos_traj)) return;
    // 保存位置与速度曲线
    traj_ = {pos_traj, pos_traj.getDerivative()};
    // 再求一次导得到加速度曲线
    traj_.push_back(traj_[1].getDerivative());
    // 记录轨迹的起始时刻 —— 用它算“已经走了多久”
    start_time_ = rclcpp::Time(msg->start_time);
    traj_id_ = msg->traj_id;
    // 轨迹总时长
    traj_duration_ = traj_[0].getTimeSum();
    // 标记“已有轨迹”
    receive_traj_ = true;
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  // 工具函数：把一组位置/速度/朝向打包成里程计消息发出去
  void publishState(const rclcpp::Time &stamp, const Eigen::Vector3d &pos,
                    const Eigen::Vector3d &vel, double yaw, double yaw_rate)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    // 填消息头：时间、坐标框架
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = pos.x();
    odom.pose.pose.position.y = pos.y();
    odom.pose.pose.position.z = pos.z();
    // 把偏航角转成四元数（姿态）
    tf2::Quaternion quaternion;
    // 只有偏航角，横滚和俯仰都为 0
    quaternion.setRPY(0.0, 0.0, yaw);
    odom.pose.pose.orientation = tf2::toMsg(quaternion);
    // 填速度
    odom.twist.twist.linear.x = vel.x();
    odom.twist.twist.linear.y = vel.y();
    odom.twist.twist.linear.z = vel.z();
    // 填转向角速度
    odom.twist.twist.angular.z = yaw_rate;
    // 发布
    odom_pub_->publish(odom);
  }

  // 定时器回调：算出现在过去了多久 elapsed，
  // 直接从轨迹上取出那一时刻的位置/速度，假装成里程计发出去。
  // 定时器回调：把轨迹上的点当成里程计发出去（这就是“开环”的含义）
  void publishOdom()
  {
    // 当前时间
    const auto now = this->now();
    // 还没收到轨迹 → 一直发布初始状态
    if (!receive_traj_)
    {
      publishState(now, current_pos_, current_vel_, last_yaw_, 0.0);
      return;
    }
    // 这条轨迹已经走了多久
    const double elapsed = (now - start_time_).seconds();
    // 时间还没到（时钟未同步）→ 先发上一状态
    if (elapsed < 0.0)
    {
      // 用上一次的状态继续发布
      publishState(now, current_pos_, current_vel_, last_yaw_, 0.0);
      return;
    }
    // 如果配置为“走完就不再发布”，此刻直接返回
    if (!hold_final_position_ && elapsed > traj_duration_) return;
    // 取轨迹上的时刻（夹在有效范围内）
    const double t = std::clamp(elapsed, 0.0, traj_duration_);
    // 该时刻的位置 —— 直接当作机器人的真实位置
    const Eigen::Vector3d pos = traj_[0].evaluateDeBoorT(t);
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    // 还没走完 → 顺便取速度和加速度来算朝向与角速度
    if (elapsed <= traj_duration_)
    {
      vel = traj_[1].evaluateDeBoorT(t);
      acc = traj_[2].evaluateDeBoorT(t);
    }
    // 水平速度大小
    const double speed = std::hypot(vel.x(), vel.y());
    // 有速度时，朝向 = 速度方向
    if (speed > yaw_min_speed_) last_yaw_ = std::atan2(vel.y(), vel.x());
    const double yaw_rate = speed > yaw_min_speed_
        // 角速度 = 速度方向的变化率（用叉积算）
        ? (vel.x() * acc.y() - vel.y() * acc.x()) / std::max(speed * speed, 1e-6) : 0.0;
    // 记下这次的状态，供下次（无轨迹时）继续发布
    current_pos_ = pos;
    current_vel_ = vel;
    // 发布里程计
    publishState(now, pos, vel, last_yaw_, yaw_rate);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool receive_traj_{false};
  bool hold_final_position_{true};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  std::int64_t traj_id_{0};
  std::string frame_id_;
  std::string child_frame_id_;
  double last_yaw_{0.0};
  double yaw_min_speed_{0.05};
  Eigen::Vector3d current_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_vel_{Eigen::Vector3d::Zero()};
};
}  // namespace scan_planner

// 程序入口：初始化 ROS，创建开环控制器并一直运行。
// 程序入口
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::OpenLoopController>());
  rclcpp::shutdown();
  return 0;
}
