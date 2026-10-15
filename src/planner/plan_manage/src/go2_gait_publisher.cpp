// ============================================================================
// 文件：src/planner/plan_manage/src/go2_gait_publisher.cpp
//
// 【这个文件是干什么的】
//   “步态发布器”：纯粹为了在 RViz 里让 Go2 的腿动起来好看。
//   它根据当前速度，虚拟地生成 12 个关节的角度并发布 joint_states（小跑步态）。
//
// 【重要】
//   它不参与规划，也不控制机器人，只是“动画”。删掉它规划照样工作。
// ============================================================================
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace scan_planner
{
// 步态发布器类（本身是一个 ROS 节点）。
class Go2GaitPublisher : public rclcpp::Node
{
public:
  // 构造函数：节点取名 go2_gait_publisher，读参数（步频、摆腿幅度等），
  // 订阅 body_pose（用它的速度决定走多快），创建定时器发布关节角。
  Go2GaitPublisher() : Node("go2_gait_publisher")
  {
    // 每秒发布多少帧关节状态
    const double rate = declare_parameter<double>("rate", 60.0);
    // 步频：每秒迈几步
    gait_frequency_ = declare_parameter<double>("gait_frequency", 2.2);
    // 低于这个速度视为“站着不动”
    min_walk_speed_ = declare_parameter<double>("min_walk_speed", 0.05);
    // 达到这个速度时摆腿幅度最大
    max_walk_speed_ = declare_parameter<double>("max_walk_speed", 1.0);
    // 是否不管速度都一直小跑
    always_trot_ = declare_parameter<bool>("always_trot", false);
    // 三条腿关节的摆动幅度（髋 / 大腿 / 小腿）
    hip_swing_ = declare_parameter<double>("hip_swing", 0.08);
    thigh_swing_ = declare_parameter<double>("thigh_swing", 0.32);
    calf_swing_ = declare_parameter<double>("calf_swing", 0.42);

    // 订阅里程计：用它的速度决定腿摆多大
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&Go2GaitPublisher::odomCallback, this, std::placeholders::_1));
    // 发布 12 个关节的角度（robot_state_publisher 用它让 RViz 里的模型动起来）
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    // 按 rate 周期性地更新并发布关节角
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, rate)),
        std::bind(&Go2GaitPublisher::timerCallback, this));

    // 12 个关节的名字：前/后腿 × 左/右 × 髋/大腿/小腿
    joint_msg_.name = {
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
    // 预备好角度和角速度数组（长度 12）
    joint_msg_.position.resize(joint_msg_.name.size(), 0.0);
    joint_msg_.velocity.resize(joint_msg_.name.size(), 0.0);
  }

private:
  static constexpr double kPi = 3.14159265358979323846;

  // 回调：从里程计里估计当前“走得多快”。
  // 既看消息里的速度，也看相邻两帧位置差（互为补充）。
  // 回调：估计当前“走得多快”
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    // 方法一：直接读消息里的速度
    const double vx = odom->twist.twist.linear.x;
    const double vy = odom->twist.twist.linear.y;
    // 速度大小
    const double twist_speed = std::hypot(vx, vy);
    // 取这一帧的时间戳
    rclcpp::Time stamp(odom->header.stamp);
    // 时间戳为空时用当前时间代替
    if (stamp.nanoseconds() == 0) stamp = now();
    const double x = odom->pose.pose.position.x;
    const double y = odom->pose.pose.position.y;
    double pose_speed = 0.0;
    // 方法二：用“相邻两帧的位置差 ÷ 时间差”算速度（作为补充）
    if (has_prev_pose_)
    {
      const double dt = (stamp - last_odom_time_).seconds();
      // 位移除以时间就是速度
      if (dt > 1e-4) pose_speed = std::hypot(x - last_odom_x_, y - last_odom_y_) / dt;
    }
    // 优先用消息里的速度；它太小就改用位置差算出的速度
    horizontal_speed_ = twist_speed > min_walk_speed_ ? twist_speed : pose_speed;
    last_odom_x_ = x;
    last_odom_y_ = y;
    last_odom_time_ = stamp;
    has_prev_pose_ = true;
    // 标记“已收到里程计”
    has_odom_ = true;
  }

  // 定时器回调：根据速度算出一个 0~1 的“幅度比例”，
  // 然后按小跑（对角腿交替）相位给 12 个关节赋值并发布。
  // 速度几乎为 0 时，发布一个站立姿势。
  // 定时器回调：根据速度算出摆腿幅度，然后发布 12 个关节角
  void timerCallback()
  {
    const auto stamp = now();
    // 还没收到里程计 → 先发布一个站立姿势
    if (!has_odom_)
    {
      publishStance(stamp);
      return;
    }
    // ratio：摆腿幅度比例 0~1（速度越快越大）；always_trot 时固定 0.45
    const double ratio = always_trot_ ? 0.45 : std::clamp(
        (horizontal_speed_ - min_walk_speed_) / std::max(1e-3, max_walk_speed_ - min_walk_speed_),
        0.0, 1.0);
    // 几乎不动 → 保持站立
    if (ratio <= 1e-3)
    {
      publishStance(stamp);
      return;
    }
    // 步态相位：随时间不断增长的正弦波相位
    const double phase = 2.0 * kPi * gait_frequency_ * stamp.seconds();
    // 小跑步态：对角的两条腿同相，另外对角相差半个周期(π)
    fillLeg(0, phase, ratio, true);
    fillLeg(3, phase + kPi, ratio, false);
    fillLeg(6, phase + kPi, ratio, true);
    fillLeg(9, phase, ratio, false);
    joint_msg_.header.stamp = stamp;
    // 发布这一帧的 12 个关节角（小跑步态）
    joint_pub_->publish(joint_msg_);
  }

  // 发布“站立不动”的关节角（四腿固定的默认姿态）。
  // 发布“站立不动”的姿势
  void publishStance(const rclcpp::Time &stamp)
  {
    // 固定的 12 个站立关节角（前/后腿、左/右腿略有差异）
    const std::array<double, 12> stance = {
        0.05, 0.82, -1.58, -0.05, 0.82, -1.58,
        0.05, 0.95, -1.62, -0.05, 0.95, -1.62};
    for (size_t i = 0; i < stance.size(); ++i)
    {
      // 填入角度，角速度置 0
      joint_msg_.position[i] = stance[i];
      joint_msg_.velocity[i] = 0.0;
    }
    joint_msg_.header.stamp = stamp;
    // 发布这一帧关节角
    joint_pub_->publish(joint_msg_);
  }

  // 给一条腿（髋/大腿/小腿三个关节）计算并填入摆动角度。
  // 计算并填入一条腿的三个关节角（髋、大腿、小腿）
  void fillLeg(size_t offset, double phase, double ratio, bool left_side)
  {
    // 相位的正弦/余弦；swing 只在正弦为正时才有值（模拟“抬腿”阶段）
    const double s = std::sin(phase), c = std::cos(phase), swing = std::max(0.0, s);
    // 左腿取正、右腿取负，形成左右镜像
    const double side = left_side ? 1.0 : -1.0;
    // 关节下标 ≥6 的是后腿（后腿站立角度略有不同）
    const bool rear = offset >= 6;
    const double thigh_stance = rear ? 0.95 : 0.82;
    const double calf_stance = rear ? -1.62 : -1.58;
    // 髋关节：随相位左右小幅摆动
    joint_msg_.position[offset] = side * (0.05 + hip_swing_ * ratio * 0.35 * s);
    // 大腿：在站立角基础上叠加余弦摆动
    joint_msg_.position[offset + 1] = thigh_stance + thigh_swing_ * ratio * c;
    // 小腿：摆动时抬脚，落地时略微弯曲缓冲
    joint_msg_.position[offset + 2] = calf_stance + calf_swing_ * ratio * swing - 0.10 * ratio * (1.0 - swing);
    // 三个角度都做限幅，避免超出关节范围看起来穿模
    joint_msg_.position[offset] = std::clamp(joint_msg_.position[offset], -1.0, 1.0);
    joint_msg_.position[offset + 1] = std::clamp(joint_msg_.position[offset + 1], -1.2, 3.0);
    joint_msg_.position[offset + 2] = std::clamp(joint_msg_.position[offset + 2], -2.6, -0.9);
    // 角频率 ω，用来把角度变化换算成角速度
    const double omega = 2.0 * kPi * gait_frequency_;
    // 角速度 = 角度对时间求导（旋转矢量插值需要）
    joint_msg_.velocity[offset] = side * hip_swing_ * ratio * 0.35 * c * omega;
    joint_msg_.velocity[offset + 1] = -thigh_swing_ * ratio * s * omega;
    joint_msg_.velocity[offset + 2] = calf_swing_ * ratio * (s > 0.0 ? c : 0.0) * omega;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState joint_msg_;
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  double horizontal_speed_{0.0}, last_odom_x_{0.0}, last_odom_y_{0.0};
  double gait_frequency_{2.2}, min_walk_speed_{0.05}, max_walk_speed_{1.0};
  double hip_swing_{0.08}, thigh_swing_{0.32}, calf_swing_{0.42};
  bool always_trot_{false}, has_odom_{false}, has_prev_pose_{false};
};
}  // namespace scan_planner

// 程序入口：初始化 ROS，创建步态发布器并一直运行。
// 程序入口
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::Go2GaitPublisher>());
  rclcpp::shutdown();
  return 0;
}
