// ============================================================================
// 文件：src/planner/plan_manage/src/go2_kinematic_sim.cpp
//
// 【这个文件是干什么的】
//   “运动学仿真器”：一个极简的假机器人。
//   它接收 cmd_vel（速度指令），用最简单的积分算出机器人跑到哪了，
//   再把结果当成里程计 body_pose 发出去，从而凑成一个闭环。
//
// 【它的运动模型】
//   二维平面：x += vx*dt, y += vy*dt, yaw += vyaw*dt。
//   高度 z 固定不变。没有加速度限制，也不模拟腿。
//
// 【安全设计】
//   如果超过 cmd_timeout（默认 0.3 秒）没收到新的速度指令，就把速度清零
//   （相当于“失联就停车”）。
// ============================================================================
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace scan_planner
{
// 仿真器类（本身是一个 ROS 节点）。
class Go2KinematicSim : public rclcpp::Node
{
public:
  // 构造函数：节点取名 go2_kinematic_sim，读参数（初始位置、限速、仿真频率），
  // 订阅 cmd_vel，创建发布器 body_pose，并创建定时器执行 simCallback()。
  Go2KinematicSim() : Node("go2_kinematic_sim")
  {
    // 机器人的初始位置和初始朝向
    x_ = declare_parameter<double>("init_x", 0.0);
    y_ = declare_parameter<double>("init_y", 0.0);
    z_ = declare_parameter<double>("init_z", 0.3);
    yaw_ = declare_parameter<double>("init_yaw", 0.0);
    // 速度上限（和规划器保持一致）
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    // 转向上限，取参数与硬上限中的较小者
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    // 多少秒没收到新指令就自动停车（失联保护）
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
    // 仿真频率（每秒积分多少次）
    const double sim_rate = declare_parameter<double>("sim_rate", 100.0);
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base");

    // 用于广播 world→base 的坐标变换
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // 发布假里程计（规划器和控制器都订阅它）
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_pose", 100);
    // 订阅控制器的速度指令
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 20, std::bind(&Go2KinematicSim::cmdCallback, this, std::placeholders::_1));
    // 记录“上次收到指令的时间”和“上次积分时间”
    last_cmd_time_ = now();
    last_sim_time_ = now();
    // 按 sim_rate 周期性积分更新位置
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, sim_rate)),
        std::bind(&Go2KinematicSim::simCallback, this));
    // 启动日志
    RCLCPP_INFO(get_logger(), "Go2 kinematic simulator ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  // 把角度规整到 [-π, π]
  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  // 回调：收到速度指令，先按上限做裁剪，然后存下来备用。
  // 回调：收到速度指令 → 裁剪到上限后存起来
  void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    // 前进速度限幅
    vx_cmd_ = std::clamp(msg->linear.x, -max_vx_, max_vx_);
    // 横移速度限幅
    vy_cmd_ = std::clamp(msg->linear.y, -max_vy_, max_vy_);
    // 转向角速度限幅
    vyaw_cmd_ = std::clamp(msg->angular.z, -max_vyaw_, max_vyaw_);
    // 记下收到指令的时间（用于失联判断）
    last_cmd_time_ = now();
  }

  // 把当前位置/朝向/速度打包成 Odometry 消息发出去（这就是假里程计）。
  // 把当前位置/朝向/速度打包成里程计消息
  void publishOdom(const rclcpp::Time &stamp)
  {
    // 偏航角 → 四元数
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw_);
    const auto orientation = tf2::toMsg(quaternion);
    // 组装里程计消息
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    // 填位置和姿态
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = z_;
    odom.pose.pose.orientation = orientation;
    // 填世界系速度（由机身速度旋转而来）
    odom.twist.twist.linear.x = vx_world_;
    odom.twist.twist.linear.y = vy_world_;
    // 填角速度
    odom.twist.twist.angular.z = vyaw_cmd_;
    // 发布
    odom_pub_->publish(odom);

    // 如果配置了要发布 TF → 再广播一次坐标变换（RViz 显示需要）
    if (publish_tf_)
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = child_frame_id_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.translation.z = z_;
      transform.transform.rotation = orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  // 【核心】定时器回调：按仿真频率执行。
  //   1. 算距上次的时间 dt；若太久没收到指令就把速度清零（失联停车）
  //   2. 把机体系速度旋转到世界系，然后积分更新 x、y、yaw
  //   3. 发布新的里程计（必要时也广播 TF 坐标变换）
  // 【核心】定时器回调：按仿真频率更新机器人位置
  void simCallback()
  {
    // 当前时间
    const auto current_time = now();
    // 距上次积分过了多久
    double dt = (current_time - last_sim_time_).seconds();
    last_sim_time_ = current_time;
    // 时间差异常 → 当作 0，避免位置跳变
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    // 取当前速度指令
    double vx = vx_cmd_, vy = vy_cmd_, wz = vyaw_cmd_;
    // 超过 cmd_timeout 没收到新指令 → 失联，停车
    if ((current_time - last_cmd_time_).seconds() > cmd_timeout_)
      // 三个速度全部清零
      vx = vy = wz = 0.0;
    // 把“机身坐标系”下的速度旋转到“世界坐标系”
    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    // 旋转公式（二维）
    vx_world_ = c * vx - s * vy;
    vy_world_ = s * vx + c * vy;
    // 用最简单的欧拉积分更新位置和朝向
    x_ += vx_world_ * dt;
    y_ += vy_world_ * dt;
    // 更新朝向并规整到 [-π, π]
    yaw_ = normalizeAngle(yaw_ + wz * dt);
    // 发布新的里程计
    publishOdom(current_time);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double x_{0.0}, y_{0.0}, z_{0.3}, yaw_{0.0};
  double vx_cmd_{0.0}, vy_cmd_{0.0}, vyaw_cmd_{0.0};
  double vx_world_{0.0}, vy_world_{0.0};
  double max_vx_{0.75}, max_vy_{0.35}, max_vyaw_{1.0}, cmd_timeout_{0.3};
  bool publish_tf_{false};
  std::string frame_id_, child_frame_id_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_sim_time_{0, 0, RCL_ROS_TIME};
};
}  // namespace scan_planner

// 程序入口：初始化 ROS，创建仿真器节点并一直运行。
// 程序入口
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::Go2KinematicSim>());
  rclcpp::shutdown();
  return 0;
}
