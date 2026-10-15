// ============================================================================
// 文件：src/waypoint_generator.cpp —— 目标点转路径的小工具
//
// 【这个文件是干什么的】
//   你在 RViz 里点一个目标点，它就把“当前位置 → 目标点”组成一条路径发出去。
//   适合快速造一条两点路径来测试。
//
// 【注意】本项目的 launch 文件里没有启动它，属于独立小工具。
// ============================================================================
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class WaypointGenerator : public rclcpp::Node
{
public:
  WaypointGenerator() : Node("waypoint_generator")
  {
    // 坐标框架名
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    // 订阅里程计：只记住“最新一帧”，作为路径的起点
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { latest_odom_ = *message; have_odom_ = true; });
    // 订阅 RViz 里 2D Goal 工具点选的目标点
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "move_base_simple/goal", 10,
        std::bind(&WaypointGenerator::goalCallback, this, std::placeholders::_1));
    // 发布“起点→目标”的路径
    path_pub_ = create_publisher<nav_msgs::msg::Path>("waypoints", 10);
    // 同时发一份 PoseArray，方便在 RViz 里把两个点显示出来
    pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("waypoints_vis", 10);
  }

private:
  // 回调：用户点了目标点 → 组装一条路径发出去
  void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal)
  {
    // 还没收到过里程计 → 不知道起点在哪，先忽略这个目标
    if (!have_odom_)
    {
      RCLCPP_WARN(get_logger(), "Ignoring goal until body_pose is available");
      return;
    }
    // 组装路径消息
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    // 起点用最新一帧里程计的位置
    geometry_msgs::msg::PoseStamped start;
    start.header = path.header;
    start.pose = latest_odom_.pose.pose;
    // 路径 = [起点, 目标点]
    path.poses = {start, *goal};
    path.poses.back().header = path.header;
    // 发布路径
    path_pub_->publish(path);

    // 再发一份位姿数组（纯可视化）
    geometry_msgs::msg::PoseArray array;
    array.header = path.header;
    for (const auto &pose : path.poses) array.poses.push_back(pose.pose);
    pose_array_pub_->publish(array);
  }

  std::string frame_id_;
  bool have_odom_{false};
  nav_msgs::msg::Odometry latest_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
};

// 程序入口：初始化 ROS 并一直运行
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointGenerator>());
  rclcpp::shutdown();
  return 0;
}
