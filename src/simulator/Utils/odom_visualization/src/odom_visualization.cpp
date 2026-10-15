// ============================================================================
// 文件：src/odom_visualization.cpp —— 机器人可视化节点（仿真用）
//
// 【这个文件是干什么的】
//   订阅机器人的里程计，把它在 RViz 里“画出来”：
//   当前位置、历史轨迹、速度箭头、机器人模型、高度。
//   它只负责显示，不参与规划。
//
// 【它发布什么】
//   pose / path / velocity / trajectory / robot / height
// ============================================================================
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

class OdomVisualization : public rclcpp::Node
{
public:
  OdomVisualization() : Node("odom_visualization")
  {
    // 读参数：坐标框架、子框架、机器人模型文件、缩放、颜色、是否发 TF
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base");
    // 机器人外形用哪个模型文件（.mesh / .dae）
    mesh_resource_ = declare_parameter<std::string>(
        "mesh_resource", "package://odom_visualization/meshes/hummingbird.mesh");
    scale_ = declare_parameter<double>("robot_scale", 1.0);
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    color_r_ = declare_parameter<double>("color.r", 0.0);
    color_g_ = declare_parameter<double>("color.g", 0.0);
    color_b_ = declare_parameter<double>("color.b", 0.0);
    color_a_ = declare_parameter<double>("color.a", 1.0);

    // 六个发布器：位姿、历史轨迹、速度箭头、轨迹线、机器人模型、高度
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    velocity_pub_ = create_publisher<visualization_msgs::msg::Marker>("velocity", 10);
    trajectory_pub_ = create_publisher<visualization_msgs::msg::Marker>("trajectory", 10);
    robot_pub_ = create_publisher<visualization_msgs::msg::Marker>("robot", 10);
    height_pub_ = create_publisher<sensor_msgs::msg::Range>("height", 10);
    // 订阅里程计 —— 这就是全部的数据来源
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&OdomVisualization::odomCallback, this, std::placeholders::_1));
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // 历史轨迹也用同一个坐标框架
    path_.header.frame_id = frame_id_;
  }

private:
  // 回调：每收到一帧里程计，就把所有可视化内容更新一遍
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    // 先整理出当前位姿
    geometry_msgs::msg::PoseStamped pose;
    pose.header = message->header;
    // 消息没带框架名就补上默认的
    if (pose.header.frame_id.empty()) pose.header.frame_id = frame_id_;
    pose.pose = message->pose.pose;
    // 发布当前位置
    pose_pub_->publish(pose);

    // 把这一帧位姿追加到历史轨迹里
    path_.header.stamp = pose.header.stamp;
    path_.poses.push_back(pose);
    // 轨迹点太多时丢掉最早的一批，避免无限增长
    if (path_.poses.size() > 10000) path_.poses.erase(path_.poses.begin(), path_.poses.begin() + 1000);
    // 发布整条历史轨迹
    path_pub_->publish(path_);

    // 画一个速度箭头：从当前位置指向“当前位置 + 速度向量”
    visualization_msgs::msg::Marker velocity;
    velocity.header = pose.header;
    velocity.ns = "velocity";
    velocity.id = 0;
    velocity.type = visualization_msgs::msg::Marker::ARROW;
    velocity.action = visualization_msgs::msg::Marker::ADD;
    velocity.scale.x = 0.04;
    velocity.scale.y = 0.08;
    velocity.scale.z = 0.08;
    velocity.color.r = 1.0;
    velocity.color.a = 1.0;
    // 箭头起点 = 当前位置；终点 = 当前位置加一个速度分量
    geometry_msgs::msg::Point start = pose.pose.position;
    geometry_msgs::msg::Point end = start;
    end.x += message->twist.twist.linear.x;
    end.y += message->twist.twist.linear.y;
    end.z += message->twist.twist.linear.z;
    // 设置箭头两端并发布
    velocity.points = {start, end};
    velocity_pub_->publish(velocity);

    // 再画一条轨迹线（沿用上面的设置，只改几项）
    visualization_msgs::msg::Marker trajectory = velocity;
    trajectory.ns = "trajectory";
    trajectory.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory.scale.x = 0.025;
    trajectory.color.r = 0.1;
    trajectory.color.g = 0.8;
    trajectory.color.b = 1.0;
    trajectory.points.reserve(path_.poses.size());
    // 把历史轨迹里所有位置连成一条线
    for (const auto &item : path_.poses) trajectory.points.push_back(item.pose.position);
    trajectory_pub_->publish(trajectory);

    // 画机器人模型（用 MESH_RESOURCE 指定模型文件）
    visualization_msgs::msg::Marker robot;
    robot.header = pose.header;
    robot.ns = "robot";
    robot.id = 0;
    robot.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    robot.action = visualization_msgs::msg::Marker::ADD;
    robot.pose = pose.pose;
    // 缩放比例与颜色
    robot.scale.x = robot.scale.y = robot.scale.z = scale_;
    robot.color.r = color_r_;
    robot.color.g = color_g_;
    robot.color.b = color_b_;
    robot.color.a = color_a_;
    // 指定模型文件路径；并启用模型自带的材质
    robot.mesh_resource = mesh_resource_;
    robot.mesh_use_embedded_materials = true;
    robot_pub_->publish(robot);

    // 再发一个“高度”话题（range 值就是当前的 z 坐标）
    sensor_msgs::msg::Range height;
    height.header = pose.header;
    height.radiation_type = sensor_msgs::msg::Range::INFRARED;
    height.field_of_view = 0.01;
    height.min_range = 0.0;
    height.max_range = 100.0;
    height.range = static_cast<float>(pose.pose.position.z);
    height_pub_->publish(height);

    // 如果配置要求发 TF，就广播 world → base 的坐标变换
    if (publish_tf_)
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = pose.header;
      transform.child_frame_id = child_frame_id_;
      transform.transform.translation.x = pose.pose.position.x;
      transform.transform.translation.y = pose.pose.position.y;
      transform.transform.translation.z = pose.pose.position.z;
      transform.transform.rotation = pose.pose.orientation;
      broadcaster_->sendTransform(transform);
    }
  }

  std::string frame_id_, child_frame_id_, mesh_resource_;
  double scale_{1.0}, color_r_{0.0}, color_g_{0.0}, color_b_{0.0}, color_a_{1.0};
  bool publish_tf_{false};
  nav_msgs::msg::Path path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velocity_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr height_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

// 程序入口：初始化 ROS 并一直运行
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomVisualization>());
  rclcpp::shutdown();
  return 0;
}
