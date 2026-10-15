// ============================================================================
// 文件：src/map_publisher.cpp —— PCD 点云地图发布节点
//
// 【这个文件是干什么的】
//   读取一个已经做好的 PCD 点云地图文件（比如仓库根目录的 map.pcd），
//   转成 ROS 点云消息后周期性发布。
//   这是“用真实场景地图做仿真”的入口（run.launch.py 里 use_pcd_map:=true）。
//
// 【它发布什么】
//   global_cloud —— launch 里会重映射成 /map_generator/global_cloud
// ============================================================================
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class MapPublisher : public rclcpp::Node
{
public:
  MapPublisher() : Node("map_pub")
  {
    // 必须提供 PCD 文件路径，否则直接抛异常退出
    const std::string file_name = declare_parameter<std::string>("file_name", "");
    if (file_name.empty()) throw std::runtime_error("file_name parameter must reference a PCD file");
    // 坐标框架、发布频率、下采样分辨率、以及整体平移偏移
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    const double publish_rate = declare_parameter<double>("publish_rate", 0.2);
    const double downsample_resolution = declare_parameter<double>("downsample_res", 0.0);
    const double offset_x = declare_parameter<double>("map_offset_x", 0.0);
    const double offset_y = declare_parameter<double>("map_offset_y", 0.0);
    const double offset_z = declare_parameter<double>("map_offset_z", 0.0);

    // 从 PCD 文件读入点云
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(file_name, cloud) != 0)
      throw std::runtime_error("failed to load PCD file: " + file_name);
    // 给所有点统一加一个偏移，把整张地图挪到需要的位置
    for (auto &point : cloud)
    {
      point.x += offset_x;
      point.y += offset_y;
      point.z += offset_z;
    }
    // 可选步骤：体素下采样（减少点数、加快后续处理）
    if (downsample_resolution > 0.0)
    {
      // 体素栅格滤波器：每个小立方体里只保留一个代表点
      pcl::VoxelGrid<pcl::PointXYZ> filter;
      pcl::PointCloud<pcl::PointXYZ> downsampled;
      filter.setInputCloud(cloud.makeShared());
      // 立方体边长 = 下采样分辨率
      filter.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      filter.filter(downsampled);
      cloud.swap(downsampled);
    }
    // 转成 ROS 点云消息。只转换一次，之后重复发布同一份数据
    pcl::toROSMsg(cloud, message_);
    message_.header.frame_id = frame_id_;
    // 发布话题 global_cloud（launch 里会重映射）
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "global_cloud", rclcpp::QoS(1).reliable().transient_local());
    // 按 publish_rate 周期性重发（默认 5 秒一次，够用）
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate)),
        std::bind(&MapPublisher::publishMap, this));
    publishMap();
    // 打印一行日志：载入了多少个点
    RCLCPP_INFO(get_logger(), "Loaded %zu PCD points from %s", cloud.size(), file_name.c_str());
  }

private:
  void publishMap()
  {
    message_.header.stamp = now();
    publisher_->publish(message_);
  }
  std::string frame_id_;
  sensor_msgs::msg::PointCloud2 message_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // 捕获异常（文件不存在 / PCD 格式错误等），打印致命日志后退出
  try
  {
    rclcpp::spin(std::make_shared<MapPublisher>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("map_pub"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
