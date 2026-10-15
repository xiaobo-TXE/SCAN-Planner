// ============================================================================
// 文件：src/mockamap.cpp —— 程序化随机地图生成节点
//
// 【这个文件是干什么的】
//   仿真时用它“造一张假地图”：随机生成一堆障碍点，发布成点云。
//   这样不用真实环境也能测试规划器。
//
// 【它发布什么】
//   mock_map（PointCloud2）—— launch 里会重映射成 /map_generator/global_cloud，
//   然后被传感器仿真器“看到”。
//
// 【地图类型由 type 参数决定】
//   1=柏林噪声  2=随机方块  3=2D 迷宫  4=3D 迷宫（具体实现在 maps.cpp）
// ============================================================================
#include <algorithm>
#include <memory>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "maps.hpp"

namespace
{
// 可选的地图“瘦身”函数：删掉被邻居包围的内部点，减少总点数（当前未被调用）
void optimizeMap(mocka::Maps::BasicInfo &info)
{
  // 记录“被邻居包围”的点的下标
  std::vector<int> enclosed;
  // 建一棵 KD 树，用于快速查邻居
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(info.cloud->makeShared());
  // 邻居查询半径
  const double radius = 1.75 / info.scale;
  // 逐点统计它周围有多少邻居
  for (size_t i = 0; i < info.cloud->size(); ++i)
  {
    std::vector<int> indices;
    std::vector<float> distances;
    // 邻居数达到 27（相当于 3×3×3 的立方体）→ 判定它在物体内部，可以删
    if (tree.radiusSearch(info.cloud->points[i], radius, indices, distances) >= 27)
      enclosed.push_back(static_cast<int>(i));
  }
  // 从后往前删（这样删除不会影响前面元素的下标）
  for (auto iterator = enclosed.rbegin(); iterator != enclosed.rend(); ++iterator)
    info.cloud->points.erase(info.cloud->points.begin() + *iterator);
  // 更新点云尺寸，再转成 ROS 消息
  info.cloud->width = static_cast<uint32_t>(info.cloud->size());
  pcl::toROSMsg(*info.cloud, *info.output);
  info.output->header.frame_id = "world";
}
}  // namespace

// 随机地图生成节点
class MockamapNode : public rclcpp::Node
{
public:
  MockamapNode() : Node("mockamap_node")
  {
    // 读取参数：随机种子、发布频率、分辨率、三个方向的尺寸、地图类型
    const int seed = declare_parameter<int>("seed", 4546);
    const double update_frequency = declare_parameter<double>("update_freq", 1.0);
    const double resolution = declare_parameter<double>("resolution", 0.38);
    const int size_x = declare_parameter<int>("x_length", 100);
    const int size_y = declare_parameter<int>("y_length", 100);
    const int size_z = declare_parameter<int>("z_length", 10);
    const int type = declare_parameter<int>("type", 1);
    // scale = 1/分辨率，用于把“米”换算成“格”
    const double scale = 1.0 / resolution;

    // 组装生成参数：尺寸换算成格子数，并把输出容器挂上去
    mocka::Maps::BasicInfo info;
    info.node = this;
    info.sizeX = static_cast<int>(size_x * scale);
    info.sizeY = static_cast<int>(size_y * scale);
    info.sizeZ = static_cast<int>(size_z * scale);
    info.seed = seed;
    info.scale = scale;
    info.output = &output_;
    info.cloud = &cloud_;
    // 真正开始生成地图（内部按 type 分派到不同算法）
    mocka::Maps map;
    map.setInfo(info);
    map.generate(type);
    // 生成的点云统一放在 world 坐标系下
    output_.header.frame_id = "world";

    // 发布话题 mock_map（launch 里会重映射成 /map_generator/global_cloud）
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "mock_map", rclcpp::QoS(1).reliable().transient_local());
    // 定时重复发布：这样 RViz 里始终能看到地图
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(0.1, update_frequency)),
        std::bind(&MockamapNode::publishMap, this));
    // 启动时先立刻发一次，不用等第一个定时周期
    publishMap();
  }

private:
  void publishMap()
  {
    // 每次发布前刷新时间戳（RViz 靠它判断数据是否是新的）
    output_.header.stamp = now();
    publisher_->publish(output_);
  }
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  sensor_msgs::msg::PointCloud2 output_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// 程序入口：初始化 ROS 并一直运行
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockamapNode>());
  rclcpp::shutdown();
  return 0;
}
