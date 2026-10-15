// ============================================================================
// 文件：include/maps.hpp —— 程序化地图生成器的接口
//
// 【它声明了什么】
//   Maps 类：四种地图生成算法（噪声地形 / 随机方块 / 2D 迷宫 / 3D 迷宫），
//             实现在 src/maps.cpp（那里有逐段注释）。
//   MazePoint 类：3D 迷宫算法内部用的“带最近两个核心点信息”的点。
// ============================================================================
#ifndef MAPS_HPP
#define MAPS_HPP

#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mocka {

class Maps {
public:
  // 【参数与结果】一次地图生成所需的全部输入输出都装在这个结构里
  typedef struct BasicInfo {
    // 节点指针（用来读参数、打日志）
    rclcpp::Node *node;
    // 地图在三个方向上的格子数（注意单位是“格”，不是米）
    int sizeX;
    int sizeY;
    int sizeZ;
    // 随机种子（保证每次生成的地图一样）
    int seed;
    // 米 ↔ 格 的换算系数（= 1/分辨率）
    double scale;
    // 输出：生成的 ROS 点云消息
    sensor_msgs::msg::PointCloud2 *output;
    // 输出：生成的 PCL 点云
    pcl::PointCloud<pcl::PointXYZ> *cloud;
  } BasicInfo;

  BasicInfo getInfo() const;
  void setInfo(const BasicInfo &value);

public:
  Maps();

public:
  // 【总入口】按 type 选择生成算法（1 噪声 / 2 随机方块 / 3 迷宫 / 4 3D 迷宫）
  void generate(int type);

private:
  BasicInfo info;

private:
  // 把 PCL 点云转成 ROS 消息
  void pcl2ros();

  // 四种生成算法的声明（实现在 maps.cpp）
  void perlin3D();
  void maze2D();
  void randomMapGenerate();
  void Maze3DGen();
  // 2D 迷宫的核心：递归分割法
  void recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi &maze);
  void recursizeDivisionMaze(Eigen::MatrixXi &maze);
  // 地图瘦身（声明了但当前未被调用）
  void optimizeMap();
};

// 3D 迷宫内部用的小结构：记录某个格点到最近两个“核心点”的距离和编号
class MazePoint {
private:
  // 这个格子本身的位置
  pcl::PointXYZ point;
  // 到最近 / 次近核心点的距离，以及它们的编号
  double dist1;
  double dist2;
  int point1;
  int point2;
  bool isdoor;

public:
  pcl::PointXYZ getPoint();
  int getPoint1();
  int getPoint2();
  double getDist1();
  double getDist2();
  void setPoint(pcl::PointXYZ p);
  void setPoint1(int p);
  void setPoint2(int p);
  void setDist1(double set);
  void setDist2(double set);
};

} // namespace mocka

#endif // MAPS_HPP
