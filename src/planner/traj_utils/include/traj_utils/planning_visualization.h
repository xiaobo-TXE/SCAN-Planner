// ============================================================================
// 文件：include/traj_utils/planning_visualization.h
//
// 【这个文件是干什么的】
//   声明“可视化”类 PlanningVisualization：把规划过程画到 RViz 里。
//   实现在 src/planning_visualization.cpp（那里有逐段注释）。
// ============================================================================
#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline_opt/uniform_bspline.h>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <traj_utils/polynomial_traj.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <stdlib.h>

using std::vector;
namespace scan_planner
{
  class PlanningVisualization
  {
  private:
    // 两个类型别名，省得每次都写一长串 Publisher 类型
    using MarkerPublisher = rclcpp::Publisher<visualization_msgs::msg::Marker>;
    using MarkerArrayPublisher = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>;
    // 节点指针
    rclcpp::Node *node_{nullptr};

    // 五个发布器：目标点、全局路径、初始路径、最优轨迹、A* 路径
    MarkerPublisher::SharedPtr goal_point_pub;
    MarkerPublisher::SharedPtr global_list_pub;
    MarkerPublisher::SharedPtr init_list_pub;
    MarkerPublisher::SharedPtr optimal_list_pub;
    MarkerPublisher::SharedPtr a_star_list_pub;

  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    // 构造函数：创建上面这五个发布器（实现在 .cpp）
    explicit PlanningVisualization(rclcpp::Node *node);

    // 智能指针别名，方便其它类持有它
    typedef std::shared_ptr<PlanningVisualization> Ptr;

    // ---------- 下面是各种“画图”接口（详见 .cpp） ----------
    // frame_id：这批 Marker 要挂在哪个坐标系下显示。默认 world；
    // navi_mode=3 时会传参考坐标系（比如 map），这样路径才会画在正确的位置上。
    void displayMarkerList(const MarkerPublisher::SharedPtr &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id, const std::string &frame_id = "world");
    void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    // 画一个目标点（白色小球）
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id,
                          const std::string &frame_id = "world");
    // 画全局参考路径（青色）
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id,
                               const std::string &frame_id = "world");
    // 画优化前的初始路径（蓝色）
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    // 【实际在用的】画优化后的 B 样条轨迹，颜色随速度变化
    void displayOptimalTraj(UniformBspline position_traj, int id);
    // 画 A* 找到的绕行路径（每段随机颜色）
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(const MarkerArrayPublisher::SharedPtr &pub, const vector<Eigen::Vector3d> &list,
                          double scale, Eigen::Vector4d color, int id);
    // void displayIntermediateState(ros::Publisher& intermediate_pub, scan_planner::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration);
    // void displayNewArrow(ros::Publisher& guide_vector_pub, scan_planner::BsplineOptimizer::Ptr optimizer);
  };
} // namespace scan_planner
#endif
