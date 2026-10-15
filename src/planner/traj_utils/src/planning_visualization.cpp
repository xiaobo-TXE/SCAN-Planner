// ============================================================================
// 文件：src/planning_visualization.cpp —— RViz 可视化
//
// 【这个文件是干什么的】
//   把规划过程中的各种东西画到 RViz 里：目标点、全局路径、初始路径、
//   最优轨迹、A* 绕行路径。
//   它只负责“画图”，不参与任何计算。
//
// 【五个话题】
//   goal_point   目标点（白球）
//   global_list  全局参考路径（青色）
//   init_list    优化前的初始路径（蓝色）
//   optimal_list 优化后的最优轨迹（颜色随速度变化）
//   a_star_list  A* 绕行路径（随机颜色）
// ============================================================================
#include <traj_utils/planning_visualization.h>
#include <cmath>
#include <limits>
#include <std_msgs/msg/color_rgba.hpp>

using std::cout;
using std::endl;
namespace scan_planner
{
  PlanningVisualization::PlanningVisualization(rclcpp::Node *node)
  {
    // 记住节点指针（发布 Marker 要用）
    node_ = node;
    // 统一的 QoS：可靠传输 + transient_local（晚连上的 RViz 也能看到最后一帧）
    const auto marker_qos = rclcpp::QoS(20).reliable().transient_local();
    // 创建五个发布器，分别对应五类可视化内容
    goal_point_pub = node_->create_publisher<visualization_msgs::msg::Marker>("goal_point", marker_qos);
    global_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("global_list", marker_qos);
    init_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("init_list", marker_qos);
    optimal_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("optimal_list", marker_qos);
    a_star_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("a_star_list", marker_qos);
  }

  // // real ids used: {id, id+1000}
  void PlanningVisualization::displayMarkerList(const MarkerPublisher::SharedPtr &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id)
  {
    // 同一组点要画两份：小球（SPHERE_LIST）和折线（LINE_STRIP）
    visualization_msgs::msg::Marker sphere, line_strip;
    // 坐标框架统一用 world，时间戳取当前时间
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = node_->now();
    // sphere 用来画点，line_strip 用来连线
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
    // 两个 Marker 用不同 id（约定：线用 id+1000），避免互相覆盖
    sphere.id = id;
    line_strip.id = id + 1000;

    // 单位旋转（不做旋转）
    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    // 颜色统一设置；如果透明度是 0，就自动补成 1（否则看不见）
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    // 小球大小与线的粗细
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::msg::Point pt;
    // 把列表里每个点同时加进两个 Marker
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    // 分别把小球和折线发布出去
    pub->publish(sphere);
    pub->publish(line_strip);
  }

  // real ids used: {id, id+1}
  // 把路径攒进 MarkerArray（不立即发布）—— 本项目当前没有被调用
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    // 与上面的 displayMarkerList 类似，只是坐标框架用 map，且折线更细
    visualization_msgs::msg::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "map";
    sphere.header.stamp = line_strip.header.stamp = node_->now();
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    // 把两个 Marker 加进数组
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  // 生成一串箭头：每两个点连成一个箭头 —— 本项目当前没有被调用
  void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    // 箭头 Marker
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = node_->now();
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    // 颜色与箭头尺寸
    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    // 每两个点组成一个箭头（起点、终点）
    geometry_msgs::msg::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);
      // 复用同一个 Marker 对象，只改起点/终点和 id
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      // id 用“序号 + 基数×1000”，保证每个箭头互不覆盖
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }

  // 画一个目标点：一个白色的小球
  void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    // 球体 Marker
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = node_->now();
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    // 颜色固定白色，透明度由参数决定
    sphere.color.r = 1.0;
    sphere.color.g = 1.0;
    sphere.color.b = 1.0;
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    // 把球放到目标点位置
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    // 发布
    goal_point_pub->publish(sphere);
  }

  // 画“全局参考路径”（青色）
  void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    // 没有订阅者就不画（省算力）
    if (global_list_pub->get_subscription_count() == 0)
    {
      return;
    }

    // 青色
    Eigen::Vector4d color(0, 0.5, 0.5, 1);
    // 交给通用函数 displayMarkerList 去画
    displayMarkerList(global_list_pub, init_pts, scale, color, id);
  }

  // 画“优化前的初始路径”（蓝色）
  void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    // 没有订阅者就跳过
    if (init_list_pub->get_subscription_count() == 0)
    {
      return;
    }

    // 蓝色
    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(init_list_pub, init_pts, scale, color, id);
  }

  // 用控制点矩阵画最优轨迹（红色）—— 本项目的调用处被注释掉了，仅备用
  void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
  {

    // 没有订阅者就跳过
    if (optimal_list_pub->get_subscription_count() == 0)
    {
      return;
    }

    // 把矩阵的每一列（一个控制点）转成三维向量
    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
      Eigen::Vector3d pt = optimal_pts.col(i).transpose();
      list.push_back(pt);
    }
    // 红色
    Eigen::Vector4d color(1, 0, 0, 1);
    displayMarkerList(optimal_list_pub, list, 0.15, color, id);
  }

  // 【实际在用的】把 B 样条轨迹采样后画出来；点的颜色随速度变化（绿=慢 → 红=快）
  void PlanningVisualization::displayOptimalTraj(UniformBspline position_traj, int id)
  {
    // 没有订阅者就跳过
    if (optimal_list_pub->get_subscription_count() == 0)
    {
      return;
    }

    // 轨迹总时长；太短就没必要画
    const double duration = position_traj.getTimeSum();
    if (duration < 1e-6)
    {
      return;
    }

    // 顺便取速度曲线，用来给每个点上色
    UniformBspline velocity_traj = position_traj.getDerivative();
    // 采样点个数：大约每 0.1 秒取一个点
    const int sample_num = std::max(2, static_cast<int>(std::ceil(duration / 0.1)) + 1);

    std::vector<Eigen::Vector3d> points;
    std::vector<double> speeds;
    points.reserve(sample_num);
    speeds.reserve(sample_num);

    // 记录速度的最小值和最大值，用于把速度归一化成颜色
    double min_speed = std::numeric_limits<double>::max();
    double max_speed = 0.0;
    // 均匀采样
    for (int i = 0; i < sample_num; ++i)
    {
      // 该采样点对应的时刻
      const double t = duration * static_cast<double>(i) / static_cast<double>(sample_num - 1);
      Eigen::Vector3d pos = position_traj.evaluateDeBoorT(t);
      // 该时刻的速度大小
      const double speed = velocity_traj.evaluateDeBoorT(t).norm();

      points.push_back(pos);
      speeds.push_back(speed);
      // 更新速度范围
      min_speed = std::min(min_speed, speed);
      max_speed = std::max(max_speed, speed);
    }

    // 同样要画“小球 + 折线”两份
    visualization_msgs::msg::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = node_->now();
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;
    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    // 小球大小与线宽
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.08;
    line_strip.scale.x = 0.08;

    // 速度跨度（防止除零）
    const double speed_range = std::max(1e-6, max_speed - min_speed);
    for (size_t i = 0; i < points.size(); ++i)
    {
      // 把点填进两个 Marker
      geometry_msgs::msg::Point pt;
      pt.x = points[i](0);
      pt.y = points[i](1);
      pt.z = points[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);

      // 把速度归一化到 0~1
      const double ratio = std::min(1.0, std::max(0.0, (speeds[i] - min_speed) / speed_range));
      // 颜色按速度插值：绿(慢) → 黄 → 红(快)
      std_msgs::msg::ColorRGBA color;
      color.a = 1.0;
      color.r = 1.0;
      color.g = 1.0 - ratio;
      color.b = 0.0;
      sphere.colors.push_back(color);
      line_strip.colors.push_back(color);
    }

    // 发布带颜色渐变的轨迹
    optimal_list_pub->publish(sphere);
    optimal_list_pub->publish(line_strip);
  }

  // 把 A* 找到的绕行路径画出来（每段一个随机颜色，方便区分是哪一段）
  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {

    // 没有订阅者就跳过
    if (a_star_list_pub->get_subscription_count() == 0)
    {
      return;
    }

    int i = 0;
    vector<Eigen::Vector3d> list;

    // 随机颜色：让每次显示的 A* 路径颜色不同，便于分辨
    Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star paths different every time.
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    // for ( int i=0; i<10; i++ )
    // {
    //   //Eigen::Vector4d color(1,1,0,0);
    //   displayMarkerList(a_star_list_pub, list, scale, color, id+i);
    // }

    // 每一段 A* 路径单独画一次
    for (auto block : a_star_paths)
    {
      list.clear();
      for (auto pt : block)
      {
        list.push_back(pt);
      }
      //Eigen::Vector4d color(0.5,0.5,0,1);
      // id 递增，避免各段之间互相覆盖
      displayMarkerList(a_star_list_pub, list, scale, color, id + i); // real ids used: [ id ~ id+a_star_paths.size() ]
      i++;
    }
  }

  // 画箭头列表（本项目当前没有被调用）
  void PlanningVisualization::displayArrowList(const MarkerArrayPublisher::SharedPtr &pub,
                                                const vector<Eigen::Vector3d> &list,
                                                double scale, Eigen::Vector4d color, int id)
  {
    // 先发一个空数组，相当于“清掉上一次画的箭头”
    visualization_msgs::msg::MarkerArray array;
    // clear
    pub->publish(array);

    // 生成箭头
    generateArrowDisplayArray(array, list, scale, color, id);

    // 再发布
    pub->publish(array);
  }

  // PlanningVisualization::
} // namespace scan_planner
