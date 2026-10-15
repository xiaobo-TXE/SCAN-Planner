// ============================================================================
// 文件：include/plan_env/grid_map.h
//
// 【这个文件是干什么的】
//   定义“栅格地图”类 GridMap。它把机器人周围的 3D 空间切成一个个小立方体
//   （叫 voxel，体素），每格记录“这里被障碍占据的可能性有多大”。
//   它还要回答两个最常用的问题：
//     “这个位置能站人吗？”（避障查询）
//     “机器人现在周围长什么样？”（给 RViz 显示）
//
// 【它怎么知道哪里是障碍】
//   传感器每扫一次，就沿着每条激光束走一遍：打中的格子概率上调，
//   沿途经过的格子概率下调。多次观测累积后，概率高的格子就算障碍。
//
// 【重要：本项目没有 ESDF（距离场）】
//   碰撞检测只用“占据概率 + 预先膨胀”，不计算到障碍的精确距离。
//
// 【机器人形状：双圆柱】
//   机器人被看成前后两个圆柱。查询碰撞时要给定朝向 yaw，
//   算法会分别检查前后两个圆柱中心所在的格子。
//
// 【结构导航】
//   MappingParameters：所有可调参数（分辨率、概率、膨胀尺寸……）
//   MappingData      ：运行时数据（占用缓冲区、各种临时变量……）
//   GridMap          ：真正的方法都在这（查询、更新、发布）
// ============================================================================
#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <random>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <tuple>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>

// logit 是概率 ↔ log-odds 的换算。用 log-odds 的好处是：
// “多次观测累加”可以直接做加法，比直接乘概率更稳、更不容易溢出。
#define logit(x) (log((x) / (1 - (x))))

using namespace std;
// voxel hashing
template <typename T>
// 给 Eigen 向量做哈希的小工具（本项目当前没有真正用到）。
struct matrix_hash {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// constant parameters

// ============================ 参数区 ============================
// 所有可调参数都放这里，由 initMap() 从 YAML 读入。
// 下面按用途分组：地图尺寸 / 膨胀 / 深度相机 / 概率模型 / 其它。
// ===============================================================
struct MappingParameters {

  /* map properties */
  /* ---------- 地图几何：原点 / 尺寸 / 边界 / 分辨率 ---------- */
  // 地图原点（世界坐标，米）：水平方向以原点为中心，竖直方向从 ground_height 开始
  Eigen::Vector3d map_origin_, map_size_;
  // 地图在世界坐标下的最小/最大边界（由 map_origin_ + map_size_ 算出），用于 isInMap 判断
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
  // 三个方向各有几个格子（= 尺寸 ÷ 分辨率），是缓冲区的形状
  Eigen::Vector3i map_voxel_num_;                        // map range in index
  // 地图边界的格子编号（最小/最大），isInMap 用索引判断时用
  Eigen::Vector3i map_bound_min_idx_, map_bound_max_idx_;
  // 地图"中心格子"的编号：滑动地图以它为中心，机器人移动后整体平移
  Eigen::Vector3i map_origin_idx_;
  // 局部更新范围（米）：只处理传感器周围这一块，范围外的旧数据会被清掉
  Eigen::Vector3d local_update_range_;
// 分辨率(米/格) 及其倒数（倒数用于“坐标→格子编号”的快速换算）
  double resolution_, resolution_inv_;
  // 障碍膨胀的竖直范围（米）：向上/向下各扩多少；上下不对称（下方障碍更危险）
  double obstacles_inflation_z_up, obstacles_inflation_z_down;
// 机器人形状参数：圆柱半径、前后两个圆柱中心离机器人中心的距离
  // 机器人被抽象成"前后两个圆柱"：radius 是圆柱半径，offset 是两圆心离中心的距离
  double double_cylinder_radius_, double_cylinder_offset_;
  // 是否启用滑动地图，以及“机器人移动多远才滑动一次”的阈值
  // map_sliding_thresh 单位是米；map_sliding_thresh_vox_ 是它换算成的格子数
  bool map_sliding_en_;
  double map_sliding_thresh_;
  int map_sliding_thresh_vox_;
  // 地图所在的世界坐标系名 / 滑动地图坐标系名（发布 TF 和 RViz 显示用）
  string frame_id_, sliding_map_frame_id_;

  /* depth camera intrinsics */
  /* ---------- 深度相机内参（sensor_type=depth 时用） ---------- */
  // 光心 (cx,cy) 与焦距 (fx,fy)，用于把深度图像素反投影成 3D 点
  double cx_, cy_, fx_, fy_;

  /* depth image projection filtering */
  /* ---------- 深度图投影过滤（sensor_type=depth 时用） ---------- */
  // 超过 maxdist 视为无遮挡、小于 mindist 视为噪点忽略；margin 是忽略的边缘像素数
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  // 深度原始值→米的缩放系数（16 位深度常用 1000，即毫米）
  double k_depth_scaling_factor_;
  // 每隔几个像素取一个点（越大越快、点越稀）
  int skip_pixel_;

  /* raycasting */
// 概率模型：p_hit 命中时上调多少、p_miss 未命中时下调多少，
// p_min/p_max 是概率的上下限，p_occ 是“判定为障碍”的门槛。
  /* ---------- 占据概率模型（建图判障碍的核心） ---------- */
  // p_hit: 光束命中时概率上调量; p_miss: 未命中(途经)时下调量
  // p_min/p_max: 概率上下限; p_occ: 超过它就算障碍（门槛）
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  // 上面五个概率预先换算成 log-odds（对数几率）：命中/未命中的增量、上下限、障碍门槛
  // 用 log-odds 的好处是"多次观测累加"可以直接做加法，比直接乘概率更稳
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                   // logit of occupancy probability
  // 一条射线的作用距离范围（米）：太近/太远都不参与建图
  double min_ray_length_, max_ray_length_;  // range of doing raycasting

  /* visualization and computation time display */
  /* ---------- 可视化与耗时显示 ---------- */
  // vis_height: RViz 里障碍显示的高度上限(相对传感器); ground_height: 地面高度
  double vis_height_, ground_height_;
  // 是否打印地图融合耗时
  bool show_occ_time_;

  /* mapping sensor input */
  /* ---------- 传感器输入方式 ---------- */
  // 传感器类型：lidar(激光点云) 或 depth(深度相机)，决定订阅哪几个话题
  string sensor_type_;
// 点云是否已经是世界坐标（真机通常给的是传感器坐标系，需要转换）
  // cloud_is_world_=true: 点云已是世界系(仿真)，直接用; false: 需用传感器位姿转到世界系(真机)
  bool cloud_is_world_;
  // 是否需要叠加"传感器安装外参"(真机=true)：把传感器位姿再修正到激光/相机的实际位置
  bool need_extrinsic_;
  // 激光/相机相对机体的安装外参矩阵（代码里写死，不从参数读）
  Eigen::Matrix4d lidar_extrinsic_;
  Eigen::Matrix4d depth_extrinsic_;

  /* active mapping */
  // 用一个"比概率下限再低一点点"的值表示"这一格还没被观测过"(未知)
  double unknown_flag_;
};

// intermediate mapping data for fusion

// ============================ 数据区 ============================
// 运行时使用的各种缓冲区。最核心的是下面三个：
//   occupancy_buffer_        每格的对数占据概率（地图本体）
//   occupancy_buffer_inflate_ 膨胀后的 0/1 标志（避障查询直接查它）
//   occupancy_buffer_inflate_cnt_ 引用计数（防止重叠障碍互相误清除）
// ===============================================================
struct MappingData {
  // main map data, occupancy of each voxel and Euclidean distance

// 每个体素存一个“对数占据概率”（log-odds），这是地图的真正内容
  // （用一维数组存放整张三维地图，靠 toAddress 把三维编号压成一维下标）
  std::vector<double> occupancy_buffer_;
// 膨胀后的障碍标志：0=可通行，1=障碍（已向外扩张过机器人半径）
  // 碰撞查询 getInflateOccupancy 直接读这块，而不是读上面的概率
  std::vector<char> occupancy_buffer_inflate_;
// 引用计数：某一格被多少个障碍膨胀覆盖。归零才把标志清掉，避免“误清”。
  std::vector<int> occupancy_buffer_inflate_cnt_;
  // 预计算的"膨胀偏移表"：机器人形状(双圆柱)对应的所有格子偏移，建一次反复用
  vector<Eigen::Vector3i> inflate_offsets_;

  // raycast origin and sensor pose data
  /* ---------- 射线起点 / 传感器位姿 ---------- */

  // 传感器(激光/相机)在世界系下的位置与姿态；射线投射的起点
  Eigen::Vector3d ray_pos_;
  Eigen::Quaterniond ray_q_;
  // 机器人位置（供发布 world→sliding_map 的 TF 用）
  Eigen::Vector3d sliding_map_frame_pos_;

  // depth image data
  /* ---------- 深度图数据（depth 模式） ---------- */

  // 当前缓存的深度图；image_cnt_ 是收到的帧计数
  cv::Mat depth_image_;
  int image_cnt_;
  // flags of map state
  /* ---------- 地图状态标志 ---------- */

  // occ_need_update_: 有新数据待融合; use_cloud_update_: 本帧是点云(而非深度图)更新
  // has_first_depth_: 是否已收到第一帧深度(首帧常不完整，跳过); has_ray_pose_/has_cloud_: 是否已收到位姿/点云
  bool occ_need_update_;
  bool use_cloud_update_;
  bool has_first_depth_;
  bool has_ray_pose_, has_cloud_;

  // depth image projected point cloud
  /* ---------- 观测点缓存 ---------- */

  // 本帧待处理的观测点(世界坐标)：深度反投影点 或 激光点云转换后的点
  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;

  // flag buffers for speeding up raycasting
  /* ---------- 加速射线遍历的临时数组（每帧复用，避免重复处理同一格） ---------- */

  // count_hit_: 被"命中"的次数; count_hit_and_miss_: 被观测(命中或途经)的总次数
  vector<short> count_hit_, count_hit_and_miss_;
  // flag_traverse_/flag_rayend_: 记录"本轮射线遍历"是否已经走过这一格
  vector<char> flag_traverse_, flag_rayend_;
  // raycast_num_ 是"本轮射线遍历"的编号（每帧+1，用于区分是否同一轮）
  char raycast_num_;
  // 待更新格子的队列：本帧打过标记的格子入队，最后统一更新概率
  queue<Eigen::Vector3i> cache_voxel_;

  // range of updating grid
  /* ---------- 本次更新的格子范围（包围盒） ---------- */

  Eigen::Vector3i local_bound_min_, local_bound_max_;

  // computation time
  /* ---------- 耗时统计 ---------- */

  // 地图融合耗时统计（平均/最大），update_num_ 为更新次数
  double fuse_time_, max_fuse_time_;
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// 地图类本体：查询、更新、发布都在这。
class GridMap {
public:
  GridMap() {}
  ~GridMap() {}

// 用一个很怪的数表示“无效格子/越界”，便于快速判断。
  // setCacheOccupancy 等函数在越界时返回它，调用方用它区分"越界"和"有效地址"
  enum { INVALID_IDX = -10000 };

  // occupancy map management
  /* ---------- 重置地图 ---------- */
  // 无参版：整张地图清空；带边界版：只清空一个长方体范围
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

// ---------- 坐标与索引换算 ----------
// 世界坐标(米)  ↔  格子编号(整数)：这是整个地图最基础的两个小工具。
  // 世界坐标 → 格子编号（floor(坐标/分辨率)）
  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  // 格子编号 → 世界坐标（格子中心）
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  // 格子编号 → 一维内存地址（内部含取模，支持滑动地图的环形缓冲）
  inline int toAddress(const Eigen::Vector3i& id);
  inline int toAddress(int& x, int& y, int& z);
  // 判断一个坐标/索引是否在地图范围内
  inline bool isInMap(const Eigen::Vector3d& pos);
  inline bool isInMap(const Eigen::Vector3i& idx);

  // 手动设置某格为占据/空闲（occ>0.5 视为障碍），会同步更新膨胀层
  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline void setOccupied(Eigen::Vector3d pos);
  // 查某格的"本体概率"是否超过障碍门槛（0=空，1=障碍，-1=出图）
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
// ---------- 碰撞查询（规划器最常用） ----------
// 问：“机器人中心在 pos、朝向 yaw 时，会不会撞到障碍？”
// 实现：分别检查前后两个圆柱中心所在的格子。
// 返回 1=有障碍，0=空，-1=超出地图范围。
  inline int getInflateOccupancy(Eigen::Vector3d pos, double yaw);

  // 把一个索引夹到地图边界内（越界时就近取边界）
  inline void boundIndex(Eigen::Vector3i& id);
  // 判断某格是否"从未被观测过"（未知）
  inline bool isUnknown(const Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3d& pos);
  // 已观测且空闲 / 已判定为障碍
  inline bool isKnownFree(const Eigen::Vector3i& id);
  inline bool isKnownOccupied(const Eigen::Vector3i& id);

// ---------- 生命周期 ----------
// 初始化：读所有参数、分配缓冲区、订阅话题、创建定时器和发布器。
  void initMap(rclcpp::Node* node);

  // 把占据格 / 膨胀格导出成点云发给 RViz（都在 visCallback 里定时调用）
  void publishMap();
  void publishMapInflate(bool all_info = false);

  // 发布未知区域、深度投影点云、滑动地图盒子 / 坐标系
  void publishUnknown();
  void publishDepth();
  void publishDepthCloud();
  void publishSlidingMapBBox();
  void publishSlidingMapFrame();

  bool hasDepthObservation();
  bool odomValid();
  // 取出地图原点与尺寸
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  inline double getResolution();
  Eigen::Vector3d getOrigin();
  int getVoxelNum();

  // 智能指针别名：外部用它持有并自动释放本对象
  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
// 参数与数据两个成员：mp_ 存参数，md_ 存运行时数据。
  MappingParameters mp_;
  MappingData md_;

  // get depth image and sensor pose
  /* ---------- 传感器数据回调（收到话题/定时器时被调用） ---------- */
  // depth 模式：深度图 + 传感器位姿（已按时间对齐）→ 反投影建图
  void depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                         const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  // lidar 模式：记录激光传感器位姿（只记录，不请求更新）
  void sensorPoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  // 记录机器人位置，供发布 world→sliding_map 的 TF
  void slidingMapFrameCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  // lidar 模式：收到点云 → 转到世界系、筛点、请求一次地图更新
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& img);

  // update occupancy by raycasting
// ---------- 两个定时器回调 ----------
// 每 50ms 执行一次：把新收到的传感器数据融合进地图（核心更新）。
  // 地图更新入口：若无新数据直接返回；否则 projectDepthImage()/raycastProcess()
  void updateOccupancyCallback();
  // 每 50ms 把地图内容发布给 RViz（无人订阅则内部跳过）
  void visCallback();

  // main update process
// 深度相机模式：把深度图里每个像素反投影成 3D 点（世界坐标）
  void projectDepthImage();
// 【地图更新的核心】沿着每条光束做“体素遍历”：
// 打中的格子标记为命中，途经的格子标记为空闲，最后统一更新概率。
  void raycastProcess();

  // 把一个格子按给定步数展开成"膨胀后的点集"（单点膨胀工具）
  inline void inflatePoint(const Eigen::Vector3i& pt, int inf_step_xy, int inf_step_z_up, int inf_step_z_down, vector<Eigen::Vector3i>& pts);
  // 在给定缓冲区(通常是膨胀标志)里查一个位置，返回 0/1/-1(出图)
  inline int getInflateOccupancyFromBuffer(Eigen::Vector3d pos, const std::vector<char>& buffer);
  // 把全局格子编号取模成"缓冲区内的局部编号"（滑动地图环形缓冲用）
  inline int getLocalIndex(int id, int dim) const;
  // 由局部编号算一维地址（滑动地图版本）
  inline int toAddressLocal(const Eigen::Vector3i& id_l) const;
  inline int toAddressLocal(int x, int y, int z) const;
  // 射线遍历时给某格打标记：occ=1 命中 / occ=0 途经；返回格子地址或 INVALID_IDX
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  // 把跑到地图外的观测点拉回到地图盒子边界上（求射线与盒子的交点）
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& ray_pos);
// 滑动地图：机器人移动超过阈值时，把地图盒子跟着挪，
// 并清掉移出盒子的那些格子（不用重建整张地图）。
  void updateSlidingMap(const Eigen::Vector3d& center);
  // 由"中心格子编号"反算地图边界（滑动后调用）
  void updateMapBoundaryFromIndex();
  // 整张地图清空回"全未知"
  void resetAllMapData();
  // 把某一格重置为"未知"，并撤销它的膨胀贡献
  void resetCellByAddress(int addr);
  // 滑动时的版本：撤销膨胀时用掩码避开"仍留在盒内"的格子
  void resetCellByAddressForSliding(int addr, const std::vector<char>& clear_mask);
  // 把"一维地址"还原成"全局格子编号"（考虑滑动取模回绕）
  void hashIdToGlobalIndex(int addr, Eigen::Vector3i& id_g) const;
  // 写一格的新概率；若"障碍/非障碍"翻转，则同步 ± 更新膨胀层
  void applyOccupancyUpdate(const Eigen::Vector3i& id, double new_log_odds);
// 预计算“膨胀偏移表”：一次算好圆柱形状对应的一堆格点偏移，
// 之后每次膨胀直接查表，速度快很多。
  void rebuildInflationOffsets();
  // 更新某格的膨胀层：按偏移表给周围格计数 += delta
  void updateInflation(const Eigen::Vector3i& id, int delta, const std::vector<char>* ignore_mask = nullptr);
  // 膨胀层核心：遍历偏移表，引用计数 cnt += delta，cnt>0 则标志置 1
  void updateInflationLayer(const Eigen::Vector3i& id, int delta,
                            const vector<Eigen::Vector3i>& offsets,
                            std::vector<int>& cnt_buffer,
                            std::vector<char>& flag_buffer,
                            const std::vector<char>* ignore_mask);

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImagePose;
  // depth 模式用"近似时间同步"把深度图与位姿配对
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;

  // 节点指针（读参数、建话题、发日志用）
  rclcpp::Node* node_{nullptr};
  // 发布 world→sliding_map 的 TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // depth 模式的两路输入（深度图 + 位姿）及其时间同步器
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> depth_pose_sub_;
  SynchronizerImagePose sync_image_pose_;

  // lidar 模式的输入：传感器位姿 + 点云
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_pose_sub_;
  // 机器人位姿（滑动地图用）
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sliding_map_frame_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  // 发布器：占据点云 / 膨胀点云 / 滑动地图盒子 / 未知区域 / 深度投影 / 加外参后的传感器位姿
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inf_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sliding_map_bbox_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr extrinsic_pose_pub_;
  // occ_timer_: 50ms 融合地图；vis_timer_: 50ms 发布可视化
  rclcpp::TimerBase::SharedPtr occ_timer_, vis_timer_;

  //
  // 噪声发生器（本项目当前未使用，保留接口）
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
};

/* ============================== definition of inline function
 * ============================== */

// 由三维格子编号算一维内存地址（滑动地图版：先取模回绕）。
// 行优先展开：addr = x*Ny*Nz + y*Nz + z，其中 x/y/z 是取模后的"局部编号"。
inline int GridMap::toAddress(const Eigen::Vector3i& id) {
  return getLocalIndex(id(0), 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(id(1), 1) * mp_.map_voxel_num_(2) + getLocalIndex(id(2), 2);
}

// 同上，只是参数是分开的 x/y/z
inline int GridMap::toAddress(int& x, int& y, int& z) {
  return getLocalIndex(x, 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(y, 1) * mp_.map_voxel_num_(2) + getLocalIndex(z, 2);
}

// 把全局格子编号取模成"缓冲区内部的局部编号"。
// 这是滑动地图的关键：地图盒子平移后，旧格子靠取模"看起来还在原位"；
// 负号要补正，保证结果落在 [0, voxel_num)。
inline int GridMap::getLocalIndex(int id, int dim) const {
  int local_id = id % mp_.map_voxel_num_(dim);
  if (local_id < 0) local_id += mp_.map_voxel_num_(dim);
  return local_id;
}

// 由"局部编号"算一维地址（不做取模，输入已经是局部编号时用）
inline int GridMap::toAddressLocal(const Eigen::Vector3i& id_l) const {
  return id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id_l(1) * mp_.map_voxel_num_(2) + id_l(2);
}

inline int GridMap::toAddressLocal(int x, int y, int z) const {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

// 把一个索引夹进地图边界内：小于下限取下限、大于上限取上限（就近截断）
inline void GridMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_bound_max_idx_(0)), mp_.map_bound_min_idx_(0));
  id1(1) = max(min(id(1), mp_.map_bound_max_idx_(1)), mp_.map_bound_min_idx_(1));
  id1(2) = max(min(id(2), mp_.map_bound_max_idx_(2)), mp_.map_bound_min_idx_(2));
  id = id1;
}

// 某格是否"从未被观测过"：概率比下限还低（初始化时就是 clamp_min_log_-unknown_flag_）
inline bool GridMap::isUnknown(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

// 同上，输入是世界坐标
inline bool GridMap::isUnknown(const Eigen::Vector3d& pos) {
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

// 某格是否"已观测且空闲"：概率高于下限且膨胀标志为 0（即可通行）
inline bool GridMap::isKnownFree(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

// 某格是否"已判定为障碍"：直接看膨胀标志是否为 1
inline bool GridMap::isKnownOccupied(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

// 手动把某格设为"占据(障碍)"：写入概率上限，并同步更新膨胀层
inline void GridMap::setOccupied(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, mp_.clamp_max_log_);
}

// 手动设置某格占据/空闲：occ 只允许 0 或 1，否则报错
inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ) {
  if (occ != 1 && occ != 0) {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, occ > 0.5 ? mp_.clamp_max_log_ : mp_.clamp_min_log_);
}

// 查"本体概率"是否超过障碍门槛：出图返回 -1，否则 1=障碍 / 0=空
inline int GridMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

// 【碰撞查询核心】机器人中心在 pos、朝向 yaw 时会不会撞：
//   ① 按朝向算出前后两个圆柱圆心（机器人抽象成前后两个圆柱）
//   ② 先查前圆柱；只要不是 0（有障碍或出图）就返回
//   ③ 前圆柱没事再查后圆柱
// 返回 1=障碍 / 0=空 / -1=出图。注意：调用方常写 if(返回值)，所以 -1 也会被当成"会撞"。
inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos, double yaw) {
  Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  Eigen::Vector3d front = pos + mp_.double_cylinder_offset_ * heading;
  Eigen::Vector3d rear = pos - mp_.double_cylinder_offset_ * heading;

  int front_occ = getInflateOccupancyFromBuffer(front, md_.occupancy_buffer_inflate_);
  if (front_occ != 0) return front_occ;

  return getInflateOccupancyFromBuffer(rear, md_.occupancy_buffer_inflate_);
}

// 在给定缓冲区里查一个点：出图 → -1，否则返回该格的标志(0/1)
inline int GridMap::getInflateOccupancyFromBuffer(Eigen::Vector3d pos, const std::vector<char>& buffer) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(buffer[toAddress(id)]);
}

// 同上，但输入是格子编号
inline int GridMap::getOccupancy(Eigen::Vector3i id) {
  if (!isInMap(id))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

// 世界坐标是否在地图盒子内（三个轴分别和边界比较，留 1e-4 余量）
inline bool GridMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

// 索引是否在地图边界内（和上面的世界坐标版本等价）
inline bool GridMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < mp_.map_bound_min_idx_(0) || idx(1) < mp_.map_bound_min_idx_(1) ||
      idx(2) < mp_.map_bound_min_idx_(2)) {
    return false;
  }
  if (idx(0) > mp_.map_bound_max_idx_(0) || idx(1) > mp_.map_bound_max_idx_(1) ||
      idx(2) > mp_.map_bound_max_idx_(2)) {
    return false;
  }
  return true;
}

// 世界坐标 → 格子编号：id = floor(坐标 / 分辨率)，用倒数做乘法更快
inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor(pos(i) * mp_.resolution_inv_);
}

// 格子编号 → 世界坐标（取格子中心：编号 + 0.5 再乘分辨率）
inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_;
}

// 单点膨胀：以 pt 为中心，在 xy 平面半径 inf_step_xy 的圆内、z 范围 [-下, +上] 内，
// 枚举所有格子偏移，输出到 pts。（上面注释掉的是更省事的"+ 字形"膨胀，本项目用圆形全膨胀）
inline void GridMap::inflatePoint(const Eigen::Vector3i& pt, int inf_step_xy, int inf_step_z_up, int inf_step_z_down, vector<Eigen::Vector3i>& pts) {

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  // 先清空输出（调用方复用一个 vector，避免反复分配）
  pts.clear();
  // 在 xy 平面上遍历正方形，只保留落在半径圆内的格子
  for (int x = -inf_step_xy; x <= inf_step_xy; ++x)
    for (int y = -inf_step_xy; y <= inf_step_xy; ++y)
    {
      if (std::sqrt(x * x + y * y) > inf_step_xy)
        continue;

      // z 方向按"向上 / 向下"两个不同高度遍历（上下不对称）
      for (int z = -inf_step_z_down; z <= inf_step_z_up; ++z) {
        pts.push_back(Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z));
      }
    }
}

// 取地图分辨率（米/格）
inline double GridMap::getResolution() { return mp_.resolution_; }

#endif
