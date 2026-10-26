// ============================================================================
// 文件：src/grid_map.cpp —— 栅格地图的实现（本项目最“底层”的文件）
//
// 【一句话说明白它在干嘛】
//   把传感器每次扫到的数据，变成一张“哪里是障碍”的 3D 小方块地图；
//   并对外提供“这个位置能不能站”的查询。
//
// 【地图更新的三步曲】（第 743 行的 updateOccupancyCallback 是入口）
//   1. projectDepthImage()：深度图 → 一堆 3D 点（激光模式跳过这步）
//   2. raycastProcess()   ：从传感器出发，沿每条光束走格子，标记“命中/空闲”
//   3. applyOccupancyUpdate()：把标记累积成概率，跨过门槛就算障碍（并更新膨胀）
//
// 【建议阅读顺序】
//   initMap → updateOccupancyCallback → raycastProcess → getInflateOccupancy(在 .h)
// ============================================================================
#include "plan_env/grid_map.h"
#include <cmath>
#include <limits>
#include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace
{
template <typename T>
// 小工具：读一个参数。若参数未声明则先声明（用默认值），再把值取到 value。
void load_parameter(rclcpp::Node *node, const std::string &name, T &value, const T &default_value)
{
  if (!node->has_parameter(name))
    node->declare_parameter<T>(name, default_value);
  node->get_parameter(name, value);
}
}  // namespace

// ==========================================================================
// initMap：整个地图的初始化，只执行一次。
//   1. 读取全部参数（分辨率、地图尺寸、概率、膨胀、相机内参……）
//   2. 把概率换算成 log-odds，并算好各种边界/索引
//   3. 分配三块核心缓冲区（占据概率、膨胀标志、膨胀计数）
//   4. 订阅话题：深度图+位姿 或 点云+位姿；另订阅 body_pose 用于滑动地图
//   5. 创建两个 50ms 定时器（更新地图 / 发布可视化）
//   6. 创建发布器（占据点云、膨胀点云、地图盒子、未知区域……）
// ==========================================================================
void GridMap::initMap(rclcpp::Node *node)
{
  // 记住节点指针（读参数、发日志、建话题都要用）
  node_ = node;

  /* get parameter */
  // 下面这一大段：从 YAML 把所有参数读进 mp_（参数结构体）
  double x_size, y_size, z_size;
  // 分辨率、地图在三个方向上的尺寸
  load_parameter(node_, "grid_map.resolution", mp_.resolution_, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_x", x_size, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_y", y_size, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_z", z_size, -1.0);
  // 局部更新范围：只处理传感器周围这一块，默认取地图尺寸的一半
  load_parameter(node_, "grid_map.local_update_range_x", mp_.local_update_range_(0), x_size / 2.0);
  load_parameter(node_, "grid_map.local_update_range_y", mp_.local_update_range_(1), y_size / 2.0);
  load_parameter(node_, "grid_map.local_update_range_z", mp_.local_update_range_(2), z_size / 2.0);
  
  // 障碍膨胀的上下范围，以及机器人“双圆柱”的半径与偏移
  load_parameter(node_, "grid_map.obstacles_inflation_z_up", mp_.obstacles_inflation_z_up, -1.0);
  load_parameter(node_, "grid_map.obstacles_inflation_z_down", mp_.obstacles_inflation_z_down, -1.0);
  load_parameter(node_, "grid_map.double_cylinder_radius", mp_.double_cylinder_radius_, -1.0);
  load_parameter(node_, "grid_map.double_cylinder_offset", mp_.double_cylinder_offset_, 0.0);
  // footprint 模型选择（0=双圆柱[默认]，1=矩形多点采样）+ 矩形参数
  load_parameter(node_, "grid_map.footprint_model", mp_.footprint_model_, 0);
  load_parameter(node_, "grid_map.footprint_length", mp_.footprint_length_, 0.0);
  load_parameter(node_, "grid_map.footprint_width", mp_.footprint_width_, 0.0);
  load_parameter(node_, "grid_map.footprint_margin", mp_.footprint_margin_, 0.06);
  load_parameter(node_, "grid_map.footprint_sample_step", mp_.footprint_sample_step_, 0.1);
  load_parameter(node_, "grid_map.map_sliding_en", mp_.map_sliding_en_, true);
  load_parameter(node_, "grid_map.map_sliding_thresh", mp_.map_sliding_thresh_, mp_.resolution_);

  // 深度相机内参 fx / fy / cx / cy
  load_parameter(node_, "grid_map.fx", mp_.fx_, -1.0);
  load_parameter(node_, "grid_map.fy", mp_.fy_, -1.0);
  load_parameter(node_, "grid_map.cx", mp_.cx_, -1.0);
  load_parameter(node_, "grid_map.cy", mp_.cy_, -1.0);

  // 深度图过滤参数：太近忽略、太远视为无遮挡、每隔几像素取一个点
  load_parameter(node_, "grid_map.depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  load_parameter(node_, "grid_map.depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  load_parameter(node_, "grid_map.depth_filter_margin", mp_.depth_filter_margin_, -1);
  load_parameter(node_, "grid_map.k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  load_parameter(node_, "grid_map.skip_pixel", mp_.skip_pixel_, -1);

  // 占据概率模型的五个参数，以及单条射线的最大长度
  load_parameter(node_, "grid_map.p_hit", mp_.p_hit_, -1.0);
  load_parameter(node_, "grid_map.p_miss", mp_.p_miss_, -1.0);
  load_parameter(node_, "grid_map.p_min", mp_.p_min_, -1.0);
  load_parameter(node_, "grid_map.p_max", mp_.p_max_, -1.0);
  load_parameter(node_, "grid_map.p_occ", mp_.p_occ_, -1.0);
  load_parameter(node_, "grid_map.max_ray_length", mp_.max_ray_length_, -0.1);

  // 可视化高度（超过它的障碍不显示）、是否打印耗时
  load_parameter(node_, "grid_map.vis_height", mp_.vis_height_, 0.3);
  load_parameter(node_, "grid_map.show_occ_time", mp_.show_occ_time_, false);

  // 坐标框架名、滑动地图框架名、地面高度
  load_parameter(node_, "grid_map.frame_id", mp_.frame_id_, string("world"));
  load_parameter(node_, "grid_map.sliding_map_frame_id", mp_.sliding_map_frame_id_, string("sliding_map"));
  load_parameter(node_, "grid_map.ground_height", mp_.ground_height_, 0.0);

  // 传感器类型、点云是否已是世界系、是否需要外参
  load_parameter(node_, "grid_map.sensor_type", mp_.sensor_type_, string("lidar"));
  load_parameter(node_, "grid_map.cloud_is_world", mp_.cloud_is_world_, true);
  load_parameter(node_, "grid_map.need_extrinsic", mp_.need_extrinsic_, true);

  // 激光与相机的外参矩阵（这里直接写死在代码里，不从参数读）
  mp_.lidar_extrinsic_ <<
      1.0, 0.0, 0.0, -0.01100,
      0.0, 1.0, 0.0, -0.02329,
      0.0, 0.0, 1.0,  0.04412,
      0.0, 0.0, 0.0,  1.00000;

  mp_.depth_extrinsic_ <<
      0.0,  0.707107, 0.707107, -0.15170,
     -1.0,  0.000000, 0.000000,  0.00000,
      0.0, -0.707107, 0.707107,  0.07510,
      0.0,  0.000000, 0.000000,  1.00000;
      
  // 传感器类型只允许 lidar / depth，写错了就回退成 lidar
  if (mp_.sensor_type_ != "lidar" && mp_.sensor_type_ != "depth")
  {
    RCLCPP_ERROR(node_->get_logger(), "[GridMap] invalid grid_map.sensor_type: %s; falling back to lidar",
                 mp_.sensor_type_.c_str());
    mp_.sensor_type_ = "lidar";
  }

  // 分辨率的倒数（坐标与格子编号互转时用乘法更快）
  mp_.resolution_inv_ = 1 / mp_.resolution_;
  // 地图原点：让地图在水平方向以原点为中心，竖直方向从地面高度开始
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  // 把概率换算成 log-odds（对数几率）：这样“多次观测累加”就直接做加法
  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  // 用一个“比下限再低一点点”的值表示“这一格还没被观测过”
  mp_.unknown_flag_ = 0.01;
  // 滑动阈值换算成格子数（至少 1 格）
  mp_.map_sliding_thresh_vox_ = std::max(1, static_cast<int>(std::ceil(mp_.map_sliding_thresh_ * mp_.resolution_inv_)));

  // 打印几个关键概率值，方便确认参数读对了
  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  // 各个方向的格子总数 = 地图尺寸 ÷ 分辨率
  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  // 地图边界的上下限（世界坐标），以及对应的格子编号
  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
  // 顺便记下“中心格子”的编号（滑动地图以它为中心）
  posToIndex(mp_.map_origin_, mp_.map_bound_min_idx_);
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  mp_.map_origin_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ / 2;
  // 由中心格子反算边界，保证前后一致
  updateMapBoundaryFromIndex();

  // initialize data buffers

  // 缓冲区大小 = 三个方向格子数的乘积（整张地图存在一块连续内存里）
  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  // 三块核心缓冲区：占据概率、膨胀标志、膨胀计数
  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_cnt_ = vector<int>(buffer_size, 0);
  rebuildInflationOffsets();
  rebuildFootprintSamples();

  // 光线遍历用的临时计数与标记（每帧会复位）
  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  // “本轮光线遍历”的编号，用来避免同一格被重复处理
  md_.raycast_num_ = 0;

  // 预分配“投影点”数组（按 640×480 深度图、隔 skip_pixel 取点来估算容量）
  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  /* init callback */
  // 准备 TF 广播器（用于发布 world→sliding_map）
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*node_);

  // 深度模式：用 message_filters 把“深度图”和“传感器位姿”按时间戳配对
  if (mp_.sensor_type_ == "depth")
  {
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
    depth_pose_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
    depth_sub_->subscribe(node_, "depth", rmw_qos_profile_sensor_data);
    depth_pose_sub_->subscribe(node_, "sensor_pose", rmw_qos_profile_sensor_data);

    // 时间同步器：只有两路消息时间足够接近时，才触发一次回调
    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *depth_pose_sub_));
    sync_image_pose_->registerCallback(
        std::bind(&GridMap::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
  }
  // 激光模式：分别订阅“传感器位姿”和“点云”两路话题
  else if (mp_.sensor_type_ == "lidar")
  {
    lidar_pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "sensor_pose", rclcpp::SensorDataQoS(),
        std::bind(&GridMap::sensorPoseCallback, this, std::placeholders::_1));
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud", rclcpp::SensorDataQoS(),
        std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));
  }

  // 两种模式都要订阅 body_pose，用于让滑动地图跟随机器人
  sliding_map_frame_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "body_pose", rclcpp::SensorDataQoS(),
      std::bind(&GridMap::slidingMapFrameCallback, this, std::placeholders::_1));

  // 50ms 定时器：融合传感器数据、更新地图
  occ_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&GridMap::updateOccupancyCallback, this));
  // 50ms 定时器：把地图内容发出去给 RViz 显示
  vis_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&GridMap::visCallback, this));

  // 发布器：占据点云、膨胀点云、地图盒子
  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", rclcpp::SensorDataQoS());
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", rclcpp::SensorDataQoS());
  sliding_map_bbox_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("grid_map/sliding_map_bbox", 10);

  // 发布器：未知区域点云、深度投影点云、传感器外参位姿
  unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/unknown", rclcpp::SensorDataQoS());
  depth_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/depth_cloud", rclcpp::SensorDataQoS());
  extrinsic_pose_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("grid_map/sensor_pose_extrinsic", 10);

  // 下面把运行时状态全部复位（此时还没收到任何数据）
  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_ray_pose_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.ray_pos_.setZero();
  md_.sliding_map_frame_pos_.setZero();
  md_.ray_q_ = Eigen::Quaterniond::Identity();

  // 计时统计量复位；把“局部更新范围”先设为整张地图
  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

// 根据“中心格子的编号”反算出地图的上下边界。滑动地图移动后要调用它。
void GridMap::updateMapBoundaryFromIndex()
{
  // 由“中心格子”推出最小/最大格子编号
  mp_.map_bound_min_idx_ = mp_.map_origin_idx_ - mp_.map_voxel_num_ / 2;
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  // 再由格子编号换回世界坐标下的边界
  mp_.map_min_boundary_ = mp_.map_bound_min_idx_.cast<double>() * mp_.resolution_;
  mp_.map_max_boundary_ = (mp_.map_bound_max_idx_.cast<double>() + Eigen::Vector3d::Ones()) * mp_.resolution_;
  mp_.map_origin_ = mp_.map_min_boundary_;
}

// 预计算膨胀用的“偏移表”。
// 思路：以机器人为中心，凡是水平距离小于半径、且高度在上下限内的格子，
// 都记录下来。之后只要按这张表去给障碍“盖章”，就是膨胀。
void GridMap::rebuildInflationOffsets()
{
  // 圆柱半径，以及它在 xy 平面、上下 z 方向各占多少格
  // 圆柱模式用满半径做大膨胀；矩形模式只用小余量（机身由采样点自己覆盖）
  const bool rect_model = (mp_.footprint_model_ == 1);
  const double xy_radius = rect_model ? std::max(0.0, mp_.footprint_margin_)
                                      : std::max(0.0, mp_.double_cylinder_radius_);
  const int inf_step_xy = ceil(xy_radius / mp_.resolution_);
  const int inf_step_z_up = ceil(mp_.obstacles_inflation_z_up / mp_.resolution_);
  const int inf_step_z_down = ceil(mp_.obstacles_inflation_z_down / mp_.resolution_);

  // 清空旧的偏移表，下面重新枚举
  md_.inflate_offsets_.clear();
  // 在 xy 平面上遍历一个正方形，只保留落在圆内的格子
  for (int x = -inf_step_xy; x <= inf_step_xy; ++x)
    for (int y = -inf_step_xy; y <= inf_step_xy; ++y)
    {
      // 换算成米，判断它离中心是否小于半径
      Eigen::Vector2d offset_xy(x * mp_.resolution_, y * mp_.resolution_);
      // 圆柱模式保持原判据(< radius)；矩形模式含边界(<= radius)，保证 margin=0 时仍保留 (0,0)
      const double nrm = offset_xy.norm();
      if (rect_model ? (nrm > xy_radius + 1e-9) : (nrm >= xy_radius))
        continue;

      // z 方向按“上膨胀 / 下膨胀”两个不同高度遍历（上下不对称）
      for (int z = -inf_step_z_down; z <= inf_step_z_up; ++z)
        md_.inflate_offsets_.push_back(Eigen::Vector3i(x, y, z));
    }
}

// 预计算 rectangle 模式的机身采样点（机身坐标系：x 向前、y 向左）。
// 在 L×W 的矩形上按 step 取网格（含四边与内部），保证不漏掉细障碍。
void GridMap::rebuildFootprintSamples()
{
  md_.footprint_samples_.clear();
  if (mp_.footprint_model_ != 1)
    return;

  const double L = std::max(0.0, mp_.footprint_length_);
  const double W = std::max(0.0, mp_.footprint_width_);
  const double step = std::max(1e-3, mp_.footprint_sample_step_);
  if (L <= 0.0 || W <= 0.0)
  {
    RCLCPP_WARN(rclcpp::get_logger("grid_map"),
                "rectangle footprint enabled but length/width not set; no samples");
    return;
  }

  const int nx = std::max(1, (int)std::ceil(L / step)) + 1;  // 含两端
  const int ny = std::max(1, (int)std::ceil(W / step)) + 1;
  md_.footprint_samples_.reserve(nx * ny);
  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < ny; ++j)
    {
      const double lx = -L / 2.0 + L * (nx > 1 ? double(i) / (nx - 1) : 0.5);
      const double ly = -W / 2.0 + W * (ny > 1 ? double(j) / (ny - 1) : 0.5);
      md_.footprint_samples_.push_back(Eigen::Vector3d(lx, ly, 0.0));
    }
}

// 把整张地图清空（回到“全部未知”）。滑动距离太大时会整体重置。
void GridMap::resetAllMapData()
{
  // 把占据概率整片填成“未知”
  std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), mp_.clamp_min_log_ - mp_.unknown_flag_);
  // 膨胀标志、膨胀计数、命中计数全部清零
  std::fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0);
  std::fill(md_.occupancy_buffer_inflate_cnt_.begin(), md_.occupancy_buffer_inflate_cnt_.end(), 0);
  std::fill(md_.count_hit_and_miss_.begin(), md_.count_hit_and_miss_.end(), 0);
  std::fill(md_.count_hit_.begin(), md_.count_hit_.end(), 0);
  // 光线遍历用的两个标记数组也全部复位
  std::fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), -1);
  std::fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), -1);
  // 清空待处理队列（用一个空队列交换过去）
  std::queue<Eigen::Vector3i> empty;
  std::swap(md_.cache_voxel_, empty);
}

// 把“缓冲区里的一维地址”还原成“全局格子编号”。
// 因为滑动地图会让缓冲区地址和全局编号的对应关系改变，需要这个换算。
void GridMap::hashIdToGlobalIndex(int addr, Eigen::Vector3i& id_g) const
{
  // 先把“一维地址”拆回三维的局部格子编号
  Eigen::Vector3i id_l;
  // 行优先展开的逆运算：先除最大的那一维
  id_l(0) = addr / (mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2));
  id_l(1) = (addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) / mp_.map_voxel_num_(2);
  id_l(2) = addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) -
            id_l(1) * mp_.map_voxel_num_(2);

  // 再把局部编号还原成全局编号（要考虑滑动地图的取模回绕）
  for (int i = 0; i < 3; ++i)
  {
    const int min_l = getLocalIndex(mp_.map_bound_min_idx_(i), i);
    int dist = id_l(i) - min_l;
    if (dist < 0)
      dist += mp_.map_voxel_num_(i);
    id_g(i) = mp_.map_bound_min_idx_(i) + dist;
  }
}

// 膨胀的核心：把一个格子按“偏移表”覆盖到周围格子，计数 += delta；
// 计数归零才把标志清零（这就是引用计数防误清的作用）。
void GridMap::updateInflationLayer(const Eigen::Vector3i& id, int delta,
                                   const vector<Eigen::Vector3i>& offsets,
                                   std::vector<int>& cnt_buffer,
                                   std::vector<char>& flag_buffer,
                                   const std::vector<char>* ignore_mask)
{
  // 遍历“偏移表”里的每一个格子
  for (const auto& offset : offsets)
  {
    // 目标格子 = 障碍格子 + 偏移量
    const Eigen::Vector3i inf_id = id + offset;
    // 超出地图范围就跳过
    if (!isInMap(inf_id))
      continue;

    // 取它的内存地址（一维）
    const int addr = toAddress(inf_id);
    // 滑动地图时，仍留在盒内的格子不能撤销膨胀（掩码保护）
    if (ignore_mask && (*ignore_mask)[addr])
      continue;

    // 计数加减 —— 这就是“引用计数”
    cnt_buffer[addr] += delta;
    // 计数不允许为负
    if (cnt_buffer[addr] < 0)
      cnt_buffer[addr] = 0;
    // 只要计数大于 0，就认为这一格被膨胀覆盖
    flag_buffer[addr] = cnt_buffer[addr] > 0 ? 1 : 0;
  }
}

// 上面函数的便捷包装：用默认的偏移表和缓冲区。
void GridMap::updateInflation(const Eigen::Vector3i& id, int delta, const std::vector<char>* ignore_mask)
{
  updateInflationLayer(id, delta, md_.inflate_offsets_, md_.occupancy_buffer_inflate_cnt_,
                       md_.occupancy_buffer_inflate_, ignore_mask);
}

// 写一个格子的新概率；如果这一格“从空变成障碍”或反之，
// 就同步更新膨胀层（+1 或 -1）。
void GridMap::applyOccupancyUpdate(const Eigen::Vector3i& id, double new_log_odds)
{
  // 越界就直接返回
  if (!isInMap(id))
    return;

  const int addr = toAddress(id);
  // 记录更新前后“这一格算不算障碍”
  const bool was_occ = md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_;
  const bool now_occ = new_log_odds > mp_.min_occupancy_log_;

  // 写入新的概率值
  md_.occupancy_buffer_[addr] = new_log_odds;
  // 如果“障碍 / 非障碍”的状态变了，就同步更新膨胀层（+1 或 -1）
  if (was_occ != now_occ)
    updateInflation(id, now_occ ? 1 : -1);
}

// 把单个格子重置为“未知”，并相应地撤销它的膨胀贡献。
void GridMap::resetCellByAddress(int addr)
{
  // 先把内存地址换回全局格子编号
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  // 如果它原本是障碍，先撤销它在膨胀层的贡献
  if (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_)
    updateInflation(id_g, -1);

  // 再把这一格恢复成“未知”，并清掉各种标记
  md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  md_.count_hit_[addr] = 0;
  md_.count_hit_and_miss_[addr] = 0;
  md_.flag_rayend_[addr] = -1;
  md_.flag_traverse_[addr] = -1;
}

// 滑动时用的版本：撤销膨胀时要避开“仍留在盒子里”的格子
void GridMap::resetCellByAddressForSliding(int addr, const std::vector<char>& clear_mask)
{
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  if (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_)
    // 传入掩码，避免把还留在盒内的格子误清
    updateInflation(id_g, -1, &clear_mask);
}

// 滑动地图：机器人移动超过阈值时，把地图盒子往机器人方向挪。
// 挪的时候要把“滑出盒子”的那一层格子清空（并撤销膨胀），
// 盒内已有内容不搬动——用取模索引让它们“看起来”还在原位。
void GridMap::updateSlidingMap(const Eigen::Vector3d& center)
{
  // 没有启用滑动地图 → 直接返回
  if (!mp_.map_sliding_en_)
    return;

  // 算机器人现在落在哪个格子，以及相对于原中心移动了几格
  Eigen::Vector3i new_origin_idx;
  posToIndex(center, new_origin_idx);
  const Eigen::Vector3i shift_num = new_origin_idx - mp_.map_origin_idx_;
  // 移动距离还不到阈值 → 地图不动（避免频繁重建）
  if (shift_num.cwiseAbs().maxCoeff() < mp_.map_sliding_thresh_vox_)
    return;

  // 一次移动超过整张地图 → 直接整张清空重来
  if ((shift_num.cwiseAbs().array() >= mp_.map_voxel_num_.array()).any())
  {
    resetAllMapData();
    mp_.map_origin_idx_ = new_origin_idx;
    updateMapBoundaryFromIndex();
    md_.local_bound_min_ = mp_.map_bound_min_idx_;
    md_.local_bound_max_ = mp_.map_bound_max_idx_;
    return;
  }

  // 否则只清理“滑出盒子”的那一层：先准备掩码和待清理地址列表
  const int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  std::vector<char> clear_mask(buffer_size, 0);
  std::vector<int> clear_addrs;
  clear_addrs.reserve(buffer_size / 8);

  // 小工具：登记一个待清理格子（自动去重）
  auto add_clear_addr = [&](const Eigen::Vector3i& id_l) {
    const int addr = toAddressLocal(id_l);
    if (!clear_mask[addr])
    {
      clear_mask[addr] = 1;
      clear_addrs.push_back(addr);
    }
  };

  // 三个方向分别处理
  for (int dim = 0; dim < 3; ++dim)
  {
    // 这个方向挪了几格；没挪就跳过
    const int shift = shift_num(dim);
    if (shift == 0)
      continue;

    // 往正方向挪 → 从“最小边界”那一侧滑出一层
    if (shift > 0)
    {
      for (int k = 0; k < shift; ++k)
      {
        const int clear_g = mp_.map_bound_min_idx_(dim) + k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b)
          {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
    else
    {
      // 往负方向挪 → 从“最大边界”那一侧滑出一层
      for (int k = 0; k < -shift; ++k)
      {
        const int clear_g = mp_.map_bound_max_idx_(dim) - k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b)
          {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
  }

  // 先撤销这些格子的膨胀贡献（受掩码保护的不动）
  for (int addr : clear_addrs)
    resetCellByAddressForSliding(addr, clear_mask);

  // 再把这些格子的数据彻底清空
  for (int addr : clear_addrs)
  {
    md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
    md_.occupancy_buffer_inflate_cnt_[addr] = 0;
    md_.occupancy_buffer_inflate_[addr] = 0;
    md_.count_hit_[addr] = 0;
    md_.count_hit_and_miss_[addr] = 0;
    md_.flag_rayend_[addr] = -1;
    md_.flag_traverse_[addr] = -1;
  }

  // 最后更新地图中心与边界，并把局部范围夹回合法区间
  mp_.map_origin_idx_ = new_origin_idx;
  updateMapBoundaryFromIndex();
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
}

// 对外提供的清空接口（整张清空 / 只清一个长方体范围）。
void GridMap::resetBuffer()
{
  // 整张清空，并把“局部范围”恢复成整张地图
  resetAllMapData();
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  Eigen::Vector3i min_id, max_id;
  // 把给定的两个世界坐标点转成格子编号
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  // 三层循环，把长方体范围内的每个格子逐个重置
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
    {
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        resetCellByAddress(toAddress(x, y, z));
      }
    }
}

// 在“光线遍历”过程中给某个格子打标记（先不急着改概率，只累计计数）。
// occ=1 表示命中，occ=0 表示只是路过。同一格子在一帧里只入队一次。
int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  // 只接受 1（命中）或 0（空闲）两种标记
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  // 转成格子编号；超出地图范围就返回“无效”
  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id))
    return INVALID_IDX;

  // 取它的一维内存地址
  int idx_ctns = toAddress(id);

  // “被观测到”的次数 +1
  md_.count_hit_and_miss_[idx_ctns] += 1;

  // 这一帧第一次碰到它 → 放进待更新队列（之后统一处理）
  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  // 如果是“命中”，命中计数也 +1
  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

// 深度相机模式：把深度图逐像素反投影成 3D 点（用相机内参和位姿）。
// 太近的像素忽略，太远或读数为 0 的当“该方向没有障碍”（用于清空该方向）。
void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  // 清空投影点计数（下面重新填）
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  // 深度图的宽和高
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  // 传感器姿态的旋转矩阵（把相机系下的点转到世界系）
  Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();

  // cout << "rotate: " << md_.ray_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.ray_pos_ << std::endl;

  // 第一帧深度图通常不完整，先跳过不用
  if (!md_.has_first_depth_)
  {
    md_.has_first_depth_ = true;
    return;
  }

  // inv_factor：把整数深度值换算成“米”
  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

  // 逐行扫描（跳过图像边缘的几行）
  for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
  {
    // 直接取这一行的裸指针，比逐个元素访问快
    row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

    // 逐列扫描（每隔 skip_pixel 个像素取一个，加快速度）
    for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_; u += mp_.skip_pixel_)
    {
      // 读出该像素的深度原始值
      const uint16_t raw_depth = *row_ptr;
      depth = raw_depth * inv_factor;
      row_ptr = row_ptr + mp_.skip_pixel_;

      // filter depth
      // depth += rand_noise_(eng_);
      // if (depth > 0.01) depth += rand_noise2_(eng_);

      // 深度为 0 → 这个方向没打到东西，按最大射程处理（用来清空该方向）
      if (raw_depth == 0)
      {
        depth = mp_.max_ray_length_ + 0.1;
      }
      // 太近（可能是噪点或自遮挡）→ 忽略这个像素
      else if (depth < mp_.depth_filter_mindist_)
      {
        continue;
      }
      // 太远 → 同样按“该方向没有障碍”处理
      else if (depth > mp_.depth_filter_maxdist_)
      {
        depth = mp_.max_ray_length_ + 0.1;
      }

      // project to world frame
      // 用针孔相机模型：把像素坐标 + 深度反投影成相机坐标系下的 3D 点
      pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
      pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
      pt_cur(2) = depth;

      // 再旋转 + 平移到世界坐标系
      pt_world = sensor_r * pt_cur + md_.ray_pos_;
      // if (!isInMap(pt_world)) {
      //   pt_world = closetPointInMap(pt_world, md_.ray_pos_);
      // }

      // 存进投影点数组，计数 +1
      md_.proj_points_[md_.proj_points_cnt++] = pt_world;
    }
  }
}

// ==========================================================================
// 【地图更新的核心】raycastProcess：把一帧观测变成概率更新。
//   对每个观测点：
//     1. 判断它在不在图内、离传感器多远，确定它是“命中”还是“到达极限的空闲”
//     2. 从传感器到该点，用 RayCaster 一格一格走，途经格子都记为“空闲”
//     3. 把所有打过标记的格子从队列里取出，统一更新概率：
//        命中多于路过 → 概率上调；否则下调（并夹在上下限之间）
// ==========================================================================
void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  // 这一帧一个观测点都没有 → 没什么可更新的
  if (md_.proj_points_cnt == 0)
    return;

  // 先让“滑动地图”跟上机器人当前位置
  updateSlidingMap(md_.ray_pos_);

  // 光线遍历轮次 +1（给标记打时间戳，避免重复处理）
  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  // 下面初始化“本次更新范围”的包围盒（先设成极端值，后面逐步扩）
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  // 射线遍历器；half 用来把格子编号换算到格子中心（+0.5）
  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  // 逐个观测点处理
  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    // 情况一：这个观测点落在地图外
    if (!isInMap(pt_w))
    {
      // 把它拉回到地图边界的交点上
      pt_w = closetPointInMap(pt_w, md_.ray_pos_);

      length = (pt_w - md_.ray_pos_).norm();
      // 如果仍然超过射程 → 截断到最大射程
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
      }
      // 这是“最远看到这里” → 标记为空闲
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    // 情况二：观测点在地图内
    else
    {
      length = (pt_w - md_.ray_pos_).norm();

      // 但离传感器超过射程 → 也只当作空闲（不可信）
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        // 射程之内 → 标记为“命中”，说明这里有障碍
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    // 用这个点来扩大“本次更新范围”的包围盒
    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between ray origin and point

    // 若这个观测点有效，就沿射线回溯（把途经格子都标成空闲）
    if (vox_idx != INVALID_IDX)
    {
      // 这一帧已经处理过这条射线 → 跳过，省时间
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    // 设置射线：从观测点回到传感器（坐标单位是“格子”）
    raycaster.setInput(pt_w / mp_.resolution_, md_.ray_pos_ / mp_.resolution_);

    // 一格一格往回走
    while (raycaster.step(ray_pt))
    {
      // 当前格子的中心在世界坐标下的位置
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.ray_pos_).norm();

      // 沿途格子标记为“空闲” —— 说明这条路上没有障碍
      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        // 这条射线之前已经走过这一格 → 提前结束回溯
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  // 把传感器自身的位置也纳入包围盒
  min_x = min(min_x, md_.ray_pos_(0));
  min_y = min(min_y, md_.ray_pos_(1));
  min_z = min(min_z, md_.ray_pos_(2));

  max_x = max(max_x, md_.ray_pos_(0));
  max_y = max(max_y, md_.ray_pos_(1));
  max_z = max(max_z, md_.ray_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  // 把包围盒换算成格子编号，并夹进合法范围
  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // update occupancy cached in queue
  // “局部更新范围”：只更新传感器周围这一块
  Eigen::Vector3d local_range_min = md_.ray_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.ray_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  // 统一处理所有被打过标记的格子
  while (!md_.cache_voxel_.empty())
  {

    // 取出一个格子，同时清掉它的计数
    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    // 多数投票：命中次数 ≥ 路过次数 → 概率上调；否则下调
    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    // 计数清零，准备下一帧
    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    // 已经在上限了 → 不用再更新
    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    // 已经在下限了 → 直接设为下限并跳过
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      applyOccupancyUpdate(idx, mp_.clamp_min_log_);
      continue;
    }

    // 判断这个格子是否还在“局部更新范围”内
    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    // 不在范围内 → 强制清成空闲，避免陈旧数据残留
    if (!in_local)
    {
      applyOccupancyUpdate(idx, mp_.clamp_min_log_);
    }

    // 新概率 = 旧概率 + 增量，并夹在上下限之间
    const double new_log_odds =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
    // 写回地图（同时会同步更新膨胀层）
    applyOccupancyUpdate(idx, new_log_odds);
  }
}

// 如果观测点跑到地图盒子外面了，就把它拉回到盒子的边界上（求射线与盒子交点）。
Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &ray_pos)
{
  // 射线方向，以及到三个方向边界的向量
  Eigen::Vector3d diff = pt - ray_pos;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - ray_pos;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - ray_pos;

  double min_t = 1000000;

  // 逐轴求“射线到达该方向边界所需的参数 t”，取最小的（最近交点）
  for (int i = 0; i < 3; ++i)
  {
    // 这个方向分量为 0 → 永远碰不到该方向的边界，跳过
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  // 交点稍微往回收一点，保证结果落在盒子内部
  return ray_pos + (min_t - 1e-3) * diff;
}

// 每 50ms 把当前地图内容发布出去（给 RViz 显示）：
// 占据点云、膨胀点云、地图盒子、深度投影点。没人订阅时会自动跳过。
void GridMap::visCallback()
{

  // 依次发布：占据点云、膨胀点云、地图坐标系、地图盒子、深度投影点
  publishMap();
  // 每个发布函数内部都会先看有没有人订阅，没人订阅就跳过
  publishMapInflate(true);
  publishSlidingMapFrame();
  publishSlidingMapBBox();
  publishDepthCloud();
}

// 每 50ms 触发一次的地图更新入口：
//   没有新数据就直接返回；深度模式先 projectDepthImage()，再 raycastProcess()。
void GridMap::updateOccupancyCallback()
{
  // 没有新数据 → 什么都不做
  if (!md_.occ_need_update_)
    return;

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  // 如果不是“点云更新”（也就是深度模式），先把深度图反投影成 3D 点
  if (!md_.use_cloud_update_)
    projectDepthImage();
  // t2 = ros::Time::now();
  // 真正的地图更新：光线遍历 + 概率融合
  raycastProcess();
  // t3 = ros::Time::now();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  // 处理完毕，清掉“待更新”和“本帧是点云更新”这两个标志
  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
}

// 深度模式的回调（深度图 + 传感器位姿 已按时间对齐后才进来）：
//   存下深度图、算出传感器在世界系的位置和朝向，然后请求一次地图更新。
void GridMap::depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img,
                                const nav_msgs::msg::Odometry::ConstSharedPtr &pose)
{
  // 不是深度模式就忽略
  if (mp_.sensor_type_ != "depth")
    return;

  /* get depth image */
  // 把 ROS 图像消息转成 OpenCV 图像
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  // 32 位浮点深度 → 统一转成 16 位整数格式
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  // 存下深度图，等定时器触发时再统一处理
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  // 从位姿消息里取出姿态（四元数）
  const geometry_msgs::msg::Pose &sensor_pose = pose->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y, sensor_pose.orientation.z);
  // 四元数退化（全 0）→ 数据无效，忽略
  if (ray_q.norm() < 1e-6)
    return;
  ray_q.normalize();

  // 传感器位置；若配置了外参，则叠加“相机相对机体”的安装偏差
  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z);
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.depth_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.depth_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }

  // 把“加过外参之后”的传感器位姿也发出去，方便在 RViz 里核对
  nav_msgs::msg::Odometry extrinsic_pose = *pose;
  extrinsic_pose.pose.pose.position.x = ray_pos.x();
  extrinsic_pose.pose.pose.position.y = ray_pos.y();
  extrinsic_pose.pose.pose.position.z = ray_pos.z();
  extrinsic_pose.pose.pose.orientation.x = ray_q.x();
  extrinsic_pose.pose.pose.orientation.y = ray_q.y();
  extrinsic_pose.pose.pose.orientation.z = ray_q.z();
  extrinsic_pose.pose.pose.orientation.w = ray_q.w();
  extrinsic_pose.child_frame_id =
      pose->child_frame_id.empty() ? "sensor_extrinsic" : pose->child_frame_id + "_extrinsic";
  extrinsic_pose_pub_->publish(extrinsic_pose);

  // 记下传感器位置与朝向；标记本次是“深度更新”
  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.use_cloud_update_ = false;
  updateSlidingMap(md_.ray_pos_);
  // 传感器在地图内才请求更新；否则不更新
  if (isInMap(md_.ray_pos_))
  {
    md_.has_ray_pose_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }
}

// 激光模式下的“传感器位姿”回调：记录激光传感器在世界系的位置/朝向。
void GridMap::sensorPoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
{
  // 不是激光模式就忽略
  if (mp_.sensor_type_ != "lidar")
    return;

  // 取姿态，做与上面相同的合法性检查和外参变换
  const geometry_msgs::msg::Pose &sensor_pose = pose_msg->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y, sensor_pose.orientation.z);
  if (ray_q.norm() < 1e-6)
    return;
  ray_q.normalize();

  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z);
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.lidar_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.lidar_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }
  // 位置里出现 NaN/Inf → 丢弃这一帧
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) || !std::isfinite(ray_pos.z()))
    return;

  // 记下传感器位姿
  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.has_ray_pose_ = true;
  // 注意：这里只记录，不请求更新；等点云到了再一起处理
  updateSlidingMap(md_.ray_pos_);
}

void GridMap::slidingMapFrameCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose)
{
  // 只记录一个位置，供发布 world→sliding_map 的坐标变换
  const geometry_msgs::msg::Point &pos = pose->pose.pose.position;
  md_.sliding_map_frame_pos_ = Eigen::Vector3d(pos.x, pos.y, pos.z);
}

// 激光模式的回调：把点云转成世界坐标，只保留地图附近的点，
// 存好之后请求一次地图更新。
void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &img)
{
  // 不是激光模式就忽略
  if (mp_.sensor_type_ != "lidar")
    return;

  // 还没收到传感器位姿 → 不知道点云在世界系哪儿，先忽略
  if (!md_.has_ray_pose_)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[GridMap] no sensor_pose received for lidar cloud update");
    return;
  }

  // 把 ROS 点云消息转成 PCL 点云
  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  // 标记“收到过点云”
  md_.has_cloud_ = true;

  // 空点云 → 什么都不做
  if (latest_cloud.points.size() == 0)
    return;

  // 传感器姿态的旋转矩阵（用来把点转到世界系）
  const Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();
  const Eigen::Vector3d ray_pos = md_.ray_pos_;
  // 传感器位置异常 → 丢弃本帧
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) || !std::isfinite(ray_pos.z()))
    return;

  // 让地图盒子跟上机器人
  updateSlidingMap(ray_pos);

  // 清空上一帧的投影点
  md_.proj_points_cnt = 0;

  // 逐个点处理
  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    const pcl::PointXYZ &pt = latest_cloud.points[i];
    // 跳过无效点（NaN/Inf）
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;

    // 把点统一到世界坐标系
    Eigen::Vector3d pt_world;
    // 如果点云本身就是世界系（仿真模式），直接用
    if (mp_.cloud_is_world_)
    {
      pt_world = Eigen::Vector3d(pt.x, pt.y, pt.z);
    }
    else
    {
      // 否则（真机）用传感器位姿把它旋转平移到世界系
      const Eigen::Vector3d pt_sensor(pt.x, pt.y, pt.z);
      pt_world = sensor_r * pt_sensor + ray_pos;
    }
    // 这个点离传感器多远
    const Eigen::Vector3d devi = pt_world - ray_pos;
    const double ray_length = devi.norm();
    // 判断它是否落在“局部更新范围”内
    const bool in_local_range =
        fabs(devi(0)) <= mp_.local_update_range_(0) && fabs(devi(1)) <= mp_.local_update_range_(1) &&
        fabs(devi(2)) <= mp_.local_update_range_(2);
    // 既不在局部范围内、又没超出射程的点用不上 → 丢弃
    if (!in_local_range && ray_length <= mp_.max_ray_length_)
      continue;

    // 预分配的数组满了就自动扩容
    if (md_.proj_points_cnt >= static_cast<int>(md_.proj_points_.size()))
      md_.proj_points_.push_back(pt_world);
    else
      md_.proj_points_[md_.proj_points_cnt] = pt_world;

    // 投影点计数 +1
    md_.proj_points_cnt++;
  }

  // 一个点都没留下 → 不用更新
  if (md_.proj_points_cnt == 0)
    return;

  // 标记“本帧用点云更新”，并请求一次地图更新
  md_.use_cloud_update_ = true;
  md_.occ_need_update_ = true;
}

// 把“判定为障碍”的格子导出成点云，发布到 grid_map/occupancy 供 RViz 显示。
void GridMap::publishMap()
{

  // 没有订阅者就不做（省算力）
  if (map_pub_->get_subscription_count() == 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // 要遍历整张地图的格子范围
  Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;

  // 三层循环扫过所有格子
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        // 概率没到门槛（不是障碍）→ 跳过
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        // 比传感器高出太多（比如天花板）→ 不显示，避免挡住视线
        if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
          continue;
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  // 打包成点云消息并发布
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  map_pub_->publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  // 没有订阅者就跳过
  if (map_inf_pub_->get_subscription_count() == 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;

  // 这次导出的依据是“膨胀后的障碍标志”
  const std::vector<char> &inflate_buffer = md_.occupancy_buffer_inflate_;
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        // 没有被膨胀覆盖 → 跳过
        if (inflate_buffer[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  map_inf_pub_->publish(cloud_msg);

  // ROS_INFO("pub map");
}

void GridMap::publishSlidingMapFrame()
{
  // 没配置框架名就跳过
  if (mp_.sliding_map_frame_id_.empty())
    return;

  // 构造一个“世界系 → 滑动地图系”的平移变换
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = mp_.frame_id_;
  transform.child_frame_id = mp_.sliding_map_frame_id_;
  transform.transform.translation.x = md_.sliding_map_frame_pos_.x();
  transform.transform.translation.y = md_.sliding_map_frame_pos_.y();
  transform.transform.translation.z = md_.sliding_map_frame_pos_.z();
  transform.transform.rotation.w = 1.0;
  // 广播出去（RViz 里就能看到滑动地图的原点）
  tf_broadcaster_->sendTransform(transform);
}

void GridMap::publishSlidingMapBBox()
{
  // 没有订阅者就跳过
  if (sliding_map_bbox_pub_->get_subscription_count() == 0)
    return;

  // 构造一个“线段列表”Marker，用来画长方体的 12 条棱
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = mp_.frame_id_;
  marker.header.stamp = node_->now();
  marker.ns = "sliding_map";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.color.r = 0.0;
  marker.color.g = 0.8;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  // 取出长方体的最小/最大两个角点，据此算出 8 个顶点
  const Eigen::Vector3d& min_pt = mp_.map_min_boundary_;
  const Eigen::Vector3d& max_pt = mp_.map_max_boundary_;
  Eigen::Vector3d corners[8] = {
      {min_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), max_pt.y(), max_pt.z()},
      {min_pt.x(), max_pt.y(), max_pt.z()},
  };

  // 小工具：把一个顶点加进 Marker 的点列表
  auto pushPoint = [&](const Eigen::Vector3d& p) {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    marker.points.push_back(point);
  };

  // 12 条棱分别由哪两个顶点相连
  const int edges[12][2] = {
      {0, 1}, {1, 2}, {2, 3}, {3, 0},
      {4, 5}, {5, 6}, {6, 7}, {7, 4},
      {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };

  // 逐条棱把两个端点加进去
  for (const auto& edge : edges)
  {
    pushPoint(corners[edge[0]]);
    pushPoint(corners[edge[1]]);
  }

  // 发布这个长方体线框
  sliding_map_bbox_pub_->publish(marker);
}

void GridMap::publishUnknown()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // 只遍历“本次更新过的范围”（比扫整张图快得多）
  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {

        // 概率比下限还低 → 说明这一格从来没被观测过，属于“未知”
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
            continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  // 发布未知区域点云
  unknown_pub_->publish(cloud_msg);
}

// 下面三个是一句话的小查询接口：有没有位姿 / 有没有深度观测 / 地图原点在哪
bool GridMap::odomValid() { return md_.has_ray_pose_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

// 返回地图的原点与尺寸
void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

// GridMap

// 把“深度图反投影出的点云”发出去（调试用）
void GridMap::publishDepthCloud()
{
  // 没人订阅、或没有投影点 → 直接跳过
  if (depth_cloud_pub_->get_subscription_count() == 0)
    return;

  if (md_.proj_points_cnt == 0)
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ pt;

  // 把投影点逐个搬进 PCL 点云
  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt.x = md_.proj_points_[i](0);
    pt.y = md_.proj_points_[i](1);
    pt.z = md_.proj_points_[i](2);
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  depth_cloud_pub_->publish(cloud_msg);
}
