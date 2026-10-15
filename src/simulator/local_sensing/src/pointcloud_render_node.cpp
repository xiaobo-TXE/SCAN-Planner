// ============================================================================
// 文件：src/pointcloud_render_node.cpp —— CPU 版传感器仿真器（默认使用）
//
// 【这个文件是干什么的】
//   它“假装”是一台激光雷达或深度相机：把全局点云地图当作真实世界，
//   根据机器人当前位姿，渲染出这一时刻传感器“看到”的数据。
//
// 【在整条数据链里的位置】
//   mockamap / map_generator  →  global_map  →  【本节点】
//        →  /quad_0/cloud（激光）或 /quad_0/depth（深度）  →  规划器的栅格地图
//
// 【两种模式的差别】
//   深度模式：把地图点直接投影到一张深度图上（简单、快）；
//   激光模式：投到“极坐标深度图”，再做面插值、扫描图案模拟（复杂、真实）。
//
// 【建议阅读顺序】
//   main → rcvGlobalPointCloudCallBack（准备地图）→ renderSensedPoints（核心）
// ============================================================================
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <cstdint>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <algorithm>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>
#include <tr1/unordered_map>
#include "FOV_Checker/FOV_Checker.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdlib.h>
#include <time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <pcl/common/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <deque>
#include <numeric>

// #define DEBUG

#define likely(x) __builtin_expect(!!(x), 1) // gcc function, for if optimization
#define unlikely(x) __builtin_expect(!!(x), 0)

using namespace std;
using namespace Eigen;

// 全局节点指针（因为很多函数是自由函数，用全局变量传递节点）
rclcpp::Node::SharedPtr ros_node;

#define MAX_INTENSITY 255
#define MIN_INTENSITY 199

// 记录最近若干次渲染耗时，用于统计平均耗时
deque<double> comp_time_vec;

std::string quad_name;
// 传感器类型（depth / lidar）—— 后面大量逻辑都按它分叉
std::string sensor_type_ = "depth";
// 哈希体素表：把地图点按“大方块”分组，查询时只看附近方块，大幅加速
std::tr1::unordered_map<int, std::vector<PointType>> point_hashmap;
std::tr1::unordered_map<int, std::vector<int>> pointindex_hashmap;

pcl::PointCloud<PointType> cloud_explored;

// 视场检查器（第三方 FOV_Checker）：快速找出落在视野内的体素方块
FOV_Checker fov_checker;
BoxPointType env_box;

PointType min_center, max_center;

int cube_numx, cube_numy, cube_numz;

// 极坐标表示：theta/fi 是水平/竖直方向的角度格子，r 是距离
struct polar3D
{
  int theta;
  int fi;
  float r;
};

// 发布器：cloud(激光点云)、sensor_cloud(传感器系点云)、dyn_cloud、uav_cloud
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud, pub_intercloud, pub_dyncloud, pub_uavcloud;
// 深度图发布器、相机位姿发布器、激光位姿发布器、耗时发布器
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pinhole_depth_pub_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr camera_pose_pub_, lidar_pose_pub_;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr comp_time_pub;

sensor_msgs::msg::PointCloud2 local_map_pcl;
sensor_msgs::msg::PointCloud2 local_depth_pcl;

// 订阅：本机里程计、全局地图、以及其它无人机的里程计
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub;
std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> other_odom_subs;

rclcpp::TimerBase::SharedPtr local_sensing_timer, dynobj_timer;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

// 四个“数据到齐了没有”的标志
bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);
bool has_dyn_map(false);

// 最新里程计；sensor2body：传感器相对机体的安装矩阵
nav_msgs::msg::Odometry odom_;
Eigen::Matrix4d sensor2body, sensor2world;

// 下面一大段都是可配置参数，由 main 函数从 YAML 读入
int output_pcd;
int collisioncheck_enable;
int is_360lidar;
int use_avia_pattern, use_vlp32_pattern, use_minicf_pattern, use_os128_pattern, use_os64_pattern;
double livox_linestep;
double sensing_horizon, sensing_rate, estimation_rate, polar_resolution, yaw_fov, vertical_fov,
    min_raylength, downsample_res, curvature_limit, hash_cubesize, collision_range;
double x_size, y_size, z_size;
double gl_xl, gl_yl, gl_zl;
double resolution, inv_resolution;
int GLX_SIZE, GLY_SIZE, GLZ_SIZE;

// 深度相机内参（这里是默认值，可被参数覆盖）
int cam_width = 640;
int cam_height = 480;
double cam_fx = 387.229248046875;
double cam_fy = 387.229248046875;
double cam_cx = 321.04638671875;
double cam_cy = 243.44969177246094;

int plane_interline = 1;

// multi uav variables
int drone_id = 0;
double lidar_pitch;
vector<Eigen::Vector3d> other_uav_pos;
vector<double> other_uav_rcv_time;
vector<vector<PointType>> otheruav_points, otheruav_points_inrender;
vector<vector<int>> otheruav_pointsindex, otheruav_pointsindex_inrender;
pcl::PointCloud<PointType> otheruav_points_vis;
double uav_size[3];
int uav_points_num;
int drone_num = 0;
int drone_drawpoints_num[3];

int use_uav_extra_model = 1;
pcl::PointCloud<PointType> uav_extra_model;

// dynamic objects variables
int dynobj_enable;
double dynobject_size;
vector<Eigen::Vector3d> dynobj_poss;
vector<int> dynobj_move_modes;
vector<int> dynobj_types;
vector<Eigen::Vector3d> dynobj_dir;
vector<PointType> dynobj_points;
pcl::PointCloud<PointType> dynobj_points_vis;
vector<int> dynobj_pointsindex;
double dyn_velocity;
int dynobject_num;
int dyn_mode;
Eigen::Vector3d map_min, map_max;
int dyn_obs_diff_size_on = 1;
vector<double> dyn_obs_size_vec;
vector<rclcpp::Time> dyn_start_time_vec;

int origin_mapptcount = 0;

pcl::PointCloud<pcl::Normal>::Ptr all_normals(new pcl::PointCloud<pcl::Normal>);
pcl::NormalEstimation<PointType, pcl::Normal> normalEstimation;

rclcpp::Time start_time(0, 0, RCL_ROS_TIME), dyn_start_time(0, 0, RCL_ROS_TIME);

pcl::PointCloud<PointType> cloud_all_map, local_map, local_map_filled, point_in_sensor;
pcl::VoxelGrid<PointType> _voxel_sampler;
sensor_msgs::msg::PointCloud2 local_map_pcd, sensor_map_pcd;

pcl::search::KdTree<PointType> _kdtreeLocalMap, kdtree_dyn;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;
double collision_check_time_sum = 0;
int collision_check_time_count = 0;

sensor_msgs::msg::PointCloud2 dynobj_points_pcd;

// 小判断：当前是深度相机模式吗
bool useDepthSensor()
{
  return sensor_type_ == "depth";
}

// 小判断：当前是激光模式吗
bool useLidarSensor()
{
  return sensor_type_ == "lidar";
}

// 拼话题名：把名字拼成 /quad_0/xxx 的形式
std::string quadTopic(const std::string &name)
{
  std::string prefix = quad_name.empty() ? std::string("quad_0") : quad_name;
  if (prefix.front() != '/')
  {
    prefix = "/" + prefix;
  }
  return prefix + "/" + name;
}

// 把 4×4 位姿矩阵打包成里程计消息（用来发布相机/激光位姿）
nav_msgs::msg::Odometry makeSensorPoseMsg(const nav_msgs::msg::Odometry &body_odom,
                                     const Eigen::Matrix4d &sensor_pose,
                                     const std::string &child_frame_id)
{
  Eigen::Quaterniond q(sensor_pose.block<3, 3>(0, 0));
  q.normalize();

  nav_msgs::msg::Odometry sensor_odom;
  sensor_odom.header = body_odom.header;
  sensor_odom.header.frame_id = "world";
  sensor_odom.child_frame_id = child_frame_id;
  sensor_odom.pose.pose.position.x = sensor_pose(0, 3);
  sensor_odom.pose.pose.position.y = sensor_pose(1, 3);
  sensor_odom.pose.pose.position.z = sensor_pose(2, 3);
  sensor_odom.pose.pose.orientation.w = q.w();
  sensor_odom.pose.pose.orientation.x = q.x();
  sensor_odom.pose.pose.orientation.y = q.y();
  sensor_odom.pose.pose.orientation.z = q.z();
  return sensor_odom;
}

// 发布相机位姿
void publishCameraPose(const nav_msgs::msg::Odometry &body_odom)
{
  camera_pose_pub_->publish(makeSensorPoseMsg(body_odom, sensor2world, "camera"));
}

// 发布激光位姿：在机体姿态上再叠加激光的俯仰安装角
void publishLidarPose(const nav_msgs::msg::Odometry &body_odom, const Eigen::Matrix4d &body2world)
{
  const double theta = lidar_pitch * M_PI / 180.0;
  Eigen::Matrix3d rot_body2lidar;
  rot_body2lidar << cos(theta), 0, sin(theta),
      0, 1, 0,
      -sin(theta), 0, cos(theta);

  Eigen::Matrix4d lidar2world = Eigen::Matrix4d::Identity();
  lidar2world.block<3, 3>(0, 0) = body2world.block<3, 3>(0, 0) * rot_body2lidar;
  lidar2world.block<3, 1>(0, 3) = body2world.block<3, 1>(0, 3);
  lidar_pose_pub_->publish(makeSensorPoseMsg(body_odom, lidar2world, "lidar"));
}

// 按当前传感器类型，发布对应的位姿
void publishSensorPoses(const nav_msgs::msg::Odometry &body_odom, const Eigen::Matrix4d &body2world)
{
  if (useDepthSensor())
  {
    publishCameraPose(body_odom);
  }
  else if (useLidarSensor())
  {
    publishLidarPose(body_odom, body2world);
  }
}

// 【深度模式】把地图点云投影成一张深度图
void publishPinholeDepth(const rclcpp::Time &stamp)
{
  // 相机内参不合法 → 打警告并直接返回
  if (cam_width <= 0 || cam_height <= 0 || cam_fx <= 0.0 || cam_fy <= 0.0)
  {
    RCLCPP_WARN_THROTTLE(ros_node->get_logger(), *ros_node->get_clock(), 1000,
                         "Invalid pinhole camera intrinsics");
    return;
  }

  // 准备一张全 0 的浮点深度图
  cv::Mat depth_image(cam_height, cam_width, CV_32FC1, cv::Scalar(0.0f));

  // 世界系 → 相机系 的变换矩阵
  const Eigen::Matrix4d world2camera = sensor2world.inverse();
  const Eigen::Vector3d camera_pos(sensor2world(0, 3), sensor2world(1, 3), sensor2world(2, 3));
  const double max_depth = std::max(0.1, sensing_horizon);

  PointType search_point;
  search_point.x = camera_pos.x();
  search_point.y = camera_pos.y();
  search_point.z = camera_pos.z();

  std::vector<int> indices;
  std::vector<float> distances;
  // 只取相机附近（射程内）的点，避免遍历整张地图
  _kdtreeLocalMap.radiusSearch(search_point, max_depth, indices, distances);

  // 逐个点做投影
  for (const int index : indices)
  {
    const PointType &pt = cloud_all_map.points[index];
    const Eigen::Vector4d pt_world(pt.x, pt.y, pt.z, 1.0);
    // 把点转到相机坐标系下
    const Eigen::Vector4d pt_camera = world2camera * pt_world;

    const double z = pt_camera.z();
    // 落在相机后方、或超出射程 → 丢弃
    if (z <= 1e-3 || z > max_depth)
    {
      continue;
    }

    // 用针孔模型算出它投到哪个像素上
    const int u_center = static_cast<int>(std::round(cam_fx * pt_camera.x() / z + cam_cx));
    const int v_center = static_cast<int>(std::round(cam_fy * pt_camera.y() / z + cam_cy));
    // 投到图像外 → 丢弃
    if (u_center < 0 || u_center >= cam_width || v_center < 0 || v_center >= cam_height)
    {
      continue;
    }

    // 近处的点会覆盖多个像素，算一个“泼溅半径”
    const int splat_radius = std::min(
        4, std::max(0, static_cast<int>(std::ceil(0.5 * cam_fx * downsample_res / z))));
    const int u_min = std::max(0, u_center - splat_radius);
    const int u_max = std::min(cam_width - 1, u_center + splat_radius);
    const int v_min = std::max(0, v_center - splat_radius);
    const int v_max = std::min(cam_height - 1, v_center + splat_radius);

    // 在这个半径内填入深度
    for (int v = v_min; v <= v_max; ++v)
    {
      for (int u = u_min; u <= u_max; ++u)
      {
        float &depth = depth_image.at<float>(v, u);
        // 只保留更近的深度 —— 这体现了遮挡关系
        if (depth <= 0.0f || z < depth)
        {
          depth = static_cast<float>(z);
        }
      }
    }
  }

  // 打包成 ROS 图像消息（32 位浮点）并发布
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = stamp;
  out_msg.header.frame_id = "camera";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_image;
  pinhole_depth_pub_->publish(*out_msg.toImageMsg());
}

// 生成一个“球壳”点云（用作动态障碍物或无人机的模型）
pcl::PointCloud<PointType> generate_sphere_cloud(double size)
{
  pcl::PointCloud<PointType> sphere_cloud;
  PointType temp_point;
  double radius = size / 2;
  double x, y, z;

  // use downsample resolution to compute how many points in a sphere
  // 按采样分辨率决定经纬方向各分多少格
  int theta_num = 2 * M_PI * radius / downsample_res;
  int fi_num = M_PI * radius / downsample_res;

  // 按球面参数方程撒点
  for (int i = 0; i < theta_num; i++)
  {
    for (int j = 0; j < fi_num; j++)
    {
      x = radius * sin(i * 2 * M_PI / theta_num) * cos(j * M_PI / fi_num);
      y = radius * sin(i * 2 * M_PI / theta_num) * sin(j * M_PI / fi_num);
      z = radius * cos(i * 2 * M_PI / theta_num);
      temp_point.x = x;
      temp_point.y = y;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 1.0 / 2.0;
      sphere_cloud.push_back(temp_point);
    }
  }

  return sphere_cloud;
}

// 生成一个“立方体表面”点云
pcl::PointCloud<PointType> generate_box_cloud(double size)
{
  pcl::PointCloud<PointType> box_cloud;
  PointType temp_point;
  double x, y, z;

  // use downsample resolution to compute how many points in a box
  int x_num = size / downsample_res;
  int y_num = size / downsample_res;
  int z_num = size / downsample_res;

  // for (int i = 0; i < x_num; i++)
  // {
  //   for (int j = 0; j < y_num; j++)
  //   {
  //     for (int k = 0; k < z_num; k++)
  //     {
  //       x = i * downsample_res - size / 2;
  //       y = j * downsample_res - size / 2;
  //       z = k * downsample_res - size / 2;
  //       temp_point.x = x;
  //       temp_point.y = y;
  //       temp_point.z = z;
  //       temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
  //       box_cloud.push_back(temp_point);
  //     }
  //   }
  // }

  // draw 2 xy plane
  // 先画上下两个水平面
  for (int i = 0; i <= x_num; i++)
  {
    for (int j = 0; j <= y_num; j++)
    {
      x = i * downsample_res - size / 2;
      y = j * downsample_res - size / 2;
      z = -size / 2;
      temp_point.x = x;
      temp_point.y = y;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
      box_cloud.push_back(temp_point);
      z = size / 2;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
      box_cloud.push_back(temp_point);
    }
  }
  // draw 4 plane
  // 再画四个竖直侧面
  for (int k = 1; k < z_num; k++)
  {
    // draw x two lines
    for (int i = 0; i <= x_num; i++)
    {
      x = i * downsample_res - size / 2;
      y = -size / 2;
      z = k * downsample_res - size / 2;
      temp_point.x = x;
      temp_point.y = y;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
      box_cloud.push_back(temp_point);

      x = i * downsample_res - size / 2;
      y = size / 2;
      z = k * downsample_res - size / 2;
      temp_point.x = x;
      temp_point.y = y;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
      box_cloud.push_back(temp_point);
    }
    for (int j = 0; j <= y_num; j++)
    {
      x = -size / 2;
      y = j * downsample_res - size / 2;
      z = k * downsample_res - size / 2;
      temp_point.x = x;
      temp_point.y = y;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
      box_cloud.push_back(temp_point);

      x = size / 2;
      y = j * downsample_res - size / 2;
      z = k * downsample_res - size / 2;
      temp_point.x = x;
      temp_point.y = y;
      temp_point.z = z;
      temp_point.intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 2.0 / 2.0;
      box_cloud.push_back(temp_point);
    }
  }

  return box_cloud;
}

// 按“障碍物类型”生成对应的点云，并平移到指定位置
void generate_ptclouds_by_pos(Eigen::Vector3d obs_pos, int obs_type, pcl::PointCloud<PointType> &obs_cloud, vector<PointType> &obs_points)
{
  // 0 for using uav model, 1 for using sphere model, 2 for using box model
  // 类型 0 = 用无人机模型，1 = 球，2 = 立方体
  switch (obs_type)
  {
  case 0:
    obs_cloud = uav_extra_model;
    for (int i = 0; i < obs_cloud.size(); i++)
    {
      obs_cloud.points[i].intensity = MIN_INTENSITY + (MAX_INTENSITY - MIN_INTENSITY) * 0.0 / 2.0;
    }
    break;

  case 1:
    obs_cloud = generate_sphere_cloud(dynobject_size);
    break;

  case 2:
    obs_cloud = generate_box_cloud(0.2);
    break;

  default:
    break;
  }

  // 把模型平移到目标位置，并追加到结果列表
  for (int i = 0; i < obs_cloud.size(); i++)
  {
    obs_cloud.points[i].x = obs_cloud.points[i].x + obs_pos(0);
    obs_cloud.points[i].y = obs_cloud.points[i].y + obs_pos(1);
    obs_cloud.points[i].z = obs_cloud.points[i].z + obs_pos(2);
    obs_points.push_back(obs_cloud.points[i]);
  }
}

// 生成/更新“动态障碍物”：按运动模式推进位置，再发布它们的点云
void dynobjGenerate()
{
  // 只有开了动态障碍、且地图已就绪时才执行
  if (has_global_map == true && dynobj_enable == 1)
  {
    dynobj_points.clear();
    dynobj_pointsindex.clear();
    dynobj_points_vis.clear();
    PointType temp_point;
    int dynpt_count = 0;
    double fly_time;
    Eigen::Vector3d gravity_vec;
    gravity_vec << 0, 0, -1;
    Eigen::Vector3d dyntemp_dir_polar;

    // rewrite generate dynamic obstacles
    // 逐个动态障碍物更新位置
    for (int n = 0; n < dynobject_num; n++)
    {
      // according to motion mode, compute the positions of dynamic obstacle
      // 运动模式：0=匀速直线，1=受重力加速，2=随机游走
      switch (dynobj_move_modes[n])
      {
      case 0:
      {
        // constant speed mode
        fly_time = (ros_node->now() - dyn_start_time_vec[n]).seconds();
        dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];
        break;
      }

      case 1:
      {
        // constant gravity acceleration mode
        fly_time = (ros_node->now() - dyn_start_time_vec[n]).seconds();
        dynobj_dir[n] = dynobj_dir[n] + gravity_vec * fly_time;
        dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];
        break;
      }

      case 2:
      {
        // random walk mode
        fly_time = (ros_node->now() - dyn_start_time_vec[n]).seconds();
        dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
        dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
        dynobj_dir[n](0) = dyn_velocity * sin(dyntemp_dir_polar(1));
        dynobj_dir[n](1) = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
        dynobj_dir[n](2) = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
        dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];
        break;
      }

      default:
        break;
      }

      // if dynamic obstacle is out of bounding box, regenerte one
      // 跑出场地范围 → 随机重生一个位置和方向
      if (dynobj_poss[n](0) < map_min(0) || dynobj_poss[n](0) > map_max(0) || dynobj_poss[n](1) < map_min(1) || dynobj_poss[n](1) > map_max(1) || dynobj_poss[n](2) < map_min(2) || dynobj_poss[n](2) > map_max(2))
      {
        dynobj_poss[n](0) = rand() / double(RAND_MAX) * (map_max(0) - map_min(0)) + map_min(0);
        dynobj_poss[n](1) = rand() / double(RAND_MAX) * (map_max(1) - map_min(1)) + map_min(1);
        dynobj_poss[n](2) = rand() / double(RAND_MAX) * (map_max(2) - map_min(2)) + map_min(2);
        Eigen::Vector3d dyntemp_dir_polar;
        dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
        dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
        dynobj_dir[n](0) = dyn_velocity * sin(dyntemp_dir_polar(1));
        dynobj_dir[n](1) = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
        dynobj_dir[n](2) = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
      }
      // 重置计时基准（下次用新的时间差算位移）
      dyn_start_time_vec[n] = ros_node->now();
      // generate point cloud of dynamic obstacle
      pcl::PointCloud<PointType> dynobj_cloud;
      vector<PointType> temp_dynobj_points;
      // 生成它的点云并累加进总列表
      generate_ptclouds_by_pos(dynobj_poss[n], dynobj_types[n], dynobj_cloud, temp_dynobj_points);
      dynobj_points_vis = dynobj_points_vis + dynobj_cloud;
      dynobj_points.insert(dynobj_points.end(), temp_dynobj_points.begin(), temp_dynobj_points.end());
      for (int i = 0; i < temp_dynobj_points.size(); i++)
      {
        dynobj_pointsindex.push_back(dynpt_count + origin_mapptcount);
        dynpt_count++;
      }
    }

    // // first type dynamic obstacles, different sizes sphere
    // //! you can add mode here
    // switch (dyn_mode)
    // {
    //   case 0:
    //     // constant speed mode
    //     // fly_time = (ros::Time::now() - dyn_start_time).toSec();
    //     for (int n = 0; n < dynobject_num; n++)
    //     {
    //       fly_time = (ros::Time::now() - dyn_start_time_vec[n]).toSec();
    //       dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];
    //       // if dynamic obstacle is out of bounding box, regenerte one
    //       if (dynobj_poss[n](0) < map_min(0) || dynobj_poss[n](0) > map_max(0) || dynobj_poss[n](1) < map_min(1) || dynobj_poss[n](1) > map_max(1) || dynobj_poss[n](2) < map_min(2) || dynobj_poss[n](2) > map_max(2))
    //       {
    //         dynobj_poss[n](0) = rand() / double(RAND_MAX) * (map_max(0) - map_min(0)) + map_min(0);
    //         dynobj_poss[n](1) = rand() / double(RAND_MAX) * (map_max(1) - map_min(1)) + map_min(1);
    //         dynobj_poss[n](2) = rand() / double(RAND_MAX) * (map_max(2) - map_min(2)) + map_min(2);
    //         Eigen::Vector3d dyntemp_dir_polar;
    //         dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
    //         dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
    //         dynobj_dir[n](0) = dyn_velocity * sin(dyntemp_dir_polar(1));
    //         dynobj_dir[n](1) = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
    //         dynobj_dir[n](2) = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
    //         dyn_start_time_vec[n] = ros::Time::now();
    //       }
    //     }
    //     break;
    //   case 1:
    //     // with gravity
    //     // fly_time = (ros::Time::now() - dyn_start_time_vec[n]).toSec();
    //     // ROS_INFO("In dyn mode 1");
    //     for (int n = 0; n < dynobject_num; n++)
    //     {
    //       fly_time = (ros::Time::now() - dyn_start_time_vec[n]).toSec();
    //       Eigen::Vector3d gravity_vec;
    //       gravity_vec<<0,0,-9.81;
    //       // cout << "dyn_obj_pos" << dynobj_poss[n].transpose() << ", gravity value = "<< 0.5 * gravity_vec.transpose() * fly_time * fly_time<<endl;
    //       dynobj_dir[n] = dynobj_dir[n] + gravity_vec * fly_time;
    //       dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];// + 0.5 * gravity_vec * fly_time * fly_time;
    //       // if dynamic obstacle is out of bounding box, regenerte one
    //       if (dynobj_poss[n](0) < map_min(0) || dynobj_poss[n](0) > map_max(0) || dynobj_poss[n](1) < map_min(1) || dynobj_poss[n](1) > map_max(1) || dynobj_poss[n](2) < map_min(2) || dynobj_poss[n](2) > map_max(2))
    //       {
    //         dynobj_poss[n](0) = rand() / double(RAND_MAX) * (map_max(0) - map_min(0)) + map_min(0);
    //         dynobj_poss[n](1) = rand() / double(RAND_MAX) * (map_max(1) - map_min(1)) + map_min(1);
    //         dynobj_poss[n](2) = rand() / double(RAND_MAX) * (map_max(2) - map_min(2)) + map_min(2);
    //         Eigen::Vector3d dyntemp_dir_polar;
    //         dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
    //         dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
    //         dynobj_dir[n](0) = dyn_velocity * sin(dyntemp_dir_polar(1));
    //         dynobj_dir[n](1) = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
    //         dynobj_dir[n](2) = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
    //         dyn_start_time_vec[n] = ros::Time::now();
    //       }
    //     }
    //     break;
    //   case 2:
    //     // random walk
    //     for (int n = 0; n < dynobject_num; n++)
    //     {
    //       fly_time = (ros::Time::now() - dyn_start_time_vec[n]).toSec();
    //         Eigen::Vector3d dyntemp_dir_polar;
    //         dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
    //         dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
    //         dynobj_dir[n](0) = dyn_velocity * sin(dyntemp_dir_polar(1));
    //         dynobj_dir[n](1) = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
    //         dynobj_dir[n](2) = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
    //       dynobj_poss[n] = dynobj_poss[n] + fly_time * dynobj_dir[n];// + 0.5 * gravity_vec * fly_time * fly_time;
    //       // if dynamic obstacle is out of bounding box, regenerte one
    //       if (dynobj_poss[n](0) < map_min(0) || dynobj_poss[n](0) > map_max(0) || dynobj_poss[n](1) < map_min(1) || dynobj_poss[n](1) > map_max(1) || dynobj_poss[n](2) < map_min(2) || dynobj_poss[n](2) > map_max(2))
    //       {
    //         dynobj_poss[n](0) = rand() / double(RAND_MAX) * (map_max(0) - map_min(0)) + map_min(0);
    //         dynobj_poss[n](1) = rand() / double(RAND_MAX) * (map_max(1) - map_min(1)) + map_min(1);
    //         dynobj_poss[n](2) = rand() / double(RAND_MAX) * (map_max(2) - map_min(2)) + map_min(2);
    //         // Eigen::Vector3d dyntemp_dir_polar;
    //         dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
    //         dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
    //         dynobj_dir[n](0) = dyn_velocity * sin(dyntemp_dir_polar(1));
    //         dynobj_dir[n](1) = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
    //         dynobj_dir[n](2) = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
    //         dyn_start_time_vec[n] = ros::Time::now();
    //       }
    //     }
    //     break;
    // }

    // if(dyn_obs_diff_size_on == 1)
    // {
    //   for (int n = 0; n < dynobject_num; n++)
    //   {
    //     // double dynobject_rand_size = rand() / double(RAND_MAX) * (dynobject_size - downsample_res) + downsample_res;
    //     for (double i = dynobj_poss[n](0) - 0.5 * dyn_obs_size_vec[n]; i < dynobj_poss[n](0) + 0.5 * dyn_obs_size_vec[n]; i = i + downsample_res)
    //     {
    //       for (double j = dynobj_poss[n](1) - 0.5 * dyn_obs_size_vec[n]; j < dynobj_poss[n](1) + 0.5 * dyn_obs_size_vec[n]; j = j + downsample_res)
    //       {
    //         for (double k = dynobj_poss[n](2) - 0.5 * dyn_obs_size_vec[n]; k < dynobj_poss[n](2) + 0.5 * dyn_obs_size_vec[n]; k = k + downsample_res)
    //         {
    //           if (sqrt((i - dynobj_poss[n](0)) * (i - dynobj_poss[n](0)) + (j - dynobj_poss[n](1)) * (j - dynobj_poss[n](1)) + (k - dynobj_poss[n](2)) * (k - dynobj_poss[n](2))) > 0.5 * dyn_obs_size_vec[n])
    //           {
    //             continue;
    //           }
    //           temp_point.x = i;
    //           temp_point.y = j;
    //           temp_point.z = k;
    //           dynobj_points_vis.push_back(temp_point);
    //           dynobj_points.push_back(temp_point);

    //           dynobj_pointsindex.push_back(dynpt_count + origin_mapptcount);
    //           dynpt_count++;
    //         }
    //       }
    //     }
    //   }
    // }else{
    //   for (int n = 0; n < dynobject_num; n++)
    //   {
    //     for (double i = dynobj_poss[n](0) - 0.5 * dynobject_size; i < dynobj_poss[n](0) + 0.5 * dynobject_size; i = i + downsample_res)
    //     {
    //       for (double j = dynobj_poss[n](1) - 0.5 * dynobject_size; j < dynobj_poss[n](1) + 0.5 * dynobject_size; j = j + downsample_res)
    //       {
    //         for (double k = dynobj_poss[n](2) - 0.5 * dynobject_size; k < dynobj_poss[n](2) + 0.5 * dynobject_size; k = k + downsample_res)
    //         {
    //           if (sqrt((i - dynobj_poss[n](0)) * (i - dynobj_poss[n](0)) + (j - dynobj_poss[n](1)) * (j - dynobj_poss[n](1)) + (k - dynobj_poss[n](2)) * (k - dynobj_poss[n](2))) > 0.5 * dynobject_size)
    //           {
    //             continue;
    //           }
    //           temp_point.x = i;
    //           temp_point.y = j;
    //           temp_point.z = k;
    //           dynobj_points_vis.push_back(temp_point);
    //           dynobj_points.push_back(temp_point);

    //           dynobj_pointsindex.push_back(dynpt_count + origin_mapptcount);
    //           dynpt_count++;
    //         }
    //       }
    //     }
    //   }
    // }

    // 整理点云尺寸，发布出去；同时把点云喂给 KD 树（供碰撞检测使用）
    dynobj_points_vis.width = dynobj_points_vis.points.size();
    dynobj_points_vis.height = 1;
    dynobj_points_vis.is_dense = true;

    pcl::toROSMsg(dynobj_points_vis, dynobj_points_pcd);
    dynobj_points_pcd.header = odom_.header;
    dynobj_points_pcd.header.frame_id = "world";
    pub_dyncloud->publish(dynobj_points_pcd);

    kdtree_dyn.setInputCloud(dynobj_points_vis.makeShared());
    has_dyn_map = true;

    dyn_start_time = ros_node->now();
  }
}

// decentralize simulation, get other uav odometry and generate point clouds
// 收到其它无人机的里程计 → 用它的模型生成点云（多机仿真用）
void multiOdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg, int drone_id)
{
  Eigen::Vector3d uav_pos;
  uav_pos(0) = msg->pose.pose.position.x;
  uav_pos(1) = msg->pose.pose.position.y;
  uav_pos(2) = msg->pose.pose.position.z;
  other_uav_pos[drone_id] = uav_pos;
  other_uav_rcv_time[drone_id] = rclcpp::Time(msg->header.stamp).seconds();

  otheruav_points[drone_id].clear();
  otheruav_pointsindex[drone_id].clear();
  otheruav_points_vis.clear();

  PointType temp_point;
  int uavpt_count = 0;

  // 方式一：用简单的立方体近似这架无人机
  if (use_uav_extra_model == 0)
  {
    // generate uav point cloud
    double x_min = uav_pos(0) - 0.5 * uav_size[0];
    double y_min = uav_pos(1) - 0.5 * uav_size[1];
    double z_min = uav_pos(2) - 0.5 * uav_size[2];

    for (int i = 0; i < drone_drawpoints_num[0]; i++)
    {
      for (int j = 0; j < drone_drawpoints_num[1]; j++)
      {
        for (int k = 0; k < drone_drawpoints_num[2]; k++)
        {
          temp_point.x = i * downsample_res + x_min;
          temp_point.y = j * downsample_res + y_min;
          temp_point.z = k * downsample_res + z_min;
          temp_point.intensity = ((float)(MAX_INTENSITY - MIN_INTENSITY)) * ((drone_id + 1.0) / (float(drone_num))) + MIN_INTENSITY; // set the intensity of the point
          otheruav_points[drone_id].push_back(temp_point);
          otheruav_pointsindex[drone_id].push_back(uavpt_count + origin_mapptcount + 100000 + drone_id * uav_points_num);
          otheruav_points_vis.push_back(temp_point);
          uavpt_count++;
        }
      }
    }
  }
  // 方式二：加载真实 PCD 模型，按该机的姿态摆放
  else
  {
    // read pcd model from file and attach it to UAVs odom
    Eigen::Quaterniond pose;
    pose.x() = msg->pose.pose.orientation.x;
    pose.y() = msg->pose.pose.orientation.y;
    pose.z() = msg->pose.pose.orientation.z;
    pose.w() = msg->pose.pose.orientation.w;
    Eigen::Matrix3d uav_rot = pose.toRotationMatrix();
    for (int i = 0; i < uav_extra_model.points.size(); i++)
    {
      Eigen::Vector3d model_point;
      model_point(0) = uav_extra_model.points[i].x;
      model_point(1) = uav_extra_model.points[i].y;
      model_point(2) = uav_extra_model.points[i].z;
      model_point = uav_rot * model_point + uav_pos;
      temp_point.x = model_point(0);
      temp_point.y = model_point(1);
      temp_point.z = model_point(2);
      temp_point.intensity = ((float)(MAX_INTENSITY - MIN_INTENSITY)) * ((drone_id + 1.0) / (float(drone_num))) + MIN_INTENSITY; // set the intensity of the point
      otheruav_points[drone_id].push_back(temp_point);
      otheruav_pointsindex[drone_id].push_back(uavpt_count + origin_mapptcount + 100000 + drone_id * uav_points_num);
      otheruav_points_vis.push_back(temp_point);
      uavpt_count++;
    }
  }

  // 发布其它无人机的点云
  // publish uav point cloud
  sensor_msgs::msg::PointCloud2 otheruav_points_vis_pcd;
  pcl::toROSMsg(otheruav_points_vis, otheruav_points_vis_pcd);
  otheruav_points_vis_pcd.header.frame_id = "world";
  otheruav_points_vis_pcd.header.stamp = ros_node->now();
  pub_uavcloud->publish(otheruav_points_vis_pcd);
}

// 【核心回调】收到本机里程计：更新位姿、发布传感器位姿、并做碰撞自检
void rcvOdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  const auto &odom = *message;
  /*if(!has_global_map)
    return;*/
  // 记录“里程计已到齐”
  has_odom = true;
  odom_ = odom;

  // 把里程计拼成一个 4×4 的“机体 → 世界”变换矩阵
  Matrix4d body2world = Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond pose;
  pose.x() = odom.pose.pose.orientation.x;
  pose.y() = odom.pose.pose.orientation.y;
  pose.z() = odom.pose.pose.orientation.z;
  pose.w() = odom.pose.pose.orientation.w;
  body2world.block<3, 3>(0, 0) = pose.toRotationMatrix();
  body2world(0, 3) = odom.pose.pose.position.x;
  body2world(1, 3) = odom.pose.pose.position.y;
  body2world(2, 3) = odom.pose.pose.position.z;

  // convert to cam pose
  // 再乘上安装矩阵，得到“传感器 → 世界”
  sensor2world = body2world * sensor2body;

  // 发布相机或激光的位姿
  publishSensorPoses(odom_, body2world);

  // collision check
  // 可选的自检：看看机器人是不是已经撞进障碍里了
  if (has_global_map && collisioncheck_enable)
  {

    // collision check running time statistics
    const auto collision_start = std::chrono::steady_clock::now();

    PointType searchPoint;
    searchPoint.x = odom.pose.pose.position.x;
    searchPoint.y = odom.pose.pose.position.y;
    searchPoint.z = odom.pose.pose.position.z;
    // 在机器人周围查一下有没有地图点 —— 查到就说明撞了
    if (_kdtreeLocalMap.radiusSearch(searchPoint, collision_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      RCLCPP_ERROR(ros_node->get_logger(), "Environment collision detected");
    }
    // 再用动态障碍物的 KD 树查一次
    if (dynobj_enable && has_dyn_map)
    {
      if (kdtree_dyn.radiusSearch(searchPoint, collision_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
      {
        RCLCPP_ERROR(ros_node->get_logger(), "Dynamic-object collision detected");
      }
    }

    double collision_check_time =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - collision_start).count();
    collision_check_time_sum += collision_check_time;
    collision_check_time_count++;
    if (collision_check_time_count % 100 == 0)
    {
      // ROS_WARN("Collision check time: %f", collision_check_time_sum / collision_check_time_count);
    }
  }

}

// 【核心回调】收到全局地图：只处理一次，做下采样/法线估计/哈希分块等准备
void rcvGlobalPointCloudCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  // 地图只处理一次，之后再来就忽略
  if (has_global_map)
    return;

  RCLCPP_INFO(ros_node->get_logger(), "Global point cloud received");

  // 兼容两种点云格式：带 intensity 的、以及纯 XYZ 的
  pcl::PointCloud<PointType> cloud_input;
  const bool has_intensity = std::any_of(
      message->fields.begin(), message->fields.end(),
      [](const sensor_msgs::msg::PointField &field) { return field.name == "intensity"; });
  if (has_intensity)
  {
    pcl::fromROSMsg(*message, cloud_input);
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
    pcl::fromROSMsg(*message, xyz_cloud);
    cloud_input.reserve(xyz_cloud.size());
    for (const auto &point : xyz_cloud.points)
    {
      PointType point_with_intensity;
      point_with_intensity.x = point.x;
      point_with_intensity.y = point.y;
      point_with_intensity.z = point.z;
      point_with_intensity.intensity = 0.0F;
      cloud_input.push_back(point_with_intensity);
    }
    cloud_input.width = static_cast<std::uint32_t>(cloud_input.size());
    cloud_input.height = 1;
    cloud_input.is_dense = xyz_cloud.is_dense;
  }

  // 体素下采样，降低点数
  _voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(cloud_all_map);

  // 记下地图点的总数（后面用它区分“地图点”和“动态物体点”）
  origin_mapptcount = cloud_all_map.points.size();

  // 给地图建 KD 树（深度投影、碰撞检测都要用）
  _kdtreeLocalMap.setInputCloud(cloud_all_map.makeShared());

  // 估计每个点的法线 —— 后面“面插值”靠它把稀疏点云补成连续表面
  normalEstimation.setInputCloud(cloud_all_map.makeShared());
  // 对于每一个点都用半径为3cm的近邻搜索方式
  normalEstimation.setRadiusSearch(3.0 * downsample_res);
  // Kd_tree是一种数据结构便于管理点云以及搜索点云，法线估计对象会使用这种结构来找到哦啊最近邻点
  pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
  normalEstimation.setSearchMethod(kdtree);
  // 计算法线
  // 真正开始计算法线
  normalEstimation.compute(*all_normals);

  RCLCPP_INFO(ros_node->get_logger(), "Normal computation finished; map size=%d", origin_mapptcount);

  // trans the map into hash map
  PointType pt_in, center;
  long int ind_x, ind_y, ind_z;

  // get max xyz
  PointType global_mapmin;
  PointType global_mapmax;
  // 求地图的包围盒，作为哈希分块的范围
  pcl::getMinMax3D(cloud_all_map, global_mapmin, global_mapmax);
  map_min(0) = global_mapmin.x;
  map_min(1) = global_mapmin.y;
  map_min(2) = global_mapmin.z;
  map_max(0) = global_mapmax.x;
  map_max(1) = global_mapmax.y;
  map_max(2) = global_mapmax.z;

  RCLCPP_INFO(ros_node->get_logger(), "map bounds=(%f %f %f) (%f %f %f)",
              map_min(0), map_min(1), map_min(2), map_max(0), map_max(1), map_max(2));
  // 三个方向各分多少个“大方块”
  cube_numx = floor((global_mapmax.x - global_mapmin.x) / hash_cubesize) + 20;
  cube_numy = floor((global_mapmax.y - global_mapmin.y) / hash_cubesize) + 20;
  cube_numz = floor((global_mapmax.z - global_mapmin.z) / hash_cubesize) + 200;

  // 环境盒子的边界（往外多留一些余量）
  env_box.vertex_min[0] = round(global_mapmin.x / hash_cubesize) * hash_cubesize - 10 * hash_cubesize;
  env_box.vertex_min[1] = round(global_mapmin.y / hash_cubesize) * hash_cubesize - 10 * hash_cubesize;
  env_box.vertex_min[2] = round(global_mapmin.z / hash_cubesize) * hash_cubesize - 100 * hash_cubesize;
  env_box.vertex_max[0] = round(global_mapmax.x / hash_cubesize) * hash_cubesize + 10 * hash_cubesize;
  env_box.vertex_max[1] = round(global_mapmax.y / hash_cubesize) * hash_cubesize + 10 * hash_cubesize;
  env_box.vertex_max[2] = round(global_mapmax.z / hash_cubesize) * hash_cubesize + 100 * hash_cubesize;
  // 把环境交给视场检查器（它需要知道方块边长和范围）
  fov_checker.Set_Env(env_box);
  fov_checker.Set_BoxLength(hash_cubesize);

  // 把每个地图点按坐标塞进对应的哈希方块
  for (int i = 0; i < int(cloud_all_map.points.size()); i++)
  {
    pt_in = cloud_all_map.points[i];
    ind_x = (round((pt_in.x - env_box.vertex_min[0] + EPSS) / hash_cubesize));
    ind_y = (round((pt_in.y - env_box.vertex_min[1] + EPSS) / hash_cubesize));
    ind_z = (round((pt_in.z - env_box.vertex_min[2] + EPSS) / hash_cubesize));

    long int box_index = ind_x + ind_y * cube_numx + ind_z * cube_numx * cube_numy;
    point_hashmap[box_index].push_back(pt_in);
    pointindex_hashmap[box_index].push_back(i);
  }

  // 如果开了动态障碍，顺便随机初始化它们的位置、类型、方向
  if (dynobj_enable == 1)
  {
    // dynamic objects initial pos generate
    srand((unsigned)time(NULL));
    for (int i = 0; i < dynobject_num; i++)
    {
      Eigen::Vector3d dyntemp_pos;
      dyntemp_pos(0) = rand() / double(RAND_MAX) * (global_mapmax.x - global_mapmin.x) + global_mapmin.x;
      dyntemp_pos(1) = rand() / double(RAND_MAX) * (global_mapmax.y - global_mapmin.y) + global_mapmin.y;
      dyntemp_pos(2) = rand() / double(RAND_MAX) * (global_mapmax.z - global_mapmin.z) + global_mapmin.z;
      dynobj_poss.push_back(dyntemp_pos);

      // random generate dynamic object type and motion mode
      int dynobj_type = rand() % 3;
      dynobj_types.push_back(dynobj_type);
      // int dynobj_motion_mode = rand() % 2;
      dynobj_move_modes.push_back(dynobj_type);

      Eigen::Vector3d dyntemp_dir, dyntemp_dir_polar;
      dyntemp_dir_polar(0) = rand() / double(RAND_MAX) * 3.1415926;
      dyntemp_dir_polar(1) = (rand() / double(RAND_MAX) - 0.5) * 3.1415926;
      dyntemp_dir[2] = dyn_velocity * sin(dyntemp_dir_polar(1));
      dyntemp_dir[1] = dyn_velocity * cos(dyntemp_dir_polar(1)) * sin(dyntemp_dir_polar(0));
      dyntemp_dir[0] = dyn_velocity * cos(dyntemp_dir_polar(1)) * cos(dyntemp_dir_polar(0));
      dynobj_dir.push_back(dyntemp_dir);

      if (dyn_obs_diff_size_on)
      {
        double dynobject_rand_size = rand() / double(RAND_MAX) * (dynobject_size - downsample_res) + downsample_res;
        dyn_obs_size_vec.push_back(dynobject_rand_size);
      }

      dyn_start_time = ros_node->now();
      dyn_start_time_vec.push_back(dyn_start_time);
    }
  }

  // 标记“地图已经准备好了”
  has_global_map = true;
}

// 直角坐标 → 极坐标格子（水平角、竖直角、距离）
inline void euc2polar(Eigen::Vector3d &euc_pt, float length, polar3D *polar_pt)
{
  polar_pt->theta = round((atan2(euc_pt[1], euc_pt[0])) / M_PI * 180.0 / polar_resolution);
  polar_pt->fi = round((atan2(euc_pt[2], euc_pt.head<2>().norm()) / M_PI * 180.0 / polar_resolution));
  polar_pt->r = length;
}

// 反过来：极坐标格子 → 直角坐标
inline void polar2euc(polar3D *polar_pt, Eigen::Vector3d &euc_pt)
{
  // trans from polar coordinate to euclidean coordinate
  // theta_angle
  euc_pt[2] = polar_pt->r * sin(polar_pt->fi * polar_resolution / 180.0 * M_PI);
  euc_pt[1] = polar_pt->r * cos(polar_pt->fi * polar_resolution / 180.0 * M_PI) * sin(polar_pt->theta * polar_resolution / 180.0 * M_PI);
  euc_pt[0] = polar_pt->r * cos(polar_pt->fi * polar_resolution / 180.0 * M_PI) * cos(polar_pt->theta * polar_resolution / 180.0 * M_PI);
}

int sense_count = 0;
double duration1 = 0.0;
double duration2 = 0.0;
double duration3 = 0.0;
double duration4 = 0.0;
double duration5 = 0.0;
double duration6 = 0.0;
double duration7 = 0.0;
double duration8 = 0.0;
double duration_interline = 0.0;
int comp_time_count = 0;

void renderSensedPoints()
{

  // 开始计时（下面按 8 个阶段分别统计耗时，方便看性能瓶颈）
  auto t1 = std::chrono::steady_clock::now();

  // 地图或里程计还没到齐 → 这一帧什么都不做
  if (!has_global_map || !has_odom)
    return;

  // 深度相机模式：走一条完全不同的、更简单的路径，然后直接返回
  if (useDepthSensor())
  {
    publishPinholeDepth(rclcpp::Time(odom_.header.stamp));
    return;
  }

  // 取出机器人的姿态四元数
  Eigen::Quaterniond q;
  q.x() = odom_.pose.pose.orientation.x;
  q.y() = odom_.pose.pose.orientation.y;
  q.z() = odom_.pose.pose.orientation.z;
  q.w() = odom_.pose.pose.orientation.w;

  // 多机仿真时，把其它无人机的点云也纳入渲染
  // trans other uav point in
  if (otheruav_points.size() > 0)
  {
    otheruav_points_inrender = otheruav_points;
    otheruav_pointsindex_inrender = otheruav_pointsindex;
  }

  // 当前传感器（机体）位置
  const Eigen::Vector3d pos(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
  // pos << odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z;

  // const Eigen::Matrix3d rot(q.toRotationMatrix());
  // Eigen::Matrix3d rot_body2lidar = Matrix3d::Identity();
  
  //add pitchW
  // 激光的俯仰安装角（斜向下扫地面）
  double theta = lidar_pitch * M_PI / 180.0;
  Eigen::Matrix3d rot_body2lidar;
  rot_body2lidar << cos(theta), 0, sin(theta),
      0, 1, 0,
      -sin(theta), 0, cos(theta);

  // rotate lidar
  // Eigen::Vector3d eulerAngle_body2lidar(0,0.5236,0);
  // Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle_body2lidar(0),Vector3d::UnitX()));
  // Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle_body2lidar(1),Vector3d::UnitY()));
  // Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle_body2lidar(2),Vector3d::UnitZ()));
  // rot_body2lidar=yawAngle*pitchAngle*rollAngle;
  // rotate lidar end

  // 传感器总旋转 = 机体姿态 × 激光安装角
  const Eigen::Matrix3d rot(q.toRotationMatrix() * rot_body2lidar);

  // rot = q.toRotationMatrix();
  const Eigen::Vector3d yaw_x(1, 0, 0);
  // yaw_x << 1,0,0;

  // 传感器朝向在水平面的投影（判断视场时要用）
  Eigen::Vector3d rotmyaw = rot * yaw_x;
  const Eigen::Vector3d yaw_vec(rotmyaw(0), rotmyaw(1), 0);
  // yaw_vec(2) = 0;

  // 清空上一帧的结果
  local_map.points.clear();
  local_map_filled.points.clear();

  Eigen::Vector3d searchpoint_pos;
  Eigen::Vector3d sensorrange_vec;
  // 搜索球心放到前方半个射程处，让搜索范围更贴合实际视野
  sensorrange_vec << 0.5 * sensing_horizon, 0, 0;
  searchpoint_pos = pos + rot * sensorrange_vec;

  PointType searchPoint;
  searchPoint.x = searchpoint_pos(0);
  searchPoint.y = searchpoint_pos(1);
  searchPoint.z = searchpoint_pos(2);

  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  // polar coordinate matrix
  // add margin
  // 视场边缘留 8 度余量，避免边缘漏点
  double margin_ = 8.0;
  const double vertical_fov_margined_ = vertical_fov + margin_;
  // 极坐标栅格的高度 = 竖直视场 ÷ 角分辨率
  const int polar_height = ceil(vertical_fov_margined_ / polar_resolution);
  double temp_yaw;
  int temp_width;
  // 360 度激光用满一圈；否则用配置的水平视场
  if (is_360lidar == 1)
  {
    temp_yaw = 360.0;
    temp_width = ceil(360.0 / polar_resolution);
  }
  else
  {
    temp_yaw = yaw_fov + margin_;
    temp_width = ceil(temp_yaw / polar_resolution);
  }

  const double yaw_fov_margined_ = temp_yaw;
  const int polar_width = temp_width;

  // 极坐标“深度图”：每格存最近命中距离，初值设为两倍射程（表示“没打到东西”）
  Eigen::MatrixXf polar_matrix = Eigen::MatrixXf::Zero(polar_width, polar_height);
  polar_matrix.setConstant(2 * sensing_horizon);
  // 每格对应的地图点序号
  Eigen::MatrixXi polarindex_matrix = Eigen::MatrixXi::Zero(polar_width, polar_height);
  // 每格的来源类型：0=直接命中 1=近处泼溅 2=面插值 4=动态物体/其它无人机
  Eigen::MatrixXi polarindextype_matrix = Eigen::MatrixXi::Zero(polar_width, polar_height);
  // 0 means no interline, 1 means depth interline, 2 means normal interline, 3 means neighbor depth interline, 4 means dynamic or other uav points
  // 每格的强度值
  Eigen::MatrixXi polarpointintensity_matrix = Eigen::MatrixXi::Zero(polar_width, polar_height);
  polarpointintensity_matrix.setConstant(MIN_INTENSITY);
  // 该格是否允许被“面插值”覆盖
  Eigen::MatrixXi allowblur_matrix = Eigen::MatrixXi::Zero(polar_width, polar_height);
  int original_pointcount = 0;
  int pointcount = 0;
  int changepointcount = 0;

  // 预计算几个视场相关的三角函数值
  const double tan_ver_ = tan(M_PI * (vertical_fov_margined_ / 2.0) / 180.0);
  const double yaw_ = (M_PI * (yaw_fov_margined_ / 2.0) / 180.0);
  const double ver_threshold = cos(M_PI * (vertical_fov_margined_ / 2.0) / 180.0);
  // 一个点大致能盖住多大范围（用于“泼溅”）
  const double cover_dis = 0.55 * 1.7321 * downsample_res; // 0.707
  // compute effective range
  const double effect_range = cover_dis / sin(0.5 * polar_resolution * M_PI / 180.0);
  // ROS_INFO("POLAR R = %f, EFFECT RANGE = %f",polar_pt.r,effect_range);

  double max_fov_angle;
  if (yaw_fov_margined_ > vertical_fov_margined_)
  {
    max_fov_angle = yaw_fov_margined_;
  }
  else
  {
    max_fov_angle = vertical_fov_margined_;
  }

  // save the points need to interline
  // 待做面插值的点下标列表
  vector<int> culling_vectheta;
  vector<int> culling_vecfi;
  vector<int> culling_kdindex;

  sense_count++;
  auto t2 = std::chrono::steady_clock::now();
  duration1 += std::chrono::duration<double>(t2 - t1).count();

  // pattern
  // 扫描图案矩阵：只有被激光真正扫到的格子才会输出点
  Eigen::MatrixXf pattern_matrix = Eigen::MatrixXf::Zero(polar_width, polar_height);
  pattern_matrix.setConstant(1);

  // avia pattern
  // 下面几个 if：按不同型号激光器生成各自的扫描花纹
  if (use_avia_pattern == 1)
  {
    float w1 = 763.82589;     // 7294.0*2.0*3.1415926/60.0
    float w2 = -488.41293788; // -4664.0*2.0*3.1415926/60.0
    // int linestep = 2;
    // double point_duration = 0.000025;
    double point_duration = 0.000004167 * 6;

    float t_duration = 1.0 / sensing_rate;
    double t_start = (ros_node->now() - start_time).seconds();

    double scale_x = 0.48 * polar_width / 2.0;
    double scale_y = 0.43 * polar_height / 2.0;

    int linestep = ceil(livox_linestep / polar_resolution);

    // 按扫描频率在时间上铺点，形成真实的花纹（非均匀、有间隔）
    for (double t_i = t_start; t_i < t_start + t_duration; t_i = t_i + point_duration)
    {
      int x = round(scale_x * (cos(w1 * t_i) + cos(w2 * t_i))) + round(0.5 * polar_width);
      int y = round(scale_y * (sin(w1 * t_i) + sin(w2 * t_i))) + round(0.5 * polar_height);

      if (x > (polar_width - 1))
      {
        x = (polar_width - 1);
      }
      else if (x < 0)
      {
        x = 0;
      }

      if (y > (polar_height - 1))
      {
        y = (polar_height - 1);
      }
      else if (y < 0)
      {
        y = 0;
      }

      pattern_matrix(x, y) = 2; // real pattern
      pattern_matrix(x, y + linestep) = 2;
      pattern_matrix(x, y + 2 * linestep) = 2;
      pattern_matrix(x, y + 3 * linestep) = 2;
      pattern_matrix(x, y - linestep) = 2;
      pattern_matrix(x, y - 2 * linestep) = 2;
    }
  }

  // vlp32 pattern
  if (use_vlp32_pattern == 1)
  {
    int step = round((40.0) / 12 / polar_resolution);
    int bottom_pattern = -ceil(25.0 / polar_resolution);
    for (int i = 0; i < 12; i++)
    {
      int y = bottom_pattern + i * step + round(0.5 * polar_height);

      for (int j = 0; j < polar_width; j++)
      {
        if (y > (polar_height - 1))
        {
          y = (polar_height - 1);
        }
        else if (y < 0)
        {
          y = 0;
        }
        pattern_matrix(j, y) = 2;
      }
    }
  }

  // os128 pattern
  if (use_os128_pattern == 1)
  {
    int step = round((45.0) / 128 / polar_resolution);
    int bottom_pattern = -ceil(22.5 / polar_resolution);
    for (int i = 0; i < 128; i++)
    {
      int y = bottom_pattern + i * step + round(0.5 * polar_height);

      for (int j = 0; j < polar_width; j++)
      {
        if (y > (polar_height - 1))
        {
          y = (polar_height - 1);
        }
        else if (y < 0)
        {
          y = 0;
        }
        pattern_matrix(j, y) = 2;
      }
    }
  }
  if (use_os64_pattern == 1)
  {
    int step = round((90.0) / 64 / polar_resolution);
    int bottom_pattern = -ceil(45.0 / polar_resolution);
    for (int i = 0; i < 64; i++)
    {
      int y = bottom_pattern + i * step + round(0.5 * polar_height);

      for (int j = 0; j < polar_width; j++)
      {
        if (y > (polar_height - 1))
        {
          y = (polar_height - 1);
        }
        else if (y < 0)
        {
          y = 0;
        }
        pattern_matrix(j, y) = 2;
      }
    }
  }
  // 本项目默认使用这一种图案
  if (use_minicf_pattern == 1)
  {
    double point_duration = 1.0 / 200000.0;

    float t_duration = 1.0 / sensing_rate;
    double t_start = (ros_node->now() - start_time).seconds();

    double scale_x = 0.48 * polar_width / 2.0;
    double scale_y = 0.43 * polar_height / 2.0;
    double PI = 3.141519265357;

    for (double t_i = t_start; t_i < t_start + t_duration; t_i = t_i + point_duration)
    {
      int x = (int(-round(-62050.63 * t_i + 3.11 * cos(314159.2 * t_i) * sin(628.318 * 2 * t_i))) % 360) / polar_resolution;
      int y = round(25.5 * cos(20 * PI * t_i) + 4 * cos(2 * PI / 0.006 * t_i) * cos(10000 * PI * t_i) + 22.5) / polar_resolution + round(0.5 * polar_height);

      // ROS_INFO("X = %d, Y = %d",x,y);
      if (x > (polar_width - 1))
      {
        x = (polar_width - 1);
      }
      else if (x < 0)
      {
        x = 0;
      }

      if (y > (polar_height - 1))
      {
        y = (polar_height - 1);
      }
      else if (y < 0)
      {
        y = 0;
      }

      pattern_matrix(x, y) = 2; // real pattern
    }
  }

  auto t3 = std::chrono::steady_clock::now();
  duration2 += std::chrono::duration<double>(t3 - t2).count();

  // 下面开始收集“落在视野内”的所有地图点
  // hashmap with fov checker
  vector<BoxPointType> fov_boxes;
  // rotmyaw << 0,1,0;
  // ROS_INFO("POS = %f,%f,%f, rotmyaw = %f,%f,%f", pos(0), pos(1),pos(2), rotmyaw(0),rotmyaw(1),rotmyaw(2));
  vector<PointType> fov_points;
  vector<int> fov_pointsindex;

  // 360 度激光：直接取传感器周围一定范围的哈希方块
  if (is_360lidar == 1)
  {
    // box expand to search
    int expand_num = ceil(sensing_horizon / hash_cubesize);
    int cen_x = (round((pos(0) - env_box.vertex_min[0] + EPSS) / hash_cubesize));
    int cen_y = (round((pos(1) - env_box.vertex_min[1] + EPSS) / hash_cubesize));
    int cen_z = (round((pos(2) - env_box.vertex_min[2] + EPSS) / hash_cubesize));
    long int ind_x, ind_y, ind_z;
    for (int i = -expand_num; i <= expand_num; i++)
    {
      for (int j = -expand_num; j <= expand_num; j++)
      {
        for (int k = -expand_num; k <= expand_num; k++)
        {
          ind_x = cen_x + i;
          ind_y = cen_y + j;
          ind_z = cen_z + k;

          long int box_index = ind_x + ind_y * cube_numx + ind_z * cube_numx * cube_numy;
          fov_points.insert(fov_points.end(), point_hashmap[box_index].begin(), point_hashmap[box_index].end());
          fov_pointsindex.insert(fov_pointsindex.end(), pointindex_hashmap[box_index].begin(), pointindex_hashmap[box_index].end());
        }
      }
    }
    // ROS_INFO("POINT SIZE = %d, box size = %d", fov_points.size(), fov_boxes.size());
  }
  else
  {
    // 非全向激光：用视场检查器算出可见的方块，再取里面的点
    fov_checker.check_fov(pos, rotmyaw, M_PI * (max_fov_angle / 2.0 + 10) / 180.0, sensing_horizon, fov_boxes);
    Eigen::Vector3d box_center_temp;
    long int ind_x, ind_y, ind_z;
    for (int i = 0; i < fov_boxes.size(); i++)
    {
      box_center_temp(0) = (fov_boxes[i].vertex_max[0] - fov_boxes[i].vertex_min[0]) * 0.5 + fov_boxes[i].vertex_min[0];
      box_center_temp(1) = (fov_boxes[i].vertex_max[1] - fov_boxes[i].vertex_min[1]) * 0.5 + fov_boxes[i].vertex_min[1];
      box_center_temp(2) = (fov_boxes[i].vertex_max[2] - fov_boxes[i].vertex_min[2]) * 0.5 + fov_boxes[i].vertex_min[2];

      ind_x = (round((box_center_temp(0) - env_box.vertex_min[0] + EPSS) / hash_cubesize));
      ind_y = (round((box_center_temp(1) - env_box.vertex_min[1] + EPSS) / hash_cubesize));
      ind_z = (round((box_center_temp(2) - env_box.vertex_min[2] + EPSS) / hash_cubesize));

      PointType box_pt;
      box_pt.x = box_center_temp(0);
      box_pt.y = box_center_temp(1);
      box_pt.z = box_center_temp(2);

      long int box_index = ind_x + ind_y * cube_numx + ind_z * cube_numx * cube_numy;

      fov_points.insert(fov_points.end(), point_hashmap[box_index].begin(), point_hashmap[box_index].end());
      fov_pointsindex.insert(fov_pointsindex.end(), pointindex_hashmap[box_index].begin(), pointindex_hashmap[box_index].end());
    }
  }

  // add dynamic objects
  // Step 1: add dynpoints int fov points
  // 把动态障碍物的点也加进来
  if (dynobj_enable == 1)
  {
    fov_points.insert(fov_points.end(), dynobj_points.begin(), dynobj_points.end());
    fov_pointsindex.insert(fov_pointsindex.end(), dynobj_pointsindex.begin(), dynobj_pointsindex.end());
  }

  // 其它无人机的点也加进来
  if (drone_num > 1)
  {
    for (int i = 0; i < drone_num; i++)
    {
      fov_points.insert(fov_points.end(), otheruav_points_inrender[i].begin(), otheruav_points_inrender[i].end());
      fov_pointsindex.insert(fov_pointsindex.end(), otheruav_pointsindex_inrender[i].begin(), otheruav_pointsindex_inrender[i].end());
    }
  }

  auto t4 = std::chrono::steady_clock::now();
  duration3 += std::chrono::duration<double>(t4 - t3).count();

  const size_t size_ = fov_points.size();

  // 下面用 32 个线程并行处理视野内的每一个点
  omp_set_num_threads(32);

#ifndef DEBUG
#pragma omp parallel /*default(none)*/                                                         \
shared(pattern_matrix, polar_matrix, cloud_all_map, pointIdxRadiusSearch,                      \
           min_raylength, downsample_res, polar_resolution, use_avia_pattern, _kdtreeLocalMap, \
           polarindex_matrix, local_map_filled, local_map,                                     \
           all_normals, vertical_fov, yaw_fov, sensing_horizon, fov_points, fov_pointsindex,   \
           polarindextype_matrix, original_pointcount, curvature_limit, allowblur_matrix,      \
           origin_mapptcount, drone_num, polarpointintensity_matrix, uav_points_num)
  {
#pragma omp for
#endif
    // 逐个点投射到极坐标深度图上
    for (size_t i = 0; i < size_; ++i)
    {
      // auto pt = cloud_all_map.points[pointIdxRadiusSearch[i]];
      auto pt = fov_points[i];
      Eigen::Vector3d pt3;
      pt3[0] = pt.x;
      pt3[1] = pt.y;
      pt3[2] = pt.z;
      // 点相对传感器的方向向量
      auto dir = pt3 - pos;

      // 超出射程 → 这一帧看不到它
      if (dir.norm() > sensing_horizon)
      {
        continue;
      }

      original_pointcount++;

      // use polar to filter the right pointclouds
      polar3D polar_pt;
      Eigen::Vector3d dir_vec;
      // dir_vec = pt3 - pos;

      // trans coordinate to lidar coordinate
      // 把方向转到传感器坐标系下
      dir_vec = rot.transpose() * dir;

      // 换算成极坐标格子（水平角、竖直角、距离）
      euc2polar(dir_vec, dir_vec.norm(), &polar_pt);
      // ROS_INFO("dir_vec = %f,%f,%f, polar = %d,%d",dir(0),dir(1),dir(2), polar_pt.theta,polar_pt.fi);
      // 格子下标要加中心偏移（极坐标以 0 为中心、下标以 0 为起点）
      int cen_theta_index = polar_pt.theta + round(0.5 * polar_width);
      int cen_fi_index = polar_pt.fi + round(0.5 * polar_height);

      // 这个点会盖住多大的角度范围（近处的点盖得更宽）
      int half_cover_angle = ceil(
          (asin(cover_dis / dir_vec.norm()) / (M_PI * polar_resolution / 180.0)));
      // ROS_INFO("half cover angle = %d",half_cover_angle);
      // int half_cover_angle = 1;

      // 远点：只写中心那一格即可
      if (polar_pt.r > effect_range)
      {

        if (unlikely((cen_theta_index > (polar_width - 1)) || (cen_theta_index < 0) || (cen_fi_index > (polar_height - 1)) || (cen_fi_index < 0)))
        {
          continue;
        }

        // 只有比已有距离更近才覆盖 —— 这就是遮挡关系
        if (polar_matrix(cen_theta_index, cen_fi_index) > polar_pt.r)
        {

          polar_matrix(cen_theta_index, cen_fi_index) = polar_pt.r;
          polarindextype_matrix(cen_theta_index, cen_fi_index) = 0;
          allowblur_matrix(cen_theta_index, cen_fi_index) = 0;
          // polarindex_matrix(cen_theta_index,cen_fi_index) = pointIdxRadiusSearch[i];

          if ((fov_pointsindex[i] >= origin_mapptcount + 100000) && drone_num > 1)
          {
            int point_temp_index = fov_pointsindex[i] - origin_mapptcount - 100000;
            int droneid_temp = (point_temp_index / (uav_points_num));
            polarpointintensity_matrix(cen_theta_index, cen_fi_index) = fov_points[i].intensity;
          }
          else
          {
            polarpointintensity_matrix(cen_theta_index, cen_fi_index) = MIN_INTENSITY;
          }
        }
      }
      // 近点：把周围一小片格子都填上，避免近处出现空洞
      else
      {

        int theta_start = cen_theta_index - half_cover_angle;
        int theta_end = cen_theta_index + half_cover_angle;
        int fi_start = cen_fi_index - half_cover_angle;
        int fi_end = cen_fi_index + half_cover_angle;

        for (int theta_index_o = theta_start;
             theta_index_o <= theta_end; theta_index_o++)
        {
          for (int fi_index_o = fi_start;
               fi_index_o <= fi_end; fi_index_o = fi_index_o + 1)
          {

            if (unlikely((theta_index_o > (polar_width - 1)) || (theta_index_o < 0) || (fi_index_o > (polar_height - 1)) || (fi_index_o < 0)))
            {
              continue;
            }
            // 近点用“距离 + 射程”作为标记，表示“这里是被近处物体挡住的”
            if (polar_matrix(theta_index_o, fi_index_o) > polar_pt.r + sensing_horizon)
            {
              polar_matrix(theta_index_o, fi_index_o) = polar_pt.r + sensing_horizon;
              polarindex_matrix(theta_index_o, fi_index_o) = fov_pointsindex[i];
              polarindextype_matrix(theta_index_o, fi_index_o) = 1;
              allowblur_matrix(theta_index_o, fi_index_o) = 0;
              if ((fov_pointsindex[i] >= origin_mapptcount + 100000) && drone_num > 1)
              {
                polarpointintensity_matrix(theta_index_o, fi_index_o) = fov_points[i].intensity;
              }
              else
              {
                polarpointintensity_matrix(theta_index_o, fi_index_o) = MIN_INTENSITY;
              }
            }
          }
        }

      } // end of for loop
    }

#ifndef DEBUG
  }
#endif
  auto t5 = std::chrono::steady_clock::now();
  duration4 += std::chrono::duration<double>(t5 - t4).count();

  // ROS_INFO("After first filter");
  auto t6 = std::chrono::steady_clock::now();

  // 第二遍：面插值 —— 用每个点的法线，把离散的点“抹”成连续表面
  if (plane_interline == 1)
  {

#pragma omp parallel /*default(none)*/ \
shared(polarindex_matrix, culling_kdindex)
    {
      std::vector<int> vec_private;

#pragma omp for nowait collapse(2)
      // compute plane interline
      // 先收集所有需要插值的点下标（去重）
      for (int i = 0; i < polar_width; i++)
      {
        for (int j = 0; j < polar_height; j++)
        {
          if (polarindex_matrix(i, j) != 0)
          {
            // check if the index in the vector
            if (std::find(vec_private.begin(), vec_private.end(), polarindex_matrix(i, j)) !=
                vec_private.end())
            {
              continue;
            }
            else
            {
              vec_private.push_back(polarindex_matrix(i, j));
            }
          }
        }
      }
#pragma omp critical
      culling_kdindex.insert(culling_kdindex.end(), vec_private.begin(), vec_private.end());
    }
    // ROS_INFO("CULLING COUNT = %d",culling_kdindex.size());

    t6 = std::chrono::steady_clock::now();
    duration5 += std::chrono::duration<double>(t6 - t5).count();
    duration_interline = 0;

    // omp_set_num_threads(32);
    std::vector<int>::iterator iter;
    int culling_size = culling_kdindex.size();

    double duration_interline = 0.0;
    double duration_direct = 0.0;

#ifndef DEBUG
#pragma omp parallel /*default(none)*/                                                            \
shared(culling_kdindex,                                                                           \
           polar_matrix, cloud_all_map,                                                           \
           downsample_res, polar_resolution,                                                      \
           all_normals, curvature_limit, fov_points, origin_mapptcount, dynobj_points,            \
           polarindextype_matrix, sensing_horizon, dynobj_enable, culling_size, allowblur_matrix, \
           otheruav_points_inrender, drone_num, uav_points_num, polarpointintensity_matrix)
    {
#pragma omp for
#endif
      // for(iter = culling_kdindex.begin();iter!=culling_kdindex.end();++iter)
      // 对每个候选点做插值计算
      for (int i = 0; i < culling_size; i++)
      {
        rclcpp::Time t_in1 = ros_node->now();

        // multi uav add
        int droneid_temp, point_temp_index;

        int point_index = culling_kdindex[i];

        // check if it is dyn points
        PointType pt;
        // 判断这个点是地图点、动态物体点、还是其它无人机的点
        if (likely(point_index < origin_mapptcount))
        {
          pt = cloud_all_map.points[point_index];
        }
        else
        {
          if (dynobj_enable == 1)
          {
            pt = dynobj_points[point_index - origin_mapptcount];
          }
          else if (drone_num > 1)
          {
            point_temp_index = point_index - origin_mapptcount - 100000;
            droneid_temp = (point_temp_index / (uav_points_num));

            if ((point_temp_index - droneid_temp * uav_points_num) > (otheruav_points_inrender[droneid_temp].size() - 1))
              continue;
            pt = otheruav_points_inrender[droneid_temp][point_temp_index - droneid_temp * uav_points_num];
          }
          else
          {
            continue;
          }
        }

        Eigen::Vector3d pt3;
        pt3[0] = pt.x;
        pt3[1] = pt.y;
        pt3[2] = pt.z;
        auto dir = pt3 - pos;

        polar3D polar_pt;
        Eigen::Vector3d dir_vec;
        // dir_vec = pt3 - pos;

        // trans coordinate to lidar coordinate
        dir_vec = rot.transpose() * dir;
        double pt_dis = dir_vec.norm();
        euc2polar(dir_vec, pt_dis, &polar_pt);
        // ROS_INFO("dir_vec = %f,%f,%f, polar = %d,%d",dir(0),dir(1),dir(2), polar_pt.theta,polar_pt.fi);
        int cen_theta_index = polar_pt.theta + round(0.5 * polar_width);
        int cen_fi_index = polar_pt.fi + round(0.5 * polar_height);

        int half_cover_angle = ceil(
            (asin(cover_dis / pt_dis) / (M_PI * polar_resolution / 180.0)));

        rclcpp::Time t_in2 = ros_node->now();
        // ROS_INFO("Init using %f s",(t_in2-t_in1).toSec());

        //! check if it is dyn points
        if (likely(point_index < origin_mapptcount))
        {

          // 取出该点的法线 —— 它代表这个点所在“面”的朝向
          Eigen::Vector3d plane_normal;
          plane_normal(0) = all_normals->points[point_index].normal_x;
          plane_normal(1) = all_normals->points[point_index].normal_y;
          plane_normal(2) = all_normals->points[point_index].normal_z;
          float curvature;
          curvature = all_normals->points[point_index].curvature;

          // 法线无效（没估计出来）→ 跳过这个点
          if (isnan(plane_normal(0)) || isnan(plane_normal(1)) || isnan(plane_normal(2))) //|| curvature > 10*(1.0/(0.5*downsample_res))
          {
            continue;
          }

          int theta_start = cen_theta_index - half_cover_angle;
          int theta_end = cen_theta_index + half_cover_angle;
          int fi_start = cen_fi_index - half_cover_angle;
          int fi_end = cen_fi_index + half_cover_angle;

          int theta_index_o = theta_start;
          int fi_index_o = fi_start;

          // 在它覆盖的每个角度格子上，做“射线与该平面求交”
          for (int theta_index_o = theta_start;
               theta_index_o <= theta_end; theta_index_o++)
          {
            for (int fi_index_o = fi_start;
                 fi_index_o <= fi_end; fi_index_o = fi_index_o + 1)
            {
              // ros::Time t8 = ros::Time::now();
              // ros::Time t_in3 = ros::Time::now();

              polar3D cur_polarpt;
              Eigen::Vector3d ray, inter_point, inter_point_world;
              double vpt, line_t;

              if (unlikely((theta_index_o > (polar_width - 1)) || (theta_index_o < 0) || (fi_index_o > (polar_height - 1)) || (fi_index_o < 0)))
              {
              }
              else
              {
                // compute plane interline
                cur_polarpt.theta = theta_index_o - round(0.5 * polar_width);
                cur_polarpt.fi = fi_index_o - round(0.5 * polar_height);
                cur_polarpt.r = 1.0;
                // 把这一格的方向还原成一条射线
                polar2euc(&cur_polarpt, ray);
                ray = rot * ray;
                ray.normalize();
                vpt = ray.dot(plane_normal);

                // 求射线与“该点所在平面”的交点
                line_t = dir.dot(plane_normal) / vpt;
                inter_point_world = pos + line_t * ray;

                // 交点离原始点太远 → 这个面不该延伸到这么远，跳过
                if ((inter_point_world - pt3).norm() > 1 * 0.8660254 * downsample_res) // sqrt 3   0.5*1.7321  2*0.8660254*downsample_res
                {
                  // ROS_INFO("OUT OF LIMIT");
                }
                else
                {
                  inter_point = rot.transpose() * (inter_point_world - pos);
                  // 交点更近才覆盖，并把该格标记为“面插值”
                  if (polar_matrix(theta_index_o, fi_index_o) > inter_point.norm())
                  {
                    polar_matrix(theta_index_o, fi_index_o) = inter_point.norm();
                    polarindextype_matrix(theta_index_o, fi_index_o) = 2;

                    // limit curvature, avoid a good plane to be interlined
                    if (curvature > 0.05)
                    {
                      // polarindextype_matrix(theta_index_o,fi_index_o) = 3;
                      allowblur_matrix(theta_index_o, fi_index_o) = 1;
                    }
                    else
                    {
                      // polarindextype_matrix(theta_index_o,fi_index_o) = 2;
                      allowblur_matrix(theta_index_o, fi_index_o) = 1;
                    }
                  }
                }
              }
              //  ros::Time t_in4 = ros::Time::now();
              //   duration_interline =duration_interline + (t_in4-t_in3).toSec();//
            }
          }
        }
        // 动态物体/其它无人机不做面插值，直接按球面覆盖
        else
        {

          rclcpp::Time t_in5 = ros_node->now();

          int theta_start = cen_theta_index - half_cover_angle;
          int theta_end = cen_theta_index + half_cover_angle;
          int fi_start = cen_fi_index - half_cover_angle;
          int fi_end = cen_fi_index + half_cover_angle;

          for (int theta_index_o = theta_start;
               theta_index_o <= theta_end; theta_index_o++)
          {
            for (int fi_index_o = fi_start;
                 fi_index_o <= fi_end; fi_index_o++)
            {
              // ros::Time t8 = ros::Time::now();
              if (unlikely((theta_index_o > (polar_width - 1)) || (theta_index_o < 0) || (fi_index_o > (polar_height - 1)) || (fi_index_o < 0)))
              {
                continue;
              }
              if (polar_matrix(theta_index_o, fi_index_o) > polar_pt.r)
              {

                polar_matrix(theta_index_o, fi_index_o) = polar_pt.r;
                polarindextype_matrix(theta_index_o, fi_index_o) = 4;
                allowblur_matrix(theta_index_o, fi_index_o) = 0;
              }
            }
          }
          //  ros::Time t_in6 = ros::Time::now();
          //   duration_direct =duration_direct + (t_in6-t_in5).toSec();
          //  ROS_INFO("End one");
        }
      }
#ifndef DEBUG
    }
#endif
  }

  // ROS_INFO("After interline");

  auto t7 = std::chrono::steady_clock::now();
  duration6 += std::chrono::duration<double>(t7 - t6).count();
  // ROS_INFO("Duration interline = %f, duration direct = %f",duration_interline,duration_direct);

  // 把上面用“距离+射程”标记的远距离空闲格，换算回真实距离
  for (int i = 0; i < polar_width; i++)
  {
    for (int j = 0; j < polar_height; j++)
    {
      if (polar_matrix(i, j) < 1.999 * sensing_horizon && polar_matrix(i, j) > sensing_horizon)
      {
        polar_matrix(i, j) = polar_matrix(i, j) - sensing_horizon;
        polarindextype_matrix(i, j) = 1;
        allowblur_matrix(i, j) = 1;
      }
    }
  }

  int free_flag = 0;

#ifndef DEBUG
// 第三遍：把极坐标深度图还原成 3D 点云（多线程）
#pragma omp parallel /*default(none)*/                                                             \
shared(use_avia_pattern, use_vlp32_pattern, use_minicf_pattern, is_360lidar,                       \
           polar_matrix, pattern_matrix, min_raylength, vertical_fov, local_map_filled, free_flag, \
           sensing_horizon, rotmyaw, polarindextype_matrix, polarpointintensity_matrix)
  {
#endif
    pcl::PointCloud<PointType> temp_cloud;
#ifndef DEBUG
#pragma omp for nowait
#endif
    // trans polar matrix to point clouds
    // 逐格遍历
    for (int i = 0; i < polar_width; i++)
    {
      for (int j = 0; j < polar_height; j++)
      {
        // 这一格没被扫描图案覆盖 → 跳过（模拟真实激光的稀疏性）
        if (use_avia_pattern == 1 || use_vlp32_pattern || use_minicf_pattern)
        {
          if (pattern_matrix(i, j) == 0 || pattern_matrix(i, j) == 1)
          {
            continue;
          }
        }
        // 射程内命中 → 生成一个真实点
        if (polar_matrix(i, j) < sensing_horizon)
        {
          Eigen::Vector3d addeuc_pt;
          polar3D polarindex_pt;
          polarindex_pt.theta = i - round(0.5 * polar_width);
          polarindex_pt.fi = j - round(0.5 * polar_height);
          polarindex_pt.r = polar_matrix(i, j);
          // 极坐标 → 3D 点，再转回世界坐标系
          polar2euc(&polarindex_pt, addeuc_pt);
          addeuc_pt = rot * addeuc_pt + pos;
          PointType add_pcl_pt;
          add_pcl_pt.x = addeuc_pt(0);
          add_pcl_pt.y = addeuc_pt(1);
          add_pcl_pt.z = addeuc_pt(2);
          // add_pcl_pt.intensity = polarindextype_matrix(i, j) * 20;

          add_pcl_pt.intensity = polarpointintensity_matrix(i, j);

          // filter the margin
          Eigen::Vector3d pt3;
          pt3[0] = add_pcl_pt.x;
          pt3[1] = add_pcl_pt.y;
          pt3[2] = add_pcl_pt.z;
          auto dir = pt3 - pos;

          // 太近的点丢掉（传感器的近端盲区）
          if (dir.norm() < min_raylength)
            continue;

          if (is_360lidar == 1)
          {
          }
          else
          {
            if (acos(dir.dot(rotmyaw) / (dir.norm() * rotmyaw.norm()) > M_PI * (vertical_fov / 2.0) / 180.0))
            {
              continue;
            }
          }
          // push back the filtered points
          // local_map_filled.push_back(add_pcl_pt);
          temp_cloud.push_back(add_pcl_pt);
        }
        // 另一种情况的命中点，同样生成
        else if (polar_matrix(i, j) < 1.999 * sensing_horizon)
        {
          // polar_matrix(i,j) = polar_matrix(i,j) - sensing_horizon;
          Eigen::Vector3d addeuc_pt;
          polar3D polarindex_pt;
          polarindex_pt.theta = i - round(0.5 * polar_width);
          polarindex_pt.fi = j - round(0.5 * polar_height);
          polarindex_pt.r = polar_matrix(i, j);
          polar2euc(&polarindex_pt, addeuc_pt);
          addeuc_pt = rot * addeuc_pt + pos; //.transpose()*
          PointType add_pcl_pt;
          add_pcl_pt.x = addeuc_pt(0);
          add_pcl_pt.y = addeuc_pt(1);
          add_pcl_pt.z = addeuc_pt(2);
          add_pcl_pt.intensity = polarindextype_matrix(i, j) * 20;

          // filter the margin
          Eigen::Vector3d pt3;
          pt3[0] = add_pcl_pt.x;
          pt3[1] = add_pcl_pt.y;
          pt3[2] = add_pcl_pt.z;
          auto dir = pt3 - pos;

          if (dir.norm() < min_raylength)
            continue;

          if (is_360lidar == 1)
          {
          }
          else
          {
            if (acos(dir.dot(rotmyaw) / (dir.norm() * rotmyaw.norm()) > M_PI * (vertical_fov / 2.0) / 180.0))
            {
              continue;
            }
          }
          // push back the filtered points
          // local_map_filled.push_back(add_pcl_pt);
          temp_cloud.push_back(add_pcl_pt);
        }
        // 完全没打到东西的方向 → 置一个标志，表示“这里是空的”
        else if (polar_matrix(i, j) >= 1.999 * sensing_horizon)
        {
          Eigen::Vector3d addeuc_pt;
          polar3D polarindex_pt;
          polarindex_pt.theta = i - round(0.5 * polar_width);
          polarindex_pt.fi = j - round(0.5 * polar_height);
          polarindex_pt.r = 2 * sensing_horizon;
          polar2euc(&polarindex_pt, addeuc_pt);
          addeuc_pt = rot * addeuc_pt + pos;
          PointType add_pcl_pt;
          add_pcl_pt.x = addeuc_pt(0);
          add_pcl_pt.y = addeuc_pt(1);
          add_pcl_pt.z = addeuc_pt(2);
          // add_pcl_pt.intensity = polarindextype_matrix(i, j) * 20;
          add_pcl_pt.intensity = MIN_INTENSITY;
          // filter the margin
          Eigen::Vector3d pt3;
          pt3[0] = add_pcl_pt.x;
          pt3[1] = add_pcl_pt.y;
          pt3[2] = add_pcl_pt.z;
          auto dir = pt3 - pos;

          if (dir.norm() < min_raylength)
            continue;

          if (is_360lidar == 1)
          {
          }
          else
          {
            if (acos(dir.dot(rotmyaw) / (dir.norm() * rotmyaw.norm()) > M_PI * (vertical_fov / 2.0) / 180.0))
            {
              continue;
            }
          }

          free_flag = 1;
          // local_map_filled.push_back(add_pcl_pt);
          // temp_cloud.push_back(add_pcl_pt);
        }
      }
    }
#ifndef DEBUG
#pragma omp critical
#endif
    // 把各线程算出来的点汇总到一起
    local_map_filled.insert(local_map_filled.end(), temp_cloud.begin(), temp_cloud.end());

#ifndef DEBUG
  }
#endif

  auto t8 = std::chrono::steady_clock::now();
  duration7 += std::chrono::duration<double>(t8 - t7).count();

  if (free_flag == 1)
  {
    // ROS_WARN("Give free far !!!!!!!!!!!!!!!!!!!!!!!");
  }

  // ROS_INFO("GET OUT OF LOOP, pointcount = %d, origin pointcount = %d, change point = %d",local_map_filled.points.size(),original_pointcount,changepointcount);

  // 整理点云尺寸信息
  local_map.width = local_map.points.size();
  local_map.height = 1;
  local_map.is_dense = true;

  local_map_filled.width = local_map_filled.points.size();
  local_map_filled.height = 1;
  local_map_filled.is_dense = true;

  // 转成 ROS 消息
  pcl::toROSMsg(local_map_filled, local_map_pcd);
  local_map_pcd.header = odom_.header;
  local_map_pcd.header.frame_id = "world";
  // 发布渲染结果 —— 规划器订阅的就是这个话题（launch 里重映射成 /quad_0/cloud）
  pub_cloud->publish(local_map_pcd);

  // transform
  // 下面广播 world → sensor 的坐标变换（RViz 显示要用）
  std::string sensor_frame_id_ = "sensor";
  geometry_msgs::msg::TransformStamped transform;
  rclcpp::Time time_stamp_(odom_.header.stamp);
  transform.header.stamp = time_stamp_;
  transform.header.frame_id = "world";
  transform.child_frame_id = sensor_frame_id_;

  transform.transform.translation.x = pos.x();
  transform.transform.translation.y = pos.y();
  transform.transform.translation.z = pos.z();
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  tf_broadcaster->sendTransform(transform);

  // 再把点云转换到“传感器坐标系”，作为另一路调试输出
  Eigen::Matrix4d sensor2world;
  sensor2world << rot(0, 0), rot(0, 1), rot(0, 2), pos.x(),
      rot(1, 0), rot(1, 1), rot(1, 2), pos.y(),
      rot(2, 0), rot(2, 1), rot(2, 2), pos.z(),
      0, 0, 0, 1;
  Eigen::Matrix4d world2sensor;
  world2sensor = sensor2world.inverse();

  // pointcloud
  point_in_sensor.points.clear();
  pcl::transformPointCloud(local_map_filled, point_in_sensor, world2sensor);
  point_in_sensor.width = point_in_sensor.points.size();
  point_in_sensor.height = 1;
  point_in_sensor.is_dense = true;
  pcl::toROSMsg(point_in_sensor, sensor_map_pcd);
  sensor_map_pcd.header.frame_id = sensor_frame_id_;
  sensor_map_pcd.header.stamp = time_stamp_;
  // 发布“传感器坐标系下的点云”（话题 sensor_cloud）
  pub_intercloud->publish(sensor_map_pcd);

  auto t_total = std::chrono::steady_clock::now();
  duration8 += std::chrono::duration<double>(t_total - t8).count();
  // ROS_INFO("Average statistics: %f, %f ,%f ,%f, %f, %f ,%f ,%f,  total_time = %f",duration1/sense_count,duration2/sense_count,duration3/sense_count,duration4/sense_count,duration5/sense_count,duration6/sense_count,duration7/sense_count,duration8/sense_count,(t_total-t1).toSec());

  // 统计并发布本次渲染的耗时（观察性能用）
  double comp_time_temp = std::chrono::duration<double>(t_total - t1).count();
  comp_time_vec.push_back(comp_time_temp);
  if (comp_time_count > 20)
  {
    comp_time_vec.pop_front();
    geometry_msgs::msg::PoseStamped totaltime_pub;
    totaltime_pub.pose.position.x = accumulate(comp_time_vec.begin(), comp_time_vec.end(), 0.0) / comp_time_vec.size();
    comp_time_pub->publish(totaltime_pub);
    // ROS_INFO_THROTTLE(0.5, "Temp compute time = %lf, average compute time = %lf", comp_time_temp, totaltime_pub.pose.position.x);
  }
  else
  {
    comp_time_count++;
  }
}

// 程序入口
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // 创建节点，名字固定为 pcl_render_node
  ros_node = std::make_shared<rclcpp::Node>("pcl_render_node");

  // 读取机器人名与传感器类型
  quad_name = ros_node->declare_parameter<std::string>("quadrotor_name", "quad_0");
  sensor_type_ = ros_node->declare_parameter<std::string>("sensor_type", "depth");
  // 传感器类型只能是 depth 或 lidar，否则直接报错退出
  if (!useDepthSensor() && !useLidarSensor())
  {
    RCLCPP_ERROR(ros_node->get_logger(), "Unsupported sensor_type '%s'. Expected 'depth' or 'lidar'.", sensor_type_.c_str());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(ros_node->get_logger(), "pcl_render_node sensor_type: %s", sensor_type_.c_str());
  // 读取激光相关的一大批参数（视场、分辨率、扫描图案开关等）
  is_360lidar = ros_node->declare_parameter<int>("is_360lidar", 1);
  sensing_horizon = ros_node->declare_parameter<double>("sensing_horizon", 40.0);
  sensing_rate = ros_node->declare_parameter<double>("sensing_rate", 10.0);
  estimation_rate = ros_node->declare_parameter<double>("estimation_rate", 10.0);
  polar_resolution = ros_node->declare_parameter<double>("polar_resolution", 0.2);
  yaw_fov = ros_node->declare_parameter<double>("yaw_fov", 360.0);
  vertical_fov = ros_node->declare_parameter<double>("vertical_fov", 90.0);
  min_raylength = ros_node->declare_parameter<double>("min_raylength", 1.0);
  downsample_res = ros_node->declare_parameter<double>("downsample_res", 0.1);
  livox_linestep = ros_node->declare_parameter<double>("livox_linestep", 1.4);
  use_avia_pattern = ros_node->declare_parameter<int>("use_avia_pattern", 0);
  curvature_limit = ros_node->declare_parameter<double>("curvature_limit", 100.0);
  hash_cubesize = ros_node->declare_parameter<double>("hash_cubesize", 5.0);
  use_vlp32_pattern = ros_node->declare_parameter<int>("use_vlp32_pattern", 0);
  use_os128_pattern = ros_node->declare_parameter<int>("use_os128_pattern", 0);
  use_os64_pattern = ros_node->declare_parameter<int>("use_os64_pattern", 0);
  use_minicf_pattern = ros_node->declare_parameter<int>("use_minicf_pattern", 1);
  // 相机内参
  cam_width = ros_node->declare_parameter<int>("cam_width", cam_width);
  cam_height = ros_node->declare_parameter<int>("cam_height", cam_height);
  cam_fx = ros_node->declare_parameter<double>("cam_fx", cam_fx);
  cam_fy = ros_node->declare_parameter<double>("cam_fy", cam_fy);
  cam_cx = ros_node->declare_parameter<double>("cam_cx", cam_cx);
  cam_cy = ros_node->declare_parameter<double>("cam_cy", cam_cy);
  // 地图尺寸相关参数（这里只用于统计，不参与渲染）
  resolution = ros_node->declare_parameter<double>("map.resolution", 0.1);
  x_size = ros_node->declare_parameter<double>("map.x_size", 100.0);
  y_size = ros_node->declare_parameter<double>("map.y_size", 100.0);
  z_size = ros_node->declare_parameter<double>("map.z_size", 10.0);

  // dyn parameters
  // 动态障碍物相关参数
  dynobj_enable = ros_node->declare_parameter<int>("dynamic.enable", 0);
  dynobject_size = ros_node->declare_parameter<double>("dynamic.object_size", 0.0);
  dynobject_num = ros_node->declare_parameter<int>("dynamic.object_count", 0);
  dyn_mode = ros_node->declare_parameter<int>("dynamic.mode", 0);
  dyn_velocity = ros_node->declare_parameter<double>("dynamic.velocity", 0.0);
  use_uav_extra_model = ros_node->declare_parameter<int>("use_uav_extra_model", 0);
  collisioncheck_enable = ros_node->declare_parameter<int>("collision_check.enable", 0);
  collision_range = ros_node->declare_parameter<double>("collision_check.range", 0.3);
  output_pcd = ros_node->declare_parameter<int>("output_pcd", 0);

  // subscribe other uav pos
  // 多机仿真相关参数
  drone_num = ros_node->declare_parameter<int>("uav_num", 1);
  drone_id = ros_node->declare_parameter<int>("drone_id", 0);
  lidar_pitch = ros_node->declare_parameter<double>("lidar_pitch", 45.0);
  other_uav_pos.resize(drone_num);
  other_uav_rcv_time.resize(drone_num);
  otheruav_points.resize(drone_num);
  otheruav_pointsindex.resize(drone_num);
  // 给除自己以外的每台无人机订阅里程计
  for (int i = 0; i < drone_num; i++)
  {
    if (i == drone_id)
    {
      continue;
    }
    string topic = "/quad_";
    topic += to_string(i);
    topic += "/body_pose";
    cout << topic << endl;
    // 话题形如 /quad_i/body_pose
    other_odom_subs.push_back(ros_node->create_subscription<nav_msgs::msg::Odometry>(
        topic, rclcpp::SensorDataQoS(),
        [i](nav_msgs::msg::Odometry::ConstSharedPtr msg) { multiOdometryCallback(msg, i); }));
  }

  pcl::VoxelGrid<PointType> sor;
  int pcd_read_status;
  // 如果配置要用真实无人机模型，就从包目录加载 PCD 模型文件
  if (use_uav_extra_model)
  {
    string uav_model_path;
    uav_model_path = ament_index_cpp::get_package_share_directory("odom_visualization");
    uav_model_path.append("/meshes/yunque001.pcd");
    std::cout << "\nFound pkg_path = " << uav_model_path << std::endl;

    // 读入模型并做体素下采样
    pcd_read_status = pcl::io::loadPCDFile<PointType>(uav_model_path, uav_extra_model);
    if (pcd_read_status == -1)
    {
      cout << "can't read uav extra model file." << endl;
      return 0;
    }
    // downsample uav model
    sor.setInputCloud(uav_extra_model.makeShared());
    sor.setLeafSize(downsample_res, downsample_res, downsample_res);
    sor.filter(uav_extra_model);
    uav_points_num = uav_extra_model.size();
  }
  else
  {
    uav_size[0] = 0.0;
    uav_size[1] = 0.0;
    uav_size[2] = 0.0;
    // uav_size[0] = 0.4;
    // uav_size[1] = 0.4;
    // uav_size[2] = 0.3;
    drone_drawpoints_num[0] = (uav_size[0] / downsample_res);
    drone_drawpoints_num[1] = (uav_size[1] / downsample_res);
    drone_drawpoints_num[2] = (uav_size[2] / downsample_res);
    uav_points_num = drone_drawpoints_num[0] * drone_drawpoints_num[1] * drone_drawpoints_num[2];
    RCLCPP_INFO(ros_node->get_logger(), "drone_drawpoints_num = %d,%d,%d, Uav points num = %d", drone_drawpoints_num[0], drone_drawpoints_num[1], drone_drawpoints_num[2], uav_points_num);
  }

  // subscribe point cloud
  // 订阅全局地图。transient_local 保证晚启动的节点也能拿到最后一份地图
  auto map_qos = rclcpp::QoS(1).reliable().transient_local();
  global_map_sub = ros_node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "global_map", map_qos, rcvGlobalPointCloudCallBack);
  // 订阅本机里程计（话题名可配置）
  const auto body_pose_topic = ros_node->declare_parameter<std::string>("body_pose_topic", "body_pose");
  odom_sub = ros_node->create_subscription<nav_msgs::msg::Odometry>(
      body_pose_topic, rclcpp::SensorDataQoS(), rcvOdometryCallback);

  // publishers
  // 创建发布器
  pub_dyncloud = ros_node->create_publisher<sensor_msgs::msg::PointCloud2>("dyn_cloud", rclcpp::SensorDataQoS());
  pub_uavcloud = ros_node->create_publisher<sensor_msgs::msg::PointCloud2>("uav_cloud", rclcpp::SensorDataQoS());
  // 深度模式发布深度图 + 相机位姿；激光模式发布点云 + 激光位姿
  if (useDepthSensor())
  {
    pinhole_depth_pub_ = ros_node->create_publisher<sensor_msgs::msg::Image>("depth", rclcpp::SensorDataQoS());
    camera_pose_pub_ = ros_node->create_publisher<nav_msgs::msg::Odometry>(quadTopic("camera_pose"), rclcpp::SensorDataQoS());
  }
  else
  {
    pub_intercloud = ros_node->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_cloud", rclcpp::SensorDataQoS());
    pub_cloud = ros_node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS());
    lidar_pose_pub_ = ros_node->create_publisher<nav_msgs::msg::Odometry>(quadTopic("lidar_pose"), rclcpp::SensorDataQoS());
  }
  comp_time_pub = ros_node->create_publisher<geometry_msgs::msg::PoseStamped>("simulator_compute_time", 10);
  // 渲染周期 = 1 ÷ 传感频率
  double sensing_duration = 1.0 / sensing_rate;

  start_time = ros_node->now();
  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node);

  // 两个定时器：一个渲染传感器数据，一个更新动态障碍物
  local_sensing_timer = ros_node->create_wall_timer(
      std::chrono::duration<double>(sensing_duration), renderSensedPoints);
  dynobj_timer = ros_node->create_wall_timer(
      std::chrono::duration<double>(sensing_duration), dynobjGenerate);

  inv_resolution = 1.0 / resolution;
  gl_xl = -x_size / 2.0;
  gl_yl = -y_size / 2.0;
  gl_zl = 0.0;
  GLX_SIZE = (int)(x_size * inv_resolution);
  GLY_SIZE = (int)(y_size * inv_resolution);
  GLZ_SIZE = (int)(z_size * inv_resolution);

  // 传感器到机体的安装矩阵（定义相机/激光的坐标轴朝向）
  sensor2body << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  // 单线程执行器，一直运行到退出
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ros_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
