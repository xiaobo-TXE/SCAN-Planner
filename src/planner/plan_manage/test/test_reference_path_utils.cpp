// ============================================================================
// 文件：test/test_reference_path_utils.cpp
//
// 【测什么】
//   include/plan_manage/reference_path_utils.h 里两个纯函数的行为：
//     prepareReferenceWaypoints  路径 → 航点（抽稀、抬高、合法性校验）
//     transformReferencePath     路径整体做坐标系变换
//
// 【怎么跑】
//   colcon test --packages-select scan_planner --ctest-args -R reference
// ============================================================================
#include <limits>

#include <gtest/gtest.h>

#include <plan_manage/reference_path_utils.h>

namespace
{

// 小工具：造一个只有位置的 PoseStamped
geometry_msgs::msg::PoseStamped pose(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped result;
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.position.z = z;
  result.pose.orientation.w = 1.0;
  return result;
}

// 空路径、只有一个点的路径都必须被拒绝（否则构不成参考路径）
TEST(ReferencePathUtils, RejectsEmptyAndSinglePointPaths)
{
  std::vector<Eigen::Vector3d> waypoints;
  std::string error;
  nav_msgs::msg::Path path;
  EXPECT_FALSE(scan_planner::prepareReferenceWaypoints(path, 0.4, 0.5, waypoints, &error));
  EXPECT_TRUE(waypoints.empty());

  path.poses.push_back(pose(0.0, 0.0, 0.0));
  EXPECT_FALSE(scan_planner::prepareReferenceWaypoints(path, 0.4, 0.5, waypoints, &error));
  EXPECT_TRUE(waypoints.empty());
}

// 抽稀：间距小于 min_distance 的点被丢掉，但最后一个点必须保留；
// 同时验证高度被统一抬高了 body_height
TEST(ReferencePathUtils, DownsamplesInThreeDimensionsAndPreservesFinalPoint)
{
  nav_msgs::msg::Path path;
  path.poses = {
      pose(0.0, 0.0, 0.1),
      pose(0.0, 0.0, 0.1),
      pose(0.2, 0.0, 0.1),
      pose(0.4, 0.0, 0.1),
      pose(0.6, 0.0, 0.1),
      pose(0.7, 0.0, 0.1),
  };

  std::vector<Eigen::Vector3d> waypoints;
  ASSERT_TRUE(scan_planner::prepareReferenceWaypoints(path, 0.4, 0.5, waypoints));
  ASSERT_EQ(waypoints.size(), 3u);
  EXPECT_NEAR(waypoints.front().z(), 0.5, 1e-9);
  EXPECT_NEAR(waypoints[1].x(), 0.6, 1e-9);
  EXPECT_NEAR(waypoints.back().x(), 0.7, 1e-9);
  EXPECT_NEAR(waypoints.back().z(), 0.5, 1e-9);

  // 只有“原地升高”的两个点：不能因为水平方向没动就被抽成 1 个点
  path.poses = {pose(1.0, 2.0, 0.0), pose(1.0, 2.0, 0.6)};
  ASSERT_TRUE(scan_planner::prepareReferenceWaypoints(path, 0.4, 0.5, waypoints));
  ASSERT_EQ(waypoints.size(), 2u);
  EXPECT_NEAR(waypoints.back().z(), 1.0, 1e-9);
}

// 坐标里出现 NaN → 整条路径作废（否则会把优化器算崩）
TEST(ReferencePathUtils, RejectsNonFiniteCoordinates)
{
  nav_msgs::msg::Path path;
  path.poses = {
      pose(0.0, 0.0, 0.0),
      pose(std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0),
  };

  std::vector<Eigen::Vector3d> waypoints;
  EXPECT_FALSE(scan_planner::prepareReferenceWaypoints(path, 0.4, 0.5, waypoints));
  EXPECT_TRUE(waypoints.empty());
}

// 坐标系变换：map 系下的路径 + “world ← map”的平移，应对应得到平移后的 world 坐标
TEST(ReferencePathUtils, TransformsPathIntoPlanningFrame)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses = {pose(1.0, 2.0, 0.5), pose(2.0, 2.0, 0.5)};
  for (auto &path_pose : path.poses)
    path_pose.header.frame_id = "map";

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "map";
  transform.transform.translation.x = 10.0;
  transform.transform.translation.y = -1.0;
  transform.transform.rotation.w = 1.0;

  nav_msgs::msg::Path transformed;
  std::string error;
  ASSERT_TRUE(scan_planner::transformReferencePath(path, transform, transformed, &error)) << error;
  ASSERT_EQ(transformed.poses.size(), 2u);
  EXPECT_EQ(transformed.header.frame_id, "world");
  EXPECT_EQ(transformed.poses[0].header.frame_id, "world");
  EXPECT_NEAR(transformed.poses[0].pose.position.x, 11.0, 1e-9);
  EXPECT_NEAR(transformed.poses[0].pose.position.y, 1.0, 1e-9);
  EXPECT_NEAR(transformed.poses[0].pose.position.z, 0.5, 1e-9);
}

// 单点坐标系与整条路径不一致 → 拒绝（避免结果错位）
TEST(ReferencePathUtils, RejectsMixedPoseFrames)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses = {pose(0.0, 0.0, 0.0), pose(1.0, 0.0, 0.0)};
  path.poses[1].header.frame_id = "odom";

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "map";
  transform.transform.rotation.w = 1.0;

  nav_msgs::msg::Path transformed;
  std::string error;
  EXPECT_FALSE(scan_planner::transformReferencePath(path, transform, transformed, &error));
  EXPECT_TRUE(transformed.poses.empty());
}

}  // namespace
