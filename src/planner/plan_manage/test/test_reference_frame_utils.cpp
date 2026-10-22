// ============================================================================
// 文件：test/test_reference_frame_utils.cpp
//
// 【测什么】
//   include/plan_manage/reference_frame_utils.h 里四个纯函数：
//     referenceToWorld            TF 变换 → Eigen 等距变换
//     referenceTransformJumped    参考系跳变检测（平移/偏航分别判）
//     referenceTransformFresh     TF 新鲜度
//     referenceTfGraceExpired     缺失时长是否超过宽限
//
// 【怎么跑】
//   colcon test --packages-select scan_planner --ctest-args -R reference
// ============================================================================
#include <cmath>
#include <gtest/gtest.h>
#include <plan_manage/reference_frame_utils.h>

namespace
{
// 小工具：造一个只有平面平移 + 偏航的变换（绕 z 轴）
geometry_msgs::msg::TransformStamped transform(double x, double y, double yaw)
{
  geometry_msgs::msg::TransformStamped result;
  result.transform.translation.x = x;
  result.transform.translation.y = y;
  result.transform.rotation.z = std::sin(yaw / 2.0);
  result.transform.rotation.w = std::cos(yaw / 2.0);
  return result;
}

// 参考系下的“目标点”应被搬到 world 系；参考系下的“速度”只应被旋转、不应被平移。
// 同时验证逆变换能把 world 系的机器人位置换回参考系。
TEST(ReferenceFrameUtils, ReprojectsPositionAndRotatesVelocityWithoutTranslation)
{
  const Eigen::Vector3d target_map(2.0, 0.0, 0.5);
  const Eigen::Vector3d velocity_map(1.0, 0.0, 0.0);
  const auto first = scan_planner::referenceToWorld(transform(1.0, 0.0, 0.0));
  const auto second = scan_planner::referenceToWorld(transform(0.0, 1.0, M_PI / 2.0));

  EXPECT_NEAR((first * target_map).x(), 3.0, 1e-9);
  EXPECT_NEAR((second * target_map).x(), 0.0, 1e-9);
  EXPECT_NEAR((second * target_map).y(), 3.0, 1e-9);
  const Eigen::Vector3d velocity_world = second.linear() * velocity_map;
  EXPECT_NEAR(velocity_world.x(), 0.0, 1e-9);
  EXPECT_NEAR(velocity_world.y(), 1.0, 1e-9);
  EXPECT_NEAR(velocity_world.z(), 0.0, 1e-9);

  const Eigen::Vector3d robot_world(0.0, 2.0, 0.5);
  const Eigen::Vector3d robot_map = second.inverse() * robot_world;
  EXPECT_NEAR(robot_map.x(), 1.0, 1e-9);
  EXPECT_NEAR(robot_map.y(), 0.0, 1e-9);
}

// 跳变检测：阈值 0.2 米 / 5 度
//   平移 0.19 米 → 不算跳变；0.21 米 → 算跳变；偏航 6 度 → 算跳变
TEST(ReferenceFrameUtils, DetectsAccumulatedTfChanges)
{
  const auto base = scan_planner::referenceToWorld(transform(0.0, 0.0, 0.0));
  const Eigen::Vector3d robot_world(1.0, 0.0, 0.0);
  EXPECT_FALSE(scan_planner::referenceTransformJumped(
      base, scan_planner::referenceToWorld(transform(0.19, 0.0, 0.0)),
      robot_world, 0.2, 5.0 * M_PI / 180.0));
  EXPECT_TRUE(scan_planner::referenceTransformJumped(
      base, scan_planner::referenceToWorld(transform(0.21, 0.0, 0.0)),
      robot_world, 0.2, 5.0 * M_PI / 180.0));
  EXPECT_TRUE(scan_planner::referenceTransformJumped(
      base, scan_planner::referenceToWorld(transform(0.0, 0.0, 6.0 * M_PI / 180.0)),
      Eigen::Vector3d::Zero(), 0.2, 5.0 * M_PI / 180.0));
}

// 新鲜度与宽限：0.3 秒内算新鲜；超时或时间戳在“未来”太多都算不可用；
// 缺失时长达到宽限值（含等于）即判定超时。
TEST(ReferenceFrameUtils, FreshnessAndGracePolicy)
{
  EXPECT_TRUE(scan_planner::referenceTransformFresh(0.1, 0.3));
  EXPECT_FALSE(scan_planner::referenceTransformFresh(0.31, 0.3));
  EXPECT_FALSE(scan_planner::referenceTransformFresh(-0.1, 0.3));
  EXPECT_FALSE(scan_planner::referenceTfGraceExpired(0.49, 0.5));
  EXPECT_TRUE(scan_planner::referenceTfGraceExpired(0.5, 0.5));
  // 拿到一次新鲜变换会把“缺失起始时刻”清零，
  // 于是下一次丢失是重新计时的全新宽限期，而不是接着上一次累计。
  EXPECT_FALSE(scan_planner::referenceTfGraceExpired(0.0, 0.5));
}
}  // namespace
