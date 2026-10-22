// ============================================================================
// 文件：include/plan_manage/reference_path_utils.h
//
// 【这个文件是干什么的】
//   navi_mode=3（参考路径模式）用到的两个纯函数工具，都围绕“外部给的一条路径”：
//     transformReferencePath    把整条路径从一个坐标系搬到另一个坐标系
//     prepareReferenceWaypoints 把路径消息整理成规划器内部用的三维航点序列
//
// 【为什么需要坐标系变换】
//   外部给（或自带发布器给）的参考路径，往往写在地图系/参考系下；
//   而规划与建图都在 world 系里做，所以必须先整体乘上“参考系 → world”的变换。
//
// 【为什么需要整理航点】
//   一条路径可能有几百上千个点，直接拿去规划又慢又没必要；
//   这里会按最小间距抽稀，并统一抬高 body_height（外部给的是地面高度）。
//
// 【错误信息怎么传】
//   两个函数失败时都会把原因写进 error（不关心就传 nullptr），方便上层打日志。
// ============================================================================
#ifndef PLAN_MANAGE_REFERENCE_PATH_UTILS_H
#define PLAN_MANAGE_REFERENCE_PATH_UTILS_H

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace scan_planner
{

// 把整条参考路径按 transform 做坐标变换。
// 要求：路径必须带 frame_id，且与 transform.child_frame_id（变换的“源”系）一致；
// 返回时所有点都在 transform.header.frame_id（变换的“目标”系）下。
inline bool transformReferencePath(
    const nav_msgs::msg::Path &path,
    const geometry_msgs::msg::TransformStamped &transform,
    nav_msgs::msg::Path &transformed_path,
    std::string *error = nullptr)
{
  transformed_path = nav_msgs::msg::Path();
  // 路径本身没有坐标系 → 不知道该按什么变换，直接失败
  if (path.header.frame_id.empty())
  {
    if (error) *error = "reference path frame_id is empty";
    return false;
  }
  // 变换缺少源系或目标系 → 无法使用
  if (transform.header.frame_id.empty() || transform.child_frame_id.empty())
  {
    if (error) *error = "reference path transform has an empty frame_id";
    return false;
  }
  // 路径的坐标系必须正好是变换的源系，否则说明拿错了变换
  if (path.header.frame_id != transform.child_frame_id)
  {
    if (error) *error = "reference path frame does not match transform source frame";
    return false;
  }

  // 输出路径的头沿用输入的（只换坐标系名）
  transformed_path.header = path.header;
  transformed_path.header.frame_id = transform.header.frame_id;
  transformed_path.poses.reserve(path.poses.size());
  // 逐点变换
  for (const auto &pose : path.poses)
  {
    geometry_msgs::msg::PoseStamped source_pose = pose;
    // 单个点没写坐标系时，默认与整条路径一致
    if (source_pose.header.frame_id.empty())
      source_pose.header.frame_id = path.header.frame_id;
    // 单个点的坐标系与整条路径不一致 → 拒绝处理（避免结果错位）
    if (source_pose.header.frame_id != path.header.frame_id)
    {
      transformed_path = nav_msgs::msg::Path();
      if (error) *error = "reference path contains poses in mixed frames";
      return false;
    }

    geometry_msgs::msg::PoseStamped transformed_pose;
    tf2::doTransform(source_pose, transformed_pose, transform);
    transformed_pose.header = transformed_path.header;
    transformed_path.poses.push_back(std::move(transformed_pose));
  }
  return true;
}

// 把路径消息整理成规划器内部用的三维航点序列（world 系）。
// body_height：统一抬高的高度（外部给的是地面高度，机器人身体要抬起来）；
// min_distance：抽稀用的最小间距（米），相邻过近的点会被丢掉。
inline bool prepareReferenceWaypoints(
    const nav_msgs::msg::Path &path,
    double body_height,
    double min_distance,
    std::vector<Eigen::Vector3d> &waypoints,
    std::string *error = nullptr)
{
  waypoints.clear();
  // 空路径没有任何意义
  if (path.poses.empty())
  {
    if (error) *error = "reference path is empty";
    return false;
  }
  // 参数合法性检查：高度与间距必须是有限值，间距不能为负
  if (!std::isfinite(body_height) || !std::isfinite(min_distance) || min_distance < 0.0)
  {
    if (error) *error = "body height and minimum distance must be finite and non-negative";
    return false;
  }

  waypoints.reserve(path.poses.size());
  Eigen::Vector3d final_waypoint;   // 路径最后一个点（无论如何都要保留）
  Eigen::Vector3d last_waypoint;    // 上一个被采纳的点，用来算间距
  bool first = true;                // 第一个点无条件采纳

  for (const auto &pose_stamped : path.poses)
  {
    const auto &position = pose_stamped.pose.position;
    // 坐标里出现 NaN/Inf → 整条路径作废（否则会把优化器算崩）
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z))
    {
      waypoints.clear();
      if (error) *error = "reference path contains a non-finite coordinate";
      return false;
    }

    // 抬高 body_height，把“地面高度”变成机器人该走的高度
    Eigen::Vector3d waypoint(position.x, position.y, position.z + body_height);
    final_waypoint = waypoint;
    // 第一个点，或者与上一个采纳点距离够远 → 采纳
    if (first || (waypoint - last_waypoint).norm() >= min_distance)
    {
      waypoints.push_back(waypoint);
      last_waypoint = waypoint;
      first = false;
    }
  }

  // 抽稀可能把最后一个点丢掉，这里补回来（终点很关键，不能缺）
  if ((waypoints.back() - final_waypoint).norm() > 1e-6)
    waypoints.push_back(final_waypoint);

  // 少于两个点无法构成一条参考路径
  if (waypoints.size() < 2)
  {
    waypoints.clear();
    if (error) *error = "reference path requires at least two distinct points";
    return false;
  }
  return true;
}

}  // namespace scan_planner

#endif  // PLAN_MANAGE_REFERENCE_PATH_UTILS_H
