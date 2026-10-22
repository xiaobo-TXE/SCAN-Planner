// ============================================================================
// 文件：include/plan_manage/reference_frame_utils.h
//
// 【这个文件是干什么的】
//   navi_mode=3 里“参考坐标系”相关的几个纯函数小工具，全部是纯计算、不依赖 ROS：
//     referenceToWorld()             把 TF 变换转成 Eigen 的等距变换矩阵
//     referenceTransformJumped()     判断参考系是不是突然跳了一大步
//     referenceTransformFresh()      判断 TF 数据是否还算“新鲜”
//     referenceTfGraceExpired()      判断 TF 已经缺失多久、是否超过宽限
//
// 【为什么需要“跳变检测”】
//   参考系（比如地图系）在定位重定位/修正时可能整体跳一下。
//   此时还沿着旧轨迹走就会撞，所以要能识别出来并触发重规划。
//
// 【为什么需要“新鲜度”】
//   TF 树里可能还留着很久以前的变换；用它换算出来的路径位置是错的，
//   所以超过一定年龄的变换一律当作不可用。
//
// 【注意】
//   这里只做数学判断，不做任何日志与状态切换——那些留给调用方（scan_replan_fsm.cpp）。
// ============================================================================
#ifndef PLAN_MANAGE_REFERENCE_FRAME_UTILS_H
#define PLAN_MANAGE_REFERENCE_FRAME_UTILS_H

#include <cmath>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace scan_planner
{
// 把 TF 里的“平移 + 四元数”转成 Eigen 的等距变换矩阵（旋转 + 平移）。
// 四元数先归一化，避免 TF 里带微小数值误差导致旋转矩阵不是正交的。
inline Eigen::Isometry3d referenceToWorld(
    const geometry_msgs::msg::TransformStamped &transform)
{
  const auto &q = transform.transform.rotation;
  Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = rotation.normalized().toRotationMatrix();
  result.translation() << transform.transform.translation.x,
      transform.transform.translation.y, transform.transform.translation.z;
  return result;
}

// 判断参考系是否相对上一次“跳变”了。
// 平移：同一个机器人位置在两个参考系下的坐标之差（能抵消机器人自身运动的影响）；
// 旋转：只取绕 z 轴的偏航角差（翻跟头式的旋转不算跳变）。
// 超过任一阈值就算跳变。
inline bool referenceTransformJumped(const Eigen::Isometry3d &previous,
                                      const Eigen::Isometry3d &current,
                                      const Eigen::Vector3d &robot_world,
                                      double max_translation, double max_yaw)
{
  const double translation =
      (previous.inverse() * robot_world - current.inverse() * robot_world).norm();
  const Eigen::Matrix3d relative = previous.linear().transpose() * current.linear();
  const double yaw = std::abs(std::atan2(relative(1, 0), relative(0, 0)));
  return translation > max_translation || yaw > max_yaw;
}

// 判断一个变换的“年龄”（秒）是否还可接受。
// 允许 -0.05 秒的小负值：不同节点时钟会有微小差异，未来的时间戳不算异常。
inline bool referenceTransformFresh(double age_seconds, double max_age_seconds)
{
  return std::isfinite(age_seconds) && age_seconds >= -0.05 &&
         age_seconds <= max_age_seconds;
}

// 判断 TF 缺失的时间是否已经超过宽限值（超过就该急停了）。
inline bool referenceTfGraceExpired(double missing_seconds, double grace_seconds)
{
  return missing_seconds >= grace_seconds;
}
}  // namespace scan_planner

#endif  // PLAN_MANAGE_REFERENCE_FRAME_UTILS_H
