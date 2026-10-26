// ============================================================================
// 文件：include/plan_manage/plan_container.hpp
//
// 【这个文件是干什么的】
//   定义几个“数据结构”（用来装数据的盒子），供规划过程传递信息。
//   本文件里没有算法，只是说明“数据长什么样”。
//
// 【三个盒子】
//   1. GlobalTrajData —— “全局参考轨迹”的数据盒子
//   2. PlanParameters —— “参数盒子”（物理限制、耗时统计）
//   3. LocalTrajData  —— “局部轨迹盒子”（当前真正在执行的短轨迹）
//
// 【一句话区分“全局”和“局部”】
//   全局 = 大方向（从起点到终点怎么走，用多项式表示，比较粗糙）；
//   局部 = 当前这一小段真正被执行的平滑轨迹（用 B 样条表示，会不断重算）。
// ============================================================================
#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/polynomial_traj.h>

using std::vector;

namespace scan_planner
{

  // ----------------------------------------------------------------
  // GlobalTrajData：全局参考轨迹的数据盒子
  // 内容：起点到终点的大方向轨迹，以及它的时长/起始时间。
  // ----------------------------------------------------------------
  class GlobalTrajData
  {
  private:
  public:
    // 全局轨迹本体（用多项式表示，见 traj_utils/polynomial_traj.h）
    PolynomialTraj global_traj_;
    // 局部轨迹的 [位置, 速度, 加速度] 三条 B 样条
    vector<UniformBspline> local_traj_;

    // 全局轨迹总时长（秒）
    double global_duration_;
    rclcpp::Time global_start_time_;
    // 局部轨迹在全局时间轴上的起止时刻（用来判断“现在该用局部还是全局轨迹”）
    double local_start_time_, local_end_time_;
    double time_increase_;
    double last_time_inc_;

    GlobalTrajData(/* args */) {}

    ~GlobalTrajData() {}

    // 判断：局部轨迹是不是已经走到全局终点附近了（差 0.1 秒以内算到达）
    bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

    // 设置一条新的全局轨迹，并把所有“局部时间”状态清零（相当于重新开始）
    void setGlobalTraj(const PolynomialTraj &traj, const rclcpp::Time &time)
    {
      global_traj_ = traj;
      global_traj_.init();
      global_duration_ = global_traj_.getTimeSum();
      global_start_time_ = time;

      local_traj_.clear();
      local_start_time_ = -1;
      local_end_time_ = -1;
      time_increase_ = 0.0;
      last_time_inc_ = 0.0;
    }

    // 把某一段局部 B 样条记录进来（同时对它求导得到速度和加速度两条样条）
    void setLocalTraj(UniformBspline traj, double local_ts, double local_te, double time_inc)
    {
      local_traj_.resize(3);
      local_traj_[0] = traj;
      local_traj_[1] = local_traj_[0].getDerivative();
      local_traj_[2] = local_traj_[1].getDerivative();

      local_start_time_ = local_ts;
      local_end_time_ = local_te;
      global_duration_ += time_inc;
      time_increase_ += time_inc;
      last_time_inc_ = time_inc;
    }

    // 问：“在时间 t 时，我应该在哪？”
    // 规则：落在局部轨迹时间段内就用局部样条，时间段外就退回全局多项式。
    Eigen::Vector3d getPosition(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluate(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    // 问：“在时间 t 时，我的速度是多少？”（规则同上）
    Eigen::Vector3d getVelocity(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateVel(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateVel(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    // 问：“在时间 t 时，我的加速度是多少？”（规则同上）
    Eigen::Vector3d getAcceleration(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateAcc(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateAcc(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    // get Bspline parameterization data of a local trajectory within a sphere
    // start_t: start time of the trajectory
    // dist_pt: distance between the discretized points
    // 把全局轨迹上“从 start_t 开始、长度约 des_radius 的一段”离散成一串点，
    // 并给出这段起点/终点的速度与加速度。后面用它来重新拟合 B 样条。
    void getTrajByRadius(const double &start_t, const double &des_radius, const double &dist_pt,
                         vector<Eigen::Vector3d> &point_set, vector<Eigen::Vector3d> &start_end_derivative,
                         double &dt, double &seg_duration)
    {
      double seg_length = 0.0; // length of the truncated segment
      double seg_time = 0.0;   // duration of the truncated segment
      double radius = 0.0;     // distance to the first point of the segment

      double delta = 0.2;
      Eigen::Vector3d first_pt = getPosition(start_t); // first point of the segment
      Eigen::Vector3d prev_pt = first_pt;              // previous point
      Eigen::Vector3d cur_pt;                          // current point

      // go forward until the traj exceed radius or global time

      while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3)
      {
        seg_time += delta;
        seg_time = min(seg_time, global_duration_ - start_t);

        cur_pt = getPosition(start_t + seg_time);
        seg_length += (cur_pt - prev_pt).norm();
        prev_pt = cur_pt;
        radius = (cur_pt - first_pt).norm();
      }

      // get parameterization dt by desired density of points
      int seg_num = floor(seg_length / dist_pt);

      // get outputs

      seg_duration = seg_time; // duration of the truncated segment
      dt = seg_time / seg_num; // time difference between two points

      for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + seg_time));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + seg_time));
    }

    // get Bspline parameterization data of a fixed duration local trajectory
    // start_t: start time of the trajectory
    // duration: time length of the segment
    // seg_num: discretized the segment into *seg_num* parts
    // 与上面类似，但用“固定时长 duration + 固定分段数 seg_num”来离散。
    void getTrajByDuration(double start_t, double duration, int seg_num,
                           vector<Eigen::Vector3d> &point_set,
                           vector<Eigen::Vector3d> &start_end_derivative, double &dt)
    {
      dt = duration / seg_num;
      Eigen::Vector3d cur_pt;
      for (double tp = 0.0; tp <= duration + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + duration));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + duration));
    }
  };

  // ----------------------------------------------------------------
  // PlanParameters：参数盒子（物理限制 + 各阶段耗时统计）
  // ----------------------------------------------------------------
  struct PlanParameters
  {
    /* planning algorithm parameters */
    // 物理限制：最大速度、最大加速度、最大加加速度（jerk）
    double max_vel_, max_acc_, max_jerk_; // physical limits
    double vel_tolerance_, acc_tolerance_;
    // 相邻两个 B 样条控制点之间的距离（决定轨迹的“精细程度”）
    double ctrl_pt_dist;                  // distance between adjacient B-spline control points
    double feasibility_tolerance_;        // permitted ratio of vel/acc exceeding limits
    double planning_horizon_;

    /* processing time */
    // 下面是各阶段的耗时统计（仅用于调试/性能观察，不影响规划结果）
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

  // ----------------------------------------------------------------
  // LocalTrajData：局部轨迹盒子（当前真正被机器人执行的那条短轨迹）
  // ----------------------------------------------------------------
  struct LocalTrajData
  {
    /* info of generated traj */

    // 轨迹编号：每重新规划一次就 +1
    int traj_id_;
    // 这条局部轨迹的总时长（秒）
    double duration_;
    double global_time_offset; // This is because when the local traj finished and is going to switch back to the global traj, the global traj time is no longer matches the world time.
    // 这条局部轨迹的开始时刻。用它算“已经走了多久”：t = 现在 - start_time_
    rclcpp::Time start_time_;
    Eigen::Vector3d start_pos_;
    // 位置 / 速度 / 加速度 三条 B 样条（速度=位置求导，加速度=速度求导）
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_;
  };

} // namespace scan_planner

#endif
