// ============================================================================
// 文件：include/plan_manage/planner_manager.h
//
// 【这个文件是干什么的】
//   定义“规划总管”类 SCANPlannerManager（可以理解成一位项目经理）。
//   它自己不实现算法细节，而是把各个算法模块按顺序串起来调用：
//       前端搜索(A*) → B样条拟合 → 后端优化(L-BFGS) → 时间重分配
//
// 【头文件(.h) 和源文件(.cpp) 的分工】
//   .h 只写“有哪些成员变量、有哪些函数、函数长什么样”，这叫【声明】；
//   真正干活的代码写在 planner_manager.cpp 里，这叫【实现】。
//   想知道某个函数具体怎么算的，就去找同名函数的实现。
//
// 【它管着哪些东西】（对应下面 public 的成员变量）
//   pp_          一堆物理限制参数（最大速度/加速度/加加速度……）
//   local_data_  当前正在执行的“局部轨迹”（真正被机器人执行的）
//   global_data_ 从起点到终点的“全局参考轨迹”（大方向）
//   grid_map_    栅格地图，用来判断哪里能走、哪里有障碍
// ============================================================================
#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>

namespace scan_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  // 规划总管：把 前端搜索 → B样条拟合 → 后端优化 → 时间重分配 串成一条流水线。
  class SCANPlannerManager
  {
    // SECTION stable
  public:
    SCANPlannerManager();
    ~SCANPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    // 【核心函数】重新规划一条局部轨迹（replan = 重新规划）。
    // 输入：起点位置/速度/加速度、局部目标位置/速度，外加两个开关标志。
    // 返回：true 表示成功（随后会把这条轨迹发出去），false 表示失败。
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    // 紧急停止：把轨迹收缩成一个“停在原地”的点（原地刹住）。
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    // 规划“全局轨迹”：给定起点和终点，生成一条大方向轨迹（用多项式表示）。
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    // 同上，但中间要依次经过若干“途经点(waypoints)”。
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    // 初始化：读参数、建地图、创建优化器。由状态机在程序启动时调用一次。
    void initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis = nullptr);

    // ↓↓↓ 下面是 public 数据，状态机会直接访问 ↓↓↓
    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

  private:
    rclcpp::Node *node_{nullptr};
    /* main planning algorithms & modules */
    // 可视化对象：负责在 RViz 里把轨迹、目标点画出来
    PlanningVisualization::Ptr visualization_;

    // B 样条优化器：真正负责“算出平滑轨迹”的工具
    BsplineOptimizer::Ptr bspline_optimizer_rebound_;

    // 连续失败了几次（用来做退避策略和随机重试）
    int continuous_failures_count_{0};

    // 把刚算出来的轨迹记录进 local_data_（开始时间、时长、轨迹本体等）
    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);
    // 检查轨迹的速度/加速度有没有超出物理限制（超了说明这条轨迹不安全）
    bool checkDynamicFeasibility(UniformBspline position_traj);

    // 重新分配时间：轨迹超速/超加速度时，把时间拉长，让同样的位移慢慢走完
    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    // 在“重新分配时间”之后再做一次小优化，让轨迹更平滑、更贴合原轨迹
    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    // 智能指针别名：外部用它来持有并自动释放本对象
    typedef unique_ptr<SCANPlannerManager> Ptr;

    // !SECTION
  };
} // namespace scan_planner

#endif
