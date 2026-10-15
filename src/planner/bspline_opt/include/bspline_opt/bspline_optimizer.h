// ============================================================================
// 文件：include/bspline_opt/bspline_optimizer.h
//
// 【这个文件是干什么的】
//   定义“B 样条优化器” BsplineOptimizer——整个规划算法的“大脑”。
//   它接过一条粗糙的初始轨迹，通过反复微调控制点，让它同时满足：
//     · 平滑（别抖）
//     · 避障（离障碍远一点）
//     · 可行（别超速、别超加速度）
//
// 【它是怎么“优化”的】
//   把上面三个目标写成一个“代价函数”（越小越好），
//   然后用 L-BFGS 这个数学工具去找一组控制点，让代价最小。
//
// 【什么时候用哪套代价】
//   Rebound（弹开）：初始规划时用，重点是避障 + 平滑 + 可行性
//   Refine（细化）  ：时间拉长之后用，重点是别离原轨迹太远（fitness 项）
//
// 【两个入口函数】
//   BsplineOptimizeTrajRebound / BsplineOptimizeTrajRefine
// ============================================================================
#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <rclcpp/rclcpp.hpp>
#include "bspline_opt/lbfgs.hpp"

// Gradient and elastic band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace scan_planner
{

// ControlPoints：优化过程中用到的控制点信息合集。
// 除了控制点坐标 points，还记录了“每个控制点该往哪个方向被顶开”（避障用）。
  class ControlPoints
  {
  public:
    double clearance;
    int size;
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the start of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it every time before using it.
    // std::vector<bool> occupancy;

    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();
      // occupancy.clear();

      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
      // occupancy.resize(size);
    }
  };

// 优化器本体。
  class BsplineOptimizer
  {

  public:
    BsplineOptimizer() {}
    ~BsplineOptimizer() {}

    /* main API */
    void setEnvironment(const GridMap::Ptr &env);
// 读参数：各代价项的权重 lambda、安全距离 dist0、速度/加速度上限等。
    void setParam(rclcpp::Node *node);
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);

    /* helper function */

    // required inputs
    void setControlPoints(const Eigen::MatrixXd &points);
    void setBsplineInterval(const double &ts);
    void setCostFunction(const int &cost_function);
    void setTerminateCond(const int &max_num_id, const int &max_time_id);

    // optional inputs
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most

    void optimize();

    Eigen::MatrixXd getControlPoints();

// A* 寻路器（由外部注入）。优化器用它来算避障的“顶开方向”。
    AStar::Ptr a_star_;
// 参考点：细化阶段用来把轨迹“拉回”到原轨迹附近（由外部填好）。
    std::vector<Eigen::Vector3d> ref_pts_;

// 【第一步】检查初始轨迹哪些控制点碰到障碍，对碰撞段跑 A*，
// 得到“该往哪个方向把控制点顶出去”，返回 A* 路径（仅作方向参考）。
    std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
// 【第二步（Rebound）】用 L-BFGS 优化控制点：平滑 + 避障 + 可行性。
// 注意：必须先调用 initControlPoints()。
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts); // must be called after initControlPoints()
// 【细化】在“时间拉长”之后再做一次优化：平滑 + 贴合原轨迹 + 可行性。
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    inline int getOrder(void) { return order_; }

  private:
    GridMap::Ptr grid_map_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    // main input
    // Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
    double bspline_interval_; // B-spline knot span
    Eigen::Vector3d end_pt_;  // end of the trajectory
    // int             dim_;                // dimension of the B-spline
    //
    vector<Eigen::Vector3d> guide_pts_; // geometric guiding path points, N-6
    vector<Eigen::Vector3d> waypoints_; // waypts constraints
    vector<int> waypt_idx_;             // waypts constraints index
                                        //
    int max_num_id_, max_time_id_;      // stopping criteria
    int cost_function_;                 // used to determine objective function
    double start_time_;                 // global time for moving obstacles

    /* optimization parameters */
    int order_;                    // bspline degree
// 代价权重：lambda1 平滑、lambda2 距离(避障)、lambda3 可行性、lambda4 贴合。
// 权重越大，优化时越优先满足对应那一项。
    double lambda1_;               // jerk smoothness weight
    double lambda2_, new_lambda2_; // distance weight
    double lambda3_;               // feasibility weight
    double lambda4_;               // curve fitting
    int a;
    //
// 期望的安全距离：离障碍小于它就会被惩罚，以及速度/加速度上限。
    double dist0_;             // safe distance
    double max_vel_, max_acc_; // dynamic limits

    int variable_num_;              // optimization variables
    int iter_num_;                  // iteration of the solver
    Eigen::VectorXd best_variable_; //
    double min_cost_;               //

    ControlPoints cps_;

    /* cost function */
    /* calculate each part of cost function with control points q as input */

    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    // q contains all control points
// ===== 四个代价项的计算函数（每个都会同时算出“代价”和“梯度”）=====
// calcSmoothnessCost：平滑项（用加加速度 jerk 衡量，越小越平滑）
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
// calcFeasibilityCost：可行性项（速度/加速度超过上限就惩罚）
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient);
// calcDistanceCostRebound：避障项（沿 A* 给出的方向把控制点顶离障碍）
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
// calcFitnessCost：贴合项（只在细化阶段用，把轨迹拉回参考点附近）
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    bool check_collision_and_rebound(void);
    double estimateSegmentYaw(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    double estimateControlPointYaw(const Eigen::MatrixXd &q, int id) const;

    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
// 交给 L-BFGS 的回调：L-BFGS 每走一步都会调用它，问“当前代价多大、梯度朝哪”。
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

// ===== 两个 L-BFGS 求解流程 =====
// rebound_optimize：反复调用 L-BFGS，找一组控制点让 Rebound 代价最小
    bool rebound_optimize();
// refine_optimize：同上，但用的是 Refine 那套代价（含贴合项）
    bool refine_optimize();
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);

    /* for benchmark evaluation only */
  public:
    typedef unique_ptr<BsplineOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner
#endif
