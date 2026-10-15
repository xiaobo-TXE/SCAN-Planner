// ============================================================================
// 文件：include/bspline_opt/uniform_bspline.h
//
// 【这个文件是干什么的】
//   定义“B 样条曲线”类 UniformBspline。B 样条是一种用少数几个点
//   （叫“控制点”）就能描述一条光滑曲线的数学工具。
//
// 【为什么用它描述轨迹】
//   只要挪动几个控制点，整条曲线就会平滑地跟着变。
//   所以优化器只需要调那几个控制点，就能得到一条平滑的轨迹——
//   比直接摆弄成百上千个轨迹点快得多。
//
// 【几个关键词】
//   控制点 control points：决定曲线形状的几个点
//   阶数 order / 次数 p   ：3 表示三次 B 样条（本项目用 3）
//   节点向量 knots u_     ：决定“每个控制点管曲线的哪一段”
//   时间间隔 interval_ ts ：相邻控制点之间隔多少秒
//
// 【最常用的几个方法】
//   evaluateDeBoorT(t) ：求 t 时刻的位置（t 从 0 到总时长）
//   getDerivative()    ：对曲线求导 → 得到速度曲线（再求导就是加速度）
//   parameterizeToBspline：把一串点拟合成 B 样条（求控制点）
// ============================================================================
#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace scan_planner
{
  // An implementation of non-uniform B-spline with different dimensions
  // It also represents uniform B-spline which is a special case of non-uniform
// B 样条类。它可以表示任意维度的曲线（本项目用 3 维：x,y,z）。
  class UniformBspline
  {
  private:
    /* ---------- 曲线本体：控制点矩阵 ---------- */
    // control points for B-spline with different dimensions.
    // Each row represents one single control point
    // The dimension is determined by column number
    // e.g. B-spline with N points in 3D space -> Nx3 matrix
// 控制点矩阵：每一列是一个控制点（3 x N 表示 N 个三维控制点）
    // 优化器调整的就是这些列；挪动它们，整条曲线平滑地跟着变
    Eigen::MatrixXd control_points_;

    /* ---------- B 样条的基本参数 ---------- */
// p=阶数(次数)，n+1=控制点个数，m = n+p+1（节点向量长度相关）
    int p_, n_, m_;     // p degree, n+1 control points, m = n+p+1
    // p_: 阶数(次数)，本项目=3(三次); n_: 控制点个数-1; m_: 节点向量长度相关量
// 节点向量（knots）：B 样条数学定义里的必需参数
    Eigen::VectorXd u_; // knots vector
    // u_: 节点向量，决定"每个控制点管曲线的哪一段"
// 时间间隔 ts：相邻控制点在时间上相差多少秒
    double interval_;   // knot span \delta t
    // interval_: 相邻控制点在时间上的间隔(秒)，也叫 knot span

    /* ---------- 内部工具与物理限制 ---------- */
    // 由控制点差分算出"导数曲线的控制点"（求导用，外部不直接调）
    Eigen::MatrixXd getDerivativeControlPoints();

    // 物理限制：最大速度/最大加速度，以及允许超限的容差比例
    double limit_vel_, limit_acc_, feasibility_tolerance_; // physical limits and feasibility tolerance

  public:
    /* ---------- 构造 / 析构 ---------- */
    // 默认构造：空对象（后续用 setUniformBspline 初始化）
    UniformBspline() {}
    // 用控制点/阶数/时间间隔构造（内部会生成节点向量）
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
    ~UniformBspline();

    // 取控制点矩阵（内联版；另有带下划线的 getControlPoint 同义）
    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    // initialize as an uniform B-spline
// 用给定的控制点/阶数/时间间隔来初始化这条曲线（会自动生成节点向量）
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // get / set basic bspline info
    /* ---------- 基本信息的读写 ---------- */

    // 直接用一批节点向量覆盖当前曲线的节点向量（控制器从消息重建曲线时用）
    void setKnot(const Eigen::VectorXd &knot);
    // 取节点向量 / 取控制点 / 取时间间隔
    Eigen::VectorXd getKnot();
    Eigen::MatrixXd getControlPoint();
    double getInterval();
// 返回曲线有效的时间范围 [um, um_p]（超出范围的部分没有定义）
    bool getTimeSpan(double &um, double &um_p);

    // compute position / derivative
    /* ---------- 求值 / 求导 ---------- */

// 【求值】用 De Boor 算法算出参数 u 处的曲线点。
// 参数 u 是“节点向量意义上的时间”，下面那个 T 版本才是常用的秒数。
    // 输入: u 是节点向量意义上的参数(范围 [u_p, u_m-p]); 输出: 该处的位置(长度=维度)
    Eigen::VectorXd evaluateDeBoor(const double &u) const;                                               // use u \in [up, u_mp]
    // 【常用版】输入秒数 t∈[0, duration]，输出 t 时刻的位置。
    // 内部只是把 t 平移成节点参数 t+u_(p_) 再调 evaluateDeBoor，所以外部一律用这个。
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) const { return evaluateDeBoor(t + u_(p_)); } // use t \in [0, duration]
// 【求导】返回这条曲线的导数曲线。位置曲线求导→速度曲线，再求导→加速度曲线。
    // 实现：导数曲线的控制点 = 相邻控制点之差 × p / interval（见 .cpp）
    UniformBspline getDerivative();

    // 3D B-spline interpolation of points in point_set, with boundary vel&acc
    // constraints
    // input : (K+2) points with boundary vel/acc; ts
    // output: (K+6) control_pts
// 【拟合】给一串点 + 起点/终点的速度和加速度，反解出 B 样条的控制点。
// 这是“把 A* 规划的一串点变成光滑曲线”的关键一步。
    // 输入: ts 相邻控制点时间间隔; point_set 一串三维点; start_end_derivative 起终点速度/加速度(4 个)
    // 输出: ctrl_pts —— 反解出的控制点矩阵(3 x N)。解的是线性方程组，见 .cpp
    static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                      const vector<Eigen::Vector3d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);

    /* check feasibility, adjust time */
    /* ---------- 可行性检查 + 时间重分配 ---------- */

// 告诉曲线物理限制（最大速度/加速度）以及允许的容差，供下面检查用
    // 输入: vel 最大速度; acc 最大加速度; tolerance 允许超限的比例
    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
// 【可行性检查】看看曲线有没有超速/超加速度；
// ratio 返回“超了几倍”，供调用者决定把时间拉长多少。
    // 输入: show 是否打印细节; 输出: ratio 超限倍数; 返回 false 表示超限了
    bool checkFeasibility(double &ratio, bool show = false);
// 【重新分配时间】把曲线的时间轴拉长 ratio 倍（同样一段路走得更慢）。
// 这样速度/加速度自然就降下来了。
    // 输入: ratio 拉长倍数; 副作用: 直接修改本对象的节点向量/时间轴
    void lengthenTime(const double &ratio);

    /* for performance evaluation */
    /* ---------- 性能/质量统计（只用于评估，不参与规划） ---------- */

// 曲线总时长。下面几个 getLength/getJerk/getMeanAndMaxVel 都是评估用的统计量。
    // 曲线总时长(秒)
    double getTimeSum();
    // 曲线长度(米)，按 res 分辨率采样后累加折线长度
    double getLength(const double &res = 0.01);
    // 加加速度(jerk)代价：衡量平滑程度，越小越平滑
    double getJerk();
    void getMeanAndMaxVel(double &mean_v, double &max_v);
    void getMeanAndMaxAcc(double &mean_a, double &max_a);

    // Eigen 类的标准要求：保证含 Eigen 成员的对象内存对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace scan_planner
#endif
