// ============================================================================
// 文件：include/bspline_opt/gradient_descent_optimizer.h
//
// 【这个文件是干什么的】
//   一个通用的“求最小值”的数学工具（梯度下降类）。
//
// 【重要提醒】
//   本项目实际使用的是另一个工具 L-BFGS（见 lbfgs.hpp），
//   这个类虽然被 bspline_optimizer.cpp #include 了，但**从未真正被调用**，
//   属于从上游项目遗留下来的“备用/历史代码”。了解即可，不影响规划结果。
// ============================================================================
#ifndef _GRADIENT_DESCENT_OPT_H_
#define _GRADIENT_DESCENT_OPT_H_

#include <iostream>
#include <limits>
#include <vector>
#include <Eigen/Eigen>

using namespace std;

// 梯度下降优化器（当前未被使用）。
class GradientDescentOptimizer
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

// 目标函数的类型定义：输入一组变量，返回“代价”并顺便填出“梯度”。
  typedef double (*objfunDef)(const Eigen::VectorXd &x, Eigen::VectorXd &grad, bool &force_return, void *data);
// 优化结果：找到最小值 / 失败 / 被要求提前返回 / 达到最大迭代次数。
  enum RESULT
  {
    FIND_MIN,
    FAILED,
    RETURN_BY_ORDER,
    REACH_MAX_ITERATION
  };

  GradientDescentOptimizer(int v_num, objfunDef objf, void *f_data)
  {
    variable_num_ = v_num;
    objfun_ = objf;
    f_data_ = f_data;
  };

  void set_maxiter(int limit) { iter_limit_ = limit; }
  void set_maxeval(int limit) { invoke_limit_ = limit; }
  void set_xtol_rel(double xtol_rel) { xtol_rel_ = xtol_rel; }
  void set_xtol_abs(double xtol_abs) { xtol_abs_ = xtol_abs; }
  void set_min_grad(double min_grad) { min_grad_ = min_grad; }

// 开始优化：从初值出发反复迭代，返回结果状态，最优代价写入 opt_f。
  RESULT optimize(Eigen::VectorXd &x_init_optimal, double &opt_f);

private:
  int variable_num_{0};
  int iter_limit_{std::numeric_limits<int>::max()};
  int invoke_limit_{std::numeric_limits<int>::max()};
  double xtol_rel_;
  double xtol_abs_;
  double min_grad_;
  double time_limit_;
  void *f_data_;
  objfunDef objfun_;
};

#endif
