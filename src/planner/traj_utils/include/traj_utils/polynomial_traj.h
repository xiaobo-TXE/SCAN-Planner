// ============================================================================
// 文件：include/traj_utils/polynomial_traj.h
//
// 【这个文件是干什么的】
//   定义“多项式轨迹”类 PolynomialTraj —— 用来表示全局参考轨迹。
//
// 【数据结构】
//   一条轨迹由若干“段”组成，每段是一条关于时间 t 的多项式：
//     x(t) = c0 + c1·t + c2·t² + ... （y、z 同理）
//   所以它存三组系数（x/y/z），外加每段的时长。
//
// 【注意系数的存放顺序】
//   系数是“从高次到低次”倒着存的（与求值时的写法有关，见 .cpp 的注释）。
// ============================================================================
#ifndef _POLYNOMIAL_TRAJ_H
#define _POLYNOMIAL_TRAJ_H

#include <Eigen/Eigen>
#include <vector>

using std::vector;

// 多项式轨迹类
class PolynomialTraj
{
private:
  // 每段的时长
  vector<double> times;       // time of each segment
  // 每段 x/y/z 的多项式系数（注意：按“高次→低次”排列）
  vector<vector<double>> cxs; // coefficient of x of each segment, from n-1 -> 0
  vector<vector<double>> cys; // coefficient of y of each segment
  vector<vector<double>> czs; // coefficient of z of each segment

  // 总时长与段数
  double time_sum;
  int num_seg;

  /* evaluation */
  vector<Eigen::Vector3d> traj_vec3d;
  double length;

public:
  PolynomialTraj(/* args */)
  {
  }
  ~PolynomialTraj()
  {
  }

  // 清空所有段，回到初始状态
  void reset()
  {
    times.clear(), cxs.clear(), cys.clear(), czs.clear();
    time_sum = 0.0, num_seg = 0;
  }

  // 追加一段：给定 x/y/z 的系数和这一段的时长
  void addSegment(vector<double> cx, vector<double> cy, vector<double> cz, double t)
  {
    cxs.push_back(cx), cys.push_back(cy), czs.push_back(cz), times.push_back(t);
  }

  // 初始化：统计总时长（必须在所有 addSegment 之后调用）
  void init()
  {
    num_seg = times.size();
    time_sum = 0.0;
    for (int i = 0; i < times.size(); ++i)
    {
      time_sum += times[i];
    }
  }

  vector<double> getTimes()
  {
    return times;
  }

  vector<vector<double>> getCoef(int axis)
  {
    switch (axis)
    {
    case 0:
      return cxs;
    case 1:
      return cys;
    case 2:
      return czs;
    default:
      std::cout << "\033[31mIllegal axis!\033[0m" << std::endl;
    }

    vector<vector<double>> empty;
    return empty;
  }

  // 【求值】算出 t 时刻的位置：先定位到哪一段，再做多项式求值
  Eigen::Vector3d evaluate(double t)
  {
    /* detetrmine segment num */
    int idx = 0;
    while (times[idx] + 1e-4 < t)
    {
      t -= times[idx];
      ++idx;
    }

    /* evaluation */
    int order = cxs[idx].size();
    Eigen::VectorXd cx(order), cy(order), cz(order), tv(order);
    for (int i = 0; i < order; ++i)
    {
      cx(i) = cxs[idx][i], cy(i) = cys[idx][i], cz(i) = czs[idx][i];
      tv(order - 1 - i) = std::pow(t, double(i));
    }

    Eigen::Vector3d pt;
    pt(0) = tv.dot(cx), pt(1) = tv.dot(cy), pt(2) = tv.dot(cz);
    return pt;
  }

  // 求 t 时刻的速度（对位置多项式求导，系数相应降一次）
  Eigen::Vector3d evaluateVel(double t)
  {
    /* detetrmine segment num */
    int idx = 0;
    while (times[idx] + 1e-4 < t)
    {
      t -= times[idx];
      ++idx;
    }

    /* evaluation */
    int order = cxs[idx].size();
    Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);

    /* coef of vel */
    for (int i = 0; i < order - 1; ++i)
    {
      vx(i) = double(i + 1) * cxs[idx][order - 2 - i];
      vy(i) = double(i + 1) * cys[idx][order - 2 - i];
      vz(i) = double(i + 1) * czs[idx][order - 2 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 1);
    for (int i = 0; i < order - 1; ++i)
      tv(i) = pow(ts, i);

    Eigen::Vector3d vel;
    vel(0) = tv.dot(vx), vel(1) = tv.dot(vy), vel(2) = tv.dot(vz);
    return vel;
  }

  // 求 t 时刻的加速度（再求一次导）
  Eigen::Vector3d evaluateAcc(double t)
  {
    /* detetrmine segment num */
    int idx = 0;
    while (times[idx] + 1e-4 < t)
    {
      t -= times[idx];
      ++idx;
    }

    /* evaluation */
    int order = cxs[idx].size();
    Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

    /* coef of vel */
    for (int i = 0; i < order - 2; ++i)
    {
      ax(i) = double((i + 2) * (i + 1)) * cxs[idx][order - 3 - i];
      ay(i) = double((i + 2) * (i + 1)) * cys[idx][order - 3 - i];
      az(i) = double((i + 2) * (i + 1)) * czs[idx][order - 3 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 2);
    for (int i = 0; i < order - 2; ++i)
      tv(i) = pow(ts, i);

    Eigen::Vector3d acc;
    acc(0) = tv.dot(ax), acc(1) = tv.dot(ay), acc(2) = tv.dot(az);
    return acc;
  }

  /* for evaluating traj, should be called in sequence!!! */
  // 总时长；下面几个都是“采样统计量”（供评估轨迹质量用）
  double getTimeSum()
  {
    return this->time_sum;
  }

  vector<Eigen::Vector3d> getTraj()
  {
    double eval_t = 0.0;
    traj_vec3d.clear();
    while (eval_t < time_sum)
    {
      Eigen::Vector3d pt = evaluate(eval_t);
      traj_vec3d.push_back(pt);
      eval_t += 0.01;
    }
    return traj_vec3d;
  }

  // 轨迹长度：把采样点连起来求折线长度
  double getLength()
  {
    length = 0.0;

    Eigen::Vector3d p_l = traj_vec3d[0], p_n;
    for (int i = 1; i < traj_vec3d.size(); ++i)
    {
      p_n = traj_vec3d[i];
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  // 平均速度（注意：这个函数少了 return，返回值不可用，属于遗留小 bug）
  double getMeanVel()
  {
    double mean_vel = length / time_sum;
  }

  // 加速度代价：衡量轨迹的平滑程度
  double getAccCost()
  {
    double cost = 0.0;
    int order = cxs[0].size();

    for (int s = 0; s < times.size(); ++s)
    {
      Eigen::Vector3d um;
      um(0) = 2 * cxs[s][order - 3], um(1) = 2 * cys[s][order - 3], um(2) = 2 * czs[s][order - 3];
      cost += um.squaredNorm() * times[s];
    }

    return cost;
  }

  // 加加速度(jerk)代价：比加速度更严格的平滑度指标
  double getJerk()
  {
    double jerk = 0.0;

    /* evaluate jerk */
    for (int s = 0; s < times.size(); ++s)
    {
      Eigen::VectorXd cxv(cxs[s].size()), cyv(cys[s].size()), czv(czs[s].size());
      /* convert coefficient */
      int order = cxs[s].size();
      for (int j = 0; j < order; ++j)
      {
        cxv(j) = cxs[s][order - 1 - j], cyv(j) = cys[s][order - 1 - j], czv(j) = czs[s][order - 1 - j];
      }
      double ts = times[s];

      /* jerk matrix */
      Eigen::MatrixXd mat_jerk(order, order);
      mat_jerk.setZero();
      for (int i = 3; i < order; i += 1)
        for (int j = 3; j < order; j += 1)
        {
          mat_jerk(i, j) =
              i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) * pow(ts, i + j - 5) / (i + j - 5);
        }

      jerk += (cxv.transpose() * mat_jerk * cxv)(0, 0);
      jerk += (cyv.transpose() * mat_jerk * cyv)(0, 0);
      jerk += (czv.transpose() * mat_jerk * czv)(0, 0);
    }

    return jerk;
  }

  // 采样统计平均速度与最大速度
  void getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    int num = 0;
    mean_v = 0.0, max_v = -1.0;
    for (int s = 0; s < times.size(); ++s)
    {
      int order = cxs[s].size();
      Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);

      /* coef of vel */
      for (int i = 0; i < order - 1; ++i)
      {
        vx(i) = double(i + 1) * cxs[s][order - 2 - i];
        vy(i) = double(i + 1) * cys[s][order - 2 - i];
        vz(i) = double(i + 1) * czs[s][order - 2 - i];
      }
      double ts = times[s];

      double eval_t = 0.0;
      while (eval_t < ts)
      {
        Eigen::VectorXd tv(order - 1);
        for (int i = 0; i < order - 1; ++i)
          tv(i) = pow(ts, i);
        Eigen::Vector3d vel;
        vel(0) = tv.dot(vx), vel(1) = tv.dot(vy), vel(2) = tv.dot(vz);
        double vn = vel.norm();
        mean_v += vn;
        if (vn > max_v)
          max_v = vn;
        ++num;

        eval_t += 0.01;
      }
    }

    mean_v = mean_v / double(num);
  }

  // 采样统计平均加速度与最大加速度
  void getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    int num = 0;
    mean_a = 0.0, max_a = -1.0;
    for (int s = 0; s < times.size(); ++s)
    {
      int order = cxs[s].size();
      Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

      /* coef of acc */
      for (int i = 0; i < order - 2; ++i)
      {
        ax(i) = double((i + 2) * (i + 1)) * cxs[s][order - 3 - i];
        ay(i) = double((i + 2) * (i + 1)) * cys[s][order - 3 - i];
        az(i) = double((i + 2) * (i + 1)) * czs[s][order - 3 - i];
      }
      double ts = times[s];

      double eval_t = 0.0;
      while (eval_t < ts)
      {
        Eigen::VectorXd tv(order - 2);
        for (int i = 0; i < order - 2; ++i)
          tv(i) = pow(ts, i);
        Eigen::Vector3d acc;
        acc(0) = tv.dot(ax), acc(1) = tv.dot(ay), acc(2) = tv.dot(az);
        double an = acc.norm();
        mean_a += an;
        if (an > max_a)
          max_a = an;
        ++num;

        eval_t += 0.01;
      }
    }

    mean_a = mean_a / double(num);
  }

  // 【静态工厂】经过多个途经点、且让四阶导(snap)总代价最小的多项式轨迹（详见 .cpp）
  static PolynomialTraj minSnapTraj(const Eigen::MatrixXd &Pos, const Eigen::Vector3d &start_vel,
                                    const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                                    const Eigen::Vector3d &end_acc, const Eigen::VectorXd &Time);

  // 【静态工厂】只有起点终点、单段五次多项式的轨迹
  static PolynomialTraj one_segment_traj_gen(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                             const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
                                             double t);
};

#endif
