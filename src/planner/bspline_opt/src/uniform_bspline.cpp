// ============================================================================
// 文件：src/uniform_bspline.cpp —— B 样条曲线的实现
//
// 【这个文件里最值得看的三个函数】
//   evaluateDeBoor       ：求 t 时刻曲线在哪（De Boor 算法）
//   getDerivative        ：求导得到速度曲线（再求导得到加速度曲线）
//   parameterizeToBspline：给一串点，反解出控制点
// ============================================================================
#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{

  UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order,
                                 const double &interval)
  {
    // 构造函数直接委托给初始化函数，逻辑不重复写
    setUniformBspline(points, order, interval);
  }

  // 析构函数：没有需要手动释放的资源
  UniformBspline::~UniformBspline() {}

// 初始化：存下控制点/阶数/时间间隔，并按“均匀”规则生成节点向量。
  void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order,
                                         const double &interval)
  {
    // 存下控制点矩阵（每一列是一个控制点）
    control_points_ = points;
    // 记下阶数 p 和时间间隔 ts
    p_ = order;
    interval_ = interval;

    // 控制点个数减 1 记为 n；节点向量长度相关量 m = n + p + 1
    n_ = points.cols() - 1;
    m_ = n_ + p_ + 1;

    // 准备节点向量，长度是 m+1
    u_ = Eigen::VectorXd::Zero(m_ + 1);
    for (int i = 0; i <= m_; ++i)
    {

      // 开头 p+1 个节点：从 -p 开始等间隔排（让曲线从第一个控制点起步）
      if (i <= p_)
      {
        u_(i) = double(-p_ + i) * interval_;
      }
      // 中间节点：每个比前一个大一个时间间隔 —— 这就是“均匀”B 样条
      else if (i > p_ && i <= m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
      // 结尾节点：继续等间隔增长（让曲线终止在最后一个控制点）
      else if (i > m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
    }
  }

  void UniformBspline::setKnot(const Eigen::VectorXd &knot) { this->u_ = knot; }

  Eigen::VectorXd UniformBspline::getKnot() { return this->u_; }

  bool UniformBspline::getTimeSpan(double &um, double &um_p)
  {
    // 参数异常（节点向量长度不够）→ 返回失败
    if (p_ > u_.rows() || m_ - p_ > u_.rows())
      return false;

    // 曲线的有效时间范围是 [u_(p_), u_(m_-p_)]
    um = u_(p_);
    um_p = u_(m_ - p_);

    return true;
  }

  Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }

// 【核心】De Boor 算法：求参数 u 处曲线点的坐标。
// 做法：先根据 u 找到它落在哪一段（节点区间），
// 然后对该段附近的 p+1 个控制点做反复加权平均，直到收敛成一个点。
  Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u) const
  {

    // 先把参数 u 夹进有效范围内
    double ub = min(max(u_(p_), u), u_(m_ - p_));

    // determine which [ui,ui+1] lay in
    // 找出 u 落在哪个节点区间 [u_k, u_(k+1)]
    int k = p_;
    while (true)
    {
      if (u_(k + 1) >= ub)
        break;
      ++k;
    }

    /* deBoor's alg */
    // De Boor 递推的初始值：该区间附近的 p+1 个控制点
    vector<Eigen::VectorXd> d;
    for (int i = 0; i <= p_; ++i)
    {
      d.push_back(control_points_.col(k - p_ + i));
      // cout << d[i].transpose() << endl;
    }

    // 递推 p 轮，每轮把点集“收缩”一个
    for (int r = 1; r <= p_; ++r)
    {
      for (int i = p_; i >= r; --i)
      {
        // 加权系数：由当前区间在节点向量中的位置决定
        double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
        // cout << "alpha: " << alpha << endl;
        // 相邻两点做加权平均 —— 这就是 De Boor 的核心一步
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
      }
    }

    // 递推结束后只剩下一个点，它就是曲线在该处的坐标
    return d[p_];
  }

  // Eigen::VectorXd UniformBspline::evaluateDeBoorT(const double& t) {
  //   return evaluateDeBoor(t + u_(p_));
  // }

// 推导“导数曲线的控制点”：由原控制点两两做差分得到（阶数相应降 1）。
  Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
  {
    // The derivative of a b-spline is also a b-spline, its order become p_-1
    // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
    // 导数曲线的控制点个数少 1 个，阶数降低 1
    Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
    for (int i = 0; i < ctp.cols(); ++i)
    {
      // 求导公式：相邻控制点之差 × 阶数 ÷ 节点间隔
      ctp.col(i) =
          p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
    }
    return ctp;
  }

// 返回导数曲线（速度曲线）。注意要裁掉首尾各一个节点，才是有效范围。
  UniformBspline UniformBspline::getDerivative()
  {
    // 先算出“导数曲线”的控制点
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    // 用它们构造一条阶数降 1 的新曲线
    UniformBspline derivative(ctp, p_ - 1, interval_);

    /* cut the first and last knot */
    // 节点向量要掐头去尾各一个，才是导数曲线的有效范围
    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);
    derivative.setKnot(knot);

    return derivative;
  }

  double UniformBspline::getInterval() { return interval_; }

// 记录物理限制，供 checkFeasibility 使用。
  void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
  {
    // 记下速度上限、加速度上限，以及允许的容差
    limit_vel_ = vel;
    limit_acc_ = acc;
    feasibility_tolerance_ = tolerance;
  }

// 逐段检查速度/加速度是否超限；
// 若超限则返回 false，并用 ratio 给出“超出的倍率”（取速度和加速度中较大的那个）。
  bool UniformBspline::checkFeasibility(double &ratio, bool show)
  {
    // 默认认为“可行”；一旦发现超限就置为 false
    bool fea = true;

    // 下面用“相邻控制点差分”来估算每一段的速度和加速度
    Eigen::MatrixXd P = control_points_;
    int dimension = control_points_.rows();

    /* check vel feasibility and insert points */
    // 记录观测到的最大速度（比较基准是“上限 × (1+容差)”）
    double max_vel = -1.0;
    double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
    // 逐段检查速度
    for (int i = 0; i < P.cols() - 1; ++i)
    {
      // 该段的速度估计值
      Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1));

      // 任一坐标轴超过上限 → 判定不可行
      if (fabs(vel(0)) > enlarged_vel_lim || fabs(vel(1)) > enlarged_vel_lim ||
          fabs(vel(2)) > enlarged_vel_lim)
      {

        if (show)
          cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
        // 标记为不可行
        fea = false;

        // 记录最大速度分量（最后用来算“超了几倍”）
        for (int j = 0; j < dimension; ++j)
        {
          max_vel = max(max_vel, fabs(vel(j)));
        }
      }
    }

    /* acc feasibility */
    // 下面用二阶差分检查加速度
    double max_acc = -1.0;
    double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
    // 逐段检查加速度
    for (int i = 0; i < P.cols() - 2; ++i)
    {

      // 加速度估计：二阶差分 ÷ 时间间隔的平方
      Eigen::VectorXd acc = p_ * (p_ - 1) *
                            ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                             (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                            (u_(i + p_ + 1) - u_(i + 2));

      if (fabs(acc(0)) > enlarged_acc_lim || fabs(acc(1)) > enlarged_acc_lim ||
          fabs(acc(2)) > enlarged_acc_lim)
      {

        if (show)
          cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
        fea = false;

        for (int j = 0; j < dimension; ++j)
        {
          max_acc = max(max_acc, fabs(acc(j)));
        }
      }
    }

    // ratio 取“速度超出倍率”和“加速度超出倍率”里较大的那个
    ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

    return fea;
  }

// 把中间各节点按比例拉伸，使整条曲线的时间变成原来的 ratio 倍。
// 首尾各留 5 个节点不动，保证边界形状稳定。
  void UniformBspline::lengthenTime(const double &ratio)
  {
    // 首尾各保留 5 个节点不动，保证曲线两端形状不变
    int num1 = 5;
    int num2 = getKnot().rows() - 1 - 5;

    // 总共需要增加多少时间
    double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
    double t_inc = delta_t / double(num2 - num1);
    // 中间节点：离起点越远，增加得越多（按比例拉伸）
    for (int i = num1 + 1; i <= num2; ++i)
      u_(i) += double(i - num1) * t_inc;
    // 末尾节点整体向后平移 delta_t
    for (int i = num2 + 1; i < u_.rows(); ++i)
      u_(i) += delta_t;
  }

  // void UniformBspline::recomputeInit() {}

// 【拟合】建立线性方程组并求解，得到控制点：
//   每个已知点对应一个“控制点的加权平均”方程（权重 1:4:1）
//   起点/终点的速度和加速度各给一个约束方程
//   解这个方程组就得到 (K+6) 个控制点。
  void UniformBspline::parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                             const vector<Eigen::Vector3d> &start_end_derivative,
                                             Eigen::MatrixXd &ctrl_pts)
  {
    // 参数检查：时间间隔必须为正
    if (ts <= 0)
    {
      cout << "[B-spline]:time step error." << endl;
      return;
    }

    // 已知点至少要 4 个，否则解不出控制点
    if (point_set.size() <= 3)
    {
      cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
      return;
    }

    // 边界导数必须是 4 个：起点速度、终点速度、起点加速度、终点加速度
    if (start_end_derivative.size() != 4)
    {
      cout << "[B-spline]:derivatives error." << endl;
    }

    // K = 已知点的个数
    int K = point_set.size();

    // write A
    // 三个“模板行”：位置(1,4,1)、速度(-1,0,1)、加速度(1,-2,1)
    Eigen::Vector3d prow(3), vrow(3), arow(3);
    prow << 1, 4, 1;
    vrow << -1, 0, 1;
    arow << 1, -2, 1;

    // 未知量是 K+2 个控制点；方程数 = K 个位置 + 4 个边界约束
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

    // 每个已知点对应一行 (1/6)(1,4,1) 的加权平均方程
    for (int i = 0; i < K; ++i)
      A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

    // 起点速度约束
    A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    // 终点速度约束
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

    // 起点加速度约束
    A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
    // 终点加速度约束
    A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();

    //cout << "A" << endl << A << endl << endl;

    // write b
    Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
    // 方程右端：前 K 项是各个已知点的 x/y/z 坐标
    for (int i = 0; i < K; ++i)
    {
      bx(i) = point_set[i](0);
      by(i) = point_set[i](1);
      bz(i) = point_set[i](2);
    }

    // 后 4 项是起点/终点的速度与加速度
    for (int i = 0; i < 4; ++i)
    {
      bx(K + i) = start_end_derivative[i](0);
      by(K + i) = start_end_derivative[i](1);
      bz(K + i) = start_end_derivative[i](2);
    }

    // solve Ax = b
    // 解线性方程组（用带列主元的 QR 分解，数值更稳定）
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

    // convert to control pts
    // 把解出的 x/y/z 三行拼成“控制点矩阵”
    ctrl_pts.resize(3, K + 2);
    ctrl_pts.row(0) = px.transpose();
    ctrl_pts.row(1) = py.transpose();
    ctrl_pts.row(2) = pz.transpose();

    // cout << "[B-spline]: parameterization ok." << endl;
  }

// 曲线总时长 = 最后节点 - 第一个有效节点。
  double UniformBspline::getTimeSum()
  {
    // 取有效时间范围；总时长 = 上界 - 下界
    double tm, tmp;
    if (getTimeSpan(tm, tmp))
      return tmp - tm;
    else
      return -1.0;
  }

  double UniformBspline::getLength(const double &res)
  {
    double length = 0.0;
    double dur = getTimeSum();
    // 下面按固定步长采样，把相邻点的距离累加起来 ≈ 曲线弧长
    Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
    // 逐段累加长度
    for (double t = res; t <= dur + 1e-4; t += res)
    {
      p_n = evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  double UniformBspline::getJerk()
  {
    // 位置曲线求三次导，得到“加加速度(jerk)”曲线
    UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

    // 用“每个控制点幅值的平方 × 时间跨度”近似 jerk 的积分（越小越平滑）
    Eigen::VectorXd times = jerk_traj.getKnot();
    Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
    int dimension = ctrl_pts.rows();

    double jerk = 0.0;
    // 逐个控制点累加
    for (int i = 0; i < ctrl_pts.cols(); ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
      }
    }

    return jerk;
  }

  void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    // 取速度曲线，按 0.01 秒采样求平均速度和最大速度
    UniformBspline vel = getDerivative();
    double tm, tmp;
    vel.getTimeSpan(tm, tmp);

    double max_vel = -1.0, mean_vel = 0.0;
    int num = 0;
    // 逐点采样
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
      double vn = vxd.norm();

      mean_vel += vn;
      ++num;
      if (vn > max_vel)
      {
        max_vel = vn;
      }
    }

    // 平均速度 = 总和 ÷ 采样次数
    mean_vel = mean_vel / double(num);
    mean_v = mean_vel;
    max_v = max_vel;
  }

  void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    // 同理，对加速度曲线采样统计
    UniformBspline acc = getDerivative().getDerivative();
    double tm, tmp;
    acc.getTimeSpan(tm, tmp);

    double max_acc = -1.0, mean_acc = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd axd = acc.evaluateDeBoor(t);
      double an = axd.norm();

      mean_acc += an;
      ++num;
      if (an > max_acc)
      {
        max_acc = an;
      }
    }

    // 平均加速度 = 总和 ÷ 采样次数
    mean_acc = mean_acc / double(num);
    mean_a = mean_acc;
    max_a = max_acc;
  }
} // namespace scan_planner
