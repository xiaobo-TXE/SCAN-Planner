// ============================================================================
// 文件：src/polynomial_traj.cpp —— 多项式轨迹的实现
//
// 【这个文件是干什么的】
//   生成“全局参考轨迹”：用一条（或多段）多项式把起点和终点连起来。
//   它是“大方向”的路，不是机器人真正执行的那条轨迹。
//
// 【两个函数】
//   minSnapTraj          ：经过多个途经点、并且让“四阶导(snap)”的总代价最小
//   one_segment_traj_gen ：只有起点和终点，解一个 6×6 方程组即可
//
// 【为什么要用多项式】
//   多项式能精确满足起点/终点的位置、速度、加速度约束，而且求导很简单。
// ============================================================================
#include <iostream>
#include <traj_utils/polynomial_traj.h>

PolynomialTraj PolynomialTraj::minSnapTraj(const Eigen::MatrixXd &Pos, const Eigen::Vector3d &start_vel,
                                           const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_acc, const Eigen::VectorXd &Time)
{
  // 段数 = 时间数组长度（每段是一条五次多项式）
  int seg_num = Time.size();
  // 结果矩阵：每段 6 个系数，三个轴（x/y/z）各占 6 列
  Eigen::MatrixXd poly_coeff(seg_num, 3 * 6);
  // 把所有段的系数拼成一个大向量（x/y/z 各一个）
  Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);

  int num_f, num_p; // number of fixed and free variables
  int num_d;        // number of all segments' derivatives

  // 阶乘小工具，下面构造矩阵要用
  const static auto Factorial = [](int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
      fac = fac * i;
    return fac;
  };

  /* ---------- end point derivative ---------- */
  // 每段多项式有 6 个“边界条件”，先摆成一个长向量
  Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dz = Eigen::VectorXd::Zero(seg_num * 6);

  // 逐段填写已知的边界条件
  for (int k = 0; k < seg_num; k++)
  {
    /* position to derivative */
    // 第 0、1 个条件：这一段起点和终点的“位置”
    Dx(k * 6) = Pos(0, k);
    Dx(k * 6 + 1) = Pos(0, k + 1);
    Dy(k * 6) = Pos(1, k);
    Dy(k * 6 + 1) = Pos(1, k + 1);
    Dz(k * 6) = Pos(2, k);
    Dz(k * 6 + 1) = Pos(2, k + 1);

    // 第一段额外给出整体起点的“速度和加速度”
    if (k == 0)
    {
      Dx(k * 6 + 2) = start_vel(0);
      Dy(k * 6 + 2) = start_vel(1);
      Dz(k * 6 + 2) = start_vel(2);

      Dx(k * 6 + 4) = start_acc(0);
      Dy(k * 6 + 4) = start_acc(1);
      Dz(k * 6 + 4) = start_acc(2);
    }
    // 最后一段额外给出整体终点的“速度和加速度”
    else if (k == seg_num - 1)
    {
      Dx(k * 6 + 3) = end_vel(0);
      Dy(k * 6 + 3) = end_vel(1);
      Dz(k * 6 + 3) = end_vel(2);

      Dx(k * 6 + 5) = end_acc(0);
      Dy(k * 6 + 5) = end_acc(1);
      Dz(k * 6 + 5) = end_acc(2);
    }
  }

  /* ---------- Mapping Matrix A ---------- */
  Eigen::MatrixXd Ab;
  // 映射矩阵 A：把“多项式系数”换算成“端点位置/速度/加速度”
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  // 每段对应一个 6×6 的小块
  for (int k = 0; k < seg_num; k++)
  {
    Ab = Eigen::MatrixXd::Zero(6, 6);
    // 小块前 3 行对应 t=0（起点），后 3 行对应 t=T（终点）
    for (int i = 0; i < 3; i++)
    {
      // t=0 处的取值：只有对应次幂的系数留下
      Ab(2 * i, i) = Factorial(i);
      for (int j = i; j < 6; j++)
        // t=T 处的取值公式（系数 × 阶乘比 × T 的幂）
        Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);
    }
    // 把这个小块放进大矩阵的对角位置
    A.block(k * 6, k * 6, 6, 6) = Ab;
  }

  /* ---------- Produce Selection Matrix C' ---------- */
  Eigen::MatrixXd Ct, C;

  // 固定变量个数 = 2m+4：各段端点 + 整体起终点的速度和加速度
  num_f = 2 * seg_num + 4; // 3 + 3 + (seg_num - 1) * 2 = 2m + 4
  // 自由变量个数 = 2m-2：中间连接点处的导数，正是我们要求解的
  num_p = 2 * seg_num - 2; //(seg_num - 1) * 2 = 2m - 2
  num_d = 6 * seg_num;
  // 选择矩阵 C'：编码“哪些量已知、哪些量待求”
  Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);
  Ct(0, 0) = 1;
  Ct(2, 1) = 1;
  // 整体起点的位置、速度、加速度
  Ct(4, 2) = 1; // stack the start point
  Ct(1, 3) = 1;
  Ct(3, 2 * seg_num + 4) = 1;
  Ct(5, 2 * seg_num + 5) = 1;

  Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1;
  Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1; // Stack the end point

  // 中间各段的连接点：保证位置、速度、加速度都连续
  for (int j = 2; j < seg_num; j++)
  {
    Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1;
    Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1;
    Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1;
    Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
  }

  // C 就是 C' 的转置
  C = Ct.transpose();

  // 把边界条件按新的顺序重排
  Eigen::VectorXd Dx1 = C * Dx;
  Eigen::VectorXd Dy1 = C * Dy;
  Eigen::VectorXd Dz1 = C * Dz;

  /* ---------- minimum snap matrix ---------- */
  // 构造代价矩阵 Q：衡量“四阶导(snap)”的大小
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  // 每段一个 6×6 小块
  for (int k = 0; k < seg_num; k++)
  {
    for (int i = 3; i < 6; i++)
    {
      for (int j = 3; j < 6; j++)
      {
        // 四次导数平方在 [0,T] 上的积分，有解析公式（所以能写成闭式解）
        Q(k * 6 + i, k * 6 + j) =
            i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
      }
    }
  }

  /* ---------- R matrix ---------- */
  // R = C·A⁻ᵀ·Q·A⁻¹·C'：把总代价表示成“边界条件”的函数
  Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;

  Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4), Dzf(2 * seg_num + 4);

  // 把边界条件拆成“固定部分 f”和“自由部分 p”
  Dxf = Dx1.segment(0, 2 * seg_num + 4);
  Dyf = Dy1.segment(0, 2 * seg_num + 4);
  Dzf = Dz1.segment(0, 2 * seg_num + 4);

  Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);
  Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);
  Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);
  Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);

  // R 也按 f / p 拆成四块（Rff、Rfp、Rpf、Rpp）
  Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
  Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
  Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
  Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

  /* ---------- close form solution ---------- */

  Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2), Dzp(2 * seg_num - 2);
  // 【闭式解】自由变量 = -Rpp⁻¹·Rfpᵀ·固定变量（代价最小的充要条件）
  Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
  Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
  Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

  // 把求出的自由变量填回边界条件向量
  Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
  Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
  Dz1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;

  // 再由边界条件反解出各段的多项式系数
  Px = (A.inverse() * Ct) * Dx1;
  Py = (A.inverse() * Ct) * Dy1;
  Pz = (A.inverse() * Ct) * Dz1;

  // 把系数按段拆开，x/y/z 三组分别存
  for (int i = 0; i < seg_num; i++)
  {
    poly_coeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();
  }

  /* ---------- use polynomials ---------- */
  PolynomialTraj poly_traj;
  // 逐段构造 PolynomialTraj 对象
  for (int i = 0; i < poly_coeff.rows(); ++i)
  {
    vector<double> cx(6), cy(6), cz(6);
    for (int j = 0; j < 6; ++j)
    {
      cx[j] = poly_coeff(i, j), cy[j] = poly_coeff(i, j + 6), cz[j] = poly_coeff(i, j + 12);
    }
    // 【注意】系数要倒序存放 —— 本项目的求值函数按“从高次到低次”读取
    reverse(cx.begin(), cx.end());
    reverse(cy.begin(), cy.end());
    reverse(cz.begin(), cz.end());
    double ts = Time(i);
    // 把这一段加进轨迹
    poly_traj.addSegment(cx, cy, cz, ts);
  }

  return poly_traj;
}

PolynomialTraj PolynomialTraj::one_segment_traj_gen(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
                                                    double t)
{
  // 单段五次多项式：6 个未知系数，对应 6 个边界条件
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6), Crow(1, 6);
  Eigen::VectorXd Bx(6), By(6), Bz(6);

  // 第 1~3 行：t=0 处的位置 / 速度 / 加速度
  C(0, 5) = 1;
  C(1, 4) = 1;
  C(2, 3) = 2;
  // 第 4 行：t=T 处的位置
  Crow << pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1;
  C.row(3) = Crow;
  // 第 5 行：t=T 处的速度（位置求导）
  Crow << 5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0;
  C.row(4) = Crow;
  // 第 6 行：t=T 处的加速度（速度再求导）
  Crow << 20 * pow(t, 3), 12 * pow(t, 2), 6 * t, 2, 0, 0;
  C.row(5) = Crow;

  // 右端向量：起点和终点的位置、速度、加速度
  Bx << start_pt(0), start_vel(0), start_acc(0), end_pt(0), end_vel(0), end_acc(0);
  By << start_pt(1), start_vel(1), start_acc(1), end_pt(1), end_vel(1), end_acc(1);
  Bz << start_pt(2), start_vel(2), start_acc(2), end_pt(2), end_vel(2), end_acc(2);

  // 解这个 6×6 线性方程组，得到 6 个多项式系数
  Eigen::VectorXd Cofx = C.colPivHouseholderQr().solve(Bx);
  Eigen::VectorXd Cofy = C.colPivHouseholderQr().solve(By);
  Eigen::VectorXd Cofz = C.colPivHouseholderQr().solve(Bz);

  vector<double> cx(6), cy(6), cz(6);
  for (int i = 0; i < 6; i++)
  {
    cx[i] = Cofx(i);
    cy[i] = Cofy(i);
    cz[i] = Cofz(i);
  }

  // 组装成一条“只有一段”的轨迹并返回
  PolynomialTraj poly_traj;
  poly_traj.addSegment(cx, cy, cz, t);

  return poly_traj;
}
