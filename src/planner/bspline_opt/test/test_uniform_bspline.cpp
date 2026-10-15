// ============================================================================
// 文件：test/test_uniform_bspline.cpp —— B 样条的单元测试
//
// 【这个测试检查什么】
//   用最简单的例子验证 B 样条的数学实现是否正确：
//   1. 控制点排在一条直线上时，求值结果应该落在预期的点上；
//   2. 求导得到的速度应该是那条直线的斜率。
//
// 【怎么跑】  colcon test --packages-select bspline_opt
// ============================================================================
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <bspline_opt/uniform_bspline.h>

// 测试 1：控制点在 x 方向等距排列时，检查 evaluateDeBoorT 求出的位置对不对。
TEST(UniformBspline, EvaluatesLinearControlPoints)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  scan_planner::UniformBspline spline(points, 3, 1.0);
  EXPECT_NEAR(spline.evaluateDeBoorT(0.0).x(), 1.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).x(), 3.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).y(), 0.0, 1e-9);
}

// 测试 2：检查 getDerivative() 求出的速度是否等于那条直线的斜率（x 方向应为 2.0）。
TEST(UniformBspline, DerivativeMatchesLinearSlope)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  auto derivative = scan_planner::UniformBspline(points, 3, 0.5).getDerivative();
  const Eigen::Vector3d velocity = derivative.evaluateDeBoorT(0.75);
  EXPECT_NEAR(velocity.x(), 2.0, 1e-9);
  EXPECT_NEAR(velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(velocity.z(), 0.0, 1e-9);
}
