// ============================================================================
// 【注意：本项目当前没有使用这个库】
//
// 【它原本是干什么的】
//   一个基于 Armadillo 数学库的位姿工具集：旋转/四元数换算、位姿复合与求逆，
//   以及一套状态估计（EKF）需要的雅可比矩阵。
//   它来自更早的仿真器（当年用于无人机状态估计），迁移到本项目后已无调用者。
//
// 【阅读建议】可以完全跳过；知道“它是历史遗留、当前不生效”即可。
// ============================================================================
#ifndef POSE_UTILS_H
#define POSE_UTILS_H

#include <iostream>
#include "armadillo"

#define PI 3.14159265359
#define NUM_INF 999999.9

using namespace arma;
using namespace std;

// 旋转与四元数换算：欧拉角↔旋转矩阵、四元数↔旋转矩阵、四元数乘 / 求逆
// Rotation ---------------------
mat ypr_to_R(const colvec& ypr);

mat yaw_to_R(double yaw);

colvec R_to_ypr(const mat& R);

mat quaternion_to_R(const colvec& q);

colvec R_to_quaternion(const mat& R);

colvec quaternion_mul(const colvec& q1, const colvec& q2);

colvec quaternion_inv(const colvec& q);

// 位姿的复合与求逆（三维版 + 二维版）
// General Pose Update ----------
colvec pose_update(const colvec& X1, const colvec& X2);

colvec pose_inverse(const colvec& X);

colvec pose_update_2d(const colvec& X1, const colvec& X2);

colvec pose_inverse_2d(const colvec& X);

// 位姿 EKF 需要的雅可比矩阵
// For Pose EKF -----------------
mat Jplus1(const colvec& X1, const colvec& X2);

mat Jplus2(const colvec& X1, const colvec& X2);

// 下面一组是 IMU 的 EKF：状态预测、对状态/输入的雅可比、观测模型
// For IMU EKF ------------------
colvec state_update(const colvec& X, const colvec& U, double dt);

mat jacobianF(const colvec& X, const colvec& U, double dt);

mat jacobianU(const colvec& X, const colvec& U, double dt);

colvec state_measure(const colvec& X);

mat jacobianH();

#endif
