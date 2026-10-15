// ============================================================================
// 文件：include/plan_env/raycast.h
//
// 【这个文件是干什么的】
//   “射线穿过格子”算法（专业名叫 Amanatides-Woo 体素遍历）。
//   给一条线段（从传感器到某个观测点），它能依次列出这条线穿过的所有格子。
//
// 【为什么需要它】
//   一条激光打出去，沿途经过的格子说明“这条路是空的”，
//   终点格子说明“这里有东西”。所以要先能求出“沿途经过了哪些格子”。
//
// 【两种用法】
//   函数 Raycast(...)  ：一次性算出整条线经过的所有格子
//   类 RayCaster       ：可以“一格一格往前走”（更灵活，地图里实际用的是它）
// ============================================================================
#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

// 三个数学小工具：
//   signum  取符号（-1/0/1）
//   mod     取模（结果保证非负）
//   intbound 求“从当前坐标走到下一个整数边界还需要多少步长”（算法的关键量）
double signum(double x);

double mod(double value, double modulus);

double intbound(double s, double ds);

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

// “可一步步前进”的射线遍历器：先 setInput 给定起点终点，
// 然后反复调用 step() 一格一格往前走。
class RayCaster {
private:
  /* data */
  Eigen::Vector3d start_;
  Eigen::Vector3d end_;
  Eigen::Vector3d direction_;
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  int x_;
  int y_;
  int z_;
  int endX_;
  int endY_;
  int endZ_;
  double maxDist_;
  double dx_;
  double dy_;
  double dz_;
  int stepX_;
  int stepY_;
  int stepZ_;
  double tMaxX_;
  double tMaxY_;
  double tMaxZ_;
  double tDeltaX_;
  double tDeltaY_;
  double tDeltaZ_;
  double dist_;

  int step_num_;

public:
  RayCaster(/* args */) {
  }
  ~RayCaster() {
  }

// 设置射线的起点和终点，并预先算好步进所需的各种量。
// 起点终点重合（没有方向）时返回 false。
  bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

// 往前走一格，把当前格子坐标写入 ray_pt；走到终点后返回 false。
  bool step(Eigen::Vector3d& ray_pt);
};

#endif  // RAYCAST_H_