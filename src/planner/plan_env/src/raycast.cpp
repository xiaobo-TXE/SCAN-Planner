// ============================================================================
// 文件：src/raycast.cpp —— 射线穿格算法的实现
//
// 【算法思路（大白话）】
//   想象一条斜线穿过方格纸。我们从起点格子出发，每次问：
//   “沿 x 方向走会先碰到竖线，还是沿 y 方向先碰到横线？”
//   谁先碰到就先跨过谁，进入下一个格子。如此重复直到终点。
//   这样每一步只做几次比较，非常快。
//
// 【注意】本文件里有两个同名 Raycast 函数（写法不同），
//   而地图实际使用的是类 RayCaster 的 setInput + step。
// ============================================================================
#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <plan_env/raycast.h>

// 取符号：负数返回 -1，0 返回 0，正数返回 1。
int signum(int x) {
  return x == 0 ? 0 : x < 0 ? -1 : 1;
}

// 对小数取模（保证结果非负，处理负数也没问题）
double mod(double value, double modulus) {
  // 先取模再加上模数再取模 —— 这样负数也能得到正结果
  return fmod(fmod(value, modulus) + modulus, modulus);
}

// 求“从位置 s 出发、每步走 ds，第一次跨过整数边界需要多少步”。
// 这是整个算法离散步进的基石。
double intbound(double s, double ds) {
  // Find the smallest positive t such that s+t*ds is an integer.
  // 如果方向是负的，就翻转成正的来算（正负是对称的）
  if (ds < 0) {
    return intbound(-s, -ds);
  } else {
    // 只保留小数部分，问题就变成“走到整数 1 还差多少”
    s = mod(s, 1);
    // problem is now s+t*ds = 1
    // 还差 (1-s)，除以每步长度 ds 就是所需步数
    return (1 - s) / ds;
  }
}

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output) {
  //    std::cout << start << ' ' << end << std::endl;
  // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
  // by John Amanatides and Andrew Woo, 1987
  // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
  // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
  // Extensions to the described algorithm:
  //   • Imposed a distance limit.
  //   • The face passed through to reach the current cube is provided to
  //     the callback.

  // The foundation of this algorithm is a parameterized representation of
  // the provided ray,
  //                    origin + t * direction,
  // except that t is not actually stored; rather, at any given point in the
  // traversal, we keep track of the *greater* t values which we would have
  // if we took a step sufficient to cross a cube boundary along that axis
  // (i.e. change the integer part of the coordinate) in the variables
  // tMaxX, tMaxY, and tMaxZ.

  // Cube containing origin point.
  // 起点所在的格子编号（向下取整）
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  // 终点所在的格子编号
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  // 射线方向；maxDist 是射线长度的平方（用于提前停止）
  Eigen::Vector3d direction = (end - start);
  double maxDist = direction.squaredNorm();

  // Break out direction vector.
  // 终点相对起点的格子数（决定要往哪个方向走）
  double dx = endX - x;
  double dy = endY - y;
  double dz = endZ - z;

  // Direction to increment x,y,z when stepping.
  // 每前进一步，在 x/y/z 上分别该 +1 还是 -1
  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);
  int stepZ = (int)signum((int)dz);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  // 从起点出发，还需要多少“步”才第一次跨过 x 方向的边界
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  // The change in t when taking a step (always positive).
  // 之后每跨一次边界，步数要固定增加这么多（三个方向各一个）
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  // Avoids an infinite loop.
  // 起点终点重合（没有方向）→ 直接返回，避免死循环
  if (stepX == 0 && stepY == 0 && stepZ == 0) return;

  double dist = 0;
  // 一步一格地往前推进，直到走出终点
  while (true) {
    // 只有当前格子落在给定范围内，才把它记录为“经过”
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
      output[output_points_cnt](0) = x;
      output[output_points_cnt](1) = y;
      output[output_points_cnt](2) = z;

      // 记录了一个格子，计数加一
      output_points_cnt++;
      // 算当前格子离起点的距离
      dist = sqrt((x - start(0)) * (x - start(0)) + (y - start(1)) * (y - start(1)) +
                  (z - start(2)) * (z - start(2)));

      // 超过射线总长度 → 提前结束
      if (dist > maxDist) return;

      /*            if (output_points_cnt > 1500) {
                      std::cerr << "Error, too many racyast voxels." <<
         std::endl;
                      throw std::out_of_range("Too many raycast voxels");
                  }*/
    }

    // 已经到达终点格子 → 结束
    if (x == endX && y == endY && z == endZ) break;

    // tMaxX stores the t-value at which we cross a cube boundary along the
    // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
    // chooses the closest cube boundary. Only the first case of the four
    // has been commented in detail.
    // 比较三个方向“下次跨边界所需步数”，谁最小就先跨过谁
    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        // Update which cube we are now in.
        // 跨过 x 方向一格，并把 x 的“下次跨边界步数”往后推
        x += stepX;
        // Adjust tMaxX to the next X-oriented boundary crossing.
        tMaxX += tDeltaX;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      // 剩下 y、z 两轴做同样的比较
      if (tMaxY < tMaxZ) {
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }
}

// 与上面同名函数逻辑完全相同，只是结果写进 std::vector（更好用，但稍慢）
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output) {
  //    std::cout << start << ' ' << end << std::endl;
  // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
  // by John Amanatides and Andrew Woo, 1987
  // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
  // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
  // Extensions to the described algorithm:
  //   • Imposed a distance limit.
  //   • The face passed through to reach the current cube is provided to
  //     the callback.

  // The foundation of this algorithm is a parameterized representation of
  // the provided ray,
  //                    origin + t * direction,
  // except that t is not actually stored; rather, at any given point in the
  // traversal, we keep track of the *greater* t values which we would have
  // if we took a step sufficient to cross a cube boundary along that axis
  // (i.e. change the integer part of the coordinate) in the variables
  // tMaxX, tMaxY, and tMaxZ.

  // Cube containing origin point.
  // 同样：先求起点、终点的格子编号
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  Eigen::Vector3d direction = (end - start);
  double maxDist = direction.squaredNorm();

  // Break out direction vector.
  double dx = endX - x;
  double dy = endY - y;
  double dz = endZ - z;

  // Direction to increment x,y,z when stepping.
  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);
  int stepZ = (int)signum((int)dz);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  // The change in t when taking a step (always positive).
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  // 清空输出容器
  output->clear();

  // Avoids an infinite loop.
  if (stepX == 0 && stepY == 0 && stepZ == 0) return;

  double dist = 0;
  while (true) {
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
      // 把当前格子加入结果列表
      output->push_back(Eigen::Vector3d(x, y, z));

      dist = (Eigen::Vector3d(x, y, z) - start).squaredNorm();

      if (dist > maxDist) return;

      // 防御性检查：格子数太多说明出异常了，直接抛错
      if (output->size() > 1500) {
        std::cerr << "Error, too many racyast voxels." << std::endl;
        throw std::out_of_range("Too many raycast voxels");
      }
    }

    // 到达终点格子 → 结束
    if (x == endX && y == endY && z == endZ) break;

    // tMaxX stores the t-value at which we cross a cube boundary along the
    // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
    // chooses the closest cube boundary. Only the first case of the four
    // has been commented in detail.
    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        // Update which cube we are now in.
        x += stepX;
        // Adjust tMaxX to the next X-oriented boundary crossing.
        tMaxX += tDeltaX;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }
}

// 初始化：记录起点终点，算出当前格子编号、每步的方向、
// 以及跨越各方向边界所需的步长（tMax / tDelta）。
bool RayCaster::setInput(const Eigen::Vector3d& start,
                         const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                         const Eigen::Vector3d& max */) {
  // 记住射线的起点和终点
  start_ = start;
  end_ = end;
  // max_ = max;
  // min_ = min;

  // 起点所在的格子编号
  x_ = (int)std::floor(start_.x());
  y_ = (int)std::floor(start_.y());
  z_ = (int)std::floor(start_.z());
  // 终点所在的格子编号
  endX_ = (int)std::floor(end_.x());
  endY_ = (int)std::floor(end_.y());
  endZ_ = (int)std::floor(end_.z());
  // 方向向量与总长度平方（用于判断何时该停）
  direction_ = (end_ - start_);
  maxDist_ = direction_.squaredNorm();

  // Break out direction vector.
  // 终点相对起点的格子数
  dx_ = endX_ - x_;
  dy_ = endY_ - y_;
  dz_ = endZ_ - z_;

  // Direction to increment x,y,z when stepping.
  // 每步在三个方向上的增量（±1）
  stepX_ = (int)signum((int)dx_);
  stepY_ = (int)signum((int)dy_);
  stepZ_ = (int)signum((int)dz_);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  // 跨过各方向第一条边界所需的步数
  tMaxX_ = intbound(start_.x(), dx_);
  tMaxY_ = intbound(start_.y(), dy_);
  tMaxZ_ = intbound(start_.z(), dz_);

  // The change in t when taking a step (always positive).
  // 之后每次跨越边界的固定步长增量
  tDeltaX_ = ((double)stepX_) / dx_;
  tDeltaY_ = ((double)stepY_) / dy_;
  tDeltaZ_ = ((double)stepZ_) / dz_;

  // 复位内部的距离与步数计数
  dist_ = 0;

  step_num_ = 0;

  // Avoids an infinite loop.
  // 起点终点完全重合 → 返回 false，表示这条射线无效
  if (stepX_ == 0 && stepY_ == 0 && stepZ_ == 0)
    return false;
  else
    return true;
}

// 走一步：先输出当前格子，若已到终点就结束；
// 否则比较三个方向的 tMax，谁最小就往那个方向跨一格。
bool RayCaster::step(Eigen::Vector3d& ray_pt) {
  // if (x_ >= min_.x() && x_ < max_.x() && y_ >= min_.y() && y_ < max_.y() &&
  // z_ >= min_.z() && z_ <
  // max_.z())
  // 先输出当前所在的格子
  ray_pt = Eigen::Vector3d(x_, y_, z_);

  // step_num_++;

  // dist_ = (Eigen::Vector3d(x_, y_, z_) - start_).squaredNorm();

  // 已经走到终点格子 → 返回 false 表示“走完了”
  if (x_ == endX_ && y_ == endY_ && z_ == endZ_) {
    return false;
  }

  // if (dist_ > maxDist_)
  // {
  //   return false;
  // }

  // tMaxX stores the t-value at which we cross a cube boundary along the
  // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
  // chooses the closest cube boundary. Only the first case of the four
  // has been commented in detail.
  // 与上面的算法完全一致：谁先跨过边界就先往谁那边走
  if (tMaxX_ < tMaxY_) {
    if (tMaxX_ < tMaxZ_) {
      // Update which cube we are now in.
      // 跨过 x 方向一格，并累加对应的步长
      x_ += stepX_;
      // Adjust tMaxX to the next X-oriented boundary crossing.
      tMaxX_ += tDeltaX_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  } else {
    if (tMaxY_ < tMaxZ_) {
      y_ += stepY_;
      tMaxY_ += tDeltaY_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  }

  // 返回 true 表示还有下一格，调用者可以继续 step
  return true;
}