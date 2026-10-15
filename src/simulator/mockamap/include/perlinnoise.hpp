// ============================================================================
// 文件：include/perlinnoise.hpp —— 柏林噪声的接口
//
// 【第三方代码】由 Ken Perlin 的官方 Java 参考实现翻译成 C++11。
//   本项目用它生成“地图类型 1”的自然地形（实现在 src/perlinnoise.cpp）。
// ============================================================================
#ifndef PERLINNOISE_HPP
#define PERLINNOISE_HPP

#include <vector>

// THIS CLASS IS A TRANSLATION TO C++11 FROM THE REFERENCE
// JAVA IMPLEMENTATION OF THE IMPROVED PERLIN FUNCTION (see
// http://mrl.nyu.edu/~perlin/noise/)
// THE ORIGINAL JAVA IMPLEMENTATION IS COPYRIGHT 2002 KEN PERLIN

// I ADDED AN EXTRA METHOD THAT GENERATES A NEW PERMUTATION VECTOR (THIS IS NOT
// PRESENT IN THE ORIGINAL IMPLEMENTATION)

// 柏林噪声类：给一个三维坐标，返回一个平滑的随机值（0~1）
class PerlinNoise
{
  // The permutation vector
  // 置换表：算法内部用来“打乱”坐标，是噪声随机性的来源
  std::vector<int> p;

public:
  // Initialize with the reference values for the permutation vector
  // 默认构造：用官方参考实现给的那张固定置换表
  PerlinNoise();
  // Generate a new permutation vector based on the value of seed
  // 带种子的构造：自己打乱生成置换表（保证地图可复现）
  PerlinNoise(unsigned int seed);
  // Get a noise value, for 2D images z can have any value
  // 【主接口】求某点的噪声值
  double noise(double x, double y, double z);

private:
  // 内部工具：平滑曲线、线性插值、梯度函数
  double fade(double t);
  double lerp(double t, double a, double b);
  double grad(int hash, double x, double y, double z);
};

#endif // PERLINNOISE_HPP
