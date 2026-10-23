// ============================================================================
// 文件：src/maps.cpp —— 各种“程序化地图”的生成算法
//
// 【这个文件是干什么的】
//   提供四种假地图的生成方式，供 mockamap_node 按 type 参数调用：
//     1  perlin3D()        柏林噪声（起伏的云状障碍）
//     2  randomMapGenerate() 随机撒立方体障碍
//     3  maze2D()          2D 迷宫（竖直拉伸成 3D）
//     4  Maze3DGen()       3D 迷宫（核心点 + 中面墙）
//
// 【生成结果怎么走】
//   每种算法最后都调用 pcl2ros()，把点云转成 ROS 消息交给节点发布。
// ============================================================================
#include "maps.hpp"

#include <algorithm>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "perlinnoise.hpp"

using namespace mocka;

namespace
{
// 小工具：读一个参数（没声明过就先声明，再取值）
template <typename T>
void loadParameter(rclcpp::Node *node, const std::string &name, T &value, const T &default_value)
{
  if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
  node->get_parameter(name, value);
}
}  // namespace

// 【地图类型 2】随机方块：在场地里随机撒一堆立方体障碍（只用表面点表示）
void
Maps::randomMapGenerate()
{

  // 用固定种子初始化随机数，保证每次生成的地图一模一样（可复现）
  std::default_random_engine eng(info.seed);

  // 分辨率（米），以及场地的边界：x/y 以原点为中心，z 从 0 到指定高度
  double _resolution = 1 / info.scale;

  double _x_l = -info.sizeX / (2 * info.scale);
  double _x_h = info.sizeX / (2 * info.scale);
  double _y_l = -info.sizeY / (2 * info.scale);
  double _y_h = info.sizeY / (2 * info.scale);
  double _h_l = 0;
  double _h_h = info.sizeZ / info.scale;

  // 从参数读取：方块宽度范围、高度范围、方块数量、表面采样密度
  double _w_l, _w_h;
  int    _ObsNum;

  loadParameter(info.node, "width_min", _w_l, 0.6);
  loadParameter(info.node, "width_max", _w_h, 1.5);
  loadParameter(info.node, "height_min", _h_l, _h_l);
  loadParameter(info.node, "height_max", _h_h, _h_h);
  loadParameter(info.node, "obstacle_number", _ObsNum, 10);
  double surface_resolution;
  loadParameter(info.node, "surface_resolution", surface_resolution, _resolution * 0.5);

  // 保护性修正：高度非负且不超过场地、采样密度不能为 0
  _h_l = std::max(0.0, _h_l);
  _h_h = std::min(info.sizeZ / info.scale, std::max(_h_l, _h_h));
  surface_resolution = std::max(1e-3, surface_resolution);

  // 准备四个均匀分布随机数发生器：x、y、宽度、高度
  std::uniform_real_distribution<double> rand_x;
  std::uniform_real_distribution<double> rand_y;
  std::uniform_real_distribution<double> rand_w;
  std::uniform_real_distribution<double> rand_h;

  pcl::PointXYZ pt_random;

  // 小工具：往点云里加一个点
  auto pushPoint = [&](double px, double py, double pz) {
    pt_random.x = px;
    pt_random.y = py;
    pt_random.z = pz;
    info.cloud->points.push_back(pt_random);
  };

  // 分别设定它们的取值范围
  rand_x = std::uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = std::uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = std::uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = std::uniform_real_distribution<double>(_h_l, _h_h);

  // 逐个生成 _ObsNum 个立方体障碍
  for (int i = 0; i < _ObsNum; i++)
  {
    double x, y;
    // 随机出这个立方体的中心位置
    x = rand_x(eng);
    y = rand_y(eng);

    double w, h;
    w = rand_w(eng);
    h = rand_h(eng);

    // 算出立方体在 XY 平面上的四个边界
    const double half_w = w * 0.5;
    const double x_min = x - half_w;
    const double x_max = x + half_w;
    const double y_min = y - half_w;
    const double y_max = y + half_w;
    // 每个面细分成多少格（决定表面点的稠密程度）
    const int xy_steps = std::max(1, static_cast<int>(std::ceil(w / surface_resolution)));
    const int z_steps = std::max(1, static_cast<int>(std::ceil(h / surface_resolution)));

    // 逐个高度层生成“四个竖直侧面”上的点
    for (int iz = 0; iz <= z_steps; ++iz)
    {
      const double z = h * iz / z_steps;
      // 前后两个面（y 取边界值）
      for (int ix = 0; ix <= xy_steps; ++ix)
      {
        const double px = x_min + w * ix / xy_steps;
        pushPoint(px, y_min, z);
        pushPoint(px, y_max, z);
      }
      // 左右两个面（x 取边界值）
      for (int iy = 0; iy <= xy_steps; ++iy)
      {
        const double py = y_min + w * iy / xy_steps;
        pushPoint(x_min, py, z);
        pushPoint(x_max, py, z);
      }
    }

    // 再生成“上下两个水平面”上的点
    for (int ix = 0; ix <= xy_steps; ++ix)
      for (int iy = 0; iy <= xy_steps; ++iy)
      {
        const double px = x_min + w * ix / xy_steps;
        const double py = y_min + w * iy / xy_steps;
        pushPoint(px, py, 0.0);
        pushPoint(px, py, h);
      }
  }

  // 更新点云尺寸并转成 ROS 消息
  info.cloud->width    = info.cloud->points.size();
  info.cloud->height   = 1;
  info.cloud->is_dense = true;

  pcl2ros();
}

// 把 PCL 点云转成 ROS 消息，顺便打印“填充率”
void
Maps::pcl2ros()
{
  pcl::toROSMsg(*info.cloud, *info.output);
  info.output->header.frame_id = "world";
  // 打印点云密度（用来判断地图生成得是否合理）
  RCLCPP_INFO(info.node->get_logger(), "finish: infill %lf%%",
              info.cloud->width / (1.0 * info.sizeX * info.sizeY * info.sizeZ));
}

// 【地图类型 1】柏林噪声：用三维噪声生成连绵起伏的障碍（像云/山）
void
Maps::perlin3D()
{
  double complexity;
  double fill;
  int    fractal;
  double attenuation;

  // 读参数：噪声复杂度、填充比例、分形层数、层间衰减
  loadParameter(info.node, "complexity", complexity, 0.142857);
  loadParameter(info.node, "fill", fill, 0.38);
  loadParameter(info.node, "fractal", fractal, 1);
  loadParameter(info.node, "attenuation", attenuation, 0.5);

  // 先按“满格”分配点云空间
  info.cloud->width  = info.sizeX * info.sizeY * info.sizeZ;
  info.cloud->height = 1;
  info.cloud->points.resize(info.cloud->width * info.cloud->height);

  // 创建柏林噪声发生器（用同一个种子保证可复现）
  PerlinNoise noise(info.seed);

  // 第一遍：把每个格子的噪声值收集到一个列表里
  std::vector<double>* v = new std::vector<double>;
  v->reserve(info.cloud->width);
  for (int i = 0; i < info.sizeX; ++i)
  {
    for (int j = 0; j < info.sizeY; ++j)
    {
      for (int k = 0; k < info.sizeZ; ++k)
      {
        double tnoise = 0;
        // 分形叠加：频率逐层加倍、幅度逐层衰减 —— 让地形更自然
        for (int it = 1; it <= fractal; ++it)
        {
          int    dfv = pow(2, it);
          double ta  = attenuation / it;
          tnoise += ta * noise.noise(dfv * i * complexity,
                                     dfv * j * complexity,
                                     dfv * k * complexity);
        }
        v->push_back(tnoise);
      }
    }
  }
  // 把噪声值排序，按 fill 比例取一个分位数当“障碍阈值”
  std::sort(v->begin(), v->end());
  // 阈值位置：只有噪声值超过它的格子才算障碍
  int    tpos = info.cloud->width * (1 - fill);
  double tmp  = v->at(tpos);
  RCLCPP_INFO(info.node->get_logger(), "threshold: %lf", tmp);

  // 第二遍：再算一次噪声，把超阈值的格子转成世界坐标存进点云
  int pos = 0;
  for (int i = 0; i < info.sizeX; ++i)
  {
    for (int j = 0; j < info.sizeY; ++j)
    {
      for (int k = 0; k < info.sizeZ; ++k)
      {
        double tnoise = 0;
        for (int it = 1; it <= fractal; ++it)
        {
          int    dfv = pow(2, it);
          double ta  = attenuation / it;
          tnoise += ta * noise.noise(dfv * i * complexity,
                                     dfv * j * complexity,
                                     dfv * k * complexity);
        }
        // 超过阈值 → 这一格是障碍
        if (tnoise > tmp)
        {
          info.cloud->points[pos].x =
            i / info.scale - info.sizeX / (2 * info.scale);
          info.cloud->points[pos].y =
            j / info.scale - info.sizeY / (2 * info.scale);
          info.cloud->points[pos].z = k / info.scale;
          pos++;
        }
      }
    }
  }
  // 只保留真正是障碍的那些点
  info.cloud->width = pos;
  RCLCPP_INFO(info.node->get_logger(), "points before optimization: %u", info.cloud->width);
  info.cloud->points.resize(info.cloud->width * info.cloud->height);
  pcl2ros();
}

void
// 【2D 迷宫核心】递归分割法：用墙把区域一分为二，再递归处理四个子区域
Maps::recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi& maze)
{
  RCLCPP_INFO(info.node->get_logger(),
    "generating maze with width %d , height %d", xh - xl + 1, yh - yl + 1);

  // 情况一：区域够大（至少 5×5）→ 同时加横墙和竖墙（十字）
  if (xl < xh - 3 && yl < yh - 3)
  { // the remaining area is larger than or equal to 5*5, need to add both x
    // wall and y wall
    // 随机选墙的位置，但必须保证不会堵住已有的通道，否则重新选
    bool valid = false; // used to judge whether the wall selection is valid
    int  xm    = 0;
    int  ym    = 0;
    RCLCPP_INFO(info.node->get_logger(), "entered 5*5 mode");
    while (valid == false)
    {
      // 在区域内部随机选一个位置作为“墙心”
      xm = (std::rand() % (xh - xl - 1) + xl +
            1); // generating random number between xl+1 and xh-1(pointless to
                // add a wall at the sides)
      ym = (std::rand() % (yh - yl - 1) + yl +
            1); // generating random number between yl+1 and yh-1(pointless to
                // add a wall at the sides)
      // 下面四段：检查墙心是否正好盖住了已有的门；是就重新随机
      if (xl - 1 >= 0)
      { // there is a point at xl-1,ym
        if (maze(xl - 1, ym) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (xh + 1 <= maze.cols() - 1)
      { // there is a point at xh+1,ym
        if (maze(xh + 1, ym) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (yl - 1 >= 0)
      { // there is a point at xm,yl-1
        if (maze(xm, yl - 1) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (yh + 1 <= maze.rows() - 1)
      { // there is a point at xm,yh+1
        if (maze(xm, yh + 1) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      // 选到合法位置，结束重试
      valid = true;

    } // xm and ym are now the valid coordinate of the center of the wall
    // 在墙心处画出十字形的墙
    for (int i = xl; i <= xh; i++)
    {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; j++)
    {
      maze(xm, j) = 1;
    } // adding walls around the center point
    // 随机选出十字四个臂上的四个候选门位
    int d1 = std::rand() % (xm - xl) + xl;
    int d2 = std::rand() % (xh - xm) + xm + 1;
    int d3 = std::rand() % (ym - yl) + yl;
    int d4 =
      std::rand() % (yh - ym) + ym + 1; // generating four possible door points

    // 随机挑 3 个作为门（留一个堵住，保证迷宫有岔路但不封闭）
    int decision = std::rand() % 4; // random selection of three doors
    switch (decision)
    {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;

      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;

      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;

      case 3:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    } // the doors are opened for this cell
    // 下面四段：如果新墙把原来的门堵住了，就在对应位置补一个门
    if (yl - 1 >= 0)
    {
      if (maze(xm, yl - 1) == 0)
      {
        maze(xm, yl) = 0;
      }
    }

    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xm, yh + 1) == 0)
      {
        maze(xm, yh) = 0;
      }
    }

    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, ym) == 0)
      {
        maze(xl, ym) = 0;
      }
    }

    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, ym) == 0)
      {
        maze(xh, ym) = 0;
      }
    }

    std::cout << maze << std::endl;
    // 递归处理四个子区域
    recursiveDivision(xl, xm - 1, yl, ym - 1, maze);
    recursiveDivision(xm + 1, xh, yl, ym - 1, maze);
    recursiveDivision(xl, xm - 1, ym + 1, yh, maze);
    recursiveDivision(xm + 1, xh, ym + 1, yh, maze);

    RCLCPP_INFO(info.node->get_logger(), "finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    std::cout << maze << std::endl;
    return;
  } // when the remaining area is larger than or equal to 5*5

  // 情况二：中等区域（至少 4×4）→ 仍加十字墙，但门位随机选三个
  else if (xl < xh - 2 && yl < yh - 2)
  {
    // bool valid     = false; // used to judge whether the wall selection is valid
    int  xm        = 0;
    int  ym        = 0;
    int  doorcount = 0;
    xm             = (std::rand() % (xh - xl - 1) + xl +
          1); // generating random number between xl+1 and xh-1(pointless to
                          // add a wall at the sides)
    ym =
      (std::rand() % (yh - yl - 1) + yl +
       1); // generating random number between yl+1 and yh-1(pointless to
           // add a wall at the sides)
           // xm and ym are now the valid coordinate of the center of the wall
    // 画出十字墙
    for (int i = xl; i <= xh; i++)
    {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; j++)
    {
      maze(xm, j) = 1;
    } // adding walls around the center point
    if (yl - 1 >= 0)
    {
      if (maze(xm, yl - 1) == 0)
      {
        maze(xm, yl) = 0;
        doorcount++;
      }
    }

    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xm, yh + 1) == 0)
      {
        maze(xm, yh) = 0;
        doorcount++;
      }
    }

    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, ym) == 0)
      {
        maze(xl, ym) = 0;
        doorcount++;
      }
    }

    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, ym) == 0)
      {
        maze(xh, ym) = 0;
        doorcount++;
      }
    }

    // 同样随机开三个门
    int d1 = std::rand() % (xm - xl) + xl;
    int d2 = std::rand() % (xh - xm) + xm + 1;
    int d3 = std::rand() % (ym - yl) + yl;
    int d4 =
      std::rand() % (yh - ym) + ym + 1; // generating four possible door points

    int decision = std::rand() % 4; // random selection of three doors
    switch (decision)
    {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;

      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;

      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;

      case 3:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    } // the doors are opened for this cell
    std::cout << maze << std::endl;

    RCLCPP_INFO(info.node->get_logger(), "finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    std::cout << maze << std::endl;
    return;
  }

  // 情况三：3×4 这类窄区域 → 只加一道竖墙
  else if (xl < xh - 1 && yl < yh - 2)
  { // the case of 3*4+
    RCLCPP_INFO(info.node->get_logger(), "entered 3*4+ mode");
    int doorcount = 0;
    int ym        = 0;
    // 在中间填一列墙
    for (int i = yl; i <= yh; i++)
    {
      maze(xl + 1, i) = 1;
    } // filling a center wall
    if (yl - 1 >= 0)
    {
      if (maze(xl + 1, yl - 1) == 0)
      {
        maze(xl + 1, yl) = 0;
        doorcount++;
      }
    }
    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xl + 1, yh + 1) == 0)
      {
        maze(xl + 1, yh) = 0;
        doorcount++;
      }
    } // opening doors if the wall blocks the old doors
    // 如果这道墙把所有门都堵死了 → 再随机开一个
    if (doorcount == 0)
    {
      ym               = std::rand() % (yh - yl + 1) + yl;
      maze(xl + 1, ym) = 0;
    }
  } // the case of 4+*3
  //
  // 情况四：4×3 → 只加一道横墙（与上面互为镜像）
  else if (xl < xh - 2 && yl < yh - 1)
  { // the case of 4+*3
    RCLCPP_INFO(info.node->get_logger(), "entered 4+*3 mode");
    int doorcount = 0;
    int xm        = 0;
    for (int i = xl; i <= xh; i++)
    {
      maze(i, yl + 1) = 1;
    } // filling a center wall
    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, yl + 1) == 0)
      {
        maze(xl, yl + 1) = 0;
        doorcount++;
      }
    }
    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, yl + 1) == 0)
      {
        maze(xh, yl + 1) = 0;
        doorcount++;
      }
    } // opening doors if the wall blocks the old doors
    if (doorcount == 0)
    {
      xm               = std::rand() % (xh - xl + 1) + xl;
      maze(xm, yl + 1) = 0;
    }
  } // the case of 4+*3

  // 情况五：3×3 → 只在中心放一根柱子
  else if (xl < xh - 1 && yl < yh - 1)
  { // the case of 3*3
    maze(xl + 1, yl + 1) = 1;
    return;
  }
  // 区域太小 → 直接返回，不再分割
  else
  {
    RCLCPP_INFO(info.node->get_logger(), "finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    return;
  }
}

// 另一套“递归分割”实现（当前没有被 maze2D 使用；源码里也注明它有 bug）
void
Maps::recursizeDivisionMaze(Eigen::MatrixXi& maze)
{
  //! @todo all bugs here...
  // 取迷宫的行列数
  int sx = maze.rows();
  int sy = maze.cols();

  int px, py;

  // 区域太小就不再分割
  if (sx > 5)
    px = (std::rand() % (sx - 3) + 1);
  else
    return;

  if (sy > 5)
    py = (std::rand() % (sy - 3) + 1);
  else
    return;

  RCLCPP_DEBUG(info.node->get_logger(), "%d %d %d %d", sx, sy, px, py);

  // 随机选出两个横向门位和两个纵向门位
  int x1, x2, y1, y2;

  if (px != 1)
    x1 = (std::rand() % (px - 1) + 1);
  else
    x1 = 1;

  if ((sx - px - 3) > 0)
    x2 = (std::rand() % (sx - px - 3) + px + 1);
  else
    x2 = px + 1;

  if (py != 1)
    y1 = (std::rand() % (py - 1) + 1);
  else
    y1 = 1;

  if ((sy - py - 3) > 0)
    y2 = (std::rand() % (sy - py - 3) + py + 1);
  else
    y2 = py + 1;
  RCLCPP_DEBUG(info.node->get_logger(), "%d %d %d %d", x1, x2, y1, y2);

  // 沿选定的行/列画墙，并在门位留空
  if (px != 1 && px != (sx - 2))
  {
    for (int i = 1; i < (sy - 1); ++i)
    {
      if (i != y1 && i != y2)
        maze(px, i) = 1;
    }
  }
  if (py != 1 && py != (sy - 2))
  {
    for (int i = 1; i < (sx - 1); ++i)
    {
      if (i != x1 && i != x2)
        maze(i, py) = 1;
    }
  }
  // 随机关掉一个门（让迷宫有唯一解）
  switch (std::rand() % 4)
  {
    case 0:
      maze(x1, py) = 1;
      break;
    case 1:
      maze(x2, py) = 1;
      break;
    case 2:
      maze(px, y1) = 1;
      break;
    case 3:
      maze(px, y2) = 1;
      break;
  }

  // 递归处理四个子区域
  if (px > 2 && py > 2)
  {
    Eigen::MatrixXi sub = maze.block(0, 0, px + 1, py + 1);
    recursizeDivisionMaze(sub);
    maze.block(0, 0, px, py) = sub;
  }
  if (px > 2 && (sy - py - 1) > 2)
  {
    Eigen::MatrixXi sub = maze.block(0, py, px + 1, sy - py);
    recursizeDivisionMaze(sub);
    maze.block(0, py, px + 1, sy - py) = sub;
  }
  if (py > 2 && (sx - px - 1) > 2)
  {
    Eigen::MatrixXi sub = maze.block(px, 0, sx - px, py + 1);
    recursizeDivisionMaze(sub);
    maze.block(px, 0, sx - px, py + 1) = sub;
  }
  if ((sx - px - 1) > 2 && (sy - py - 1) > 2)
  {

    Eigen::MatrixXi sub = maze.block(px, py, sy - px, sy - py);

    recursizeDivisionMaze(sub);
    maze.block(px, py, sy - px, sy - py) = sub;
  }
}

// 【地图类型 3】2D 迷宫：先在二维网格上生成迷宫，再竖直拉伸成 3D 点云
void
Maps::maze2D()
{
  double width;
  int    type;
  int    addWallX;
  int    addWallY;
  // 读参数：通道宽度、是否加外墙、迷宫类型
  loadParameter(info.node, "road_width", width, 1.0);
  loadParameter(info.node, "add_wall_x", addWallX, 0);
  loadParameter(info.node, "add_wall_y", addWallY, 0);
  loadParameter(info.node, "maze_type", type, 1);

  // 迷宫网格的尺寸（按通道宽度换算）
  int mx = info.sizeX / (width * info.scale);
  int my = info.sizeY / (width * info.scale);

  // 迷宫矩阵：0 = 通道，1 = 墙
  Eigen::MatrixXi maze(mx, my);
  maze.setZero();

  // 目前只实现了一种生成算法（递归分割）
  switch (type)
  {
    case 1:
      recursiveDivision(0, maze.cols() - 1, 0, maze.rows() - 1, maze);
      break;
  }

  // 可选：沿 x 方向给场地四周加外墙
  if (addWallX)
  {
    for (int i = 0; i < mx; ++i)
    {
      maze(i, 0)      = 1;
      maze(i, my - 1) = 1;
    }
  }
  if (addWallY)
  {
    for (int i = 0; i < my; ++i)
    {
      maze(0, i)      = 1;
      maze(mx - 1, i) = 1;
    }
  }

  std::cout << maze << std::endl;

  // 把二维迷宫“竖直拉伸”：凡是墙的格子，沿 z 方向填满点
  for (int i = 0; i < mx; ++i)
  {
    for (int j = 0; j < my; ++j)
    {
      // 只有墙的格子才需要生成点
      if (maze(i, j))
      {
        for (int ii = 0; ii < width * info.scale; ++ii)
        {
          for (int jj = 0; jj < width * info.scale; ++jj)
          {
            for (int k = 0; k < info.sizeZ; ++k)
            {
              pcl::PointXYZ pt_random;
              pt_random.x =
                i * width + ii / info.scale - info.sizeX / (2.0 * info.scale);
              pt_random.y =
                j * width + jj / info.scale - info.sizeY / (2.0 * info.scale);
              pt_random.z = k / info.scale;
              info.cloud->points.push_back(pt_random);
            }
          }
        }
      }
    }
  }
  info.cloud->width    = info.cloud->points.size();
  info.cloud->height   = 1;
  info.cloud->is_dense = true;
  pcl2ros();
}

// 参数的读取 / 设置接口
Maps::BasicInfo
Maps::getInfo() const
{
  return info;
}

void
Maps::setInfo(const BasicInfo& value)
{
  info = value;
}

Maps::Maps()
{
}

// 【总入口】按 type 分派到不同的生成算法
void
Maps::generate(int type)
{
  switch (type)
  {
    default:
    case 1:
      perlin3D();
      break;
    case 2:
      randomMapGenerate();
      break;
    case 3:
      // 迷宫类算法用 srand 设随机种子
      std::srand(info.seed);
      maze2D();
      break;
    case 4: // generating 3d maze
      std::srand(info.seed);
      Maze3DGen();
      break;
  }
}

// 下面一批都是 MazePoint 的简单读写函数（3D 迷宫算法内部要用）
pcl::PointXYZ
MazePoint::getPoint()
{
  return point;
}

int
MazePoint::getPoint1()
{
  return point1;
}

int
MazePoint::getPoint2()
{
  return point2;
}

double
MazePoint::getDist1()
{
  return dist1;
}

double
MazePoint::getDist2()
{
  return dist2;
}

void
MazePoint::setPoint(pcl::PointXYZ p)
{
  point = p;
}

void
MazePoint::setPoint1(int p)
{
  point1 = p;
}

void
MazePoint::setPoint2(int p)
{
  point2 = p;
}

void
MazePoint::setDist1(double set)
{
  dist1 = set;
}

void
MazePoint::setDist2(double set)
{
  dist2 = set;
}

void
// 【地图类型 4】3D 迷宫：随机撒“核心点”，在它们之间的中面上生成墙
Maps::Maze3DGen()
{
  // getting required info parameters from the given node
  int    numNodes;
  double connectivity;
  int    nodeRad;
  int    roadRad;

  // 读参数：核心点数量、连通度、核心半径、通道半径
  loadParameter(info.node, "numNodes", numNodes, 10);
  loadParameter(info.node, "connectivity", connectivity, 0.5);
  loadParameter(info.node, "nodeRad", nodeRad, 3);
  loadParameter(info.node, "roadRad", roadRad, 2);
  RCLCPP_INFO(info.node->get_logger(), "received parameters : numNodes: %d connectivity: "
           "%f nodeRad: %d roadRad: %d",
           numNodes,
           connectivity,
           nodeRad,
           roadRad);
  // generating random points
  std::vector<pcl::PointXYZ> base;

  // 随机生成 numNodes 个“核心点”作为迷宫的骨架
  for (int i = 0; i < numNodes; i++)
  {
    double rx = std::rand() / RAND_MAX +
                (std::rand() % info.sizeX) / info.scale -
                info.sizeX / (2 * info.scale);
    double ry = std::rand() / RAND_MAX +
                (std::rand() % info.sizeY) / info.scale -
                info.sizeY / (2 * info.scale);
    double rz = std::rand() / RAND_MAX +
                (std::rand() % info.sizeZ) / info.scale -
                info.sizeZ / (2 * info.scale);
    RCLCPP_DEBUG(info.node->get_logger(), "point: x: %f , y: %f , z: %f", rx, ry, rz);

    pcl::PointXYZ pt_random;
    pt_random.x = rx;
    pt_random.y = ry;
    pt_random.z = rz;
    base.push_back(pt_random);
  } // generating random cores in the space

  // 遍历空间里的每一个格子
  for (int i = 0; i < info.sizeX; i++)
  {
    for (int j = 0; j < info.sizeY; j++)
    {
      for (int k = 0; k < info.sizeZ; k++)
      { // for every scaled coordinate points
        pcl::PointXYZ test;
        test.x = i / info.scale - info.sizeX / (2 * info.scale);
        test.y = j / info.scale - info.sizeY / (2 * info.scale);
        test.z = k / info.scale -
                 info.sizeZ /
                   (2 * info.scale); // marking the corresponding point location

        MazePoint mp;
        mp.setPoint(test);
        mp.setPoint2(-1);
        mp.setPoint1(-1);
        mp.setDist1(10000.0);
        mp.setDist2(100000.0); // setting super large starting values
        // 对每个格子，找出离它最近和次近的两个核心点
        for (int ii = 0; ii < numNodes; ii++)
        {
          double dist =
            std::sqrt((base[ii].x - test.x) * (base[ii].x - test.x) +
                      (base[ii].y - test.y) * (base[ii].y - test.y) +
                      (base[ii].z - test.z) * (base[ii].z - test.z));
          // 比“最近的”还近 → 原来的最近降级为次近
          if (dist < mp.getDist1())
          {

            mp.setDist2(mp.getDist1());
            mp.setDist1(dist);

            mp.setPoint2(mp.getPoint1());
            mp.setPoint1(ii);
          }
          // 否则只需要更新“次近”
          else if (dist < mp.getDist2())
          {
            mp.setDist2(dist);
            mp.setPoint2(ii);
          } // finding the distances to the nearest two cores
        }
        // 到最近和次近核心点的距离相近 → 说明它在两者的“中面”上
        if (std::abs(mp.getDist2() - mp.getDist1()) < 1 / info.scale)
        { // the tested location is on one of the middle planes
          // 只有两个核心点的编号之和落在某个区间内才留墙（区间宽窄由 connectivity 决定）
          if ((mp.getPoint1() + mp.getPoint2()) >
                int((1 - connectivity) * numNodes) &&
              (mp.getPoint1() + mp.getPoint2()) <
                int((1 + connectivity) * numNodes))
          { // this is a holed wall
            // 两个核心点之间距离够远 → 中面上开洞，形成通道
            double judge =
              std::sqrt((base[mp.getPoint1()].x - base[mp.getPoint2()].x) *
                          (base[mp.getPoint1()].x - base[mp.getPoint2()].x) +
                        (base[mp.getPoint1()].y - base[mp.getPoint2()].y) *
                          (base[mp.getPoint1()].y - base[mp.getPoint2()].y) +
                        (base[mp.getPoint1()].z - base[mp.getPoint2()].z) *
                          (base[mp.getPoint1()].z - base[mp.getPoint2()].z));
            if (mp.getDist1() + mp.getDist2() - judge >=
                roadRad / (info.scale * 3))
            {
              // 该格子成为障碍点
              info.cloud->points.push_back(mp.getPoint());
            }
          }
          else
          {
            // 另一个分支：不开洞，直接留墙
            info.cloud->points.push_back(mp.getPoint());
          }
        }
      }
    }
  }

  info.cloud->width  = info.cloud->points.size();
  info.cloud->height = 1;
  RCLCPP_INFO(info.node->get_logger(), "points before optimization: %u", info.cloud->width);
  info.cloud->points.resize(info.cloud->width * info.cloud->height);
  pcl2ros();
}
