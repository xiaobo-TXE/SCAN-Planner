// ============================================================================
// 文件：src/bspline_optimizer.cpp —— 轨迹优化器的实现（最核心的算法文件）
//
// 【一分钟看懂它在干嘛】
//   给一条“会撞墙/有点抖”的初始轨迹，通过微调 B 样条的控制点，
//   让它变成一条“平滑、离障碍远、不超速”的好轨迹。
//
// 【怎么知道往哪调？——靠“梯度”】
//   代价函数对每个控制点求导，导数告诉我们“把控制点往哪个方向挪代价会变小”。
//   每个代价项（平滑/避障/可行性/贴合）都会贡献一份梯度，加权求和就是总方向。
//
// 【注意】优化只动 x、y；z 的梯度被清零（四足走平地，高度另行处理）。
//
// 【建议阅读顺序】
//   initControlPoints → calcSmoothnessCost → calcDistanceCostRebound
//   → calcFeasibilityCost → combineCostRebound → rebound_optimize
// ============================================================================
#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
#include <algorithm>
#include <chrono>
#include <cmath>
// using namespace std;

namespace scan_planner
{

// 读取所有优化参数（从 planner.yaml 的 optimization.* 段）。
  void BsplineOptimizer::setParam(rclcpp::Node *node)
  {
    // 小工具：读取一个 double 参数（没声明过就先声明）
    const auto get_double = [node](const std::string &name, double default_value) {
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    // 四个代价权重：平滑 / 避障 / 可行性 / 贴合
    lambda1_ = get_double("optimization.lambda_smooth", -1.0);
    lambda2_ = get_double("optimization.lambda_collision", -1.0);
    lambda3_ = get_double("optimization.lambda_feasibility", -1.0);
    lambda4_ = get_double("optimization.lambda_fitness", -1.0);
    // 期望安全距离、速度上限、加速度上限
    dist0_ = get_double("optimization.dist0", -1.0);
    max_vel_ = get_double("optimization.max_vel", -1.0);
    max_acc_ = get_double("optimization.max_acc", -1.0);
    // B 样条阶数（默认 3）
    if (!node->has_parameter("optimization.order")) node->declare_parameter<int>("optimization.order", 3);
    order_ = static_cast<int>(node->get_parameter("optimization.order").as_int());
  }

// 注入地图，供避障查询使用。
  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &env)
  {
    // 保存地图指针，避障查询要用
    this->grid_map_ = env;
  }

// 用一段线段的方向估计机器人朝向 yaw（避障查询需要朝向）。
  double BsplineOptimizer::estimateSegmentYaw(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    // 用两点连线的水平方向估计朝向（碰撞检测需要朝向）
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    // 两点几乎重合 → 无法判断，返回 0
    if (diff.squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(diff(1), diff(0));
  }

  double BsplineOptimizer::estimateControlPointYaw(const Eigen::MatrixXd &q, int id) const
  {
    // 控制点太少 → 返回 0
    if (q.cols() <= 1)
      return 0.0;

    // 用“前一个点 → 后一个点”的方向，估计该控制点处的朝向
    int prev_id = std::max(0, id - 1);
    int next_id = std::min(static_cast<int>(q.cols()) - 1, id + 1);
    return estimateSegmentYaw(q.col(prev_id), q.col(next_id));
  }

  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    // 直接设置控制点（Refine 流程用）
    cps_.points = points;
  }

  void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

  /* This function is very similar to check_collision_and_rebound(). 
   * It was written separately, just because I did it once and it has been running stably since March 2020.
   * But I will merge then someday.*/
// ==========================================================================
// 【避障准备】initControlPoints：找出“哪些控制点撞了障碍”，并算出脱困方向。
//   1. 沿轨迹逐段采样，用带朝向的碰撞检测找出“碰撞段”
//   2. 对每段碰撞的起点/终点跑一次 A*，得到一条绕行路线
//   3. 对每个撞墙的控制点：找到它与 A* 路线的交点方向，记为 direction
//      （优化时就把这个控制点沿着 direction 往外推）
// ==========================================================================
  std::vector<std::vector<Eigen::Vector3d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
  {

    // 第一次调用时：初始化 ControlPoints 结构（容量、安全距离、控制点）
    if (flag_first_init)
    {
      cps_.clearance = dist0_;
      cps_.resize(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    // 判断“碰撞状态是否稳定”的连续次数阈值
    constexpr int ENOUGH_INTERVAL = 2;
    // 采样步长：按轨迹长短自适应，保证两点之间采样足够密
    double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 2;
    // in_id / out_id：当前碰撞段的起点与终点控制点下标
    int in_id = -1, out_id = -1;
    vector<std::pair<int, int>> segment_ids;
    // 连续处于同一碰撞状态的次数
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    // 当前采样点、上一次采样点是否碰撞
    bool occ, last_occ = false;
    // 三个状态标志：是否找到碰撞段起点 / 终点 / 疑似终点
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    // 只检查前 2/3 的控制点
    int i_end = (int)init_points.cols() - order_ - ((int)init_points.cols() - 2 * order_) / 3; // only check closed 2/3 points.
    // 逐个控制点区间扫描
    for (int i = order_; i <= i_end; ++i)
    {
      // 在相邻两个控制点之间做细密采样
      for (double a = 1.0; a >= 0.0; a -= step_size)
      {
        // 采样点位置：在相邻两个控制点之间线性插值
        Eigen::Vector3d sample_pt = a * init_points.col(i - 1) + (1 - a) * init_points.col(i);
        // 该处的机器人朝向
        double sample_yaw = estimateSegmentYaw(init_points.col(i - 1), init_points.col(i));
        // 问地图：这里撞吗
        occ = grid_map_->getInflateOccupancy(sample_pt, sample_yaw);
        // cout << setprecision(5);
        // cout << (a * init_points.col(i-1) + (1-a) * init_points.col(i)).transpose() << " occ1=" << occ << endl;

        // 从“空”变成“撞” → 出现一个新的碰撞段
        if (occ && !last_occ)
        {
          // 状态稳定之后才确认起点（避免噪声误判）
          if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
          {
            // 记录碰撞段的起点下标
            in_id = i - 1;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        // 从“撞”变成“空” → 这一碰撞段结束
        else if (!occ && last_occ)
        {
          // 记录碰撞段的终点下标
          out_id = i;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;
        }

        // 稳定几拍之后才确认终点
        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        // 起点和终点都确认了 → 把这一段记录下来
        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          // 合法性检查通过后存入碰撞段列表
          if (in_id >= 0 && out_id >= in_id && out_id < init_points.cols())
            segment_ids.push_back(std::pair<int, int>(in_id, out_id));
          else
            RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                        "Skip invalid collision segment: in_id=%d, out_id=%d", in_id, out_id);
        }
      }
    }

    /*** a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_paths;
    // 对每一段碰撞各跑一次 A*
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      //cout << "in=" << in.transpose() << " out=" << out.transpose() << endl;
      // 用这段的起点/终点控制点作为 A* 的起终点
      Eigen::Vector3d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
      // 开始 A* 搜索
      ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
      if (ret == ASTAR_RET::SUCCESS)
      {
        // 取出 A* 找到的绕行路径
        vector<Eigen::Vector3d> path = a_star_->getPath();
        if (path.size() < 2)
        {
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"), "A-star path has fewer than 2 points");
          return a_star_paths;
        }
        // 保存这条路径（后面只用来确定“往哪边推”）
        a_star_paths.push_back(path);
      }
      else
      {
        // A* 都失败了 → 放弃这次优化
        RCLCPP_ERROR(rclcpp::get_logger("bspline_opt"), "A-star failed; aborting optimization");
        return a_star_paths;
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    // 给每段计算一个“允许影响的控制点范围”，避免相邻段互相干扰
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      // 第一段：下界从第 order_ 个控制点开始
      if (i == 0) // first segment
      {
        id_low_bound = order_;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - order_ - 1;
        }
      }
      // 最后一段：上界到倒数第 order_ 个控制点
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - order_ - 1;
      }
      // 中间段：上下界取相邻两段的中点
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      // 记录这一段的可用范围
      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    // cout << "+++++++++" << endl;
    // for ( int j=0; j<bounds.size(); ++j )
    // {
    //   cout << bounds[j].first << "  " << bounds[j].second << endl;
    // }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> final_segment_ids(segment_ids.size());
    // 每段至少要有的点数比例（0 表示不做扩段）
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient thrust
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      // 这一段现在有多少个控制点
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
      // 太短就向两侧扩展一点（保证有足够点数产生推力）
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        // 扩展后不能超出 bounds 允许的范围
        final_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

        final_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
      }
      // 长度够，就用原来的范围
      else
      {
        final_segment_ids[i].first = segment_ids[i].first;
        final_segment_ids[i].second = segment_ids[i].second;
      }

      //cout << "final:" << "i = " << i << " first = " << final_segment_ids[i].first << " second = " << final_segment_ids[i].second << endl;
    }

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // A* 路径缺失 → 跳过这一段
      if (i >= a_star_paths.size() || a_star_paths[i].size() < 2)
      {
        RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                    "Skip invalid A-star path while assigning control point directions");
        continue;
      }

      // step 1
      // step1：先清空这一段的“已处理”标记
      for (int j = final_segment_ids[i].first; j <= final_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      int got_intersection_id = -1;
      // step2：逐个控制点求它与 A* 路径的交点
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        // 控制点处的切线方向（用来判断交点在哪一侧）
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
        // 从 A* 路径的中点开始，向两侧搜索
        int Astar_id = a_star_paths[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        // 用“投影值”的符号变化来判断是否跨过交点
        double val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
        // 沿 A* 路径逐点步进
        while (Astar_id >= 0 && Astar_id < (int)a_star_paths[i].size())
        {
          last_Astar_id = Astar_id;

          // 投影为正往一侧走、为负往另一侧走
          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          if (Astar_id < 0 || Astar_id >= (int)a_star_paths[i].size())
            break;

          val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

          // 符号翻转 → 说明找到了交点
          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            // 求交点的分母（接近 0 说明平行，放弃）
            const double denom = ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]);
            if (std::abs(denom) < 1e-8)
              break;

            // 用线性插值算出精确的交点
            intersection_point =
                a_star_paths[i][Astar_id] +
                ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(cps_.points.col(j) - a_star_paths[i][Astar_id]) / denom) // = t
                );

            //cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

            got_intersection_id = j;
            break;
          }
        }

        // 找到交点后：从控制点朝交点方向走，一直找到障碍表面
        if (got_intersection_id >= 0)
        {
          cps_.flag_temp[j] = true;
          // 控制点到交点的距离
          double length = (intersection_point - cps_.points.col(j)).norm();
          if (length > 1e-5)
          {
            // 从交点往回逐格采样
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              // 采样点位置（在控制点与交点之间按比例插值）
              Eigen::Vector3d sample_pt = (a / length) * intersection_point + (1 - a / length) * cps_.points.col(j);
              double sample_yaw = estimateControlPointYaw(cps_.points, j);
              // 该采样点是否被障碍占据
              occ = grid_map_->getInflateOccupancy(sample_pt, sample_yaw);

              // 撞到了、或已经退回到控制点附近 → 在此记录推出方向和障碍面点
              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                // 障碍表面上的点（作为推出方向的起点）
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                // 推出方向（单位向量），优化时沿它把控制点往外推
                cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                break;
              }
            }
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      // 特例：碰撞段只跨 1 个控制点（此时交点可能算错，方向会反，需要特殊处理）
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        // 用这一小段的方向和两端点的中点来找交点
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(segment_ids[i].second) - cps_.points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (cps_.points.col(segment_ids[i].second) + cps_.points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_paths[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_paths[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_paths[i].size())
        {
          last_Astar_id = Astar_id;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          if (Astar_id < 0 || Astar_id >= (int)a_star_paths[i].size())
            break;

          val = (a_star_paths[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            const double denom = ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]);
            if (std::abs(denom) < 1e-8)
              break;

            intersection_point =
                a_star_paths[i][Astar_id] +
                ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_paths[i][Astar_id]) / denom) // = t
                );

            // 交点必须离中点足够远（超过 1cm）才算有效
            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              // 记录方向和障碍面点
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(cps_.points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //step 3
      // step3：把求出的方向向两侧“传播”给还没处理的控制点
      if (got_intersection_id >= 0)
      {
        // 向后传播
        for (int j = got_intersection_id + 1; j <= final_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        // 向前传播
        for (int j = got_intersection_id - 1; j >= final_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    return a_star_paths;
  }

  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    // earlyExit 回调：把 user data 转回优化器对象
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // cout << "k=" << k << endl;
    // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
    // 返回 true 就是告诉 L-BFGS：“请提前停止优化”
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

// L-BFGS 回调（Rebound）：把一维数组 x 还原成控制点，算出总代价和梯度。
  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    // L-BFGS 回调：把 user data 转回优化器对象
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    // 交给 combineCostRebound 计算总代价与梯度
    opt->combineCostRebound(x, grad, cost, n);

    // 迭代次数 +1（避障项靠它判断“轨迹是否已经足够平滑”）
    opt->iter_num_ += 1;
    return cost;
  }

  double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    // Refine 版：换成含“贴合项”的代价函数
    opt->combineCostRefine(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

// 【代价项 2：避障】对每个撞墙的控制点，算它离障碍多远、够不够安全距离，
// 不够就产生一个“沿 direction 往外推”的梯度。
// 注意：只有等轨迹已经比较平滑时（iter_num 较大）才开始考虑避障，避免一开始乱推。
  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
  {
    // 代价清零
    cost = 0.0;
    // 只处理中间的控制点
    int end_idx = q.cols() - order_;
    // demarcation：安全距离的分界点，用来切换两段不同的惩罚曲线
    double demarcation = cps_.clearance;
    // 二次曲线系数（让两段惩罚在分界点处能平滑衔接）
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    // 默认不打断优化
    force_stop_type_ = DONT_STOP;
    // 只有当“已经迭代几次、轨迹比较平滑”时，才真正开始避障
    if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
    {
      // 检查是否出现新碰撞，并更新推出方向
      check_collision_and_rebound();
    }

    /*** calculate distance cost and gradient ***/
    // 遍历每个需要避障的控制点
    for (auto i = order_; i < end_idx; ++i)
    {
      // 一个控制点可能对应多个推出方向
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        // 控制点沿推出方向“已经脱离障碍”的距离
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        // 距离安全值还差多少
        double dist_err = cps_.clearance - dist;
        Eigen::Vector3d dist_grad = cps_.direction[i][j];

        // 已经足够安全 → 不惩罚
        if (dist_err < 0)
        {
          /* do nothing */
        }
        // 轻微不足 → 用三次曲线惩罚（靠近分界点时增长柔和）
        else if (dist_err < demarcation)
        {
          // 代价增加
          cost += pow(dist_err, 3);
          // 梯度：沿推出方向往回推（负号 = 远离障碍的方向）
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        // 严重不足 → 用二次曲线惩罚（增长更快）
        else
        {
          // 代价增加
          cost += a * dist_err * dist_err + b * dist_err + c;
          // 梯度
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
  }

// 【代价项 3：贴合】让轨迹别离参考点 ref_pts_ 太远。
// 而且是“各向异性”的：纵向允许偏 25 倍，横向只允许偏 1 倍（横向更严格）。
  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {

    // 贴合项：让优化后的轨迹不要离参考点太远
    cost = 0.0;

    int end_idx = q.cols() - order_;

    // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
    // 各向异性权重：纵向（沿轨迹方向）允许偏 25 倍，横向只允许 1 倍
    double a2 = 25, b2 = 1;
    // 遍历控制点
    for (auto i = order_ - 1; i < end_idx + 1; ++i)
    {
      // 偏差 x = 曲线上的点 与 参考点 之差
      Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
      // 参考点处的切线方向 v（用来区分纵向与横向）
      Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

      // 把偏差拆成“纵向分量”和“横向分量”
      double xdotv = x.dot(v);
      Eigen::Vector3d xcrossv = x.cross(v);

      // 代价 = 纵向偏差²/25 + 横向偏差²/1
      double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
      cost += f;

      Eigen::Matrix3d m;
      // 叉乘的反对称矩阵形式，方便求导
      m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
      // 代价对偏差 x 的导数
      Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

      // 用 (1,4,1)/6 的权重把导数分摊回三个控制点
      gradient.col(i - 1) += df_dx / 6;
      gradient.col(i) += 4 * df_dx / 6;
      gradient.col(i + 1) += df_dx / 6;
    }
  }

// 【代价项 1：平滑】用相邻控制点的四阶差分（近似加加速度 jerk）衡量抖动，
// 越抖代价越大；梯度就是把 jerk 按 [-1,+3,-3,+1] 的权重分摊回四个控制点。
  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
  {

    // 平滑项：用四阶差分近似“加加速度 jerk”衡量抖动
    cost = 0.0;

    // 默认用 jerk（比用加速度更严格、更平滑）
    if (falg_use_jerk)
    {
      Eigen::Vector3d jerk, temp_j;

      // 遍历每四个相邻控制点
      for (int i = 0; i < q.cols() - 3; i++)
      {
        /* evaluate jerk */
        // 四阶差分 = 近似的 jerk
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        // 代价 = jerk 的平方（越抖代价越大）
        cost += jerk.squaredNorm();
        temp_j = 2.0 * jerk;
        /* jerk gradient */
        // 梯度：把 jerk 按 (-1, +3, -3, +1) 的权重分摊回四个控制点
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
      }
    }
    // 备用方案：用加速度（二阶差分）代替 jerk
    else
    {
      Eigen::Vector3d acc, temp_acc;

      for (int i = 0; i < q.cols() - 2; i++)
      {
        /* evaluate acc */
        // 二阶差分 = 近似的加速度
        acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
        cost += acc.squaredNorm();
        temp_acc = 2.0 * acc;
        /* acc gradient */
        // 梯度按 (1, -2, 1) 分摊到三个控制点
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
      }
    }
  }

// 【代价项 4：可行性】速度/加速度超过上限就按超出的平方惩罚，
// 梯度把控制点往“减速”的方向推。
  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {

    // 下面这一大段被 #ifdef 关掉了（是另一种更平滑的可行性实现），实际编译不到
    //#define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS

    cost = 0.0;
    double demarcation = 1.0; // 1m/s, 1m/s/s
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
    double al = ar, bl = -br, cl = cr;

    /* abbreviation */
    double ts, ts_inv2, ts_inv3;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    ts_inv3 = 1 / ts / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_ + demarcation)
        {
          double diff = vi(j) - max_vel_;
          cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

          double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) > max_vel_)
        {
          double diff = vi(j) - max_vel_;
          cost += pow(diff, 3) * ts_inv3;
          ;

          double grad = 3 * diff * diff / ts * ts_inv3;
          ;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -(max_vel_ + demarcation))
        {
          double diff = vi(j) + max_vel_;
          cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

          double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -max_vel_)
        {
          double diff = vi(j) + max_vel_;
          cost += -pow(diff, 3) * ts_inv3;

          double grad = -3 * diff * diff / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_ + demarcation)
        {
          double diff = ai(j) - max_acc_;
          cost += ar * diff * diff + br * diff + cr;

          double grad = (2.0 * ar * diff + br) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) > max_acc_)
        {
          double diff = ai(j) - max_acc_;
          cost += pow(diff, 3);

          double grad = 3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -(max_acc_ + demarcation))
        {
          double diff = ai(j) + max_acc_;
          cost += al * diff * diff + bl * diff + cl;

          double grad = (2.0 * al * diff + bl) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -max_acc_)
        {
          double diff = ai(j) + max_acc_;
          cost += -pow(diff, 3);

          double grad = -3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

#else

    // 注意：上面那一段（#ifdef 内的）是另一种实现，本编译配置下不生效；实际走下面这段
    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    // 时间间隔，以及它的平方的倒数（算法里反复用到）
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* velocity feasibility */
    // 逐段检查速度是否超限
    for (int i = 0; i < q.cols() - 1; i++)
    {
      // 该段速度 = 相邻控制点之差 ÷ 时间间隔
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      //cout << "temp_v * vi=" ;
      // 三个坐标轴分别判断
      for (int j = 0; j < 3; j++)
      {
        // 正向超速 → 按“超出量的平方”惩罚
        if (vi(j) > max_vel_)
        {
          // cout << "fuck VEL" << endl;
          // cout << vi(j) << endl;
          // 代价增加
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          // 梯度：把相邻两个控制点往“彼此靠近（减速）”的方向推
          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        // 反向超速同理
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }

    /* acceleration feasibility */
    // 逐段检查加速度是否超限（用二阶差分）
    for (int i = 0; i < q.cols() - 2; i++)
    {
      // 加速度 = 二阶差分 ÷ 时间间隔的平方
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      //cout << "temp_a * ai=" ;
      for (int j = 0; j < 3; j++)
      {
        // 正向超加速度 → 惩罚
        if (ai(j) > max_acc_)
        {
          // cout << "fuck ACC" << endl;
          // cout << ai(j) << endl;
          // 代价增加
          cost += pow(ai(j) - max_acc_, 2);

          // 梯度按 (1, -2, 1) 的权重分摊到三个控制点
          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        // 反向超加速度同理
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
      //cout << endl;
    }

#endif
  }

// 优化过程中“回头看”：检查是否出现新的碰撞，若发现就重新跑 A*、
// 重新算 direction，并请求 L-BFGS 中止当前这轮（STOP_FOR_REBOUND）。
  bool BsplineOptimizer::check_collision_and_rebound(void)
  {

    // 只检查到倒数第 order_ 个控制点
    int end_idx = cps_.size - order_;

    /*** Check and segment the initial trajectory according to obstacles ***/
    // in_id / out_id：本次碰撞段的起点与终点下标
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    // 只看前 2/3（后段反正会重规划）
    int i_end = end_idx - (end_idx - order_) / 3;
    for (int i = order_ - 1; i <= i_end; ++i)
    {

      // 问地图：这个控制点撞上了吗
      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i), estimateControlPointYaw(cps_.points, i));

      /*** check if the new collision will be valid ***/
      // 如果撞了，再判断这是不是“真正的新碰撞”
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          cout.precision(2);
          // 若它还没被推离上一次的障碍面 → 说明是同一个老碰撞，不算新的
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false; // Not really takes effect, just for better hunman understanding.
            break;
          }
        }
      }

      // 确认是新碰撞 → 开始记录这一段
      if (occ)
      {
        flag_new_obs_valid = true;

        // 向前找第一个“没撞”的控制点，作为这一段起点
        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j), estimateControlPointYaw(cps_.points, j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        // 一路找到头都没找到 → 说明机器人本身已经在障碍里
        if (j < 0) // fail to get the obs free point
        {
          RCLCPP_ERROR(rclcpp::get_logger("bspline_opt"), "The robot is inside an obstacle");
          in_id = 0;
        }

        // 向后找第一个“没撞”的控制点，作为这一段终点
        for (j = i + 1; j < cps_.size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j), estimateControlPointYaw(cps_.points, j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        // 连终点都在障碍里 → 这条轨迹救不回来，标记为错误并停止
        if (j >= cps_.size) // fail to get the obs free point
        {
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                      "Trajectory terminal point is in an obstacle; skip this plan");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        // 跳过这一整段，继续往后检查
        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    // 确实发现了新碰撞 → 对每段重新跑 A*，重新算脱困方向
    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_paths;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
        // 在这一段的起终点之间跑一次 A*
        ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
        if (ret == ASTAR_RET::SUCCESS)
        {
          vector<Eigen::Vector3d> path = a_star_->getPath();
          if (path.size() < 2)
          {
            RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                        "A-star path has fewer than 2 points; drop collision segment");
            // A* 路径太短 → 丢掉这一段
            segment_ids.erase(segment_ids.begin() + i);
            i--;
            continue;
          }
          a_star_paths.push_back(path);
        }
        // 搜索失败但后面还有段落 → 把它和下一段合并再试
        else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size())
        {
          segment_ids[i].second = segment_ids[i + 1].second;
          segment_ids.erase(segment_ids.begin() + i + 1);
          --i;
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                      "A-star failed on a collision segment; merge it with the next segment");
        }
        else
        {
          // 其它错误 → 同样丢掉这一段
          RCLCPP_ERROR(rclcpp::get_logger("bspline_opt"), "A-star error");
          segment_ids.erase(segment_ids.begin() + i);
          i--;
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        if (i >= a_star_paths.size() || a_star_paths[i].size() < 2)
        {
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                      "Skip invalid A-star path while assigning rebound directions");
          continue;
        }

        // step 1
        // step1：先把这一段的“已处理”标记清零
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        // step2：逐点求“控制点连线”与 A* 路径的交点，据此确定推出方向
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          // 控制点处的切线方向，用来判断交点落在哪一侧
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          // 从 A* 路径的中点开始，向两侧搜索交点
          int Astar_id = a_star_paths[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
          // 沿 A* 路径逐步试探
          while (Astar_id >= 0 && Astar_id < (int)a_star_paths[i].size())
          {
            last_Astar_id = Astar_id;

            // 投影为正往一侧走、为负往另一侧走
            if (val >= 0)
              --Astar_id;
            else
              ++Astar_id;

            if (Astar_id < 0 || Astar_id >= (int)a_star_paths[i].size())
              break;

            // 重新计算投影值
            val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            // cout << val << endl;

            // 符号发生翻转 → 说明跨过了交点
            if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
            {
              // 求交点的分母（接近 0 说明两线平行，放弃）
              const double denom = ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]);
              if (std::abs(denom) < 1e-8)
                break;

              intersection_point =
                  a_star_paths[i][Astar_id] +
                  ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_paths[i][Astar_id]) / denom) // = t
                  );

              // 记录“已经处理到这个控制点”
              got_intersection_id = j;
              break;
            }
          }

          // 找到交点后：从控制点朝交点方向逐格采样，一直走到障碍表面
          if (got_intersection_id >= 0)
          {
            cps_.flag_temp[j] = true;
            // 控制点到交点的距离
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              // 从交点往回（朝控制点）逐格采样
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                // 采样点：在控制点与交点之间按比例插值
                Eigen::Vector3d sample_pt = (a / length) * intersection_point + (1 - a / length) * cps_.points.col(j);
                double sample_yaw = estimateControlPointYaw(cps_.points, j);
                // 这个采样点撞了吗
                bool occ = grid_map_->getInflateOccupancy(sample_pt, sample_yaw);

                // 撞到了、或已经退回控制点附近 → 就在这里记录推出方向
                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  // 记录障碍表面的点（base_point）和推出方向（direction）
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            // 控制点几乎就落在交点上 → 这一段找不到有效方向
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3
        // step3：把找到的方向向两侧“传播”给还没处理的控制点
        if (got_intersection_id >= 0)
        {
          // 向后传播
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          // 向前传播
          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          RCLCPP_WARN(rclcpp::get_logger("bspline_opt"), "Failed to generate rebound direction");
      }

      // 告诉 L-BFGS：方向已更新，请中止这一轮、从头重新优化
      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    // 没有发现新碰撞 → 不打断优化
    return false;
  }

// 【对外入口 1】Rebound 优化：调用 rebound_optimize() 并把结果写回 optimal_points。
  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
  {
    // 【对外入口 1】先保存时间间隔
    setBsplineInterval(ts);

    // 执行 Rebound 优化
    bool flag_success = rebound_optimize();

    // 把优化后的控制点交还给调用者
    optimal_points = cps_.points;

    return flag_success;
  }

// 【对外入口 2】Refine 优化：时间拉长后的细化，调用 refine_optimize()。
  bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
  {

    // 【对外入口 2】Refine：先接收控制点与时间间隔
    setControlPoints(init_points);
    setBsplineInterval(ts);

    bool flag_success = refine_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

// Rebound 求解流程：
//   1. 把控制点拉直成一维数组（L-BFGS 只认一维数组）
//   2. 配置 L-BFGS 参数并求解（回调是 costFunctionRebound）
//   3. 解完后再检查一次碰撞：若还撞就加倍避障权重、重启（最多 3 次）
// ==========================================================================
  bool BsplineOptimizer::rebound_optimize()
  {
    // 迭代计数清零
    iter_num_ = 0;
    // 只优化中间的控制点：首尾各留下 order_ 个不动（保证边界约束不被破坏）
    int start_id = order_;
    int end_id = this->cps_.size - order_;
    // 变量个数 = 控制点个数 × 3（每个点有 x/y/z 三个分量）
    variable_num_ = 3 * (end_id - start_id);
    double final_cost;

    auto t0 = std::chrono::steady_clock::now();
    auto t1 = t0;
    auto t2 = t0;
    // restart_nums：重启次数；rebound_times：因发现新障碍而中断的次数
    int restart_nums = 0, rebound_times = 0;
    ;
    bool flag_force_return, flag_occ, success;
    // 避障权重（重启时会翻倍，所以单独用一个变量）
    new_lambda2_ = lambda2_;
    // 最多重启 3 次
    constexpr int MAX_RESART_NUMS_SET = 3;
    // 外层循环：优化 → 检查 → 必要时重启
    do
    {
      /* ---------- prepare ---------- */
      // 每轮开始时把“最优代价”重置为无穷大
      min_cost_ = std::numeric_limits<double>::max();
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;

      // 把控制点从矩阵“拉直”成一维数组（L-BFGS 只认一维数组）
      double q[variable_num_];
      memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

      // 配置 L-BFGS：记忆 16 步、最多 200 次迭代、梯度阈值 0.01
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.01;

      /* ---------- optimize ---------- */
      t1 = std::chrono::steady_clock::now();
      // 开始求解：L-BFGS 每走一步都会回调 costFunctionRebound 来要代价和梯度
      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      t2 = std::chrono::steady_clock::now();
      double time_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
      double total_time_ms = std::chrono::duration<double, std::milli>(t2 - t0).count();

      /* ---------- success temporary, check collision again ---------- */
      // 求解正常结束（收敛 / 达到迭代上限 / 已是最优 / 被主动停止）
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
        flag_force_return = false;

        // 用优化后的控制点重建轨迹，自己再检查一遍碰撞
        UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
        double tm, tmp;
        traj.getTimeSpan(tm, tmp);
        double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution());
        // 只检查前 2/3（后段反正会重规划）
        for (double t = tm; t < tmp * 2 / 3; t += t_step) // Only check the closest 2/3 partition of the whole trajectory.
        {
          Eigen::Vector3d pos = traj.evaluateDeBoorT(t);
          Eigen::Vector3d pos_next = traj.evaluateDeBoorT(std::min(t + t_step, tmp));
          flag_occ = grid_map_->getInflateOccupancy(pos, estimateSegmentYaw(pos, pos_next));
          // 发现碰撞
          if (flag_occ)
          {
            //cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

            // 如果一开头就撞 → 前几个控制点就在障碍里，这条轨迹没法用，直接失败
            if (t <= bspline_interval_) // First 3 control points in obstacles!
            {
              cout << cps_.points.col(1).transpose() << "\n"
                   << cps_.points.col(2).transpose() << "\n"
                   << cps_.points.col(3).transpose() << "\n"
                   << cps_.points.col(4).transpose() << endl;
              RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                          "First three control points are in obstacles; t=%f", t);
              return false;
            }

            // 否则跳出检查，走下面的“重启”流程
            break;
          }
        }

        // 全程没有碰撞 → 这次优化成功
        if (!flag_occ)
        {
          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          // 标记成功
          success = true;
        }
        else // restart
        {
          // 还撞 → 重启：重新计算脱困方向，并把避障权重翻倍
          restart_nums++;
          initControlPoints(cps_.points, false);
          // 避障权重翻倍（逼优化器更重视避障）
          new_lambda2_ *= 2;

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,keep optimizing\n\033[0m", iter_num_, time_ms);
        }
      }
      // 被“提前退出”打断 → 说明优化中途发现了新障碍
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        // 记录一次中断
        rebound_times++;
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
      }
      else
      {
        // 其它求解错误 → 打警告并放弃本次规划
        RCLCPP_WARN(rclcpp::get_logger("bspline_opt"),
                    "Solver error. Return=%d, %s; skip this plan", result, lbfgs::lbfgs_strerror(result));
        // while (ros::ok());
      }

    // 继续循环的条件：还在碰撞且重启次数没用完，或还有新障碍待处理
    } while ((flag_occ && restart_nums < MAX_RESART_NUMS_SET) ||
             (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    return success;
  }

// Refine 求解流程：与上面类似，但用 costFunctionRefine（含贴合项），
// 并且若结果仍不安全就把贴合权重加倍再试。
  bool BsplineOptimizer::refine_optimize()
  {
    // Refine 流程：同样只优化中间的控制点
    iter_num_ = 0;
    int start_id = order_;
    int end_id = this->cps_.points.cols() - order_;
    variable_num_ = 3 * (end_id - start_id);

    // 一维变量数组
    double q[variable_num_];
    double final_cost;

    // 把中间控制点拷进一维数组
    memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

    // 保存原始贴合权重（退出时要还原）
    double origin_lambda4 = lambda4_;
    bool flag_safe = true;
    int iter_count = 0;
    // 循环：优化 → 检查安全性 → 不安全就把贴合权重加倍
    do
    {
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      // 这里梯度阈值更小（0.001），要求解得更精细
      lbfgs_params.g_epsilon = 0.001;

      // 用 Refine 的代价函数求解（注意没有 earlyExit 回调）
      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
      // 求解状态检查
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //pass
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("bspline_opt"),
                     "Solver error while refining: return=%d, %s", result, lbfgs::lbfgs_strerror(result));
      }

      UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
      double tm, tmp;
      traj.getTimeSpan(tm, tmp);
      double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution()); // Step size is defined as the maximum size that can passes through every grid.
      // 同样只检查前 2/3 有没有碰撞
      for (double t = tm; t < tmp * 2 / 3; t += t_step)
      {
        Eigen::Vector3d pos = traj.evaluateDeBoorT(t);
        Eigen::Vector3d pos_next = traj.evaluateDeBoorT(std::min(t + t_step, tmp));
        // 撞了 → 判定为不安全
        if (grid_map_->getInflateOccupancy(pos, estimateSegmentYaw(pos, pos_next)))
        {
          // cout << "Refined traj hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

          Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
          for (size_t i = 0; i < ref_pts_.size(); i++)
          {
            ref_pts.row(i) = ref_pts_[i].transpose();
          }

          // 标记不安全（下面会把贴合权重加倍，再试一轮）
          flag_safe = false;
          break;
        }
      }

      if (!flag_safe)
        // 贴合权重翻倍
        lambda4_ *= 2;

      iter_count++;
    // 不安全就再试一次（注意 iter_count 从 0 开始，所以实际只跑一轮）
    } while (!flag_safe && iter_count <= 0);

    // 还原贴合权重，避免影响下一次调用
    lambda4_ = origin_lambda4;

    //cout << "iter_num_=" << iter_num_ << endl;

    // 返回“是否安全”
    return flag_safe;
  }

// 【代价汇总（Rebound）】把上面四项按权重相加：
//   f = λ1·平滑 + λ2·避障 + λ3·可行性
// 并把 z 方向的梯度清零（只优化水平面）。
  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {

    // 【代价汇总（Rebound）】先把一维数组 x 还原回控制点矩阵
    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_distance, f_feasibility;

    // 三个代价项各自的梯度矩阵（都先清零）
    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);

    // 分别计算三项代价：平滑、避障、可行性
    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    // 总代价 = λ1·平滑 + λ2·避障 + λ3·可行性
    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility;
    //printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

    // 总梯度 = 三项梯度按同样权重加权相加
    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance +
                              lambda3_ * g_feasibility;
    // 【关键】把 z 方向的梯度清零 —— 只优化水平面（四足走平地，高度另行处理）
    grad_3D.row(2).setZero();
    // 把梯度压回一维数组，交回给 L-BFGS
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

// 【代价汇总（Refine）】把权重换成：
//   f = λ1·平滑 + λ4·贴合 + λ3·可行性
  void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
  {

    // 【代价汇总（Refine）】同样先把控制点还原
    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_fitness, f_feasibility;

    // 三项梯度矩阵：平滑、贴合、可行性
    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    //time_satrt = ros::Time::now();

    // 分别计算三项代价
    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcFitnessCost(cps_.points, f_fitness, g_fitness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    /* ---------- convert to solver format...---------- */
    // 总代价 = λ1·平滑 + λ4·贴合 + λ3·可行性（注意这里换成了“贴合项”）
    f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;
    // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);

    // 总梯度同理加权相加
    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness +
                              lambda3_ * g_feasibility;
    // 同样把 z 方向梯度清零
    grad_3D.row(2).setZero();
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

} // namespace scan_planner
