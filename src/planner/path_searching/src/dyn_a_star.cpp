// ============================================================================
// 文件：src/dyn_a_star.cpp —— A* 寻路算法的实现
//
// 【一句话说明白 A* 在干嘛】
//   把地图切成小格子，从起点开始一层层往外“扩散”，
//   每次优先扩展“看起来最有希望最快到终点”的格子，直到碰到终点；
//   然后顺着“我是从哪来的”倒推出一条路径。
//
// 【本项目特有的两点】
//   1. 只在水平面 XY 上做八方向搜索，高度 z 由起点到终点的连线插值决定；
//   2. 查障碍时带朝向 yaw（因为机器人是前后两个圆柱）。
//
// 【建议阅读顺序】
//   initGridMap → AstarSearch（核心）→ retrievePath（倒推路径）
// ============================================================================
#include "path_searching/dyn_a_star.h"
#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std;
using namespace Eigen;

// 析构函数：释放 initGridMap 里 new 出来的所有搜索节点，防止内存泄漏。
AStar::~AStar()
{
    // 三层循环遍历整个搜索池，逐个节点释放
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                // 释放这个格子上的搜索节点
                delete GridNodeMap_[i][j][k];
}

// 初始化搜索空间：
//   - 记住地图指针（后面用它查障碍）
//   - 分配 pool_size 大小的三维数组，每个格子放一个 GridNode
void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    // 保存搜索池大小；把池子正中间记为“中心索引”
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    // 分配一个三维指针数组（每个格子对应一个搜索节点）
    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    // C++ 里三维动态数组要一层一层手动分配
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                // 每个格子创建一个搜索节点
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    // 保存地图指针，搜索时用它查“这里是不是障碍”
    grid_map_ = occ_map;
}

// 启发函数：估计“从当前格子还要走多远才到终点”。
// 这里用对角距离（允许斜着走时的最短距离），估计得越准，A* 搜得越快。
double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    // 两个格子在 x、y、z 三个方向上的距离
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    // 对角距离公式：先尽量走“三个方向同时前进”的斜步
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    // 哪个方向先归零，就用对应的那组公式计算剩余距离
    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

// 曼哈顿距离（只能横竖走时的距离）。本文件里没被使用。
double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

// 欧氏直线距离。本文件里没被使用。
double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

// 找到终点后，顺着 cameFrom 一路往回走，得到整条路径（此时是“终点→起点”的顺序）。
vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    // 从终点开始，逐个把节点加入路径
    path.push_back(current);

    // 顺着“我是从哪来的(cameFrom)”一路往回走，直到起点
    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

// 把世界坐标转成格子坐标；如果起点或终点正好落在障碍里，
// 就沿着“起点→终点”方向把它往外挪，直到挪到空地上为止。
bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    // 先把世界坐标转成格子坐标；超出搜索范围就直接失败
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    // 起点指向终点的方向（用来算朝向，也用来把点往外挪）
    Eigen::Vector3d start_to_end = end_pt - start_pt;
    if (start_to_end.norm() < 1e-6)
        return false;
    // 这条搜索路径的朝向 —— 查障碍时要带上朝向
    const double path_yaw = std::atan2(start_to_end(1), start_to_end(0));
    start_to_end.normalize();

    // 检查起点是否正好落在障碍里
    int occ = checkOccupancy(Index2Coord(start_idx), path_yaw);
    if (occ)
    {
        //ROS_WARN("Start point is insdide an obstacle.");
        // 如果起点在障碍里：每次沿反方向退一格，直到退到空地上
        do
        {
            start_pt -= start_to_end * step_size_;
            if (!Coord2Index(start_pt, start_idx))
                return false;

            occ = checkOccupancy(Index2Coord(start_idx), path_yaw);
            // 退到搜索范围外了 → 无解，返回失败
            if (occ == -1)
            {
                RCLCPP_WARN(rclcpp::get_logger("path_searching"), "[Astar] Start point outside the map region.");
                return false;
            }
        } while (occ);
    }

    // 终点同理：如果落在障碍里就往前挪
    occ = checkOccupancy(Index2Coord(end_idx), path_yaw);
    if (occ)
    {
        //ROS_WARN("End point is insdide an obstacle.");
        do
        {
            // 每次沿方向前进一格
            end_pt += start_to_end * step_size_;
            if (!Coord2Index(end_pt, end_idx))
                return false;

            occ = checkOccupancy(Index2Coord(end_idx), path_yaw);
            if (occ == -1)
            {
                RCLCPP_WARN(rclcpp::get_logger("path_searching"), "[Astar] End point outside the map region.");
                return false;
            }
        } while (occ);
    }

    return true;
}

// ==========================================================================
// 【核心】AstarSearch：真正执行搜索的地方。
//   1. 把起点/终点转成格子坐标（必要时从障碍里挪出来）
//   2. 建立一个“只看起点到终点这条连线附近”的搜索平面（z 由插值决定）
//   3. 把起点放进待扩展队列
//   4. 循环：每次取出 fScore 最小的格子，向周围 8 个方向扩展
//   5. 碰到终点 → 倒推路径返回 SUCCESS；超过 0.2 秒或搜完 → 返回失败
// ==========================================================================
ASTAR_RET AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    // 记下开始时间（用于后面的超时判断）
    const auto time_1 = std::chrono::steady_clock::now();
    // 轮次 +1：用来区分本次和上次搜索，避免每次都清空所有节点
    ++rounds_;

    // 记录格子边长、其倒数，以及搜索中心（起点与终点的中点）
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    // 把起终点转成格子坐标（必要时从障碍里挪出来）
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        RCLCPP_ERROR(rclcpp::get_logger("path_searching"),
                     "Unable to handle the initial or end point, force return!");
        // 起终点有问题 → 返回“初始化错误”
        return ASTAR_RET::INIT_ERR;
    }

    // 下面构造“搜索平面”：只在起点到终点这条连线附近搜索，而不是搜整张图
    const Eigen::Vector3d search_start = Index2Coord(start_idx);
    const Eigen::Vector3d search_end = Index2Coord(end_idx);
    const Eigen::Vector2d search_start_xy = search_start.head<2>();
    const Eigen::Vector2d search_xy_delta = search_end.head<2>() - search_start_xy;
    const double search_xy_len2 = search_xy_delta.squaredNorm();

    // 关键函数：给定 XY 格子坐标，算出它在连线上对应的高度格子编号
    auto interpolateZIndexOnSearchPlane = [&](const int x_idx, const int y_idx) -> int {
        // 起点终点在水平面上重合 → 高度就直接取起点的高度
        if (search_xy_len2 < 1e-8)
            return start_idx(2);

        // 把这个 XY 格子投影到连线上，算出它在连线上的比例
        Eigen::Vector3i sample_idx(x_idx, y_idx, start_idx(2));
        const Eigen::Vector2d sample_xy = Index2Coord(sample_idx).head<2>();
        double ratio = (sample_xy - search_start_xy).dot(search_xy_delta) / search_xy_len2;
        // 比例夹在 0~1 之间（超出连线范围就取端点）
        ratio = std::max(0.0, std::min(1.0, ratio));

        // 高度按比例线性插值
        const double z = search_start(2) + ratio * (search_end(2) - search_start(2));
        return static_cast<int>((z - center_(2)) * inv_step_size_ + 0.5) + CENTER_IDX_(2);
    };

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    // 取出起点、终点对应的搜索节点
    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    // 用一个空队列把“待扩展队列”清空（比逐个弹出快得多）
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    // 记录起终点的格子坐标
    endPtr->index = end_idx;

    startPtr->index = start_idx;
    // 起点：标记为“本轮已访问”，代价初始化为 0
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    // 总代价估计 = 已走代价 + 到终点的估计距离
    startPtr->fScore = getHeu(startPtr, endPtr);
    // 状态设为“待扩展”；起点没有来路
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    // 把起点放进待扩展队列，搜索正式开始
    openSet_.push(startPtr); //put start in open set

    double tentative_gScore;

    int num_iter = 0;
    // 主循环：只要还有待扩展的格子就继续
    while (!openSet_.empty())
    {
        // 迭代次数计数（仅用于统计）
        num_iter++;
        // 取出总代价估计最小的格子 —— 它最有希望通向终点
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        // 如果取出的正好是终点 → 搜索成功
        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            // 顺着来路倒推出整条路径
            gridPath_ = retrievePath(current);
            // 返回成功
            return ASTAR_RET::SUCCESS;
        }
        // 否则标记为“已扩展”，以后不再重复考虑
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        // 向周围 8 个方向扩展（左右上下及四个斜向）
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            {
                // 跳过“原地不动”的情况
                if (dx == 0 && dy == 0)
                    continue;

                // 邻居的格子坐标：XY 加上偏移，Z 由上面的“搜索平面”插值得到
                Vector3i neighborIdx;
                neighborIdx(0) = (current->index)(0) + dx;
                neighborIdx(1) = (current->index)(1) + dy;
                neighborIdx(2) = interpolateZIndexOnSearchPlane(neighborIdx(0), neighborIdx(1));

                // 越界检查（四周各留一圈，避免数组越界）
                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                {
                    continue;
                }

                // 取出这个邻居对应的搜索节点
                neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                neighborPtr->index = neighborIdx;

                // 判断这个邻居本轮是否已经被访问过
                bool flag_explored = neighborPtr->rounds == rounds_;

                // 已经扩展过 → 跳过，不用再算
                if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                {
                    continue; //in closed set.
                }

                // 标记为“本轮访问过”
                neighborPtr->rounds = rounds_;

                // 从当前格子走向邻居的方向，作为机器人的朝向
                const double neighbor_yaw = std::atan2(static_cast<double>(dy), static_cast<double>(dx));
                // 邻居是障碍 → 走不通，跳过
                if (checkOccupancy(Index2Coord(neighborPtr->index), neighbor_yaw))
                {
                    continue;
                }

                // 算这一步的代价：三维欧氏距离（直走 1、斜走 √2 或 √3）
                const int dz = neighborIdx(2) - current->index(2);
                double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                // 试探性总代价 = 当前已走代价 + 这一步的代价
                tentative_gScore = current->gScore + static_cost;

                // 情况一：第一次发现这个邻居 → 记录来路和代价，放进队列
                if (!flag_explored)
                {
                    //discover a new node
                    neighborPtr->state = GridNode::OPENSET;
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    // 总代价估计 = 试探代价 + 到终点的估计距离
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    openSet_.push(neighborPtr); //put neighbor in open set and record it.
                }
                // 情况二：以前来过，但这次走的更便宜 → 更新它的来路和代价
                else if (tentative_gScore < neighborPtr->gScore)
                { //in open set and need update
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                }
            }
        // 每扩展完一轮都检查一次是否超时
        const auto time_2 = std::chrono::steady_clock::now();
        // 超过 0.2 秒还没搜完 → 放弃本次搜索
        if (std::chrono::duration<double>(time_2 - time_1).count() > 0.2)
        {
            RCLCPP_WARN(rclcpp::get_logger("path_searching"),
                        "Failed in A-star path search: 0.2 second time limit exceeded");
            return ASTAR_RET::SEARCH_ERR;
        }
    }

    // 队列都空了还没碰到终点 → 说明这条路根本走不通
    const auto time_2 = std::chrono::steady_clock::now();

    const double elapsed = std::chrono::duration<double>(time_2 - time_1).count();
    // 如果这次搜索特别慢（超过 0.1 秒），打一条警告方便排查
    if (elapsed > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("path_searching"),
                    "A-star path search took %.3fs, iter=%d", elapsed, num_iter);

    // 返回“搜索失败”
    return ASTAR_RET::SEARCH_ERR;
}

// 把刚才搜到的格子路径换算成世界坐标，并反转成“起点→终点”的顺序返回。
vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    // 把格子路径里每个节点换算成世界坐标
    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    // 翻转顺序：从“终点→起点”变成“起点→终点”
    reverse(path.begin(), path.end());
    return path;
}
