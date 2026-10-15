// ============================================================================
// 文件：include/path_searching/dyn_a_star.h
//
// 【这个文件是干什么的】
//   定义 A*（读作 A-star）寻路算法。A* 是最经典的“找一条最短通路”算法。
//   它在一张网格地图上，从起点一步步往外扩散，直到找到终点。
//
// 【在本项目里的角色】
//   它不负责生成最终轨迹，只负责“告诉 B 样条优化器往哪边绕开障碍”。
//   当轨迹中间有一段撞上障碍时，就用 A* 在这段的两端之间找一条绕行路线，
//   然后用这条路线去“顶开”B 样条的控制点。
//
// 【注意】虽然文件名叫 dyn_a_star（dynamic），但实现其实是普通的网格 A*，
//   没有速度/加速度状态，输出的是纯位置点列。
//
// 【建议阅读顺序】
//   先看 GridNode（搜索节点）→ 再看 AStar 类 → 最后看 dyn_a_star.cpp 的 AstarSearch
// ============================================================================
#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>

// 注意：这里本意是“无穷大”，但 1 >> 20 在 C++ 里等于 0（这是个遗留小 bug）。
// 实际上每个节点的 gScore/fScore 在第一次被发现时就会被覆盖，所以影响不大。
constexpr double inf = 1 >> 20;
struct GridNode;
typedef GridNode *GridNodePtr;

// A* 的返回值：SUCCESS=成功找到路径，INIT_ERR=起点/终点有问题，SEARCH_ERR=没搜到。
enum ASTAR_RET
{
	SUCCESS,
	INIT_ERR,
	SEARCH_ERR
};

// 搜索用的“格子节点”：网格上的每一个格子，在搜索时就是一个 GridNode。
//   index    这个格子的整数坐标 (x,y,z)
//   gScore   从起点走到这里的实际花费
//   fScore   预估总花费 = gScore + 到终点的估计距离（用来决定先扩展谁）
//   cameFrom 我是从哪个格子走过来的（用于最后倒推出整条路径）
struct GridNode
{
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};

// 每调用一次搜索就递增，用来区分“这一次搜索”和“上一次搜索”，
// 避免每次搜索都把所有节点重置一遍（省时间的小技巧）。
	int rounds{0}; // Distinguish every call
	enum enum_state state
	{
		UNDEFINED
	};
// 这个格子在网格里的整数坐标
	Eigen::Vector3i index;

	double gScore{inf}, fScore{inf};
	GridNodePtr cameFrom{NULL};
};

// 优先队列的比较器：让 fScore 小的节点排在前面（先被取出、先被扩展）。
class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

// A* 算法本体。
class AStar
{
private:
	GridMap::Ptr grid_map_;

	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

// 三种“到终点的估计距离”写法。实际只用 getDiagHeu（对角距离），另两个没用上。
	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline int checkOccupancy(const Eigen::Vector3d &pos, const double yaw) { return grid_map_->getInflateOccupancy(pos, yaw); }

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	double step_size_, inv_step_size_;
	Eigen::Vector3d center_;
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> gridPath_;

	GridNodePtr ***GridNodeMap_;
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	int rounds_{0};

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

// 初始化：关联地图，并分配一块 pool_size 大小的搜索空间（本项目是 100x100x100 格）。
	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

// 【核心】开始搜索。step_size 是格子边长（这里直接用地图像素分辨率），
// start_pt / end_pt 是世界坐标里的起点和终点。
	ASTAR_RET AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

// 取出上一步搜到的路径（一串世界坐标点，从起点排到终点）。
	std::vector<Eigen::Vector3d> getPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		RCLCPP_ERROR(rclcpp::get_logger("path_searching"),
		             "Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}

	return true;
};

#endif
