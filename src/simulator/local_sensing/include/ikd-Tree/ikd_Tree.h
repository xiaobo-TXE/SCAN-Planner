// ============================================================================
// 【第三方代码，非本项目编写】
//   来源：ikd-Tree（作者 Yixi Cai / Wei Xu / Fu Zhang，香港大学 MARS 实验室）
//   论文：“ikd-Tree: An Incremental KD Tree for Robotic Applications”
//
// 【它是什么（一句话）】
//   一棵支持“增量更新”的 KD 树：可以随时加点、删点、删一片区域，
//   而不必整棵树推倒重建，特别适合机器人一边走一边建图。
//
// 【本项目的用法】
//   FOV_Checker 用它来判断“哪些体素方块落在视场内”。
//   注意：传感器渲染的主流程并不依赖它（默认走哈希体素表）。
//
// 【阅读建议】完整移植的第三方库，了解接口即可，不必细读实现。
// ============================================================================
#pragma once
#include <pcl/point_types.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <stdio.h>
#include <queue>
#include <pthread.h>
#include <chrono>
#include <time.h>


// 算法参数：极小值、树不平衡阈值、多线程重建的触发点数、操作队列长度
#define EPSS 1e-6
#define Minimal_Unbalanced_Tree_Size 10
#define Multi_Thread_Rebuild_Point_Num 1500
#define DOWNSAMPLE_SWITCH true
#define ForceRebuildPercentage 0.2
#define Q_LEN 1000000

using namespace std;

// 本文件内部统一用 pcl::PointXYZI 作为点类型
typedef pcl::PointXYZI PointType;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;

const PointType ZeroP;

// KD 树的节点：除左右子树外，还带了很多“增量更新”用的辅助字段
struct KD_TREE_NODE
{
    // point：节点存的点；division_axis：按哪个轴划分
    PointType point;
    int division_axis;  
    // 子树大小与失效点数 —— 用来判断是否需要重建
    int TreeSize = 1;
    int invalid_point_num = 0;
    int down_del_num = 0;
    bool point_deleted = false;
    bool tree_deleted = false; 
    bool point_downsample_deleted = false;
    bool tree_downsample_deleted = false;
    bool need_push_down_to_left = false;
    bool need_push_down_to_right = false;
    bool working_flag = false;
    // 多线程环境下保护该节点的互斥锁
    pthread_mutex_t push_down_mutex_lock;
    // 以该节点为根的子树覆盖的包围盒（加速查询与重建）
    float node_range_x[2], node_range_y[2], node_range_z[2];   
    // 左右子节点与父节点（父指针用于向上回溯更新）
    KD_TREE_NODE *left_son_ptr = nullptr;
    KD_TREE_NODE *right_son_ptr = nullptr;
    KD_TREE_NODE *father_ptr = nullptr;
    // For paper data record
    float alpha_del;
    float alpha_bal;
};

// 带距离的点：最近邻搜索时按距离排序用
struct PointType_CMP{
    PointType point;
    float dist = 0.0;
    PointType_CMP (PointType p = ZeroP, float d = INFINITY){
        this->point = p;
        this->dist = d;
    };
    bool operator < (const PointType_CMP &a)const{
        if (fabs(dist - a.dist) < 1e-10) return point.x < a.point.x;
          else return dist < a.dist;
    }    
};

// 一个轴对齐的立方体盒子（记录顶点最小/最大坐标）
struct BoxPointType{
    float vertex_min[3];
    float vertex_max[3];
};

// 增量操作种类：加点 / 删点 / 删区域 / 加区域 / 降采样删除 / 下推
enum operation_set {ADD_POINT, DELETE_POINT, DELETE_BOX, ADD_BOX, DOWNSAMPLE_DELETE, PUSH_DOWN};

enum delete_point_storage_set {NOT_RECORD, DELETE_POINTS_REC, MULTI_THREAD_REC};

// 记录一次待执行的操作，供多线程重建时“回放”到新树上
struct Operation_Logger_Type{
    PointType point;
    BoxPointType boxpoint;
    bool tree_deleted, tree_downsample_deleted;
    operation_set op;
};

// 手写的环形队列（比 std::queue 快，用来缓存操作日志）
class MANUAL_Q{
    private:
        int head = 0,tail = 0, counter = 0;
        Operation_Logger_Type q[Q_LEN];
        bool is_empty;
    public:
        void pop();
        Operation_Logger_Type front();
        Operation_Logger_Type back();
        void clear();
        void push(Operation_Logger_Type op);
        bool empty();
        int size();
};

// 手写的最小堆（最近邻搜索的候选队列）
class MANUAL_HEAP
{
    public:
        MANUAL_HEAP(int max_capacity = 100);
        ~MANUAL_HEAP();
        void pop();
        PointType_CMP top();
        void push(PointType_CMP point);
        int size();
        void clear();
    private:
        PointType_CMP * heap;
        void MoveDown(int heap_index);
        void FloatUp(int heap_index);
        int heap_size = 0;
        int cap = 0;
};


// 【核心类】增量式 KD 树
class KD_TREE
{
private:
    // Multi-thread Tree Rebuild
    bool termination_flag = false;
    bool rebuild_flag = false;
    pthread_t rebuild_thread;
    pthread_mutex_t termination_flag_mutex_lock, rebuild_ptr_mutex_lock, working_flag_mutex, search_flag_mutex;
    pthread_mutex_t rebuild_logger_mutex_lock, points_deleted_rebuild_mutex_lock;
    // queue<Operation_Logger_Type> Rebuild_Logger;
    MANUAL_Q Rebuild_Logger;    
    PointVector Rebuild_PCL_Storage;
    KD_TREE_NODE ** Rebuild_Ptr;
    int search_mutex_counter = 0;
    static void * multi_thread_ptr(void *arg);
    void multi_thread_rebuild();
    void start_thread();
    void stop_thread();
    void run_operation(KD_TREE_NODE ** root, Operation_Logger_Type operation);
    // KD Tree Functions and augmented variables
    int Treesize_tmp = 0, Validnum_tmp = 0;
    float alpha_bal_tmp = 0.5, alpha_del_tmp = 0.0;
    float delete_criterion_param = 0.5f;
    float balance_criterion_param = 0.7f;
    float downsample_size = 0.2f;
    bool Delete_Storage_Disabled = false;
    KD_TREE_NODE * STATIC_ROOT_NODE = nullptr;
    PointVector Points_deleted;
    PointVector Downsample_Storage;
    PointVector Multithread_Points_deleted;
    void InitTreeNode(KD_TREE_NODE * root);
    void Test_Lock_States(KD_TREE_NODE *root);
    void BuildTree(KD_TREE_NODE ** root, int l, int r, PointVector & Storage);
    void Rebuild(KD_TREE_NODE ** root);
    int Delete_by_range(KD_TREE_NODE ** root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample);
    void Delete_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild);
    void Add_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild, int father_axis);
    void Add_by_range(KD_TREE_NODE ** root, BoxPointType boxpoint, bool allow_rebuild);
    void Search(KD_TREE_NODE * root, int k_nearest, PointType point, MANUAL_HEAP &q, double max_dist);//priority_queue<PointType_CMP>
    void Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector &Storage);
    bool Criterion_Check(KD_TREE_NODE * root);
    void Push_Down(KD_TREE_NODE * root);
    void Update(KD_TREE_NODE * root); 
    void delete_tree_nodes(KD_TREE_NODE ** root);
    void downsample(KD_TREE_NODE ** root);
    bool same_point(PointType a, PointType b);
    float calc_dist(PointType a, PointType b);
    float calc_box_dist(KD_TREE_NODE * node, PointType point);    
    static bool point_cmp_x(PointType a, PointType b); 
    static bool point_cmp_y(PointType a, PointType b); 
    static bool point_cmp_z(PointType a, PointType b); 
    void print_treenode(KD_TREE_NODE * root, int index, FILE *fp, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);

public:
    KD_TREE(float delete_param = 0.5, float balance_param = 0.6 , float box_length = 0.2);
    ~KD_TREE();
    void Set_delete_criterion_param(float delete_param);
    void Set_balance_criterion_param(float balance_param);
    void set_downsample_param(float box_length);
    void InitializeKDTree(float delete_param = 0.5, float balance_param = 0.7, float box_length = 0.2); 
    // ---------- 下面是给外部使用的接口 ----------
    // 树的规模、有效点数，以及根节点的平衡度/删除率指标
    int size();
    int validnum();
    void root_alpha(float &alpha_bal, float &alpha_del);
    // 用一批点重建整棵树
    void Build(PointVector point_cloud);
    // 最近邻搜索：找距离给定点最近的 k 个点
    void Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points, vector<float> & Point_Distance, double max_dist = INFINITY);
    // 增量加点（可以顺便做体素降采样）
    int Add_Points(PointVector & PointToAdd, bool downsample_on);
    // 把若干区域内的点重新加回来
    void Add_Point_Boxes(vector<BoxPointType> & BoxPoints);
    // 删除给定的点
    void Delete_Points(PointVector & PointToDel);
    // 删除若干区域内的所有点
    int Delete_Point_Boxes(vector<BoxPointType> & BoxPoints);
    // 把树里的点摊平成一个列表
    void flatten(KD_TREE_NODE * root, PointVector &Storage, delete_point_storage_set storage_type);
    // 取回被删除的点（供外部重建使用）
    void acquire_removed_points(PointVector & removed_points);
    void print_tree(int index, FILE *fp, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
    // 整棵树覆盖的包围盒
    BoxPointType tree_range();
    // 存放树中所有点的原始数组
    PointVector PCL_Storage;     
    KD_TREE_NODE * Root_Node = nullptr;
    int max_queue_size = 0;
};
