// ============================================================================
// 文件：src/planner/plan_manage/src/scan_planner_node.cpp
//
// 【这个文件是干什么的】
//   这是整个规划程序的“入口大门”（main 函数所在的地方）。
//   操作系统启动这个可执行程序时，会从 main() 开始执行。
//
// 【它按顺序做了这几件事】
//   1. 初始化 ROS 2（rclcpp::init）——相当于向 ROS 报到：“我启动了”。
//   2. 创建一个 ROS 2 节点，名字叫 scan_planner_node。
//      节点 = ROS 网络里的一个参与者，可以收发“话题(topic)”。
//   3. 创建一个状态机对象 SCANReplanFSM，并调用它的 init() 做全部准备工作
//      （读参数、订阅话题、创建定时器……全都在 init() 里完成）。
//   4. 用一个“单线程执行器”(SingleThreadedExecutor) 让程序持续运行：
//      它不断取出收到的消息和到点的定时器，依次执行对应的回调函数。
//
// 【为什么用“单线程”】
//   所有回调（定时器、消息）都在同一根线程上排队执行，不会真正同时跑，
//   所以不需要加锁；代价是某个回调太慢会拖住其他回调。
//
// 【想继续往下读，去哪里】
//   真正的逻辑几乎都在 SCANReplanFSM::init() 里，见：
//     include/plan_manage/scan_replan_fsm.h   （声明：有哪些函数/变量）
//     src/scan_replan_fsm.cpp                 （实现：函数体）
// ============================================================================

#include <memory>
#include <exception>

#include <rclcpp/rclcpp.hpp>
#include <plan_manage/scan_replan_fsm.h>

int main(int argc, char **argv)
{
  // 初始化 ROS 2：把命令行参数交给 ROS 解析（例如 --ros-args 等）。
  rclcpp::init(argc, argv);

  // 创建节点：make_shared 是 C++ 里“创建智能指针对象”的写法。
  // 节点名字 "scan_planner_node" 会被 ROS 用来给话题加前缀。
  auto node = std::make_shared<rclcpp::Node>("scan_planner_node");

  try
  {
    // 创建状态机（“大脑”），它持有规划器、地图、可视化等所有组件。
    scan_planner::SCANReplanFSM planner;

    // 做初始化：读 YAML 参数、订阅话题、创建定时器。出问题会抛异常。
    planner.init(node.get());

    // 单线程执行器：不断处理消息/定时器回调，相当于“主循环”。
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin(); // 一直循环，直到收到关闭信号（Ctrl+C 等）
  }
  catch (const std::exception &error)
  {
    // 只要初始化阶段出错，就打印致命日志并退出（返回非 0 表示失败）。
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize SCAN-Planner: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  // 正常退出。
  rclcpp::shutdown();
  return 0;
}
