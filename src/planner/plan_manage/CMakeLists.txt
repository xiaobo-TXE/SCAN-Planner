# ============================================================================
# 文件：CMakeLists.txt —— 构建脚本（告诉编译器要编译出哪些程序）
#
# 【这个文件是干什么的】
#   colcon build 时会读这个文件，决定：
#     1. 需要依赖哪些其他包（find_package）
#     2. 要生成哪些可执行程序（add_executable）
#     3. 哪些文件要安装到 share 目录（install）
#
# 【本包会生成 5 个可执行程序】
#   scan_planner_node     规划器主程序（最重要）
#   closed_loop_controller 闭环控制器
#   open_loop_controller   开环控制器
#   go2_kinematic_sim      假机器人
#   go2_gait_publisher     步态动画
# ============================================================================
cmake_minimum_required(VERSION 3.8)
# 注意：包名叫 scan_planner，但目录名是 plan_manage（两者不一致是历史原因）。
# 所有 launch / config 里引用包名时都用 scan_planner。
project(scan_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

set(scan_planner_dependencies
  ament_index_cpp bspline_opt geometry_msgs nav_msgs path_searching plan_env
  rclcpp scan_planner_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
  traj_utils visualization_msgs)

add_executable(scan_planner_node
  src/scan_planner_node.cpp
  src/scan_replan_fsm.cpp
  src/planner_manager.cpp)
target_include_directories(scan_planner_node PRIVATE include)
ament_target_dependencies(scan_planner_node ${scan_planner_dependencies})
target_link_libraries(scan_planner_node Eigen3::Eigen)

# 下面用一个循环批量生成 4 个较小的可执行程序，它们的依赖是一样的。
foreach(target open_loop_controller closed_loop_controller go2_kinematic_sim go2_gait_publisher)
  add_executable(${target} src/${target}.cpp)
  target_include_directories(${target} PRIVATE include)
  ament_target_dependencies(${target}
    bspline_opt geometry_msgs nav_msgs rclcpp scan_planner_msgs sensor_msgs
    std_msgs tf2 tf2_geometry_msgs tf2_ros)
  target_link_libraries(${target} Eigen3::Eigen)
endforeach()

# 安装规则：可执行程序装到 lib/scan_planner，头文件装到 include，
# launch/config 装到 share/scan_planner（运行时靠它找参数文件和启动脚本）。
install(TARGETS
  scan_planner_node open_loop_controller closed_loop_controller
  go2_kinematic_sim go2_gait_publisher
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(FILES launch/default.rviz DESTINATION share/${PROJECT_NAME}/rviz OPTIONAL)
install(PROGRAMS
  scripts/keypoint_recorder.py
  scripts/reference_path_publisher.py
  DESTINATION lib/${PROJECT_NAME} OPTIONAL)

# 如果开启测试（colcon test），就注册两个“启动测试”。
# 它们只检查程序能不能正常启动，不检查规划结果对不对。
# 下面另外还有参考路径/参考坐标系相关的单元测试与启动测试。
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_planner_startup.py TIMEOUT 30)
  add_launch_test(test/test_waypoint_parameters.py TIMEOUT 30)

  # ---- 参考路径 / 参考坐标系相关的测试 ----
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # 两个 C++ 单测：直接测纯函数，不需要起节点
  ament_add_gtest(test_reference_path_utils test/test_reference_path_utils.cpp)
  target_include_directories(test_reference_path_utils PRIVATE include)
  ament_target_dependencies(test_reference_path_utils geometry_msgs nav_msgs tf2_geometry_msgs)
  target_link_libraries(test_reference_path_utils Eigen3::Eigen)
  ament_add_gtest(test_reference_frame_utils test/test_reference_frame_utils.cpp)
  target_include_directories(test_reference_frame_utils PRIVATE include)
  ament_target_dependencies(test_reference_frame_utils geometry_msgs)
  target_link_libraries(test_reference_frame_utils Eigen3::Eigen)

  # Python 单测临时关掉 pytest 插件自动加载，避免环境里的第三方插件干扰
  set(_pytest_disable_plugin_autoload "$ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD}")
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(
    test_reference_path_publisher_unit
    test/test_reference_path_publisher_unit.py
    PYTHON_EXECUTABLE /usr/bin/python3
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "${_pytest_disable_plugin_autoload}")

  # 两个启动测试：把发布器 / 规划器真正起起来，验证动态 TF 下的行为
  add_launch_test(test/test_reference_path_publisher_launch.py TIMEOUT 30)
  add_launch_test(test/test_dynamic_reference_tf_launch.py TIMEOUT 35)
endif()

ament_package()
