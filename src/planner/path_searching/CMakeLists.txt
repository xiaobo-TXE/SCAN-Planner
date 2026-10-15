# ============================================================================
# 文件：path_searching/CMakeLists.txt —— 构建脚本
#
# 【这个包会生成什么】
#   一个静态库 libpath_searching（不是可执行程序），
#   供 bspline_opt 调用（A* 是优化器的辅助工具）。
#
# 【依赖】plan_env（要用地图查障碍）、rclcpp、Eigen
# ============================================================================
cmake_minimum_required(VERSION 3.8)
project(path_searching)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(plan_env REQUIRED)
find_package(rclcpp REQUIRED)

# 只编译一个源文件 dyn_a_star.cpp，打包成库 path_searching。
add_library(path_searching src/dyn_a_star.cpp)
target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(path_searching plan_env rclcpp)
target_link_libraries(path_searching Eigen3::Eigen)

# 安装库和头文件，并导出给别的包使用。
install(TARGETS path_searching EXPORT export_path_searching
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_path_searching HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 plan_env rclcpp)
ament_package()
