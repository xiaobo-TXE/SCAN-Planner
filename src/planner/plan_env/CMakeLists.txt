# ============================================================================
# 文件：plan_env/CMakeLists.txt —— 构建脚本
#
# 【这个包会生成什么】
#   一个静态库 libplan_env（由 grid_map.cpp + raycast.cpp 编译而成），
#   供规划器、A*、B 样条优化器共同使用（它们都要查地图）。
#
# 【依赖】PCL（点云库）、OpenCV（图像处理）、cv_bridge、message_filters、
#          Eigen、tf2、以及各种 ROS 消息类型。
# ============================================================================
cmake_minimum_required(VERSION 3.8)
project(plan_env)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# 只编译两个源文件，打包成库 plan_env。
add_library(plan_env src/grid_map.cpp src/raycast.cpp)
target_include_directories(plan_env PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(plan_env
  cv_bridge geometry_msgs message_filters nav_msgs pcl_conversions rclcpp
  sensor_msgs tf2 tf2_ros visualization_msgs
)
target_link_libraries(plan_env Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS plan_env EXPORT export_plan_env
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_plan_env HAS_LIBRARY_TARGET)
ament_export_dependencies(
  cv_bridge Eigen3 geometry_msgs message_filters nav_msgs pcl_conversions
  rclcpp sensor_msgs tf2 tf2_ros visualization_msgs)
ament_package()
