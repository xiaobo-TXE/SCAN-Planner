# ============================================================================
# 文件：src/simulator/Utils/go2_description/CMakeLists.txt —— 构建脚本
#
# 本包没有 C++ 代码，只安装模型、配置与 launch 等资源文件
# ============================================================================
cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

foreach(directory config dae launch meshes urdf xacro worlds)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${directory}")
    install(DIRECTORY ${directory} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
