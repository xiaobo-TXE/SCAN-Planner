# ============================================================================
# 文件：src/simulator/Utils/a2_description/CMakeLists.txt —— 构建脚本
#
# 本包没有 C++ 代码，只安装模型、网格与 launch 等资源文件
# ============================================================================
cmake_minimum_required(VERSION 3.8)
project(a2_description)

find_package(ament_cmake REQUIRED)

foreach(directory launch meshes urdf)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${directory}")
    install(DIRECTORY ${directory} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
