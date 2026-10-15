// ============================================================================
// 【第三方代码，非本项目编写】
//   来源：ZJU-FAST-Lab 的“快速视场检查”工具（配合 ikd-Tree 使用）。
//
// 【它解决什么问题】
//   传感器视野是一个“锥形”。要渲染点云时，如果遍历整张地图去找可见的点会很慢。
//   这个工具把空间切成体素方块，只判断“哪些方块与视锥相交”，
//   从而把需要检查的点缩小到很少一部分。
//
// 【本项目的用法】
//   非 360 度激光时，CPU 版渲染器用它筛出视场内的方块。
// ============================================================================
// Include Files
#pragma once
#include <math.h>
#include <cmath>
#include "ikd-Tree/ikd_Tree.h"
#include <Eigen/Core>
#include <algorithm>

// 数值容差
#define eps_value 1e-6

// 一个用 4 个点表示的面片
struct PlaneType{
    Eigen::Vector3d p[4];
};


// 【核心类】视场检查器：判断体素方块是否落在“锥形视野”内
class FOV_Checker{
public:
    FOV_Checker();
    ~FOV_Checker();
    // 设置整个环境的边界
    void Set_Env(BoxPointType env_param);
    // 设置体素方块的边长
    void Set_BoxLength(double box_len_param);
    // 【主接口】给定位姿/朝向/半视场角/射程，输出所有可见的体素方块
    void check_fov(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, vector<BoxPointType> &boxes);
    // 判断单个方块是否可见（内部会检查它的面与棱是否和视锥相交）
    bool check_box(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, const BoxPointType box);
    // 判断一条线段是否与视锥相交
    bool check_line(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, Eigen::Vector3d line_p, Eigen::Vector3d line_vec);
    // 判断一个面片是否与视锥相交
    bool check_surface(Eigen::Vector3d cur_pose, Eigen::Vector3d axis,  double theta, double depth, PlaneType plane);
    // 判断一个点是否在视锥内
    bool check_point(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, Eigen::Vector3d point);
    // 判断方块是否落在环境范围内
    bool check_box_in_env(BoxPointType box);    
private:
    BoxPointType env;
    double box_length;
    FILE *fp;

};
