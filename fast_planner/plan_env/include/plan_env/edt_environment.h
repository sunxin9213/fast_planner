/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <utility>

#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>

using std::cout;
using std::endl;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class EDTEnvironment {
private:
  /* data */
  ObjPrediction obj_prediction_;
  ObjScale obj_scale_;
  double resolution_inv_;
  /// @brief 计算点到轴对齐包围盒(AABB)的距离
  /// @param idx 障碍物索引
  /// @param pos 查询点位置
  /// @param time 当前时间(用于动态障碍物)
  /// @return 点到包围盒的最小距离
  /// @details 使用解析方法计算点到AABB的距离：如果点在盒内返回0，否则返回到最近面的距离
  double distToBox(int idx, const Eigen::Vector3d& pos, const double& time);
  
  /// @brief 计算点到所有动态障碍物包围盒的最小距离
  /// @param pos 查询点位置
  /// @param time 当前时间
  /// @return 到所有障碍物的最小距离
  /// @details 遍历所有障碍物，找到最近的障碍物距离
  double minDistToAllBox(const Eigen::Vector3d& pos, const double& time);

public:
  EDTEnvironment(/* args */) {
  }
  ~EDTEnvironment() {
  }

  /// @brief SDF地图智能指针，用于访问静态障碍物信息
  SDFMap::Ptr sdf_map_;

  /// @brief 初始化环境类
  void init();
  
  /// @brief 设置SDF地图
  /// @param map SDF地图智能指针
  void setMap(SDFMap::Ptr map);
  
  /// @brief 设置动态障碍物预测轨迹
  /// @param prediction 障碍物预测轨迹列表
  void setObjPrediction(ObjPrediction prediction);
  
  /// @brief 设置动态障碍物尺寸
  /// @param scale 障碍物尺寸列表
  void setObjScale(ObjScale scale);
  
  /// @brief 获取包围某点的8个体素角点的距离值
  /// @param pts 8个体素角点坐标 (2x2x2数组)
  /// @param dists 对应的距离值数组 (2x2x2数组)
  /// @details 用于三线性插值前的距离场采样
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
  
  /// @brief 三线性插值计算任意位置的距离和梯度
  /// @param values 8个体素角点的距离值
  /// @param diff 相对于角点的偏移量
  /// @param value 插值得到的位置距离
  /// @param grad 距离场梯度(指向最近障碍物)
  /// @details 使用三线性插值公式:
  /// f(x,y,z) = f(0,0,0)*(1-x)*(1-y)*(1-z) + f(1,0,0)*x*(1-y)*(1-z) + ...
  void interpolateTrilinear(double values[2][2][2], const Eigen::Vector3d& diff,
                            double& value, Eigen::Vector3d& grad);
  
  /// @brief 评估带梯度的欧几里得距离场
  /// @param pos 查询位置
  /// @param time 当前时间
  /// @param dist 输出的距离值
  /// @param grad 输出的梯度向量
  /// @details 综合考虑静态障碍物(SDF)和动态障碍物，返回最小距离和对应梯度
  void evaluateEDTWithGrad(const Eigen::Vector3d& pos, double time,
                           double& dist, Eigen::Vector3d& grad);
  
  /// @brief 快速评估距离场(仅距离，无梯度)
  /// @param pos 查询位置
  /// @param time 当前时间
  /// @return 到最近障碍物的距离
  /// @details 用于碰撞检测等需要快速判断的场景，比带梯度的版本更快
  double evaluateCoarseEDT(Eigen::Vector3d& pos, double time);
  
  /// @brief 获取地图区域信息
  /// @param ori 地图原点输出
  /// @param size 地图尺寸输出
  void getMapRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
    sdf_map_->getRegion(ori, size);
  }

  /// @brief 智能指针类型定义
  typedef shared_ptr<EDTEnvironment> Ptr;
};

}  // namespace fast_planner

#endif