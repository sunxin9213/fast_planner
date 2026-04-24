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



#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline_opt/bspline_optimizer.h>
#include <bspline/non_uniform_bspline.h>

#include <path_searching/astar.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/topo_prm.h>

#include <plan_env/edt_environment.h>

#include <plan_manage/plan_container.hpp>

#include <ros/ros.h>

namespace fast_planner {

/**
 * @brief FastPlannerManager类 - 快速规划器管理器
 *
 * Fast-Planner的核心管理类，负责协调各个规划模块：
 * 1. 地图模块(SDFMap): 提供环境表示
 * 2. 路径搜索模块(A*, Kinodynamic A*, Topology PRM): 前端路径搜索
 * 3. 轨迹优化模块(BsplineOptimizer): 后端轨迹优化
 *
 * 主要功能：
 * - 动力学重规划(kinodynamicReplan): 结合A*和B样条优化
 * - 全局轨迹规划(planGlobalTraj): 多目标点全局规划
 * - 拓扑重规划(topoReplan): 拓扑路径引导的轨迹优化
 * - 偏航角规划(planYaw): 规划无人机朝向
 *
 * @see Fast-Planner相关论文
 */
class FastPlannerManager {
  // SECTION 稳定功能
public:
  /// @brief 构造函数
  FastPlannerManager();
  
  /// @brief 析构函数
  ~FastPlannerManager();

  /* ===== 主要规划接口 ===== */

  /**
   * @brief 动力学重规划
   * @param start_pt 起点位置
   * @param start_vel 起点速度
   * @param start_acc 起点加速度
   * @param end_pt 终点位置
   * @param end_vel 终点速度
   * @return 是否规划成功
   * @details
   * 1. 使用Kinodynamic A*搜索初始路径（考虑动力学约束）
   * 2. 将路径转换为B样条曲线
   * 3. 使用梯度下降优化B样条（考虑时间最优、障碍物避让等）
   */
  bool kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                         Eigen::Vector3d end_pt, Eigen::Vector3d end_vel);
                         
  /**
   * @brief 全局轨迹规划
   * @param start_pos 起始位置
   * @return 是否规划成功
   * @details 使用A*搜索全局路径，然后优化为B样条轨迹
   */
  bool planGlobalTraj(const Eigen::Vector3d& start_pos);
  
  /**
   * @brief 拓扑重规划
   * @param collide 是否发生碰撞
   * @return 是否规划成功
   * @details
   * 1. 检测碰撞范围
   * 2. 使用Topology PRM搜索多条拓扑路径
   * 3. 选择最优路径进行优化
   */
  bool topoReplan(bool collide);

  /**
   * @brief 偏航角规划
   * @param start_yaw 起始偏航角
   * @details 基于轨迹切线方向平滑过渡偏航角
   */
  void planYaw(const Eigen::Vector3d& start_yaw);

  /**
   * @brief 初始化规划模块
   * @param nh ROS节点句柄
   * @details 创建所有子模块并初始化
   */
  void initPlanModules(ros::NodeHandle& nh);
  
  /**
   * @brief 设置全局路点
   * @param waypoints 路点列表
   */
  void setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints);

  /**
   * @brief 检查轨迹碰撞
   * @param distance 碰撞前距离输出
   * @return 是否发生碰撞
   */
  bool checkTrajCollision(double& distance);

  /* ===== 规划数据 ===== */
  
  /// @brief 规划参数
  PlanParameters pp_;
  
  /// @brief 局部轨迹数据
  LocalTrajData local_data_;
  
  /// @brief 全局轨迹数据
  GlobalTrajData global_data_;
  
  /// @brief 中间规划数据
  MidPlanData plan_data_;
  
  /// @brief 欧几里得距离场环境
  EDTEnvironment::Ptr edt_environment_;

private:
  /* 主要规划算法和模块 */
  
  /// @brief SDF地图
  SDFMap::Ptr sdf_map_;

  /// @brief 几何A*路径搜索器（用于全局规划）
  unique_ptr<Astar> geo_path_finder_;
  
  /// @brief 动力学A*路径搜索器（用于局部规划）
  unique_ptr<KinodynamicAstar> kino_path_finder_;
  
  /// @brief 拓扑PRM路径搜索器
  unique_ptr<TopologyPRM> topo_prm_;
  
  /// @brief B样条优化器列表（支持并行优化多条轨迹）
  vector<BsplineOptimizer::Ptr> bspline_optimizers_;

  /// @brief 更新轨迹信息
  void updateTrajInfo();

  /* 拓扑引导优化相关函数 */

  /**
   * @brief 查找碰撞范围
   * @param colli_start 碰撞起始点列表
   * @param colli_end 碰撞终止点列表
   * @param start_pts 轨迹起点列表
   * @param end_pts 轨迹终点列表
   * @details 分析碰撞位置，确定需要重规划的区域
   */
  void findCollisionRange(vector<Eigen::Vector3d>& colli_start, vector<Eigen::Vector3d>& colli_end,
                          vector<Eigen::Vector3d>& start_pts, vector<Eigen::Vector3d>& end_pts);

  /**
   * @brief 优化拓扑B样条轨迹
   * @param start_t 起始时间
   * @param duration 持续时间
   * @param guide_path 引导路径
   * @param traj_id 轨迹ID
   * @details 使用引导路径约束的梯度优化
   */
  void optimizeTopoBspline(double start_t, double duration, vector<Eigen::Vector3d> guide_path,
                           int traj_id);
                           
  /// @brief 重参数化局部轨迹（时间优化）
  Eigen::MatrixXd reparamLocalTraj(double start_t, double& dt, double& duration);
  Eigen::MatrixXd reparamLocalTraj(double start_t, double duration, int seg_num, double& dt);

  /// @brief 选择最优轨迹
  void selectBestTraj(NonUniformBspline& traj);
  
  /// @brief 细化轨迹
  void refineTraj(NonUniformBspline& best_traj, double& time_inc);
  
  /// @brief 重参数化B样条（调整控制点和时间分配）
  void reparamBspline(NonUniformBspline& bspline, double ratio, Eigen::MatrixXd& ctrl_pts, double& dt,
                      double& time_inc);

  // 偏航角规划
  /**
   * @brief 计算下一时刻偏航角
   * @param last_yaw 上一时刻偏航角
   * @param yaw 输出的下一时刻偏航角
   */
  void calcNextYaw(const double& last_yaw, double& yaw);

  // !SECTION 稳定功能

  // SECTION 开发中功能

public:
  /// @brief 智能指针类型定义
  typedef unique_ptr<FastPlannerManager> Ptr;

  // !SECTION 开发中
};
}  // namespace fast_planner

#endif