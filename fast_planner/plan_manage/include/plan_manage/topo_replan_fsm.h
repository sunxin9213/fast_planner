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



#ifndef _TOPO_REPLAN_FSM_H_
#define _TOPO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace fast_planner {

/**
 * @brief TopoReplanFSM类 - 拓扑重规划有限状态机
 *
 * 基于拓扑的轨迹重规划有限状态机。
 * 与KinoReplanFSM的区别：
 * - 使用Topology PRM搜索多条拓扑等价的路径
 * - 支持主动探索(Active Mapping)模式
 * - 使用前沿(frontier)信息指导探索
 *
 * 状态机状态：
 * - INIT: 系统初始化
 * - WAIT_TARGET: 等待目标点
 * - GEN_NEW_TRAJ: 生成新轨迹
 * - REPLAN_TRAJ: 重新规划轨迹
 * - EXEC_TRAJ: 执行轨迹
 * - REPLAN_NEW: 新目标重规划
 *
 * 特点：
 * - 主动地图更新(act_map_模式)
 * - 拓扑路径多样性
 * - 基于前沿的探索策略
 */
class TopoReplanFSM {
private:
  /* ---------- 状态机状态枚举 ---------- */
  
  /**
   * @brief FSM执行状态枚举
   * - INIT: 初始化状态
   * - WAIT_TARGET: 等待目标点
   * - GEN_NEW_TRAJ: 生成新轨迹
   * - REPLAN_TRAJ: 重新规划轨迹
   * - EXEC_TRAJ: 执行轨迹
   * - REPLAN_NEW: 新目标重规划
   */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  
  /**
   * @brief 目标类型枚举
   * - MANUAL_TARGET: 手动选择目标
   * - PRESET_TARGET: 预设目标点
   * - REFENCE_PATH: 参考路径
   */
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* 规划工具 */
  
  /// @brief 规划管理器
  FastPlannerManager::Ptr planner_manager_;
  
  /// @brief 可视化工具
  PlanningVisualization::Ptr visualization_;

  /* 参数 */
  
  /// @brief 目标类型
  int target_type_;  // 1 手动选择, 2 硬编码
  
  /// @brief 重规划阈值
  double replan_distance_threshold_;  // 距离阈值
  double replan_time_threshold_;       // 时间阈值
  
  double waypoints_[50][3];  // 预设路点
  int waypoint_num_;         // 路点数量
  
  /// @brief 是否启用主动地图更新
  bool act_map_;

  /* 规划数据 */
  
  bool trigger_;      // 触发标志
  bool have_target_;  // 是否有目标
  bool have_odom_;    // 是否有里程计
  bool collide_;      // 是否发生碰撞
  FSM_EXEC_STATE exec_state_;  // 当前状态

  /// @brief 里程计状态
  Eigen::Vector3d odom_pos_, odom_vel_;  // 位置和速度
  Eigen::Quaterniond odom_orient_;       // 姿态

  /// @brief 规划状态
  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // 起始状态
  Eigen::Vector3d target_point_, end_vel_;                        // 目标状态
  int current_wp_;  // 当前路点索引

  /* ROS工具 */
  
  ros::NodeHandle node_;
  
  /// @brief 定时器
  /// - exec_timer_: 执行状态机
  /// - safety_timer_: 安全检查
  /// - vis_timer_: 可视化
  /// - frontier_timer_: 前沿更新（主动地图）
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  
  ros::Subscriber waypoint_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  /* 辅助函数 */
  
  /**
   * @brief 调用搜索和优化
   * @return 是否成功
   * @details 前端+后端方法
   */
  bool callSearchAndOptimization();
  
  /**
   * @brief 调用拓扑轨迹生成
   * @param step 步骤：1新轨迹，2重规划
   * @return 是否成功
   * @details 拓扑路径引导的梯度优化
   */
  bool callTopologicalTraj(int step);
  
  /**
   * @brief 改变FSM状态
   */
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  
  /// @brief 打印状态
  void printFSMExecState();

  /* ROS回调函数 */
  
  /// @brief FSM执行回调
  void execFSMCallback(const ros::TimerEvent& e);
  
  /// @brief 碰撞检测回调
  void checkCollisionCallback(const ros::TimerEvent& e);
  
  /// @brief 路点回调
  void waypointCallback(const nav_msgs::PathConstPtr& msg);
  
  /// @brief 里程计回调
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

public:
  TopoReplanFSM(/* args */) {}
  ~TopoReplanFSM() {}

  /**
   * @brief 初始化
   */
  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif