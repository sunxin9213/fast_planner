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



#ifndef _KINO_REPLAN_FSM_H_
#define _KINO_REPLAN_FSM_H_

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
 * @brief 测试类（调试用）
 */
class Test {
private:
  /* data */
  int test_;
  std::vector<int> test_vec_;
  ros::NodeHandle nh_;

public:
  Test(const int& v) {
    test_ = v;
  }
  Test(ros::NodeHandle& node) {
    nh_ = node;
  }
  ~Test() {
  }
  void print() {
    std::cout << "test: " << test_ << std::endl;
  }
};

/**
 * @brief KinoReplanFSM类 - 动力学重规划有限状态机
 *
 * 实现基于有限状态机(FSM)的无人机运动规划逻辑。
 * 状态机包含以下状态：
 * - INIT: 系统初始化
 * - WAIT_TARGET: 等待目标点
 * - GEN_NEW_TRAJ: 生成新轨迹
 * - REPLAN_TRAJ: 重新规划轨迹
 * - EXEC_TRAJ: 执行轨迹
 * - REPLAN_NEW: 新目标重规划
 *
 * 工作流程：
 * 1. 接收目标点（手动选择/预设/参考路径）
 * 2. 调用动力学重规划生成安全轨迹
 * 3. 周期性检查碰撞，如需要则重规划
 * 4. 发送B样条轨迹给控制器执行
 *
 * 触发重规划的条件：
 * - 轨迹执行时间超过阈值
 * - 无人机偏离轨迹超过阈值
 * - 检测到前方障碍物碰撞
 */
class KinoReplanFSM {

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
  double no_replan_thresh_;  // 不重规划的距离阈值
  double replan_thresh_;     // 重规划的距离阈值
  double waypoints_[50][3];  // 预设路点
  int waypoint_num_;         // 路点数量

  /* 规划数据 */
  
  bool trigger_;      // 触发标志
  bool have_target_;  // 是否有目标
  bool have_odom_;    // 是否有里程计数据
  FSM_EXEC_STATE exec_state_;  // 当前状态

  /// @brief 里程计状态
  Eigen::Vector3d odom_pos_, odom_vel_;  // 位置和速度
  Eigen::Quaterniond odom_orient_;       // 姿态（四元数）

  /// @brief 规划起始状态
  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // 位置、速度、加速度、偏航角
  
  /// @brief 目标状态
  Eigen::Vector3d end_pt_, end_vel_;  // 目标位置、速度
  
  /// @brief 当前路点索引
  int current_wp_;

  /* ROS工具 */
  
  ros::NodeHandle node_;  // ROS节点句柄
  
  /// @brief 定时器
  /// - exec_timer_: 执行状态机循环
  /// - safety_timer_: 安全检查（碰撞检测）
  /// - vis_timer_: 可视化
  /// - test_something_timer_: 测试用
  ros::Timer exec_timer_, safety_timer_, vis_timer_, test_something_timer_;
  
  /// @brief 订阅者
  ros::Subscriber waypoint_sub_, odom_sub_;
  
  /// @brief 发布者
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  /* 辅助函数 */
  
  /**
   * @brief 调用动力学重规划
   * @return 是否规划成功
   * @details 结合前端(Kinodynamic A*)和后端(B样条优化)的规划方法
   */
  bool callKinodynamicReplan();
  
  /**
   * @brief 调用拓扑轨迹生成
   * @param step 步骤标识：1新轨迹, 2重规划
   * @return 是否成功
   * @details 拓扑路径引导的梯度优化方法
   */
  bool callTopologicalTraj(int step);
  
  /**
   * @brief 改变FSM执行状态
   * @param new_state 新状态
   * @param pos_call 调用位置描述
   */
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  
  /// @brief 打印当前FSM状态
  void printFSMExecState();

  /* ROS回调函数 */
  
  /**
   * @brief FSM执行定时器回调
   * @param e 定时器事件
   * @details 周期性执行状态机逻辑
   */
  void execFSMCallback(const ros::TimerEvent& e);
  
  /**
   * @brief 碰撞检测定时器回调
   * @param e 定时器事件
   * @details 检查轨迹是否与障碍物碰撞
   */
  void checkCollisionCallback(const ros::TimerEvent& e);
  
  /**
   * @brief 路点话题回调
   * @param msg 路径消息
   * @details 接收目标路点
   */
  void waypointCallback(const nav_msgs::PathConstPtr& msg);
  
  /**
   * @brief 里程计话题回调
   * @param msg 里程计消息
   * @details 更新无人机当前位置和速度
   */
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

public:
  KinoReplanFSM(/* args */) {
  }
  ~KinoReplanFSM() {
  }

  /**
   * @brief 初始化有限状态机
   * @param nh ROS节点句柄
   */
  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif