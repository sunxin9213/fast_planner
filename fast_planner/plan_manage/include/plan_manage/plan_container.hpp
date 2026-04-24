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



#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <bspline/non_uniform_bspline.h>
#include <poly_traj/polynomial_traj.h>
#include <path_searching/topo_prm.h>

using std::vector;

namespace fast_planner {

/**
 * @brief GlobalTrajData类 - 全局轨迹数据结构
 *
 * 存储和管理全局+局部轨迹数据。
 *
 * 数据结构说明：
 * - global_traj_: 全局多项式轨迹
 * - local_traj_: 局部B样条轨迹数组（位置、速度、加速度）
 *
 * 时间管理：
 * - 局部轨迹会更新全局轨迹的时间（time_increase_）
 * - 通过时间偏移实现全局和局部轨迹的无缝切换
 */
class GlobalTrajData {
private:
public:
  /// @brief 全局多项式轨迹
  PolynomialTraj global_traj_;
  
  /// @brief 局部B样条轨迹数组
  /// local_traj_[0]: 位置
  /// local_traj_[1]: 一阶导数（速度）
  /// local_traj_[2]: 二阶导数（加速度）
  vector<NonUniformBspline> local_traj_;

  /// @brief 全局轨迹总时长
  double global_duration_;
  
  /// @brief 全局轨迹开始时间（ROS时间）
  ros::Time global_start_time_;
  
  /// @brief 局部轨迹时间范围
  double local_start_time_, local_end_time_;
  
  /// @brief 时间增量（局部规划导致的时间调整）
  double time_increase_;
  
  /// @brief 上一次时间增量
  double last_time_inc_;

  GlobalTrajData(/* args */) {}

  ~GlobalTrajData() {}

  /// @brief 检查局部轨迹是否到达目标
  /// @return 是否到达
  bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

  /**
   * @brief 设置全局轨迹
   * @param traj 多项式轨迹
   * @param time 轨迹开始时间
   */
  void setGlobalTraj(const PolynomialTraj& traj, const ros::Time& time) {
    global_traj_ = traj;
    global_traj_.init();
    global_duration_ = global_traj_.getTimeSum();
    global_start_time_ = time;

    // 清空局部轨迹
    local_traj_.clear();
    local_start_time_ = -1;
    local_end_time_ = -1;
    time_increase_ = 0.0;
    last_time_inc_ = 0.0;
  }

  /**
   * @brief 设置局部轨迹
   * @param traj B样条轨迹
   * @param local_ts 局部轨迹开始时间
   * @param local_te 局部轨迹结束时间
   * @param time_inc 时间增量
   * @details
   * 1. 创建位置、速度、加速度三条B样条
   * 2. 更新全局时间参数
   */
  void setLocalTraj(NonUniformBspline traj, double local_ts, double local_te, double time_inc) {
    local_traj_.resize(3);
    local_traj_[0] = traj;
    local_traj_[1] = local_traj_[0].getDerivative();  // 速度
    local_traj_[2] = local_traj_[1].getDerivative();  // 加速度

    local_start_time_ = local_ts;
    local_end_time_ = local_te;
    global_duration_ += time_inc;
    time_increase_ += time_inc;
    last_time_inc_ = time_inc;
  }

  /**
   * @brief 获取任意时刻的位置
   * @param t 绝对时间
   * @return 3D位置
   * @details 根据时间判断使用全局还是局部轨迹
   * - t <= local_start_time: 使用全局轨迹（调整后）
   * - t >= local_end_time: 使用全局轨迹
   * - 其他: 使用局部轨迹
   */
  Eigen::Vector3d getPosition(double t) {
    if (t >= -1e-3 && t <= local_start_time_) {
      return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
    } else if (t >= local_end_time_ && t <= global_duration_ + 1e-3) {
      return global_traj_.evaluate(t - time_increase_);
    } else {
      double tm, tmp;
      local_traj_[0].getTimeSpan(tm, tmp);
      return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
    }
  }

  /// @brief 获取任意时刻的速度
  Eigen::Vector3d getVelocity(double t) {
    if (t >= -1e-3 && t <= local_start_time_) {
      return global_traj_.evaluateVel(t);
    } else if (t >= local_end_time_ && t <= global_duration_ + 1e-3) {
      return global_traj_.evaluateVel(t - time_increase_);
    } else {
      double tm, tmp;
      local_traj_[0].getTimeSpan(tm, tmp);
      return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
    }
  }

  /// @brief 获取任意时刻的加速度
  Eigen::Vector3d getAcceleration(double t) {
    if (t >= -1e-3 && t <= local_start_time_) {
      return global_traj_.evaluateAcc(t);
    } else if (t >= local_end_time_ && t <= global_duration_ + 1e-3) {
      return global_traj_.evaluateAcc(t - time_increase_);
    } else {
      double tm, tmp;
      local_traj_[0].getTimeSpan(tm, tmp);
      return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_);
    }
  }

  /// @brief 获取指定半径范围内的轨迹数据（B样条参数化）
  /// @param start_t 轨迹开始时间
  /// @param des_radius 搜索半径
  /// @param dist_pt 离散点间距
  /// @param point_set 输出的轨迹点集合
  /// @param start_end_derivative 起点/终点导数 [v0, v1, a0, a1]
  /// @param dt 时间步长
  /// @param seg_duration 轨迹段时长
  void getTrajByRadius(const double& start_t, const double& des_radius, const double& dist_pt,
                       vector<Eigen::Vector3d>& point_set, vector<Eigen::Vector3d>& start_end_derivative,
                       double& dt, double& seg_duration) {
    double seg_length = 0.0;  // length of the truncated segment
    double seg_time = 0.0;    // duration of the truncated segment
    double radius = 0.0;      // distance to the first point of the segment

    double delt = 0.2;
    Eigen::Vector3d first_pt = getPosition(start_t);  // first point of the segment
    Eigen::Vector3d prev_pt = first_pt;               // previous point
    Eigen::Vector3d cur_pt;                           // current point

    // go forward until the traj exceed radius or global time

    while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3) {
      seg_time += delt;
      seg_time = min(seg_time, global_duration_ - start_t);

      cur_pt = getPosition(start_t + seg_time);
      seg_length += (cur_pt - prev_pt).norm();
      prev_pt = cur_pt;
      radius = (cur_pt - first_pt).norm();
    }

    // get parameterization dt by desired density of points
    int seg_num = floor(seg_length / dist_pt);

    // get outputs

    seg_duration = seg_time;  // duration of the truncated segment
    dt = seg_time / seg_num;  // time difference between to points

    for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt) {
      cur_pt = getPosition(start_t + tp);
      point_set.push_back(cur_pt);
    }

    start_end_derivative.push_back(getVelocity(start_t));
    start_end_derivative.push_back(getVelocity(start_t + seg_time));
    start_end_derivative.push_back(getAcceleration(start_t));
    start_end_derivative.push_back(getAcceleration(start_t + seg_time));
  }

  /// @brief 获取固定时长的轨迹数据（B样条参数化）
  /// @param start_t 开始时间
  /// @param duration 时长
  /// @param seg_num 段数
  /// @param point_set 轨迹点输出
  /// @param start_end_derivative 起点/终点导数
  /// @param dt 时间步长
  void getTrajByDuration(double start_t, double duration, int seg_num,
                         vector<Eigen::Vector3d>& point_set,
                         vector<Eigen::Vector3d>& start_end_derivative, double& dt) {
    dt = duration / seg_num;
    Eigen::Vector3d cur_pt;
    for (double tp = 0.0; tp <= duration + 1e-4; tp += dt) {
      cur_pt = getPosition(start_t + tp);
      point_set.push_back(cur_pt);
    }

    start_end_derivative.push_back(getVelocity(start_t));
    start_end_derivative.push_back(getVelocity(start_t + duration));
    start_end_derivative.push_back(getAcceleration(start_t));
    start_end_derivative.push_back(getAcceleration(start_t + duration));
  }
};

/**
 * @brief PlanParameters结构体 - 规划参数
 *
 * 存储规划算法使用的所有参数，包括物理限制和计算时间统计
 */
struct PlanParameters {
  /* 规划算法参数 */
  
  /// @brief 物理限制
  double max_vel_;    // 最大速度 (m/s)
  double max_acc_;    // 最大加速度 (m/s²)
  double max_jerk_;   // 最大加加速度 (m/s³)
  
  double local_traj_len_;   // 局部重规划轨迹长度
  double ctrl_pt_dist;      // B样条控制点间距
  double clearance_;        // 间隙（安全距离）
  int dynamic_;            // 动态障碍物避让开关
  
  /* 处理时间统计 */
  double time_search_ = 0.0;   // 搜索时间
  double time_optimize_ = 0.0; // 优化时间
  double time_adjust_ = 0.0;   // 调整时间
};

struct LocalTrajData {
  /* info of generated traj */

  int traj_id_;
  double duration_;
  ros::Time start_time_;
  Eigen::Vector3d start_pos_;
  NonUniformBspline position_traj_, velocity_traj_, acceleration_traj_, yaw_traj_, yawdot_traj_,
      yawdotdot_traj_;
};

class MidPlanData {
public:
  MidPlanData(/* args */) {}
  ~MidPlanData() {}

  vector<Eigen::Vector3d> global_waypoints_;

  // initial trajectory segment
  NonUniformBspline initial_local_segment_;
  vector<Eigen::Vector3d> local_start_end_derivative_;

  // kinodynamic path
  vector<Eigen::Vector3d> kino_path_;

  // topological paths
  list<GraphNode::Ptr> topo_graph_;
  vector<vector<Eigen::Vector3d>> topo_paths_;
  vector<vector<Eigen::Vector3d>> topo_filtered_paths_;
  vector<vector<Eigen::Vector3d>> topo_select_paths_;

  // multiple topological trajectories
  vector<NonUniformBspline> topo_traj_pos1_;
  vector<NonUniformBspline> topo_traj_pos2_;
  vector<NonUniformBspline> refines_;

  // visibility constraint
  vector<Eigen::Vector3d> block_pts_;
  Eigen::MatrixXd ctrl_pts_;

  // heading planning
  vector<double> path_yaw_;
  double dt_yaw_;
  double dt_yaw_path_;

  void clearTopoPaths() {
    topo_traj_pos1_.clear();
    topo_traj_pos2_.clear();
    topo_graph_.clear();
    topo_paths_.clear();
    topo_filtered_paths_.clear();
    topo_select_paths_.clear();
  }

  void addTopoPaths(list<GraphNode::Ptr>& graph, vector<vector<Eigen::Vector3d>>& paths,
                    vector<vector<Eigen::Vector3d>>& filtered_paths,
                    vector<vector<Eigen::Vector3d>>& selected_paths) {
    topo_graph_ = graph;
    topo_paths_ = paths;
    topo_filtered_paths_ = filtered_paths;
    topo_select_paths_ = selected_paths;
  }
};

}  // namespace fast_planner

#endif