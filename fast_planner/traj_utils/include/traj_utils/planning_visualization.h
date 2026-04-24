/**
 * @file planning_visualization.h
 * @brief 规划可视化工具头文件
 *
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

#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline/non_uniform_bspline.h>
#include <iostream>
#include <path_searching/topo_prm.h>
#include <plan_env/obj_predictor.h>
#include <poly_traj/polynomial_traj.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>

using std::vector;

/**
 * @brief fast_planner命名空间
 *
 * 包含Fast-Planner规划系统的所有类和函数
 */
namespace fast_planner {

/**
 * @brief PlanningVisualization类 - 规划可视化工具
 *
 * 提供ROS可视化功能，用于在RViz中显示：
 * - 轨迹
 * - 拓扑图和路径
 * - B样条曲线
 * - 多项式轨迹
 * - 目标点
 * - 障碍物预测
 * - 偏航角轨迹
 */
class PlanningVisualization {
private:
  /**
   * @brief 轨迹规划可视化ID枚举
   */
  enum TRAJECTORY_PLANNING_ID {
    GOAL = 1,              /**< 目标点 */
    PATH = 200,            /**< 路径 */
    BSPLINE = 300,         /**< B样条轨迹 */
    BSPLINE_CTRL_PT = 400, /**< B样条控制点 */
    POLY_TRAJ = 500        /**< 多项式轨迹 */
  };

  /**
   * @brief 拓扑路径规划可视化ID枚举
   */
  enum TOPOLOGICAL_PATH_PLANNING_ID {
    GRAPH_NODE = 1,       /**< 图节点 */
    GRAPH_EDGE = 100,     /**< 图边 */
    RAW_PATH = 200,       /**< 原始路径 */
    FILTERED_PATH = 300,  /**< 过滤后的路径 */
    SELECT_PATH = 400     /**< 选中的路径 */
  };

  /* ========== ROS发布者 ========== */
  ros::NodeHandle node;                    /**< ROS节点句柄 */
  ros::Publisher traj_pub_;               /**< 轨迹可视化发布者 */
  ros::Publisher topo_pub_;               /**< 拓扑路径可视化发布者 */
  ros::Publisher predict_pub_;            /**< 障碍物预测可视化发布者 */
  ros::Publisher visib_pub_;              /**< 可见性约束可视化发布者 */
  ros::Publisher frontier_pub_;            /**< 前沿搜索可视化发布者 */
  ros::Publisher yaw_pub_;                 /**< 偏航角轨迹可视化发布者 */
  vector<ros::Publisher> pubs_;           /**< 发布者列表 */

  /* ========== 状态追踪 ========== */
  int last_topo_path1_num_;               /**< 上次拓扑路径1数量 */
  int last_topo_path2_num_;               /**< 上次拓扑路径2数量 */
  int last_bspline_phase1_num_;           /**< 上次B样条阶段1数量 */
  int last_bspline_phase2_num_;           /**< 上次B样条阶段2数量 */
  int last_frontier_num_;                  /**< 上次前沿数量 */

public:
  PlanningVisualization(/* args */) {}
  ~PlanningVisualization() {}
  
  /**
   * @brief 构造函数
   * @param nh ROS节点句柄
   */
  PlanningVisualization(ros::NodeHandle& nh);

  /* ========== 基础图形绘制 ========== */

  /**
   * @brief 显示球体列表
   * @param list 3D点列表
   * @param resolution 球体大小
   * @param color RGBA颜色
   * @param id 可视化ID
   * @param pub_id 发布者ID（0-5对应不同话题）
   */
  void displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                         const Eigen::Vector4d& color, int id, int pub_id = 0);
                         
  /**
   * @brief 显示立方体列表
   * @param list 3D点列表
   * @param resolution 立方体大小
   * @param color RGBA颜色
   * @param id 可视化ID
   * @param pub_id 发布者ID
   */
  void displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                       const Eigen::Vector4d& color, int id, int pub_id = 0);
                       
  /**
   * @brief 显示线段列表（成对点）
   * @param list1 起点列表
   * @param list2 终点列表
   * @param line_width 线宽
   * @param color RGBA颜色
   * @param id 可视化ID
   * @param pub_id 发布者ID
   */
  void displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                       double line_width, const Eigen::Vector4d& color, int id, int pub_id = 0);

  /* ========== 路径绘制 ========== */

  /**
   * @brief 绘制几何路径（分段直线）
   * @param path 路径点列表
   * @param resolution 点的大小
   * @param color 颜色
   * @param id 可视化ID
   */
  void drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution,
                         const Eigen::Vector4d& color, int id = 0);

  /* ========== 轨迹绘制 ========== */

  /**
   * @brief 绘制多项式轨迹
   * @param poly_traj 多项式轨迹对象
   * @param resolution 点的大小
   * @param color 颜色
   * @param id 可视化ID
   */
  void drawPolynomialTraj(PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d& color,
                         int id = 0);

  /**
   * @brief 绘制B样条轨迹
   * @param bspline B样条对象
   * @param size 轨迹点大小
   * @param color 轨迹颜色
   * @param show_ctrl_pts 是否显示控制点
   * @param size2 控制点大小
   * @param color2 控制点颜色
   * @param id1 轨迹ID
   * @param id2 控制点ID
   */
  void drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color,
                   bool show_ctrl_pts = false, double size2 = 0.1,
                   const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0,
                   int id2 = 0);

  /**
   * @brief 绘制第一阶段B样条轨迹集合
   * @param bsplines B样条向量
   * @param size 点的大小
   */
  void drawBsplinesPhase1(vector<NonUniformBspline>& bsplines, double size);
  
  /**
   * @brief 绘制第二阶段B样条轨迹集合
   * @param bsplines B样条向量
   * @param size 点的大小
   */
  void drawBsplinesPhase2(vector<NonUniformBspline>& bsplines, double size);

  /* ========== 拓扑图绘制 ========== */

  /**
   * @brief 绘制拓扑图
   * @param graph 图节点列表
   * @param point_size 节点大小
   * @param line_width 边线宽
   * @param color1 节点颜色
   * @param color2 边颜色
   * @param color3 选中路径颜色
   * @param id 可视化ID
   */
  void drawTopoGraph(list<GraphNode::Ptr>& graph, double point_size, double line_width,
                     const Eigen::Vector4d& color1, const Eigen::Vector4d& color2,
                     const Eigen::Vector4d& color3, int id = 0);

  /**
   * @brief 绘制第一阶段拓扑路径
   * @param paths 路径集合
   * @param line_width 线宽
   */
  void drawTopoPathsPhase1(vector<vector<Eigen::Vector3d>>& paths, double line_width);
  
  /**
   * @brief 绘制第二阶段拓扑路径
   * @param paths 路径集合
   * @param line_width 线宽
   */
  void drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>>& paths, double line_width);

  /* ========== 其他可视化 ========== */

  /**
   * @brief 绘制目标点
   * @param goal 目标位置
   * @param resolution 目标点大小
   * @param color 颜色
   * @param id 可视化ID
   */
  void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);
  
  /**
   * @brief 绘制障碍物预测
   * @param pred 预测对象
   * @param resolution 大小
   * @param color 颜色
   * @param id 可视化ID
   */
  void drawPrediction(ObjPrediction pred, double resolution, const Eigen::Vector4d& color, int id = 0);

  /**
   * @brief 获取HSV颜色
   * @param h 色相(0-1)
   * @param alpha 透明度(0-1)
   * @return RGBA颜色向量
   */
  Eigen::Vector4d getColor(double h, double alpha = 1.0);

  /**
   * @brief 智能指针类型定义
   */
  typedef std::shared_ptr<PlanningVisualization> Ptr;

  /* ========== 偏航角可视化 ========== */

  /**
   * @brief 绘制偏航角轨迹
   * @param pos 位置B样条
   * @param yaw 偏航角B样条
   * @param dt 时间步长
   */
  void drawYawTraj(NonUniformBspline& pos, NonUniformBspline& yaw, const double& dt);
  
  /**
   * @brief 绘制偏航角路径
   * @param pos 位置B样条
   * @param yaw 偏航角列表
   * @param dt 时间步长
   */
  void drawYawPath(NonUniformBspline& pos, const vector<double>& yaw, const double& dt);
};
}  // namespace fast_planner
#endif