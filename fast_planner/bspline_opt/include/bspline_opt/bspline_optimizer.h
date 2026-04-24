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



#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <plan_env/edt_environment.h>
#include <ros/ros.h>

/**
 * @brief B样条轨迹优化器
 *
 * 使用梯度下降和弹性带(elastic band)方法优化B样条轨迹
 *
 * 输入: 符号距离场(SDF)和一系列路径点
 * 输出: 优化后的路径点序列
 * 路径点格式: N x 3 矩阵，每行是一个3D点
 *
 * 优化目标:
 * - 平滑度(Smoothness): 最小化加加速度(jerk)
 * - 距离(Distance): 保持与障碍物的安全距离
 * - 可行性(Feasibility): 满足机器人的动力学约束
 * - 端点(Endpoint): 到达目标点
 * - 引导路径(Guide): 跟随参考路径
 * - 路标点(Waypoints): 经过指定的路标点
 * - 可见性(Visibility): 保持良好的视野
 *
 * 优化方法:
 * - GUIDE_PHASE: 引导阶段，更依赖引导路径
 * - NORMAL_PHASE: 正常阶段，平衡各项代价
 */
namespace fast_planner {
class BsplineOptimizer {

public:
  /** @brief 代价函数类型常量 */
  static const int SMOOTHNESS;    /**< @brief 平滑度代价 (jerk) */
  static const int DISTANCE;      /**< @brief 距离代价 (障碍物避让) */
  static const int FEASIBILITY;   /**< @brief 可行性代价 (动力学约束) */
  static const int ENDPOINT;      /**< @brief 端点代价 (目标到达) */
  static const int GUIDE;         /**< @brief 引导路径代价 */
  static const int WAYPOINTS;     /**< @brief 路标点代价 */

  /** @brief 优化阶段常量 */
  static const int GUIDE_PHASE;   /**< @brief 引导阶段: 优先跟随引导路径 */
  static const int NORMAL_PHASE;  /**< @brief 正常阶段: 平衡各项代价 */

  /**
   * @brief 默认构造函数
   */
  BsplineOptimizer() {}
  
  /**
   * @brief 析构函数
   */
  ~BsplineOptimizer() {}

  /* ==================== 主API ==================== */

  /**
   * @brief 设置环境地图
   * 用于获取障碍物距离信息
   * @param env EDTEnvironment智能指针
   */
  void setEnvironment(const EDTEnvironment::Ptr& env);

  /**
   * @brief 从ROS参数服务器加载优化参数
   * @param nh ROS节点句柄
   */
  void setParam(ros::NodeHandle& nh);

  /**
   * @brief B样条轨迹优化主函数
   *
   * @param points 初始控制点 (Nx3矩阵)
   * @param ts B样条节点间隔时间
   * @param cost_function 代价函数类型组合
   * @param max_num_id 最大迭代次数索引
   * @param max_time_id 最大优化时间索引
   * @return 优化后的控制点矩阵
   */
  Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                      const int& cost_function, int max_num_id, int max_time_id);

  /* ==================== 辅助函数 ==================== */

  /**
   * @brief 设置控制点 (必需输入)
   * @param points 控制点矩阵
   */
  void setControlPoints(const Eigen::MatrixXd& points);

  /**
   * @brief 设置B样条时间间隔 (必需输入)
   * @param ts 节点间隔时间
   */
  void setBsplineInterval(const double& ts);

  /**
   * @brief 设置代价函数类型 (必需输入)
   * @param cost_function 代价函数类型组合
   */
  void setCostFunction(const int& cost_function);

  /**
   * @brief 设置终止条件
   * @param max_num_id 最大迭代次数索引
   * @param max_time_id 最大优化时间索引
   */
  void setTerminateCond(const int& max_num_id, const int& max_time_id);

  /**
   * @brief 设置引导路径 (可选输入)
   * 提供一个几何参考路径引导优化方向
   * @param guide_pt 引导路径点序列
   */
  void setGuidePath(const vector<Eigen::Vector3d>& guide_pt);

  /**
   * @brief 设置路标点约束 (可选输入)
   * 轨迹必须经过指定的路标点
   * @param waypts 路标点序列
   * @param waypt_idx 路标点在控制点中的索引 (最多N-2个)
   */
  void setWaypoints(const vector<Eigen::Vector3d>& waypts,
                    const vector<int>&             waypt_idx);  // N-2 constraints at most

  /**
   * @brief 执行优化
   * 使用Ceres求解器进行优化
   */
  void optimize();

  /**
   * @brief 获取优化后的控制点
   * @return 控制点矩阵
   */
  Eigen::MatrixXd getControlPoints();

  /**
   * @brief 将矩阵转换为向量数组
   * @param ctrl_pts 控制点矩阵
   * @return vector形式的控制点
   */
  vector<Eigen::Vector3d> matrixToVectors(const Eigen::MatrixXd& ctrl_pts);

private:
  /** @brief 环境地图指针 */
  EDTEnvironment::Ptr edt_environment_;

  /* ==================== 主输入 ==================== */
  
  /** @brief B样条控制点矩阵 (N x dim) */
  Eigen::MatrixXd control_points_;
  /** @brief B样条节点间隔时间 */
  double          bspline_interval_;
  /** @brief 轨迹终点 */
  Eigen::Vector3d end_pt_;
  /** @brief 空间维度 (2D或3D) */
  int             dim_;

  /** @brief 引导路径点 (N-6个) */
  vector<Eigen::Vector3d> guide_pts_;
  /** @brief 路标点 */
  vector<Eigen::Vector3d> waypoints_;
  /** @brief 路标点索引 */
  vector<int>             waypt_idx_;

  /** @brief 终止条件索引 */
  int    max_num_id_, max_time_id_;
  /** @brief 代价函数类型 */
  int    cost_function_;
  /** @brief 是否考虑动态障碍物 */
  bool   dynamic_;
  /** @brief 全局开始时间 (用于动态障碍物) */
  double start_time_;

  /* ==================== 优化参数 ==================== */
  
  /** @brief B样条阶数 */
  int    order_;
  /** @brief 平滑度权重 (jerk最小化) */
  double lambda1_;
  /** @brief 距离权重 (障碍物避让) */
  double lambda2_;
  /** @brief 可行性权重 (动力学约束) */
  double lambda3_;
  /** @brief 端点权重 */
  double lambda4_;
  /** @brief 引导路径权重 */
  double lambda5_;
  /** @brief 可见性权重 */
  double lambda6_;
  /** @brief 路标点权重 */
  double lambda7_;
  /** @brief 加速度平滑权重 */
  double lambda8_;
                                  
  /** @brief 安全距离阈值 */
  double dist0_;
  /** @brief 最大速度限制 */
  double max_vel_,
  /** @brief 最大加速度限制 */
         max_acc_;
  /** @brief 最小可见性阈值 */
  double visib_min_;
  /** @brief 权重归一化参数 */
  double wnl_;
  /** @brief 最小时间间隔 */
  double dlmin_;
                                  
  /** @brief 二次代价优化算法 */
  int    algorithm1_;
  /** @brief 一般代价优化算法 */
  int    algorithm2_;
  /** @brief 可用的最大迭代次数 */
  int    max_iteration_num_[4];
  /** @brief 可用的最大优化时间 */
  double max_iteration_time_[4];

  /* ==================== 中间变量 ==================== */
  
  /** @brief 梯度缓冲区 - 避免重复内存分配 */
  vector<Eigen::Vector3d> g_q_;            /**< @brief 总体梯度 */
  vector<Eigen::Vector3d> g_smoothness_;   /**< @brief 平滑度梯度 */
  vector<Eigen::Vector3d> g_distance_;     /**< @brief 距离梯度 */
  vector<Eigen::Vector3d> g_feasibility_; /**< @brief 可行性梯度 */
  vector<Eigen::Vector3d> g_endpoint_;    /**< @brief 端点梯度 */
  vector<Eigen::Vector3d> g_guide_;       /**< @brief 引导路径梯度 */
  vector<Eigen::Vector3d> g_waypoints_;   /**< @brief 路标点梯度 */

  /** @brief 优化变量数量 */
  int                 variable_num_;
  /** @brief 迭代次数 */
  int                 iter_num_;
  /** @brief 最佳变量值 */
  std::vector<double> best_variable_;
  /** @brief 最小代价 */
  double              min_cost_;

  /** @brief 障碍物/阻挡点 (用于计算可见性) */
  vector<Eigen::Vector3d> block_pts_;

  /* ==================== 代价函数 ==================== */
  
  /**
   * @brief Ceres优化问题的代价函数回调
   * @param x 优化变量数组
   * @param grad 梯度输出
   * @param func_data 用户数据
   * @return 代价值
   */
  static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);

  /**
   * @brief 合并各项代价函数
   * @param x 优化变量
   * @param grad 梯度输出
   * @param cost 代价输出
   */
  void combineCost(const std::vector<double>& x, vector<double>& grad, double& cost);

  /**
   * @brief 计算平滑度代价 (jerk)
   * @param q 控制点
   * @param cost 代价输出
   * @param gradient 梯度输出
   */
  void calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                          vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 计算距离代价 (障碍物避让)
   * @param q 控制点
   * @param cost 代价输出
   * @param gradient 梯度输出
   */
  void calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                        vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 计算可行性代价 (动力学约束)
   * @param q 控制点
   * @param cost 代价输出
   * @param gradient 梯度输出
   */
  void calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                           vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 计算端点代价
   * @param q 控制点
   * @param cost 代价输出
   * @param gradient 梯度输出
   */
  void calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                        vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 计算引导路径代价
   * @param q 控制点
   * @param cost 代价输出
   * @param gradient 梯度输出
   */
  void calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 计算可见性代价
   * @param q 控制点
   * @param cost 代价输出
   * @param gradient 梯度输出
   */
  void calcVisibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                          vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 计算路标点代价
   * @param q 控制点
   * @param cost 代价输出
   * @param gradient 梯度输出
   */
  void calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                         vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 计算视角代价
   * @param q 控制点
   * @param cost 代价输出
   * @param gradient 梯度输出
   */
  void calcViewCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 判断是否为二次代价函数
   * 二次代价可以使用更高效的优化算法
   * @return true表示所有代价都是二次的
   */
  bool isQuadratic();

  /* ==================== 性能评估 ==================== */
public:
  /** @brief 代价历史记录 */
  vector<double> vec_cost_;
  /** @brief 时间历史记录 */
  vector<double> vec_time_;
  /** @brief 优化开始时间 */
  ros::Time      time_start_;

  /**
   * @brief 获取代价曲线
   * @param cost 代价历史
   * @param time 时间历史
   */
  void getCostCurve(vector<double>& cost, vector<double>& time) {
    cost = vec_cost_;
    time = vec_time_;
  }

  /** @brief 智能指针类型定义 */
  typedef unique_ptr<BsplineOptimizer> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace fast_planner
#endif