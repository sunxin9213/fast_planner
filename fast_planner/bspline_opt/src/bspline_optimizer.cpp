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



#include "bspline_opt/bspline_optimizer.h"
#include <nlopt.hpp>
// using namespace std;

namespace fast_planner {

/**
 * @brief 代价函数类型常量定义
 * 使用位掩码表示，可组合使用
 */
const int BsplineOptimizer::SMOOTHNESS  = (1 << 0);  /**< @brief 平滑度代价 (bit 0) */
const int BsplineOptimizer::DISTANCE    = (1 << 1);  /**< @brief 距离代价 (bit 1) */
const int BsplineOptimizer::FEASIBILITY = (1 << 2);  /**< @brief 可行性代价 (bit 2) */
const int BsplineOptimizer::ENDPOINT    = (1 << 3);  /**< @brief 端点代价 (bit 3) */
const int BsplineOptimizer::GUIDE       = (1 << 4);  /**< @brief 引导路径代价 (bit 4) */
const int BsplineOptimizer::WAYPOINTS   = (1 << 6);  /**< @brief 路标点代价 (bit 6) */

/**
 * @brief 优化阶段常量定义
 * - GUIDE_PHASE: 引导阶段，主要使用平滑度和引导路径代价
 * - NORMAL_PHASE: 正常阶段，使用平滑度、距离和可行性代价
 */
const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
const int BsplineOptimizer::NORMAL_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;

/**
 * @brief 从ROS参数服务器加载优化参数
 *
 * 参数列表:
 * - lambda1-8: 各项代价的权重
 * - dist0: 安全距离阈值
 * - max_vel/max_acc: 速度和加速度限制
 * - visib_min: 可见性最小值
 * - max_iteration_num1-4: 最大迭代次数
 * - max_iteration_time1-4: 最大优化时间
 * - algorithm1/2: 优化算法选择
 * - order: B样条阶数
 *
 * @param nh ROS节点句柄
 */
void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
  // 代价权重参数
  nh.param("optimization/lambda1", lambda1_, -1.0);  // 平滑度权重
  nh.param("optimization/lambda2", lambda2_, -1.0);  // 距离权重
  nh.param("optimization/lambda3", lambda3_, -1.0);  // 可行性权重
  nh.param("optimization/lambda4", lambda4_, -1.0);  // 端点权重
  nh.param("optimization/lambda5", lambda5_, -1.0);  // 引导路径权重
  nh.param("optimization/lambda6", lambda6_, -1.0);  // 可见性权重
  nh.param("optimization/lambda7", lambda7_, -1.0);  // 路标点权重
  nh.param("optimization/lambda8", lambda8_, -1.0);  // 加速度平滑权重

  // 物理限制参数
  nh.param("optimization/dist0", dist0_, -1.0);     // 安全距离
  nh.param("optimization/max_vel", max_vel_, -1.0); // 最大速度
  nh.param("optimization/max_acc", max_acc_, -1.0); // 最大加速度
  nh.param("optimization/visib_min", visib_min_, -1.0); // 最小可见性
  nh.param("optimization/dlmin", dlmin_, -1.0);     // 最小时间间隔
  nh.param("optimization/wnl", wnl_, -1.0);        // 权重归一化

  // 终止条件参数
  nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
  nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
  nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
  nh.param("optimization/max_iteration_num4", max_iteration_num_[3], -1);
  nh.param("optimization/max_iteration_time1", max_iteration_time_[0], -1.0);
  nh.param("optimization/max_iteration_time2", max_iteration_time_[1], -1.0);
  nh.param("optimization/max_iteration_time3", max_iteration_time_[2], -1.0);
  nh.param("optimization/max_iteration_time4", max_iteration_time_[3], -1.0);

  // 算法参数
  nh.param("optimization/algorithm1", algorithm1_, -1); // 二次代价优化算法
  nh.param("optimization/algorithm2", algorithm2_, -1); // 一般代价优化算法
  nh.param("optimization/order", order_, -1);          // B样条阶数
}

/**
 * @brief 设置环境地图
 * 用于获取障碍物距离信息和梯度
 * @param env EDT环境指针
 */
void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

/**
 * @brief 设置控制点
 * @param points 控制点矩阵 (Nxdim)
 */
void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd& points) {
  control_points_ = points;
  dim_            = control_points_.cols();  // 空间维度
}

/**
 * @brief 设置B样条时间间隔
 * @param ts 节点间隔时间
 */
void BsplineOptimizer::setBsplineInterval(const double& ts) { bspline_interval_ = ts; }

/**
 * @brief 设置终止条件
 * @param max_num_id 最大迭代次数索引
 * @param max_time_id 最大优化时间索引
 */
void BsplineOptimizer::setTerminateCond(const int& max_num_id, const int& max_time_id) {
  max_num_id_  = max_num_id;
  max_time_id_ = max_time_id;
}

/**
 * @brief 设置代价函数类型
 * @param cost_code 代价函数类型掩码
 */
void BsplineOptimizer::setCostFunction(const int& cost_code) {
  cost_function_ = cost_code;

  // 打印使用的代价函数类型
  string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
  if (cost_function_ & DISTANCE) cost_str += " dist  |";
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
  if (cost_function_ & ENDPOINT) cost_str += " endpt |";
  if (cost_function_ & GUIDE) cost_str += " guide |";
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";

  ROS_INFO_STREAM("cost func: " << cost_str);
}

/**
 * @brief 设置引导路径
 * @param guide_pt 引导路径点序列
 */
void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) { guide_pts_ = guide_pt; }

/**
 * @brief 设置路标点约束
 * @param waypts 路标点序列
 * @param waypt_idx 路标点索引
 */
void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d>& waypts,
                                    const vector<int>&             waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

/**
 * @brief B样条轨迹优化主函数
 *
 * 完整流程:
 * 1. 设置控制点、时间间隔、代价函数、终止条件
 * 2. 调用optimize()执行优化
 * 3. 返回优化后的控制点
 *
 * @param points 初始控制点
 * @param ts B样条时间间隔
 * @param cost_function 代价函数类型
 * @param max_num_id 最大迭代次数索引
 * @param max_time_id 最大优化时间索引
 * @return 优化后的控制点矩阵
 */
Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                                      const int& cost_function, int max_num_id,
                                                      int max_time_id) {
  setControlPoints(points);
  setBsplineInterval(ts);
  setCostFunction(cost_function);
  setTerminateCond(max_num_id, max_time_id);

  optimize();
  return this->control_points_;
}

/**
 * @brief 执行B样条轨迹优化
 *
 * 使用NLopt库进行非线性优化:
 * 1. 初始化求解器和梯度缓冲区
 * 2. 确定优化变量数量
 * 3. 设置优化算法(根据是否为二次代价选择)
 * 4. 执行优化迭代
 * 5. 更新控制点
 */
void BsplineOptimizer::optimize() {
  /* 步骤1: 初始化求解器 */
  iter_num_        = 0;
  min_cost_        = std::numeric_limits<double>::max();  // 初始最小代价设为无穷大
  const int pt_num = control_points_.rows();  // 控制点数量
  
  // 调整梯度缓冲区大小
  g_q_.resize(pt_num);
  g_smoothness_.resize(pt_num);
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);

  /* 步骤2: 计算优化变量数量 */
  if (cost_function_ & ENDPOINT) {
    // 端点作为硬约束，优化变量不包括最后order_个控制点
    variable_num_ = dim_ * (pt_num - order_);
    // 计算终点位置(用于硬约束)
    end_pt_ = (1 / 6.0) *
        (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
         control_points_.row(pt_num - 1));
  } else {
    // 普通情况，优化中间的控制点
    variable_num_ = max(0, dim_ * (pt_num - 2 * order_)) ;
  }

  /* 步骤3: 配置NLopt求解器 */
  // 根据是否为二次代价选择优化算法
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);  // 设置代价函数
  opt.set_maxeval(max_iteration_num_[max_num_id_]);            // 最大迭代次数
  opt.set_maxtime(max_iteration_time_[max_time_id_]);          // 最大优化时间
  opt.set_xtol_rel(1e-5);                                     // 相对误差容忍度

  /* 步骤4: 初始化优化变量 */
  vector<double> q(variable_num_);
  for (int i = order_; i < pt_num; ++i) {
    // 跳过不需要优化的控制点
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - order_) + j] = control_points_(i, j);
    }
  }

  /* 步骤5: 设置变量边界 */
  if (dim_ != 1) {
    vector<double> lb(variable_num_), ub(variable_num_);
    const double   bound = 10.0;  // 边界范围
    for (int i = 0; i < variable_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  /* 步骤6: 执行优化 */
  try {
    double        final_cost;
    nlopt::result result = opt.optimize(q, final_cost);
  } catch (std::exception& e) {
    ROS_WARN("[Optimization]: nlopt exception");
    cout << e.what() << endl;
  }

  /* 步骤7: 更新控制点 */
  for (int i = order_; i < control_points_.rows(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
    }
  }

  if (!(cost_function_ & GUIDE)) ROS_INFO_STREAM("iter num: " << iter_num_);
}

/**
 * @brief 计算平滑度代价 (Jerk最小化)
 *
 * J = Σ ||jerk||²
 * jerk = q[i+3] - 3*q[i+2] + 3*q[i+1] - q[i]
 *
 * 梯度: dJ/dq[i] = -2*jerk, +6*jerk, -6*jerk, +2*jerk
 *
 * @param q 控制点序列
 * @param cost 代价值输出
 * @param gradient 梯度输出
 */
void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                                          vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  for (int i = 0; i < q.size() - order_; i++) {
    /* 计算加加速度 (jerk) */
    // 三次B样条的二阶差分: q[i+3] - 3*q[i+2] + 3*q[i+1] - q[i]
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    cost += jerk.squaredNorm();  // 累加平方范数
    
    temp_j = 2.0 * jerk;
    /* 计算梯度 (链式法则) */
    gradient[i + 0] += -temp_j;
    gradient[i + 1] += 3.0 * temp_j;
    gradient[i + 2] += -3.0 * temp_j;
    gradient[i + 3] += temp_j;
  }
}

/**
 * @brief 计算距离代价 (障碍物避让)
 *
 * 当控制点靠近障碍物时增加代价
 * 代价函数: max(0, dist0 - dist)²
 *
 * @param q 控制点序列
 * @param cost 代价值输出
 * @param gradient 梯度输出
 */
void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double          dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

  // 确定需要检查的控制点索引范围
  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    // 获取距离和梯度
    edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
    if (dist_grad.norm() > 1e-4) dist_grad.normalize();

    // 当距离小于安全距离时施加惩罚
    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);
      gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
  }
}

/**
 * @brief 计算可行性代价 (动力学约束)
 * 惩罚超出速度和加速度限制的情况
 *
 * @param q 控制点序列
 * @param cost 代价值输出
 * @param gradient 梯度输出
 */
/**
 * @brief 计算可行性代价 (动力学约束)
 *
 * 惩罚超出速度和加速度限制的情况:
 * - 速度代价: max(0, v² - vm²)²
 * - 加速度代价: max(0, a² - am²)²
 *
 * @param q 控制点序列
 * @param cost 代价值输出
 * @param gradient 梯度输出
 */
void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                                           vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  /* 预计算变量以提高效率 */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;  // 速度平方上限
  am2 = max_acc_ * max_acc_;  // 加速度平方上限

  ts      = bspline_interval_;
  ts_inv2 = 1 / ts / ts;  // 时间间隔平方的倒数
  ts_inv4 = ts_inv2 * ts_inv2;  // 时间间隔四次方的倒数

  /* 步骤1: 检查速度可行性 */
  for (int i = 0; i < q.size() - 1; i++) {
    // 计算速度: v = (q[i+1] - q[i]) / ts
    Eigen::Vector3d vi = q[i + 1] - q[i];

    for (int j = 0; j < 3; j++) {
      // 速度平方 / ts² - vm²
      double vd = vi(j) * vi(j) * ts_inv2 - vm2;
      if (vd > 0.0) {
        cost += pow(vd, 2);  // 惩罚超速

        // 梯度计算
        double temp_v = 4.0 * vd * ts_inv2;
        gradient[i + 0](j) += -temp_v * vi(j);
        gradient[i + 1](j) += temp_v * vi(j);
      }
    }
  }

  /* 步骤2: 检查加速度可行性 */
  for (int i = 0; i < q.size() - 2; i++) {
    // 加速度: a = (q[i+2] - 2*q[i+1] + q[i]) / ts²
    Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];

    for (int j = 0; j < 3; j++) {
      double ad = ai(j) * ai(j) * ts_inv4 - am2;
      if (ad > 0.0) {
        cost += pow(ad, 2);

        double temp_a = 4.0 * ad * ts_inv4;
        gradient[i + 0](j) += temp_a * ai(j);
        gradient[i + 1](j) += -2 * temp_a * ai(j);
        gradient[i + 2](j) += temp_a * ai(j);
      }
    }
  }
}

/**
 * @brief 计算端点代价
 * 惩罚轨迹终点与目标点的偏差
 * 使用B样条基函数计算终点位置
 *
 * @param q 控制点序列
 * @param cost 代价值输出
 * @param gradient 梯度输出
 */
void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // 获取最后三个控制点
  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  // 使用B样条基函数计算曲线终点: (1/6)*(q_3 + 4*q_2 + q_1)
  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
  cost += dq.squaredNorm();

  // 梯度计算 (链式法则)
  gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
}

/**
 * @brief 计算路标点代价
 * 惩罚轨迹经过路标点时的偏差
 *
 * @param q 控制点序列
 * @param cost 代价值输出
 * @param gradient 梯度输出
 */
void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                                         vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // 遍历所有路标点
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int             idx   = waypt_idx_[i];

    // 获取相关的三个控制点
    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    // 使用B样条基函数计算曲线上的点
    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    // 梯度计算
    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);
  }
}

/**
 * @brief 计算引导路径代价
 *
 * 使用几何路径上的采样点引导轨迹优化方向
 * 对每个待优化的控制点，分配一个引导点，惩罚它们之间的距离
 *
 * 代价函数: Σ ||q[i] - gpt[i]||²
 *
 * @param q 控制点序列
 * @param cost 代价值输出
 * @param gradient 梯度输出
 */
void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost,
                                     vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    // 获取对应的引导点
    Eigen::Vector3d gpt = guide_pts_[i - order_];
    // 累加距离平方代价
    cost += (q[i] - gpt).squaredNorm();
    // 梯度: 2*(q[i] - gpt)
    gradient[i] += 2 * (q[i] - gpt);
  }
}

/**
 * @brief 合并各项代价函数
 *
 * 根据cost_function_标志位计算各项代价并加权求和
 * 同时计算总梯度
 *
 * @param x 优化变量数组
 * @param grad 梯度输出
 * @param f_combine 总代价输出
 */
void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
  /* convert the NLopt format vector to control points. */

  // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control point
  // For 1D case, the second and third elements are zero, and similar for the 2D case.
  for (int i = 0; i < order_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i][j] = control_points_(i, j);
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i][j] = 0.0;
    }
  }

  for (int i = 0; i < variable_num_ / dim_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i + order_][j] = x[dim_ * i + j];
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i + order_][j] = 0.0;
    }
  }

  if (!(cost_function_ & ENDPOINT)) {
    for (int i = 0; i < order_; i++) {

      for (int j = 0; j < dim_; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] =
            control_points_(control_points_.rows() - order_ + i, j);
      }
      for (int j = dim_; j < 3; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
      }
    }
  }

  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);

  /*  evaluate costs and their gradient  */
  double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints;
  f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;

  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
    f_combine += lambda1_ * f_smoothness;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);
  }
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += lambda2_ * f_distance;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);
  }
  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
    f_combine += lambda3_ * f_feasibility;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);
  }
  if (cost_function_ & ENDPOINT) {
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
    f_combine += lambda4_ * f_endpoint;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);
  }
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += lambda5_ * f_guide;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);
  }
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += lambda7_ * f_waypoints;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);
  }
  /*  print cost  */
  // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
  //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
  //        << ", waypt: " << lambda7_ * f_waypoints << endl;
  // }

  // if (optimization_phase_ == SECOND_PHASE) {
  //  << ", smooth: " << lambda1_ * f_smoothness
  //  << " , dist:" << lambda2_ * f_distance
  //  << ", fea: " << lambda3_ * f_feasibility << endl;
  // << ", end: " << lambda4_ * f_endpoint
  // << ", guide: " << lambda5_ * f_guide
  // }
}

/**
 * @brief NLopt代价函数回调
 *
 * 作为NLopt求解器的目标函数
 *
 * @param x 优化变量
 * @param grad 梯度输出
 * @param func_data 用户数据(BsplineOptimizer指针)
 * @return 代价值
 */
double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  // 从用户数据获取优化器指针
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double            cost;
  
  // 计算合并后的代价和梯度
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* 保存最小代价结果 */
  if (cost < opt->min_cost_) {
    opt->min_cost_      = cost;
    opt->best_variable_ = x;
  }
  return cost;

  // /* evaluation */
  // ros::Time te1 = ros::Time::now();
  // double time_now = (te1 - opt->time_start_).toSec();
  // opt->vec_time_.push_back(time_now);
  // if (opt->vec_cost_.size() == 0)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else if (opt->vec_cost_.back() > f_combine)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else
  // {
  //   opt->vec_cost_.push_back(opt->vec_cost_.back());
  // }
}

/**
 * @brief 将矩阵转换为向量数组
 * 将Nx3的控制点矩阵转换为vector<Eigen::Vector3d>
 * @param ctrl_pts 控制点矩阵
 * @return 向量形式控制点
 */
vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

/**
 * @brief 获取优化后的控制点
 * @return 控制点矩阵
 */
Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

/**
 * @brief 判断是否为二次代价函数
 *
 * 二次代价函数可以使用更高效的优化算法
 * 以下情况被认为是二次代价:
 * - GUIDE_PHASE: 引导阶段代价
 * - SMOOTHNESS | WAYPOINTS: 平滑度+路标点代价
 *
 * @return true表示所有代价都是二次的
 */
bool BsplineOptimizer::isQuadratic() {
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}

}  // namespace fast_planner