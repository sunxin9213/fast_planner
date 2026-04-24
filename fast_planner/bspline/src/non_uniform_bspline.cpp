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



#include "bspline/non_uniform_bspline.h"
#include <ros/ros.h>

namespace fast_planner {

/**
 * @brief 构造函数 - 使用控制点、阶数和时间间隔初始化均匀B样条
 *
 * @param points 控制点矩阵 (NxD, N为控制点数量, D为空间维度)
 * @param order B样条阶数 (p), 阶数为p表示使用p+1个基函数
 * @param interval 节点间隔时间 (δt)
 */
NonUniformBspline::NonUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                     const double& interval) {
  setUniformBspline(points, order, interval);
}

/**
 * @brief 析构函数
 * 释放B样条对象占用的资源
 */
NonUniformBspline::~NonUniformBspline() {}

/**
 * @brief 设置均匀B样条参数
 *
 * 初始化B样条的基本参数:
 * - 控制点: 直接赋值
 * - 阶数: 存储样条阶数
 * - 节点向量: 构造均匀分布的节点向量
 *
 * 节点向量构造规则:
 * - 前p+1个节点: [0, δt, 2δt, ..., pδt]
 * - 中间节点: 等间距δt递增
 * - 后p+1个节点: 等间距δt递增至终点
 *
 * @param points 控制点矩阵 (NxD)
 * @param order 样条阶数 p
 * @param interval 基础时间间隔 δt
 */
void NonUniformBspline::setUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                           const double& interval) {
  control_points_ = points;  // 设置控制点
  p_              = order;   // 设置样条阶数
  interval_       = interval; // 设置基础时间间隔

  // 计算n和m: n+1个控制点, m+1个节点, 满足 m = n + p + 1
  n_ = points.rows() - 1;
  m_ = n_ + p_ + 1;

  // 初始化节点向量为均匀分布
  u_ = Eigen::VectorXd::Zero(m_ + 1);
  for (int i = 0; i <= m_; ++i) {
    if (i <= p_) {
      // 前p+1个节点: 从负值开始 (允许参数u为负)
      u_(i) = double(-p_ + i) * interval_;
    } else if (i > p_ && i <= m_ - p_) {
      // 中间节点: 均匀递增
      u_(i) = u_(i - 1) + interval_;
    } else if (i > m_ - p_) {
      // 后p+1个节点: 继续均匀递增
      u_(i) = u_(i - 1) + interval_;
    }
  }
}

/**
 * @brief 设置节点向量
 * 允许用户自定义非均匀节点分布
 * @param knot 节点向量
 */
void NonUniformBspline::setKnot(const Eigen::VectorXd& knot) { this->u_ = knot; }

/**
 * @brief 获取节点向量
 * @return 当前使用的节点向量副本
 */
Eigen::VectorXd NonUniformBspline::getKnot() { return this->u_; }

/**
 * @brief 获取曲线的时间跨度
 *
 * 有效参数范围为 [u_p, u_{m-p}]
 * 在此范围内曲线有定义，超出范围则需要外推
 *
 * @param um 输出: 曲线起始参数 (u_p)
 * @param um_p 输出: 曲线结束参数 (u_{m-p})
 */
void NonUniformBspline::getTimeSpan(double& um, double& um_p) {
  um   = u_(p_);      // 有效参数范围下界
  um_p = u_(m_ - p_); // 有效参数范围上界
}

/**
 * @brief 获取控制点矩阵
 * @return 控制点矩阵副本
 */
Eigen::MatrixXd NonUniformBspline::getControlPoint() { return control_points_; }

/**
 * @brief 获取曲线的起点和终点
 * 使用De Boor算法在参数域边界求值
 * @return pair(起点坐标, 终点坐标)
 */
pair<Eigen::VectorXd, Eigen::VectorXd> NonUniformBspline::getHeadTailPts() {
  // 在有效参数范围的起点求值
  Eigen::VectorXd head = evaluateDeBoor(u_(p_));
  // 在有效参数范围的终点求值
  Eigen::VectorXd tail = evaluateDeBoor(u_(m_ - p_));
  return make_pair(head, tail);
}

/**
 * @brief 使用De Boor算法计算曲线上的点
 *
 * De Boor算法是B样条求值的标准稳定算法，类似于贝塞尔曲线的De Casteljau算法
 * 但效率更高，是B样条求值的主流方法
 *
 * 算法步骤:
 * 1. 确定参数u所在的节点区间 [u_k, u_{k+1}]
 * 2. 初始化: d_i = P_{k-p+i}, i=0,...,p (取p+1个控制点)
 * 3. 递归插值: 对r=1,...,p:
 *    d_i = (1-α_i) * d_{i-1} + α_i * d_i, 其中α_i = (u-u_{i+k-p})/(u_{i+k+1-r}-u_{i+k-p})
 * 4. 结果: d_p 即为曲线上的点
 *
 * @param u 参数值，范围应在 [u_p, u_{m-p}] 内
 * @return 曲线上的点坐标
 */
Eigen::VectorXd NonUniformBspline::evaluateDeBoor(const double& u) {
  // 将参数限制在有效范围内
  double ub = min(max(u_(p_), u), u_(m_ - p_));

  // 步骤1: 确定参数u所在的节点区间索引k
  // 找到最小的k使得 u_k+1 >= ub
  int k = p_;
  while (true) {
    if (u_(k + 1) >= ub) break;
    ++k;
  }

  // 步骤2: 初始化d向量，取相关控制点
  vector<Eigen::VectorXd> d;
  for (int i = 0; i <= p_; ++i) {
    d.push_back(control_points_.row(k - p_ + i));
  }

  // 步骤3: De Boor递归插值
  // 外层循环: 递归层数r
  for (int r = 1; r <= p_; ++r) {
    // 内层循环: 从后向前更新d向量
    for (int i = p_; i >= r; --i) {
      // 计算插值因子α
      double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
      // 线性插值计算新的d_i
      d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
    }
  }

  // 步骤4: 返回结果
  return d[p_];
}

/**
 * @brief 使用时间参数t计算曲线上的点
 *
 * 将时间t ∈ [0, duration] 映射到参数域 [u_p, u_{m-p}]
 * 便于直接使用时间进行轨迹求值
 *
 * @param t 时间参数，从0开始
 * @return 曲线上的点坐标
 */
Eigen::VectorXd NonUniformBspline::evaluateDeBoorT(const double& t) {
  // 时间t映射到参数u: u = t + u_p
  return evaluateDeBoor(t + u_(p_));
}

/**
 * @brief 获取导数曲线的控制点
 *
 * B样条的重要性质: 导数也是B样条
 * 导数曲线的阶数 = 原曲线阶数 - 1
 * 导数控制点计算公式: Q_i = p * (P_{i+1} - P_i) / (u_{i+p+1} - u_{i+1})
 *
 * @return 导数曲线的控制点矩阵
 */
Eigen::MatrixXd NonUniformBspline::getDerivativeControlPoints() {
  // 导数曲线的控制点数量 = 原控制点数量 - 1
  Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(control_points_.rows() - 1, control_points_.cols());
  for (int i = 0; i < ctp.rows(); ++i) {
    // Q_i = p * (P_{i+1} - P_i) / (u_{i+p+1} - u_{i+1})
    ctp.row(i) =
        p_ * (control_points_.row(i + 1) - control_points_.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
  }
  return ctp;
}

/**
 * @brief 获取B样条的导数曲线
 *
 * 导数曲线仍是B样条，阶数减1
 * 节点向量也需要相应调整: 去掉首尾各一个节点
 *
 * @return 导数曲线的NonUniformBspline对象
 */
NonUniformBspline NonUniformBspline::getDerivative() {
  // 获取导数控制点，阶数减1
  Eigen::MatrixXd   ctp = getDerivativeControlPoints();
  NonUniformBspline derivative(ctp, p_ - 1, interval_);

  /* 调整节点向量: 去掉首尾各一个节点 */
  // 原节点向量: u_0, u_1, ..., u_m
  // 导数节点: u_1, u_2, ..., u_{m-1}
  Eigen::VectorXd knot(u_.rows() - 2);
  knot = u_.segment(1, u_.rows() - 2);
  derivative.setKnot(knot);

  return derivative;
}

/**
 * @brief 获取基础时间间隔
 * @return 节点间隔 δt
 */
double NonUniformBspline::getInterval() { return interval_; }

/**
 * @brief 设置物理限制参数
 *
 * 设置速度和加速度的最大限制，用于轨迹可行性检查和时间调整
 *
 * @param vel 最大速度限制 (m/s)
 * @param acc 最大加速度限制 (m/s²)
 */
void NonUniformBspline::setPhysicalLimits(const double& vel, const double& acc) {
  limit_vel_   = vel;    // 最大速度限制
  limit_acc_   = acc;    // 最大加速度限制
  limit_ratio_ = 1.1;    // 时间调整比例上限 (最多增加10%)
}

/**
 * @brief 检查轨迹是否满足速度和加速度限制
 *
 * 通过计算控制点之间的速度和加速度来检查轨迹可行性
 * 速度计算: v_i = p * (P_{i+1} - P_i) / (u_{i+p+1} - u_{i+1})
 * 加速度计算: a_i = p*(p-1)*((P_{i+2}-P_{i+1})/Δu_2 - (P_{i+1}-P_i)/Δu_1) / Δu_mid
 *
 * @param show 是否打印详细的超限信息
 * @return true表示满足限制，false表示存在超限
 */
bool NonUniformBspline::checkFeasibility(bool show) {
  bool fea = true;  // 初始假设可行

  Eigen::MatrixXd P         = control_points_;  // 获取控制点矩阵
  int             dimension = control_points_.cols();  // 空间维度

  /* 步骤1: 检查速度可行性 */
  double max_vel = -1.0;
  for (int i = 0; i < P.rows() - 1; ++i) {
    // 计算相邻控制点之间的速度: v_i = p * ΔP / Δu
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    // 检查速度是否超过限制
    if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
        fabs(vel(2)) > limit_vel_ + 1e-4) {
      if (show)
        cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
      fea = false;  // 标记为不可行

      // 记录最大速度
      for (int j = 0; j < dimension; ++j) {
        max_vel = max(max_vel, fabs(vel(j)));
      }
    }
  }

  /* 步骤2: 检查加速度可行性 */
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {
    // 计算加速度: 使用二阶差分近似
    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    // 检查加速度是否超过限制
    if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
        fabs(acc(2)) > limit_acc_ + 1e-4) {
      if (show)
        cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
      fea = false;  // 标记为不可行

      // 记录最大加速度
      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)));
      }
    }
  }

  // 计算综合超限比例 (速度超限比例和加速度超限比例的最大值)
  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

  return fea;
}

/**
 * @brief 检查超出限制的比例
 * 计算轨迹超出物理限制的程度，用于判断需要多少时间调整
 * @return 超出限制的最大比例 (1.0表示刚好在限制内，>1.0表示超限)
 */
double NonUniformBspline::checkRatio() {
  Eigen::MatrixXd P         = control_points_;
  int             dimension = control_points_.cols();

  // 步骤1: 找最大速度
  double max_vel = -1.0;
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
    for (int j = 0; j < dimension; ++j) {
      max_vel = max(max_vel, fabs(vel(j)));
    }
  }
  
  // 步骤2: 找最大加速度
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));
    for (int j = 0; j < dimension; ++j) {
      max_acc = max(max_acc, fabs(acc(j)));
    }
  }
  
  // 计算综合超限比例
  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));
  
  // 如果超限比例过大，打印错误信息
  ROS_ERROR_COND(ratio > 2.0, "max vel: %lf, max acc: %lf.", max_vel, max_acc);

  return ratio;
}

/**
 * @brief 重新分配时间以满足速度和加速度约束
 * 对超出物理限制的段落自动增加时间
 * @param show 是否打印调整详细信息
 * @return true表示调整后满足限制，false表示仍需进一步调整
 */
bool NonUniformBspline::reallocateTime(bool show) {
  bool fea = true;  // 初始假设可行

  Eigen::MatrixXd P         = control_points_;
  int             dimension = control_points_.cols();

  double max_vel, max_acc;

  /* 步骤1: 处理速度超限 */
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
        fabs(vel(2)) > limit_vel_ + 1e-4) {
      fea = false;
      if (show) cout << "[Realloc]: Infeasible vel " << i << " :" << vel.transpose() << endl;

      // 计算需要的速度缩放比例
      max_vel = -1.0;
      for (int j = 0; j < dimension; ++j) {
        max_vel = max(max_vel, fabs(vel(j)));
      }

      double ratio = max_vel / limit_vel_ + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_;

      // 计算时间调整量
      double time_ori = u_(i + p_ + 1) - u_(i + 1);
      double time_new = ratio * time_ori;
      double delta_t  = time_new - time_ori;
      double t_inc    = delta_t / double(p_);

      // 调整该段内节点的节点值
      for (int j = i + 2; j <= i + p_ + 1; ++j) {
        u_(j) += double(j - i - 1) * t_inc;
      }
      // 调整后续所有节点
      for (int j = i + p_ + 2; j < u_.rows(); ++j) {
        u_(j) += delta_t;
      }
    }
  }

  /* 步骤2: 处理加速度超限 */
  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
        fabs(acc(2)) > limit_acc_ + 1e-4) {
      fea = false;
      if (show) cout << "[Realloc]: Infeasible acc " << i << " :" << acc.transpose() << endl;

      max_acc = -1.0;
      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)));
      }

      double ratio = sqrt(max_acc / limit_acc_) + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_;

      double time_ori = u_(i + p_ + 1) - u_(i + 2);
      double time_new = ratio * time_ori;
      double delta_t  = time_new - time_ori;
      double t_inc    = delta_t / double(p_ - 1);

      // 特殊处理起始段
      if (i == 1 || i == 2) {
        for (int j = 2; j <= 5; ++j) {
          u_(j) += double(j - 1) * t_inc;
        }
        for (int j = 6; j < u_.rows(); ++j) {
          u_(j) += 4.0 * t_inc;
        }
      } else {
        // 普通段落
        for (int j = i + 3; j <= i + p_ + 1; ++j) {
          u_(j) += double(j - i - 2) * t_inc;
        }
        for (int j = i + p_ + 2; j < u_.rows(); ++j) {
          u_(j) += delta_t;
        }
      }
    }
  }

  return fea;
}

/**
 * @brief 按比例延长轨迹时间
 * 将轨迹的总时间按指定比例延长
 * @param ratio 时间延长比例 (例如1.2表示增加20%时间)
 */
void NonUniformBspline::lengthenTime(const double& ratio) {
  int num1 = 5;  // 起始节点索引
  int num2 = getKnot().rows() - 1 - 5;  // 结束节点索引

  double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
  double t_inc   = delta_t / double(num2 - num1);
  // 线性增加中间节点的时间
  for (int i = num1 + 1; i <= num2; ++i) u_(i) += double(i - num1) * t_inc;
  // 统一增加后续节点的时间
  for (int i = num2 + 1; i < u_.rows(); ++i) u_(i) += delta_t;
}

/**
 * @brief 重新计算初始化参数 (占位函数)
 */
void NonUniformBspline::recomputeInit() {}

/**
 * @brief 将一系列3D点参数化为B样条控制点
 *
 * 给定一组路径点和起止点的速度/加速度约束，
 * 计算满足这些约束的B样条控制点
 *
 * 算法: 构造线性方程组 Ax = b 求解控制点
 * - 内部点约束: 使用B样条基函数矩阵 (1/6)[1,4,1]
 * - 边界速度约束: (1/2ts)[-1,0,1]
 * - 边界加速度约束: (1/ts²)[1,-2,1]
 *
 * 方程组结构:
 * - 前K行: 内部路径点约束 (每个点由3个控制点决定)
 * - 第K行: 起点速度约束
 * - 第K+1行: 终点速度约束
 * - 第K+2行: 起点加速度约束
 * - 第K+3行: 终点加速度约束
 *
 * @param ts 时间步长
 * @param point_set 路径点集合 (K个点)
 * @param start_end_derivative 起终点导数 [起点速度, 起点加速度, 终点速度, 终点加速度]
 * @param ctrl_pts 输出: 计算得到的控制点 (K+2个)
 */
void NonUniformBspline::parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                              const vector<Eigen::Vector3d>& start_end_derivative,
                                              Eigen::MatrixXd&               ctrl_pts) {
  // 参数有效性检查
  if (ts <= 0) {
    cout << "[B-spline]:time step error." << endl;
    return;
  }

  if (point_set.size() < 2) {
    cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
    return;
  }

  if (start_end_derivative.size() != 4) {
    cout << "[B-spline]:derivatives error." << endl;
  }

  int K = point_set.size();

  /* 构造系数矩阵A (K+4) x (K+2) */
  // 内部点基函数系数: [1,4,1]/6 (来自三次B样条基函数)
  Eigen::Vector3d prow(3), vrow(3), arow(3);
  prow << 1, 4, 1;    // 位置基函数系数
  vrow << -1, 0, 1;  // 速度基函数系数
  arow << 1, -2, 1;  // 加速度基函数系数

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

  // 填充内部点约束 (前K行)
  for (int i = 0; i < K; ++i)
    A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

  // 填充速度约束 (第K和K+1行)
  A.block(K, 0, 1, 3)         = (1 / 2.0 / ts) * vrow.transpose();
  A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

  // 填充加速度约束 (第K+2和K+3行)
  A.block(K + 2, 0, 1, 3)     = (1 / ts / ts) * arow.transpose();
  A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();

  /* 构造右端向量b */
  // 内部点坐标
  Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
  for (int i = 0; i < K; ++i) {
    bx(i) = point_set[i](0);
    by(i) = point_set[i](1);
    bz(i) = point_set[i](2);
  }

  // 边界导数约束
  for (int i = 0; i < 4; ++i) {
    bx(K + i) = start_end_derivative[i](0);
    by(K + i) = start_end_derivative[i](1);
    bz(K + i) = start_end_derivative[i](2);
  }

  /* 求解线性方程组 Ax = b */
  Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
  Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
  Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

  /* 转换为控制点矩阵 */
  ctrl_pts.resize(K + 2, 3);
  ctrl_pts.col(0) = px;
  ctrl_pts.col(1) = py;
  ctrl_pts.col(2) = pz;
}

/**
 * @brief 获取轨迹总时间
 * 计算有效参数范围内的时间跨度
 * @return 时间跨度 duration = u_{m-p} - u_p
 */
double NonUniformBspline::getTimeSum() {
  double tm, tmp;
  getTimeSpan(tm, tmp);
  return tmp - tm;
}

/**
 * @brief 计算轨迹长度
 * 通过在曲线上采样求弧长近似
 * @param res 采样分辨率 (默认0.01秒)
 * @return 轨迹总长度 (米)
 */
double NonUniformBspline::getLength(const double& res) {
  double length = 0.0;  // 累计长度
  double dur    = getTimeSum();  // 获取总时长
  
  // 起点位置
  Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
  
  // 沿轨迹采样累加弧长
  for (double t = res; t <= dur + 1e-4; t += res) {
    p_n = evaluateDeBoorT(t);  // 获取当前时刻位置
    length += (p_n - p_l).norm();  // 累加弧长
    p_l = p_n;  // 更新起点
  }
  
  return length;
}

/**
 * @brief 计算加加速度(jerk)代价
 *
 * 加加速度是加速度的变化率，与轨迹的平滑度密切相关
 * 三次导数(加加速度)的积分:
 * J = ∫ ||j(t)||² dt = Σ (Δu_i * ||c_i||²)
 *
 * 其中c_i是三阶导数(加加速度)曲线的控制点
 *
 * @return jerk代价值
 */
double NonUniformBspline::getJerk() {
  // 获取三阶导数曲线 (加加速度曲线)
  NonUniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

  Eigen::VectorXd times     = jerk_traj.getKnot();      // 节点向量
  Eigen::MatrixXd ctrl_pts  = jerk_traj.getControlPoint();  // 控制点
  int             dimension = ctrl_pts.cols();

  // 计算jerk代价: J = Σ (Δu_i * ||c_i||²)
  double jerk = 0.0;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    for (int j = 0; j < dimension; ++j) {
      jerk += (times(i + 1) - times(i)) * ctrl_pts(i, j) * ctrl_pts(i, j);
    }
  }

  return jerk;
}

/**
 * @brief 获取平均和最大速度
 * 通过对速度曲线采样计算
 * @param mean_v 输出: 平均速度
 * @param max_v 输出: 最大速度
 */
/**
 * @brief 获取平均和最大速度
 * 通过对速度曲线采样计算
 * @param mean_v 输出: 平均速度
 * @param max_v 输出: 最大速度
 */
void NonUniformBspline::getMeanAndMaxVel(double& mean_v, double& max_v) {
  // 获取速度曲线 (一阶导数)
  NonUniformBspline vel = getDerivative();
  double            tm, tmp;
  vel.getTimeSpan(tm, tmp);

  double max_vel = -1.0, mean_vel = 0.0;
  int    num = 0;
  // 采样计算速度和最大速度
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
    double          vn  = vxd.norm();

    mean_vel += vn;
    ++num;
    if (vn > max_vel) {
      max_vel = vn;
    }
  }

  mean_vel = mean_vel / double(num);
  mean_v   = mean_vel;
  max_v    = max_vel;
}

/**
 * @brief 获取平均和最大加速度
 * 通过对加速度曲线采样计算
 * @param mean_a 输出: 平均加速度
 * @param max_a 输出: 最大加速度
 */
void NonUniformBspline::getMeanAndMaxAcc(double& mean_a, double& max_a) {
  // 获取加速度曲线 (二阶导数)
  NonUniformBspline acc = getDerivative().getDerivative();
  double            tm, tmp;
  acc.getTimeSpan(tm, tmp);

  double max_acc = -1.0, mean_acc = 0.0;
  int    num = 0;
  // 采样计算加速度和最大加速度
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd axd = acc.evaluateDeBoor(t);
    double          an  = axd.norm();

    mean_acc += an;
    ++num;
    if (an > max_acc) {
      max_acc = an;
    }
  }

  mean_acc = mean_acc / double(num);
  mean_a   = mean_acc;
  max_a    = max_acc;
}
}  // namespace fast_planner
