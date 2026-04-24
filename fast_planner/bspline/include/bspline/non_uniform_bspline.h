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



#ifndef _NON_UNIFORM_BSPLINE_H_
#define _NON_UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace fast_planner {

/**
 * @brief 非均匀B样条曲线类
 *
 * B样条是一种用于曲线拟合的参数多项式，与贝塞尔曲线相比具有局部支撑性，
 * 修改控制点只会影响曲线的一部分而非整体。本类实现了非均匀B样条，
 * 允许在参数域上有不均匀的节点分布，这比均匀B样条更灵活。
 *
 * 数学基础:
 * - B样条基函数: B_{i,p}(u) 通过Cox-de Boor递归公式定义
 * - 曲线表示: C(u) = Σ P_i * B_{i,p}(u), 其中P_i是控制点
 * - De Boor算法: 用于在给定参数u处求取曲线上点的稳定算法(类似De Casteljau)
 *
 * 主要特性:
 * - 支持任意维度的控制点(2D, 3D等)
 * - 支持任意阶数(degree)的B样条
 * - 提供可行性检查和时间重分配功能
 * - 可计算曲线长度、加加速度(jerk)等性能指标
 */
class NonUniformBspline {
private:
  /**
   * @brief 控制点矩阵
   * 每一行代表一个控制点，列数决定维度
   * 例如: 3D空间的N个控制点 -> Nx3矩阵
   */
  Eigen::MatrixXd control_points_;

  /**
   * @brief B样条参数
   * p_: 样条阶数(order/degree)，阶数为p表示使用p+1个基函数
   * n_: 控制点数量减1 (n+1个控制点)
   * m_: 节点数量减1 (m+1个节点，满足 m = n + p + 1)
   */
  int             p_, n_, m_;
  
  /**
   * @brief 节点向量(knot vector)
   * 定义参数域上的节点位置，非均匀B样条的节点可以是不等间距的
   * 节点向量必须满足: u_0 <= u_1 <= ... <= u_m
   */
  Eigen::VectorXd u_;
  
  /**
   * @brief 基础时间间隔
   * 均匀B样条中相邻节点间的基础距离δt
   */
  double          interval_;

  /**
   * @brief 获取导数的控制点
   * B样条的导数也是B样条，阶数减1
   * 导数控制点 Q_i = p * (P_{i+1} - P_i) / (u_{i+p+1} - u_{i+1})
   * @return 导数曲线的控制点矩阵
   */
  Eigen::MatrixXd getDerivativeControlPoints();

  /**
   * @brief 物理限制参数
   * limit_vel_: 最大速度限制 (m/s)
   * limit_acc_: 最大加速度限制 (m/s²)
   * limit_ratio_: 时间调整比例上限
   */
  double limit_vel_, limit_acc_, limit_ratio_;

public:
  /**
   * @brief 默认构造函数
   */
  NonUniformBspline() {}
  
  /**
   * @brief 构造均匀B样条
   * @param points 控制点矩阵
   * @param order 样条阶数
   * @param interval 节点间隔时间
   */
  NonUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);
  
  /**
   * @brief 析构函数
   */
  ~NonUniformBspline();

  /**
   * @brief 初始化为均匀B样条
   * 均匀B样条是非均匀B样条的特例，所有节点间距相等
   * @param points 控制点矩阵 (NxD, N为点数, D为维度)
   * @param order 样条阶数 (p)
   * @param interval 节点间隔时间 (δt)
   */
  void setUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);

  // ============ 基本信息获取/设置 ============

  /**
   * @brief 设置节点向量
   * @param knot 节点向量
   */
  void setKnot(const Eigen::VectorXd& knot);
  
  /**
   * @brief 获取节点向量
   * @return 节点向量副本
   */
  Eigen::VectorXd getKnot();
  
  /**
   * @brief 获取控制点
   * @return 控制点矩阵副本
   */
  Eigen::MatrixXd getControlPoint();
  
  /**
   * @brief 获取基础时间间隔
   * @return 节点间隔δt
   */
  double getInterval();
  
  /**
   * @brief 获取曲线的时间跨度
   * 有效参数范围为[u_p, u_{m-p}]
   * @param um 输出: 曲线起始参数
   * @param um_p 输出: 曲线结束参数
   */
  void getTimeSpan(double& um, double& um_p);
  
  /**
   * @brief 获取曲线的起点和终点
   * @return pair(起点坐标, 终点坐标)
   */
  pair<Eigen::VectorXd, Eigen::VectorXd> getHeadTailPts();

  // ============ 位置和导数计算 ============

  /**
   * @brief 使用De Boor算法计算曲线上的点
   * De Boor算法是B样条求值的稳定算法，类似于贝塞尔曲线的De Casteljau算法
   * 通过逐步插值计算在给定参数u处的曲线点
   * @param u 参数值，范围在[u_p, u_{m-p}]内
   * @return 曲线上的点坐标(向量)
   */
  Eigen::VectorXd evaluateDeBoor(const double& u);
  
  /**
   * @brief 使用时间参数t计算曲线上的点
   * 将时间t ∈ [0, duration] 映射到参数u并求值
   * @param t 时间参数，从0开始
   * @return 曲线上的点坐标
   */
  Eigen::VectorXd evaluateDeBoorT(const double& t);
  
  /**
   * @brief 获取B样条的导数曲线
   * 导数曲线仍是B样条，阶数减1
   * @return 导数曲线的NonUniformBspline对象
   */
  NonUniformBspline getDerivative();

  /**
   * @brief 将一系列3D点参数化为B样条控制点
   *
   * 给定一组路径点和起止点的速度/加速度约束，
   * 计算满足这些约束的B样条控制点
   *
   * 算法: 构造线性方程组 Ax = b 求解控制点
   * - 内部点: 使用B样条基函数矩阵 (1/6)[1,4,1]
   * - 边界速度: (1/2ts)[-1,0,1]
   * - 边界加速度: (1/ts²)[1,-2,1]
   *
   * @param ts 时间步长
   * @param point_set 路径点集合 (K+2个点，包含起点终点)
   * @param start_end_derivative 起终点导数 [起点速度, 起点加速度, 终点速度, 终点加速度]
   * @param ctrl_pts 输出: 计算得到的控制点 (K+6个)
   */
  static void parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                    const vector<Eigen::Vector3d>& start_end_derivative,
                                    Eigen::MatrixXd&               ctrl_pts);

  // ============ 可行性检查和时间调整 ============

  /**
   * @brief 设置物理限制
   * @param vel 最大速度限制 (m/s)
   * @param acc 最大加速度限制 (m/s²)
   */
  void setPhysicalLimits(const double& vel, const double& acc);
  
  /**
   * @brief 检查轨迹是否满足速度和加速度限制
   *
   * 速度计算: v_i = p * (P_{i+1} - P_i) / (u_{i+p+1} - u_{i+1})
   * 加速度计算: a_i = p*(p-1)*(...)/delta_u
   *
   * @param show 是否打印详细信息
   * @return true表示满足限制，false表示需要调整
   */
  bool checkFeasibility(bool show = false);
  
  /**
   * @brief 检查超出限制的比例
   * @return 超出限制的最大比例
   */
  double checkRatio();
  
  /**
   * @brief 按比例延长轨迹时间
   * @param ratio 时间延长比例 (例如1.2表示增加20%时间)
   */
  void lengthenTime(const double& ratio);
  
  /**
   * @brief 重新分配时间以满足约束
   *
   * 对超出限制的段落自动增加时间:
   * - 速度超限: 增加对应段落的节点间隔
   * - 加速度超限: 增加相关段落的节点间隔
   *
   * @param show 是否打印详细信息
   * @return true表示调整后满足限制
   */
  bool reallocateTime(bool show = false);

  // ============ 性能评估 ============

  /**
   * @brief 获取轨迹总时间
   * @return 时间跨度 duration
   */
  double getTimeSum();
  
  /**
   * @brief 计算轨迹长度
   * 通过在曲线上采样求弧长近似
   * @param res 采样分辨率 (默认0.01秒)
   * @return 轨迹总长度 (米)
   */
  double getLength(const double& res = 0.01);
  
  /**
   * @brief 计算加加速度(jerk)代价
   *
   * 加加速度是加速度的变化率，与轨迹的平滑度密切相关
   * J = ∫ ||j(t)||² dt = Σ (Δu_i * ||c_i||²)
   *
   * @return jerk代价值
   */
  double getJerk();
  
  /**
   * @brief 获取平均和最大速度
   * @param mean_v 输出: 平均速度
   * @param max_v 输出: 最大速度
   */
  void getMeanAndMaxVel(double& mean_v, double& max_v);
  
  /**
   * @brief 获取平均和最大加速度
   * @param mean_a 输出: 平均加速度
   * @param max_a 输出: 最大加速度
   */
  void getMeanAndMaxAcc(double& mean_a, double& max_a);

  /**
   * @brief 重新计算初始化参数 (占位函数)
   */
  void recomputeInit();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace fast_planner
#endif