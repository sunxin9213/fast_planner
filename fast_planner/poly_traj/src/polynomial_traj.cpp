/**
 * @file polynomial_traj.cpp
 * @brief 多项式轨迹实现 - 最小跳跃轨迹生成与快速直线轨迹
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

#include <iostream>
#include <poly_traj/polynomial_traj.h>

/**
 * @brief minSnapTraj - 最小跳跃(Minimum Snap)轨迹生成
 *
 * 使用闭式解方法生成最小跳跃轨迹。
 *
 * 算法原理：
 * 1. 5次多项式有6个系数，可以指定位置、速度、加速度边界条件
 * 2. 最小跳跃问题转化为二次规划(QP)问题：min ∫|j(t)|² dt
 * 3. 使用映射矩阵A将多项式系数映射到各阶导数
 * 4. 使用选择矩阵C分离固定变量和自由变量
 * 5. 通过求解简化后的QP问题得到闭式解
 *
 * @param Pos 航点位置矩阵 (seg_num+1) x 3
 * @param start_vel 起始速度向量
 * @param end_vel 终止速度向量
 * @param start_acc 起始加速度向量
 * @param end_acc 终止加速度向量
 * @param Time 每段时间长度向量
 * @return PolynomialTraj 生成的多项式轨迹
 */
PolynomialTraj minSnapTraj(const Eigen::MatrixXd& Pos, const Eigen::Vector3d& start_vel,
                           const Eigen::Vector3d& end_vel, const Eigen::Vector3d& start_acc,
                           const Eigen::Vector3d& end_acc, const Eigen::VectorXd& Time) {
  int seg_num = Time.size();  // 段数
  Eigen::MatrixXd poly_coeff(seg_num, 3 * 6);  // 多项式系数矩阵
  Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);  // x,y,z系数向量

  int num_f, num_p;  // 固定变量和自由变量的数量
  int num_d;         // 所有段导数的总数

  // 阶乘函数 lambda
  const static auto Factorial = [](int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
      fac = fac * i;
    return fac;
  };

  /* ========== 端点导数约束 ========== */
  // D向量存储各阶导数的约束值
  // 每段6个约束：位置(0,1阶)、速度(2,3阶)、加速度(4,5阶)
  Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dz = Eigen::VectorXd::Zero(seg_num * 6);

  for (int k = 0; k < seg_num; k++) {
    /* 位置约束 */
    Dx(k * 6) = Pos(k, 0);       // 段起点位置
    Dx(k * 6 + 1) = Pos(k + 1, 0);  // 段终点位置
    Dy(k * 6) = Pos(k, 1);
    Dy(k * 6 + 1) = Pos(k + 1, 1);
    Dz(k * 6) = Pos(k, 2);
    Dz(k * 6 + 1) = Pos(k + 1, 2);

    // 第一段：设置起始速度和加速度约束
    if (k == 0) {
      Dx(k * 6 + 2) = start_vel(0);   // 起始速度x
      Dy(k * 6 + 2) = start_vel(1);   // 起始速度y
      Dz(k * 6 + 2) = start_vel(2);   // 起始速度z

      Dx(k * 6 + 4) = start_acc(0);   // 起始加速度x
      Dy(k * 6 + 4) = start_acc(1);   // 起始加速度y
      Dz(k * 6 + 4) = start_acc(2);   // 起始加速度z
    }
    // 最后一段：设置终止速度和加速度约束
    else if (k == seg_num - 1) {
      Dx(k * 6 + 3) = end_vel(0);   // 终止速度x
      Dy(k * 6 + 3) = end_vel(1);   // 终止速度y
      Dz(k * 6 + 3) = end_vel(2);   // 终止速度z

      Dx(k * 6 + 5) = end_acc(0);   // 终止加速度x
      Dy(k * 6 + 5) = end_acc(1);   // 终止加速度y
      Dz(k * 6 + 5) = end_acc(2);   // 终止加速度z
    }
  }

  /* ========== 映射矩阵A ========== */
  // A矩阵将多项式系数映射到各阶导数在端点的值
  // 对角块结构，每个块对应一段
  Eigen::MatrixXd Ab;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++) {
    Ab = Eigen::MatrixXd::Zero(6, 6);
    // 构建6x6的映射矩阵块
    for (int i = 0; i < 3; i++) {
      // 对应0,2,4阶导数（位置、速度、加速度）
      Ab(2 * i, i) = Factorial(i);
      for (int j = i; j < 6; j++)
        // 对应1,3,5阶导数
        Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);
    }
    A.block(k * 6, k * 6, 6, 6) = Ab;
  }

  /* ========== 选择矩阵C' ========== */
  // C矩阵用于选择固定变量（约束）和自由变量
  // 固定变量：端点位置、速度、加速度
  // 自由变量：中间连接点的速度、加速度
  Eigen::MatrixXd Ct, C;

  num_f = 2 * seg_num + 4;  // 固定变量数 = 2*段数 + 4
  num_p = 2 * seg_num - 2;  // 自由变量数 = 2*(段数-1)
  num_d = 6 * seg_num;       // 导数总数
  
  Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);
  
  // 起点约束
  Ct(0, 0) = 1;
  Ct(2, 1) = 1;
  Ct(4, 2) = 1;  // stack the start point
  Ct(1, 3) = 1;
  Ct(3, 2 * seg_num + 4) = 1;
  Ct(5, 2 * seg_num + 5) = 1;

  // 终点约束
  Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1;  // Stack the end point
  Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1;  // Stack the end point
  Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1;
  Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1;  // Stack the end point

  // 中间连接点约束（速度、加速度连续）
  for (int j = 2; j < seg_num; j++) {
    Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1;
    Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1;
    Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1;
    Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
  }

  C = Ct.transpose();  // 选择矩阵

  // 应用选择矩阵到导数约束
  Eigen::VectorXd Dx1 = C * Dx;
  Eigen::VectorXd Dy1 = C * Dy;
  Eigen::VectorXd Dz1 = C * Dz;

  /* ========== 最小跳跃矩阵Q ========== */
  // Q矩阵是跳跃代价的 Hessian 矩阵
  // 只考虑3-5阶导数（加速度和跳跃）
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++) {
    for (int i = 3; i < 6; i++) {
      for (int j = 3; j < 6; j++) {
        // 跳跃代价积分公式
        Q(k * 6 + i, k * 6 + j) =
            i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
      }
    }
  }

  /* ========== 约化 Hessian 矩阵R ========== */
  // R = C * A^(-1)^T * Q * A^(-1) * C^T
  // 这是约化后的QP问题的 Hessian 矩阵
  Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;

  // 提取固定和自由部分的约束
  Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4), Dzf(2 * seg_num + 4);

  Dxf = Dx1.segment(0, 2 * seg_num + 4);
  Dyf = Dy1.segment(0, 2 * seg_num + 4);
  Dzf = Dz1.segment(0, 2 * seg_num + 4);

  // 分解R矩阵为四个子块
  Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);  // 固定-固定
  Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);  // 固定-自由
  Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);  // 自由-固定
  Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);  // 自由-自由

  Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
  Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
  Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
  Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

  /* ========== 闭式求解 ========== */
  // 求解自由变量：Dp = -Rpp^(-1) * Rfp^T * Df
  Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2), Dzp(2 * seg_num - 2);
  Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
  Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
  Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

  // 合并固定和自由变量
  Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
  Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
  Dz1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;

  // 恢复多项式系数：P = A^(-1) * C^T * D
  Px = (A.inverse() * Ct) * Dx1;
  Py = (A.inverse() * Ct) * Dy1;
  Pz = (A.inverse() * Ct) * Dz1;

  // 整理系数矩阵
  for (int i = 0; i < seg_num; i++) {
    poly_coeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();
  }

  /* ========== 构造轨迹对象 ========== */
  PolynomialTraj poly_traj;
  for (int i = 0; i < poly_coeff.rows(); ++i) {
    vector<double> cx(6), cy(6), cz(6);
    for (int j = 0; j < 6; ++j) {
      cx[j] = poly_coeff(i, j);
      cy[j] = poly_coeff(i, j + 6);
      cz[j] = poly_coeff(i, j + 12);
    }
    // 系数顺序反转（从高次到低次转为从低次到高次）
    reverse(cx.begin(), cx.end());
    reverse(cy.begin(), cy.end());
    reverse(cz.begin(), cz.end());
    double ts = Time(i);
    poly_traj.addSegment(cx, cy, cz, ts);
  }

  return poly_traj;
}

/**
 * @brief fastLine4deg - 快速直线轨迹(4次多项式，7段)
 *
 * 生成沿直线的轨迹，使用梯形速度曲线。
 * 适用于需要平滑加减速的运动。
 *
 * @param start 起始位置
 * @param end 终止位置
 * @param max_vel 最大速度
 * @param max_acc 最大加速度
 * @param max_jerk 最大跳跃
 * @return PolynomialTraj 生成的多项式轨迹
 */
/**
 * @brief fastLine4deg - 快速直线轨迹(4次多项式，7段)
 *
 * 生成沿直线的轨迹，使用梯形速度曲线。
 * 适用于需要平滑加减速的运动。
 *
 * 轨迹段划分：
 *  - 第1段：匀加速度启动 (j = +max_jerk)
 *  - 第2段：匀速上升 (j = 0, a = max_acc)
 *  - 第3段：匀减速 (j = -max_jerk)
 *  - 第4段：匀速巡航 (j = 0, a = 0)
 *  - 第5段：匀减速 (j = -max_jerk)
 *  - 第6段：匀速下降 (j = 0, a = -max_acc)
 *  - 第7段：匀减速度停止 (j = +max_jerk)
 *
 * @param start 起始位置
 * @param end 终止位置
 * @param max_vel 最大速度
 * @param max_acc 最大加速度
 * @param max_jerk 最大跳跃
 * @return PolynomialTraj 生成的多项式轨迹
 */
PolynomialTraj fastLine4deg(Eigen::Vector3d start, Eigen::Vector3d end, double max_vel, double max_acc,
                            double max_jerk) {
  // 计算位移和方向
  Eigen::Vector3d disp = end - start;
  double len = disp.norm();
  Eigen::Vector3d dir = disp.normalized();

  // 获取缩放向量：确保各轴同时到达限制
  int max_id = -1;
  double max_dist = -1.0;
  for (int i = 0; i < 3; ++i) {
    if (fabs(disp(i)) > max_dist) {
      max_dist = disp(i);
      max_id = i;
    }
  }
  Eigen::Vector3d scale_vec = disp / max_dist;

  PolynomialTraj poly_traj;
  vector<double> cx(6), cy(6), cz(6), zero(6);
  for (int i = 0; i < 6; ++i)
    zero[i] = 0.0;

  // 初始状态
  Eigen::Vector3d p0 = start;
  Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d a0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d j0 = max_jerk * scale_vec;

  /* ========== 第一段：匀加速度启动 ========== */
  cx = cy = cz = zero;
  double t1 = max_acc / max_jerk;  // 加速时间

  cx[5] = p0(0);  // 位置常数项
  cy[5] = p0(1);
  cz[5] = p0(2);

  cx[4] = v0(0);  // 速度系数
  cy[4] = v0(1);
  cz[4] = v0(2);

  cx[3] = (1 / 2.0) * a0(0);  // 加速度系数
  cy[3] = (1 / 2.0) * a0(1);
  cz[3] = (1 / 2.0) * a0(2);

  cx[2] = (1 / 6.0) * j0(0);  // 跳跃系数
  cy[2] = (1 / 6.0) * j0(1);
  cz[2] = (1 / 6.0) * j0(2);

  poly_traj.addSegment(cx, cy, cz, t1);

  // 更新状态
  Eigen::Vector3d p1 = p0 + v0 * t1 + (1 / 2.0) * a0 * pow(t1, 2) + (1 / 6.0) * j0 * pow(t1, 3);
  Eigen::Vector3d v1 = v0 + a0 * t1 + (1 / 2.0) * j0 * pow(t1, 2);
  Eigen::Vector3d a1 = a0 + j0 * t1;
  Eigen::Vector3d j1 = Eigen::Vector3d::Zero();

  /* ========== 第二段：匀速上升 ========== */
  cx = cy = cz = zero;
  double t2 = max_vel / max_acc - t1;

  cx[5] = p1(0);
  cy[5] = p1(1);
  cz[5] = p1(2);

  cx[4] = v1(0);
  cy[4] = v1(1);
  cz[4] = v1(2);

  cx[3] = (1 / 2.0) * a1(0);
  cy[3] = (1 / 2.0) * a1(1);
  cz[3] = (1 / 2.0) * a1(2);

  cx[2] = (1 / 6.0) * j1(0);
  cy[2] = (1 / 6.0) * j1(1);
  cz[2] = (1 / 6.0) * j1(2);

  poly_traj.addSegment(cx, cy, cz, t2);

  // 更新状态
  Eigen::Vector3d p2 = p1 + v1 * t2 + (1 / 2.0) * a1 * pow(t2, 2) + (1 / 6.0) * j1 * pow(t2, 3);
  Eigen::Vector3d v2 = v1 + a1 * t2 + (1 / 2.0) * j1 * pow(t2, 2);
  Eigen::Vector3d a2 = a1 + j1 * t2;
  Eigen::Vector3d j2 = -max_jerk * scale_vec;

  /* ========== 第三段：匀减速 ========== */
  cx = cy = cz = zero;
  double t3 = t1;

  cx[5] = p2(0);
  cy[5] = p2(1);
  cz[5] = p2(2);

  cx[4] = v2(0);
  cy[4] = v2(1);
  cz[4] = v2(2);

  cx[3] = (1 / 2.0) * a2(0);
  cy[3] = (1 / 2.0) * a2(1);
  cz[3] = (1 / 2.0) * a2(2);

  cx[2] = (1 / 6.0) * j2(0);
  cy[2] = (1 / 6.0) * j2(1);
  cz[2] = (1 / 6.0) * j2(2);

  poly_traj.addSegment(cx, cy, cz, t3);

  // 更新状态
  Eigen::Vector3d p3 = p2 + v2 * t3 + (1 / 2.0) * a2 * pow(t3, 2) + (1 / 6.0) * j2 * pow(t3, 3);
  Eigen::Vector3d v3 = v2 + a2 * t3 + (1 / 2.0) * j2 * pow(t3, 2);
  Eigen::Vector3d a3 = a2 + j2 * t3;
  Eigen::Vector3d j3 = Eigen::Vector3d::Zero();

  /* ========== 第四段：匀速巡航 ========== */
  cx = cy = cz = zero;
  double t4 = max_dist / max_vel - 2 * t1 - t2;

  cx[5] = p3(0);
  cy[5] = p3(1);
  cz[5] = p3(2);

  cx[4] = v3(0);
  cy[4] = v3(1);
  cz[4] = v3(2);

  cx[3] = (1 / 2.0) * a3(0);
  cy[3] = (1 / 2.0) * a3(1);
  cz[3] = (1 / 2.0) * a3(2);

  cx[2] = (1 / 6.0) * j3(0);
  cy[2] = (1 / 6.0) * j3(1);
  cz[2] = (1 / 6.0) * j3(2);

  poly_traj.addSegment(cx, cy, cz, t4);

  // 更新状态
  Eigen::Vector3d p4 = p3 + v3 * t4 + (1 / 2.0) * a3 * pow(t4, 2) + (1 / 6.0) * j3 * pow(t4, 3);
  Eigen::Vector3d v4 = v3 + a3 * t4 + (1 / 2.0) * j3 * pow(t4, 2);
  Eigen::Vector3d a4 = a3 + j3 * t4;
  Eigen::Vector3d j4 = -max_jerk * scale_vec;

  /* ========== 第五段：匀减速 ========== */
  cx = cy = cz = zero;
  double t5 = t1;

  cx[5] = p4(0);
  cy[5] = p4(1);
  cz[5] = p4(2);

  cx[4] = v4(0);
  cy[4] = v4(1);
  cz[4] = v4(2);

  cx[3] = (1 / 2.0) * a4(0);
  cy[3] = (1 / 2.0) * a4(1);
  cz[3] = (1 / 2.0) * a4(2);

  cx[2] = (1 / 6.0) * j4(0);
  cy[2] = (1 / 6.0) * j4(1);
  cz[2] = (1 / 6.0) * j4(2);

  poly_traj.addSegment(cx, cy, cz, t5);

  // 更新状态
  Eigen::Vector3d p5 = p4 + v4 * t5 + (1 / 2.0) * a4 * pow(t5, 2) + (1 / 6.0) * j4 * pow(t5, 3);
  Eigen::Vector3d v5 = v4 + a4 * t5 + (1 / 2.0) * j4 * pow(t5, 2);
  Eigen::Vector3d a5 = a4 + j4 * t5;
  Eigen::Vector3d j5 = Eigen::Vector3d::Zero();

  /* ========== 第六段：匀速下降 ========== */
  cx = cy = cz = zero;
  double t6 = t2;

  cx[5] = p5(0);
  cy[5] = p5(1);
  cz[5] = p5(2);

  cx[4] = v5(0);
  cy[4] = v5(1);
  cz[4] = v5(2);

  cx[3] = (1 / 2.0) * a5(0);
  cy[3] = (1 / 2.0) * a5(1);
  cz[3] = (1 / 2.0) * a5(2);

  cx[2] = (1 / 6.0) * j5(0);
  cy[2] = (1 / 6.0) * j5(1);
  cz[2] = (1 / 6.0) * j5(2);

  poly_traj.addSegment(cx, cy, cz, t6);

  // 更新状态
  Eigen::Vector3d p6 = p5 + v5 * t6 + (1 / 2.0) * a5 * pow(t6, 2) + (1 / 6.0) * j5 * pow(t6, 3);
  Eigen::Vector3d v6 = v5 + a5 * t6 + (1 / 2.0) * j5 * pow(t6, 2);
  Eigen::Vector3d a6 = a5 + j5 * t6;
  Eigen::Vector3d j6 = max_jerk * scale_vec;

  /* ========== 第七段(最后)：匀减速停止 ========== */
  cx = cy = cz = zero;
  double t7 = t1;

  cx[5] = p6(0);
  cy[5] = p6(1);
  cz[5] = p6(2);

  cx[4] = v6(0);
  cy[4] = v6(1);
  cz[4] = v6(2);

  cx[3] = (1 / 2.0) * a6(0);
  cy[3] = (1 / 2.0) * a6(1);
  cz[3] = (1 / 2.0) * a6(2);

  cx[2] = (1 / 6.0) * j6(0);
  cy[2] = (1 / 6.0) * j6(1);
  cz[2] = (1 / 6.0) * j6(2);

  poly_traj.addSegment(cx, cy, cz, t7);

  return poly_traj;
}

/**
 * @brief fastLine3deg - 快速直线轨迹(3次多项式，3段)
 *
 * 生成沿直线的轨迹，使用梯形加速度曲线。
 * 比fastLine4deg更简单，但跳跃不连续（只有加速度连续）。
 *
 * 轨迹分为3段：
 * 1. 头段：匀加速度（从0加速到max_vel）
 * 2. 中段：匀速
 * 3. 尾段：匀减速
 *
 * @param start 起始位置
 * @param end 终止位置
 * @param max_vel 最大速度
 * @param max_acc 最大加速度
 * @return PolynomialTraj 生成的多项式轨迹
 */
PolynomialTraj fastLine3deg(Eigen::Vector3d start, Eigen::Vector3d end, double max_vel, double max_acc) {
  // 计算位移和方向
  Eigen::Vector3d disp = end - start;
  double len = disp.norm();
  Eigen::Vector3d dir = disp.normalized();

  // 获取缩放向量
  double max_dist = std::max(disp(0), disp(1));
  max_dist = std::max(max_dist, disp(2));
  Eigen::Vector3d scale_vec = disp / max_dist;

  // 计算三段时间
  // 头尾段时间
  double t_ht = max_vel / max_acc;

  // 中段时间
  double len_ht = 0.5 * (max_acc * scale_vec.norm()) * t_ht * t_ht;
  double t_m = (len - 2 * len_ht) / (max_vel * scale_vec.norm());

  // 两个中间航点
  Eigen::Vector3d mpt1 = start + dir * len_ht;
  Eigen::Vector3d mpt2 = end - dir * len_ht;

  // 3段轨迹
  PolynomialTraj poly_traj;
  vector<double> cx(6), cy(6), cz(6), zero(6);
  for (int i = 0; i < 6; ++i)
    zero[i] = 0.0;

  /* ========== 头段：匀加速度启动 ========== */
  cx = cy = cz = zero;

  cx[5] = start(0);  // p0
  cy[5] = start(1);
  cz[5] = start(2);

  cx[4] = 0.0;  // v0
  cy[4] = 0.0;
  cz[4] = 0.0;

  cx[3] = 0.5 * max_acc * scale_vec(0);  // a0
  cy[3] = 0.5 * max_acc * scale_vec(1);
  cz[3] = 0.5 * max_acc * scale_vec(2);

  poly_traj.addSegment(cx, cy, cz, t_ht);

  /* ========== 中段：匀速 ========== */
  cx = cy = cz = zero;

  cx[5] = mpt1(0);  // p0
  cy[5] = mpt1(1);
  cz[5] = mpt1(2);

  cx[4] = max_vel * scale_vec(0);  // v0
  cy[4] = max_vel * scale_vec(1);
  cz[4] = max_vel * scale_vec(2);

  cx[3] = 0.0;  // a0
  cy[3] = 0.0;
  cz[3] = 0.0;

  poly_traj.addSegment(cx, cy, cz, t_m);

  /* ========== 尾段：匀减速 ========== */
  cx = cy = cz = zero;

  cx[5] = mpt2(0);  // p0
  cy[5] = mpt2(1);
  cz[5] = mpt2(2);

  cx[4] = max_vel * scale_vec(0);  // v0
  cy[4] = max_vel * scale_vec(1);
  cz[4] = max_vel * scale_vec(2);

  cx[3] = -0.5 * max_acc * scale_vec(0);  // a0
  cy[3] = -0.5 * max_acc * scale_vec(1);
  cz[3] = -0.5 * max_acc * scale_vec(2);

  poly_traj.addSegment(cx, cy, cz, t_ht);

  return poly_traj;
}