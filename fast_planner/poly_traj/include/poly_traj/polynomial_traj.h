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



#ifndef _POLYNOMIAL_TRAJ_H
#define _POLYNOMIAL_TRAJ_H

#include <Eigen/Eigen>
#include <vector>

using std::vector;

/**
 * @brief PolynomialTraj类 - 分段多项式轨迹
 *
 * 表示由多个多项式段组成的连续轨迹。
 * 支持任意阶数的多项式，每段可以是不同阶数。
 *
 * 数据结构：
 * - times: 每段的时间长度
 * - cxs, cys, czs: 每段的多项式系数（从低阶到高阶存储）
 *
 * 评估函数：
 * - evaluate(): 位置
 * - evaluateVel(): 速度（一阶导数）
 * - evaluateAcc(): 加速度（二阶导数）
 */
class PolynomialTraj {
private:
  /// @brief 每段时间长度
  vector<double> times;
  
  /// @brief 每段x轴多项式系数（从0次到n-1次）
  vector<vector<double>> cxs;
  
  /// @brief 每段y轴多项式系数
  vector<vector<double>> cys;
  
  /// @brief 每段z轴多项式系数
  vector<vector<double>> czs;

  /// @brief 轨迹总时间
  double time_sum;
  
  /// @brief 段数
  int num_seg;

  /* 评估用数据 */
  
  /// @brief 离散轨迹点
  vector<Eigen::Vector3d> traj_vec3d;
  
  /// @brief 轨迹长度
  double length;

public:
  PolynomialTraj(/* args */) {
  }
  ~PolynomialTraj() {
  }

  /// @brief 重置轨迹数据
  void reset() {
    times.clear(), cxs.clear(), cys.clear(), czs.clear();
    time_sum = 0.0, num_seg = 0;
  }

  /// @brief 添加一段多项式
  /// @param cx x系数向量
  /// @param cy y系数向量
  /// @param cz z系数向量
  /// @param t 时间长度
  void addSegment(vector<double> cx, vector<double> cy, vector<double> cz, double t) {
    cxs.push_back(cx), cys.push_back(cy), czs.push_back(cz), times.push_back(t);
  }

  /// @brief 初始化轨迹
  void init() {
    num_seg = times.size();
    time_sum = 0.0;
    for (int i = 0; i < times.size(); ++i) {
      time_sum += times[i];
    }
  }

  /**
   * @brief 评估位置
   * @param t 时间
   * @return 3D位置
   */
  Eigen::Vector3d evaluate(double t) {
    /* 确定段索引 */
    int idx = 0;
    while (times[idx] + 1e-4 < t) {
      t -= times[idx];
      ++idx;
    }

    /* 评估 */
    int order = cxs[idx].size();
    Eigen::VectorXd cx(order), cy(order), cz(order), tv(order);
    for (int i = 0; i < order; ++i) {
      cx(i) = cxs[idx][i], cy(i) = cys[idx][i], cz(i) = czs[idx][i];
      tv(order - 1 - i) = std::pow(t, double(i));
    }

    Eigen::Vector3d pt;
    pt(0) = tv.dot(cx), pt(1) = tv.dot(cy), pt(2) = tv.dot(cz);
    return pt;
  }

  /**
   * @brief 评估速度（一阶导数）
   * @param t 时间
   * @return 3D速度
   */
  Eigen::Vector3d evaluateVel(double t) {
    /* 确定段索引 */
    int idx = 0;
    while (times[idx] + 1e-4 < t) {
      t -= times[idx];
      ++idx;
    }

    /* 评估 */
    int order = cxs[idx].size();
    Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);

    /* 速度多项式系数 = 导数系数 */
    for (int i = 0; i < order - 1; ++i) {
      vx(i) = double(i + 1) * cxs[idx][order - 2 - i];
      vy(i) = double(i + 1) * cys[idx][order - 2 - i];
      vz(i) = double(i + 1) * czs[idx][order - 2 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 1);
    for (int i = 0; i < order - 1; ++i)
      tv(i) = pow(ts, i);

    Eigen::Vector3d vel;
    vel(0) = tv.dot(vx), vel(1) = tv.dot(vy), vel(2) = tv.dot(vz);
    return vel;
  }

  /**
   * @brief 评估加速度（二阶导数）
   * @param t 时间
   * @return 3D加速度
   */
  Eigen::Vector3d evaluateAcc(double t) {
    /* 确定段索引 */
    int idx = 0;
    while (times[idx] + 1e-4 < t) {
      t -= times[idx];
      ++idx;
    }

    /* 评估 */
    int order = cxs[idx].size();
    Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

    /* 加速度多项式系数 */
    for (int i = 0; i < order - 2; ++i) {
      ax(i) = double((i + 2) * (i + 1)) * cxs[idx][order - 3 - i];
      ay(i) = double((i + 2) * (i + 1)) * cys[idx][order - 3 - i];
      az(i) = double((i + 2) * (i + 1)) * czs[idx][order - 3 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 2);
    for (int i = 0; i < order - 2; ++i)
      tv(i) = pow(ts, i);

    Eigen::Vector3d acc;
    acc(0) = tv.dot(ax), acc(1) = tv.dot(ay), acc(2) = tv.dot(az);
    return acc;
  }

  /* 获取轨迹属性 */
  
  /// @brief 获取总时间
  double getTimeSum() {
    return this->time_sum;
  }

  /// @brief 获取离散轨迹
  /// @return 轨迹点列表（0.01s间隔）
  vector<Eigen::Vector3d> getTraj() {
    double eval_t = 0.0;
    traj_vec3d.clear();
    while (eval_t < time_sum) {
      Eigen::Vector3d pt = evaluate(eval_t);
      traj_vec3d.push_back(pt);
      eval_t += 0.01;
    }
    return traj_vec3d;
  }

  /// @brief 获取轨迹长度
  double getLength() {
    length = 0.0;

    Eigen::Vector3d p_l = traj_vec3d[0], p_n;
    for (int i = 1; i < traj_vec3d.size(); ++i) {
      p_n = traj_vec3d[i];
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  /// @brief 获取平均速度
  double getMeanVel() {
    double mean_vel = length / time_sum;
  }

  /// @brief 获取加速度代价
  double getAccCost() {
    double cost = 0.0;
    int order = cxs[0].size();

    for (int s = 0; s < times.size(); ++s) {
      Eigen::Vector3d um;
      um(0) = 2 * cxs[s][order - 3], um(1) = 2 * cys[s][order - 3], um(2) = 2 * czs[s][order - 3];
      cost += um.squaredNorm() * times[s];
    }

    return cost;
  }

  /**
   * @brief 获取总跳跃(jerk)代价
   *
   * 跳跃是加速度的导数，是衡量轨迹平滑度的重要指标。
   * 最小跳跃轨迹是无人机轨迹规划中的常用方法。
   *
   * 计算公式：J = ∫|j(t)|² dt
   *
   * @return double 总跳跃积分值
   */
  double getJerk() {
    double jerk = 0.0;

    /* 评估跳跃 */
    for (int s = 0; s < times.size(); ++s) {
      Eigen::VectorXd cxv(cxs[s].size()), cyv(cys[s].size()), czv(czs[s].size());
      /* 转换系数顺序 */
      int order = cxs[s].size();
      for (int j = 0; j < order; ++j) {
        cxv(j) = cxs[s][order - 1 - j], cyv(j) = cys[s][order - 1 - j], czv(j) = czs[s][order - 1 - j];
      }
      double ts = times[s];

      /* 跳跃矩阵 */
      Eigen::MatrixXd mat_jerk(order, order);
      mat_jerk.setZero();
      for (double i = 3; i < order; i += 1)
        for (double j = 3; j < order; j += 1) {
          mat_jerk(i, j) =
              i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) * pow(ts, i + j - 5) / (i + j - 5);
        }

      jerk += (cxv.transpose() * mat_jerk * cxv)(0, 0);
      jerk += (cyv.transpose() * mat_jerk * cyv)(0, 0);
      jerk += (czv.transpose() * mat_jerk * czv)(0, 0);
    }

    return jerk;
  }

  /**
   * @brief 获取平均和最大速度
   *
   * 对轨迹进行离散采样，计算所有采样点的速度统计信息。
   *
   * @param[out] mean_v 平均速度
   * @param[out] max_v 最大速度
   */
  void getMeanAndMaxVel(double& mean_v, double& max_v) {
    int num = 0;
    mean_v = 0.0, max_v = -1.0;
    for (int s = 0; s < times.size(); ++s) {
      int order = cxs[s].size();
      Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);

      /* 速度系数 */
      for (int i = 0; i < order - 1; ++i) {
        vx(i) = double(i + 1) * cxs[s][order - 2 - i];
        vy(i) = double(i + 1) * cys[s][order - 2 - i];
        vz(i) = double(i + 1) * czs[s][order - 2 - i];
      }
      double ts = times[s];

      double eval_t = 0.0;
      while (eval_t < ts) {
        Eigen::VectorXd tv(order - 1);
        for (int i = 0; i < order - 1; ++i)
          tv(i) = pow(ts, i);
        Eigen::Vector3d vel;
        vel(0) = tv.dot(vx), vel(1) = tv.dot(vy), vel(2) = tv.dot(vz);
        double vn = vel.norm();
        mean_v += vn;
        if (vn > max_v) max_v = vn;
        ++num;

        eval_t += 0.01;
      }
    }

    mean_v = mean_v / double(num);
  }

  /**
   * @brief 获取平均和最大加速度
   *
   * 对轨迹进行离散采样，计算所有采样点的加速度统计信息。
   *
   * @param[out] mean_a 平均加速度
   * @param[out] max_a 最大加速度
   */
  void getMeanAndMaxAcc(double& mean_a, double& max_a) {
    int num = 0;
    mean_a = 0.0, max_a = -1.0;
    for (int s = 0; s < times.size(); ++s) {
      int order = cxs[s].size();
      Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

      /* 加速度系数 */
      for (int i = 0; i < order - 2; ++i) {
        ax(i) = double((i + 2) * (i + 1)) * cxs[s][order - 3 - i];
        ay(i) = double((i + 2) * (i + 1)) * cys[s][order - 3 - i];
        az(i) = double((i + 2) * (i + 1)) * czs[s][order - 3 - i];
      }
      double ts = times[s];

      double eval_t = 0.0;
      while (eval_t < ts) {
        Eigen::VectorXd tv(order - 2);
        for (int i = 0; i < order - 2; ++i)
          tv(i) = pow(ts, i);
        Eigen::Vector3d acc;
        acc(0) = tv.dot(ax), acc(1) = tv.dot(ay), acc(2) = tv.dot(az);
        double an = acc.norm();
        mean_a += an;
        if (an > max_a) max_a = an;
        ++num;

        eval_t += 0.01;
      }
    }

    mean_a = mean_a / double(num);
  }
};

// ========== 全局函数声明 ==========

/**
 * @brief 生成最小跳跃(Minimum Snap)轨迹
 *
 * 最小跳跃轨迹优化是一种轨迹生成方法，通过最小化跳跃（即加速度的导数）
 * 来生成平滑的无人机轨迹。该方法确保轨迹具有连续的速度和加速度。
 *
 * 数学原理：
 * - 5次多项式有6个系数，可以指定位置、速度、加速度边界条件
 * - 最小跳跃问题转化为二次规划(QP)问题
 * - 使用闭式解求解，构造映射矩阵和汉森矩阵
 *
 * 输入参数：
 * - Pos: 航点位置矩阵 (seg_num+1) x 3，每行是一个3D位置
 * - start_vel: 起始速度
 * - end_vel: 终止速度
 * - start_acc: 起始加速度
 * - end_acc: 终止加速度
 * - Time: 每段时间长度向量
 *
 * @return PolynomialTraj 生成的多项式轨迹对象
 */
PolynomialTraj minSnapTraj(const Eigen::MatrixXd& Pos, const Eigen::Vector3d& start_vel,
                           const Eigen::Vector3d& end_vel, const Eigen::Vector3d& start_acc,
                           const Eigen::Vector3d& end_acc, const Eigen::VectorXd& Time);

/**
 * @brief 快速直线轨迹(4次多项式，7段)
 *
 * 生成沿直线的轨迹，使用梯形速度曲线（加速-匀速-减速模式）。
 * 轨迹分为7段：
 * 1. 匀加速度段 (jerk = max_jerk)
 * 2. 匀速段 (j = 0, a = max_acc)
 * 3. 匀减速度段 (j = -max_jerk)
 * 4. 匀速段 (a = 0)
 * 5. 匀加速度段 (j = -max_jerk)
 * 6. 匀速段 (j = 0, a = -max_acc)
 * 7. 匀减速度段 (j = max_jerk)
 *
 * @param start 起始位置
 * @param end 终止位置
 * @param max_vel 最大速度约束
 * @param max_acc 最大加速度约束
 * @param max_jerk 最大跳跃约束
 * @return PolynomialTraj 生成的多项式轨迹
 */
PolynomialTraj fastLine4deg(Eigen::Vector3d start, Eigen::Vector3d end, double max_vel, double max_acc,
                            double max_jerk);

/**
 * @brief 快速直线轨迹(3次多项式，3段)
 *
 * 生成沿直线的轨迹，使用3段多项式（梯形加速度曲线）。
 * 比fastLine4deg更简单，但跳跃不连续。
 *
 * 轨迹分为3段：
 * 1. 头段：匀加速度（从0加速到max_vel）
 * 2. 中段：匀速
 * 3. 尾段：匀减速
 *
 * @param start 起始位置
 * @param end 终止位置
 * @param max_vel 最大速度约束
 * @param max_acc 最大加速度约束
 * @return PolynomialTraj 生成的多项式轨迹
 */
PolynomialTraj fastLine3deg(Eigen::Vector3d start, Eigen::Vector3d end, double max_vel, double max_acc);

#endif