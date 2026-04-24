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
 * 表示由多个多项式段组成的连续轨迹。每段可以是不同阶数的多项式。
 * 支持轨迹的位置、速度、加速度评估，以及轨迹长度、 jerks 等计算。
 *
 * 数据结构：
 * - times: 每段的时间长度
 * - cxs, cys, czs: 每段的多项式系数（从高阶到低阶存储）
 *
 * 示例：5次多项式 p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
 * 系数存储为 [c5, c4, c3, c2, c1, c0]（从高阶到低阶）
 */
class PolynomialTraj {
private:
  /// @brief 每段的时间长度
  vector<double> times;
  
  /// @brief 每段x轴多项式系数（从高阶到低阶）
  vector<vector<double>> cxs;
  
  /// @brief 每段y轴多项式系数
  vector<vector<double>> cys;
  
  /// @brief 每段z轴多项式系数
  vector<vector<double>> czs;

  /// @brief 轨迹总时间
  double time_sum;
  
  /// @brief 段数
  int num_seg;

  /* 评估用数据成员 */
  
  /// @brief 离散化的轨迹点序列
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
  /// @param cx x系数向量（从高阶到低阶）
  /// @param cy y系数向量
  /// @param cz z系数向量
  /// @param t 该段时间长度
  void addSegment(vector<double> cx, vector<double> cy, vector<double> cz, double t) {
    cxs.push_back(cx), cys.push_back(cy), czs.push_back(cz), times.push_back(t);
  }

  /// @brief 初始化轨迹参数
  /// @details 计算总时间和段数
  void init() {
    num_seg = times.size();
    time_sum = 0.0;
    for (int i = 0; i < times.size(); ++i) {
      time_sum += times[i];
    }
  }

  /// @brief 评估轨迹位置
  /// @param t 查询时间（从轨迹开始）
  /// @return 3D位置
  /// @details 算法：
  /// 1. 确定时间t属于哪一段（累减times[i]直到t < times[idx]）
  /// 2. 使用该段的多项式系数计算位置
  Eigen::Vector3d evaluate(double t) {
    /* detetrmine segment num */
    int idx = 0;
    while (times[idx] < t) {
      t -= times[idx];
      ++idx;
    }

    /* evaluation */
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

  /// @brief 评估轨迹速度
  /// @param t 查询时间
  /// @return 3D速度向量
  /// @details 速度是位置对时间的导数：
  /// v(t) = dc/dt = c1 + 2*c2*t + 3*c3*t^2 + ...
  /// 系数关系：v_coef[i] = (order-1-i) * p_coef[i]
  Eigen::Vector3d evaluateVel(double t) {
    /* 确定段索引 */
    int idx = 0;
    while (times[idx] < t) {
      t -= times[idx];
      ++idx;
    }

    /* 评估速度 */
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

  /// @brief 评估轨迹加速度
  /// @param t 查询时间
  /// @return 3D加速度向量
  /// @details 加速度是速度的导数（位置的二阶导数）：
  /// a(t) = d²p/dt² = 2*c2 + 6*c3*t + ...
  Eigen::Vector3d evaluateAcc(double t) {
    /* detetrmine segment num */
    int idx = 0;
    while (times[idx] < t) {
      t -= times[idx];
      ++idx;
    }

    /* evaluation */
    int order = cxs[idx].size();
    Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

    /* coef of vel */
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

  /// @brief 获取轨迹总时间
  /// @return 轨迹总时长
  /* 用于评估轨迹，应按顺序调用!!! */
  double getTimeSum() {
    return this->time_sum;
  }

  /// @brief 获取离散化的轨迹点序列
  /// @return 轨迹点向量（间隔0.01秒采样）
  /// @details 使用0.01秒间隔对轨迹进行离散化
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

  /// @brief 计算轨迹总长度
  /// @return 轨迹长度（米）
  /// @details 对离散轨迹点累加欧几里得距离
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

  /// @brief 计算平均速度
  /// @return 平均速度 = 轨迹长度 / 总时间
  double getMeanVel() {
    double mean_vel = length / time_sum;
  }

  /// @brief 计算加速度代价
  /// @return 加速度代价积分
  /// @details 使用2范数平方积分：∫||a(t)||² dt
  /// 对于n次多项式，加速度代价 = ∑ (2*c_n-2)² * Δt
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

  /// @brief 计算加加速度(Jerk)代价
  /// @return Jerk代价积分
  /// @details Jerk是加速度的导数（三阶导数），反映运动平滑度
  /// 公式：J = ∫||j(t)||² dt
  double getJerk() {
    double jerk = 0.0;

    /* evaluate jerk */
    for (int s = 0; s < times.size(); ++s) {
      Eigen::VectorXd cxv(cxs[s].size()), cyv(cys[s].size()), czv(czs[s].size());
      /* convert coefficient */
      int order = cxs[s].size();
      for (int j = 0; j < order; ++j) {
        cxv(j) = cxs[s][order - 1 - j], cyv(j) = cys[s][order - 1 - j], czv(j) = czs[s][order - 1 - j];
      }
      double ts = times[s];

      /* jerk matrix */
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

  void getMeanAndMaxVel(double& mean_v, double& max_v) {
    int num = 0;
    mean_v = 0.0, max_v = -1.0;
    for (int s = 0; s < times.size(); ++s) {
      int order = cxs[s].size();
      Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);

      /* coef of vel */
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

  void getMeanAndMaxAcc(double& mean_a, double& max_a) {
    int num = 0;
    mean_a = 0.0, max_a = -1.0;
    for (int s = 0; s < times.size(); ++s) {
      int order = cxs[s].size();
      Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

      /* coef of acc */
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

#endif