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



#ifndef _LINEAR_OBJ_MODEL_H_
#define _LINEAR_OBJ_MODEL_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

/**
 * @brief LinearObjModel类 - 线性障碍物运动模型
 *
 * 用于模拟动态障碍物的简单运动模型（匀速/匀加速模型）。
 * 主要用于仿真环境和轨迹预测。
 *
 * 运动学模型：
 * - 位置更新: p(t+dt) = p(t) + v(t) * dt
 * - 速度更新: v(t+dt) = v(t) + a(t) * dt
 * - 偏航角: yaw(t+dt) = yaw(t) + yaw_dot * dt
 *
 * 边界处理：
 * - 当障碍物超出边界时反弹（速度取反）
 */
class LinearObjModel {
private:
  /* 私有成员变量 */
  
  /// @brief 位置
  Eigen::Vector3d pos_;
  
  /// @brief 速度
  Eigen::Vector3d vel_;
  
  /// @brief 加速度
  Eigen::Vector3d acc_;
  
  /// @brief 颜色（用于可视化）
  Eigen::Vector3d color_;
  
  /// @brief 尺寸（长宽高）
  Eigen::Vector3d scale_;
  
  /// @brief 偏航角
  double yaw_;
  
  /// @brief 偏航角变化率
  double yaw_dot_;

  /// @brief 运动边界
  Eigen::Vector3d bound_;
  
  /// @brief 速度限制 [min, max]
  Eigen::Vector2d limit_v_;
  
  /// @brief 加速度限制 [min, max]
  Eigen::Vector2d limit_a_;
  
public:
  /// @brief 默认构造函数
  LinearObjModel(/* args */);
  
  /// @brief 析构函数
  ~LinearObjModel();

  /// @brief 初始化障碍物模型
  /// @param p 初始位置
  /// @param v 初始速度
  /// @param a 初始加速度
  /// @param yaw 初始偏航角
  /// @param yaw_dot 偏航角变化率
  /// @param color 可视化颜色 (RGB)
  /// @param scale 尺寸 (XYZ)
  void initialize(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw, double yaw_dot,
                  Eigen::Vector3d color, Eigen::Vector3d scale);

  /// @brief 设置运动限制
  /// @param bound 位置边界
  /// @param vel 速度限制 [min, max]
  /// @param acc 加速度限制 [min, max]
  void setLimits(Eigen::Vector3d bound, Eigen::Vector2d vel, Eigen::Vector2d acc);

  /// @brief 更新障碍物状态
  /// @param dt 时间步长
  /// @details 使用线性三重积分器模型（匀速模型）：
  /// 1. 位置更新: pos = pos + vel * dt
  /// 2. 边界检测与反弹处理
  /// 3. 偏航角更新: yaw = yaw + yaw_dot * dt
  void update(double dt);

  /// @brief 检测两个障碍物是否碰撞
  /// @param obj1 障碍物1
  /// @param obj2 障碍物2
  /// @return 是否发生碰撞
  /// @details 使用轴对齐包围盒(AABB)碰撞检测
  static bool collide(LinearObjModel& obj1, LinearObjModel& obj2);

  /// @brief 设置输入速度（用于控制）
  /// @param vel 速度向量
  void setInput(Eigen::Vector3d vel) {
    vel_ = vel;
  }

  /// @brief 设置偏航角变化率
  /// @param yaw_dot 偏航角变化率
  void setYawDot(double yaw_dot) {
    yaw_dot_ = yaw_dot;
  }

  /// @brief 获取位置
  Eigen::Vector3d getPosition() {
    return pos_;
  }
  
  /// @brief 设置位置
  void setPosition(Eigen::Vector3d pos) {
    pos_ = pos;
  }

  /// @brief 获取速度
  Eigen::Vector3d getVelocity() {
    return vel_;
  }

  /// @brief 设置速度
  /// @param x, y, z 速度分量
  void setVelocity(double x, double y, double z) {
    vel_ = Eigen::Vector3d(x, y, z);
  }

  /// @brief 获取颜色
  Eigen::Vector3d getColor() {
    return color_;
  }
  
  /// @brief 获取尺寸
  Eigen::Vector3d getScale() {
    return scale_;
  }

  /// @brief 获取偏航角
  double getYaw() {
    return yaw_;
  }

private:
};

LinearObjModel::LinearObjModel(/* args */) {
}

LinearObjModel::~LinearObjModel() {
}

void LinearObjModel::initialize(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw,
                                double yaw_dot, Eigen::Vector3d color, Eigen::Vector3d scale) {
  pos_ = p;
  vel_ = v;
  acc_ = a;
  color_ = color;
  scale_ = scale;

  yaw_ = yaw;
  yaw_dot_ = yaw_dot;
}

void LinearObjModel::setLimits(Eigen::Vector3d bound, Eigen::Vector2d vel, Eigen::Vector2d acc) {
  bound_ = bound;
  limit_v_ = vel;
  limit_a_ = acc;
}

void LinearObjModel::update(double dt) {
  Eigen::Vector3d p0, v0, a0;
  p0 = pos_, v0 = vel_, a0 = acc_;

  /* ---------- use acc as input ---------- */
  // vel_ = v0 + acc_ * dt;
  // for (int i = 0; i < 3; ++i)
  // {
  //   if (vel_(i) > 0) vel_(i) = std::max(limit_v_(0), std::min(vel_(i),
  //   limit_v_(1)));
  //   if (vel_(i) <= 0) vel_(i) = std::max(-limit_v_(1), std::min(vel_(i),
  //   -limit_v_(0)));
  // }

  // pos_ = p0 + v0 * dt + 0.5 * acc_ * pow(dt, 2);
  // for (int i = 0; i < 2; ++i)
  // {
  //   pos_(i) = std::min(pos_(i), bound_(i));
  //   pos_(i) = std::max(pos_(i), -bound_(i));
  // }
  // pos_(2) = std::min(pos_(2), bound_(2));
  // pos_(2) = std::max(pos_(2), 0.0);

  /* ---------- use vel as input ---------- */
  pos_ = p0 + v0 * dt;
  for (int i = 0; i < 2; ++i) {
    pos_(i) = std::min(pos_(i), bound_(i));
    pos_(i) = std::max(pos_(i), -bound_(i));
  }
  pos_(2) = std::min(pos_(2), bound_(2));
  pos_(2) = std::max(pos_(2), 0.0);

  yaw_ += yaw_dot_ * dt;

  const double PI = 3.1415926;
  if (yaw_ > 2 * PI) yaw_ -= 2 * PI;

  /* ---------- reflect when collide with bound ---------- */
  const double tol = 0.1;
  if (pos_(0) > bound_(0) - tol) {
    pos_(0) = bound_(0) - tol;
    vel_(0) = -vel_(0);
  }
  if (pos_(0) < -bound_(0) + tol) {
    pos_(0) = -bound_(0) + tol;
    vel_(0) = -vel_(0);
  }

  if (pos_(1) > bound_(1) - tol) {
    pos_(1) = bound_(1) - tol;
    vel_(1) = -vel_(1);
  }
  if (pos_(1) < -bound_(1) + tol) {
    pos_(1) = -bound_(1) + tol;
    vel_(1) = -vel_(1);
  }

  if (pos_(2) > bound_(2) - tol) {
    pos_(2) = bound_(2) - tol;
    vel_(2) = -vel_(2);
  }
  if (pos_(2) < tol) {
    pos_(2) = tol;
    vel_(2) = -vel_(2);
  }
}

bool LinearObjModel::collide(LinearObjModel& obj1, LinearObjModel& obj2) {
  Eigen::Vector3d pos1, pos2, vel1, vel2, scale1, scale2;
  pos1 = obj1.getPosition();
  vel1 = obj1.getVelocity();
  scale1 = obj1.getScale();

  pos2 = obj2.getPosition();
  vel2 = obj2.getVelocity();
  scale2 = obj2.getScale();

  /* ---------- collide ---------- */
  bool collide = fabs(pos1(0) - pos2(0)) < 0.5 * (scale1(0) + scale2(0)) &&
      fabs(pos1(1) - pos2(1)) < 0.5 * (scale1(1) + scale2(1)) &&
      fabs(pos1(2) - pos2(2)) < 0.5 * (scale1(2) + scale2(2));

  if (collide) {
    double tol[3];
    tol[0] = 0.5 * (scale1(0) + scale2(0)) - fabs(pos1(0) - pos2(0));
    tol[1] = 0.5 * (scale1(1) + scale2(1)) - fabs(pos1(1) - pos2(1));
    tol[2] = 0.5 * (scale1(2) + scale2(2)) - fabs(pos1(2) - pos2(2));

    for (int i = 0; i < 3; ++i) {
      if (tol[i] < tol[(i + 1) % 3] && tol[i] < tol[(i + 2) % 3]) {
        vel1(i) = -vel1(i);
        vel2(i) = -vel2(i);
        obj1.setVelocity(vel1(0), vel1(1), vel1(2));
        obj2.setVelocity(vel2(0), vel2(1), vel2(2));

        if (pos1(i) >= pos2(i)) {
          pos1(i) += tol[i];
          pos2(i) -= tol[i];
        } else {
          pos1(i) -= tol[i];
          pos2(i) += tol[i];
        }
        obj1.setPosition(pos1);
        obj2.setPosition(pos2);

        break;
      }
    }

    return true;
  } else {
    return false;
  }
}

#endif