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



#ifndef _OBJ_PREDICTOR_H_
#define _OBJ_PREDICTOR_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <list>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {

/// @brief 多项式预测轨迹类型定义
class PolynomialPrediction;

/// @brief 障碍物预测轨迹列表类型定义
/// @details 存储所有动态障碍物的预测轨迹
typedef shared_ptr<vector<PolynomialPrediction>> ObjPrediction;

/// @brief 障碍物尺寸列表类型定义
typedef shared_ptr<vector<Eigen::Vector3d>> ObjScale;

/* ========== 多项式预测轨迹类 ========== */
/**
 * @brief PolynomialPrediction类 - 5次多项式轨迹预测
 *
 * 使用5次多项式描述障碍物的未来运动轨迹：
 * p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
 *
 * 多项式系数存储格式：
 * polys[0]: x轴系数 [c0, c1, c2, c3, c4, c5]
 * polys[1]: y轴系数
 * polys[2]: z轴系数
 */
class PolynomialPrediction {
private:
  /// @brief 多项式系数（x, y, z三个维度）
  vector<Eigen::Matrix<double, 6, 1>> polys;
  
  /// @brief 预测时间范围 [t1, t2]
  double t1, t2;

public:
  PolynomialPrediction(/* args */) {
  }
  ~PolynomialPrediction() {
  }

  /// @brief 设置多项式系数
  /// @param pls 系数向量（x, y, z三个6维向量）
  void setPolynomial(vector<Eigen::Matrix<double, 6, 1>>& pls) {
    polys = pls;
  }
  
  /// @brief 设置预测时间范围
  /// @param t1 起始时间
  /// @param t2 结束时间
  void setTime(double t1, double t2) {
    this->t1 = t1;
    this->t2 = t2;
  }

  /// @brief 检查预测是否有效
  /// @return 是否有完整的三维多项式
  bool valid() {
    return polys.size() == 3;
  }

  /// @brief 评估5次多项式轨迹（精确版本）
  /// @param t 查询时间（应在[t1, t2]范围内）
  /// @return 预测位置
  /// @details 使用完整5次多项式计算：
  /// p(t) = [1, t, t^2, t^3, t^4, t^5] · [c0, c1, c2, c3, c4, c5]^T
  Eigen::Vector3d evaluate(double t) {
    Eigen::Matrix<double, 6, 1> tv;
    tv << 1.0, pow(t, 1), pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);

    Eigen::Vector3d pt;
    pt(0) = tv.dot(polys[0]), pt(1) = tv.dot(polys[1]), pt(2) = tv.dot(polys[2]);

    return pt;
  }

  /// @brief 评估常速度模型轨迹（简化版本）
  /// @param t 查询时间
  /// @return 预测位置
  /// @details 假设障碍物做匀速运动，使用2次多项式：
  /// p(t) = p0 + v0*t （线性模型）
  /// 仅使用多项式的前两个系数（常数项和一次项）
  Eigen::Vector3d evaluateConstVel(double t) {
    Eigen::Matrix<double, 2, 1> tv;
    tv << 1.0, pow(t, 1);

    Eigen::Vector3d pt;
    pt(0) = tv.dot(polys[0].head(2)), pt(1) = tv.dot(polys[1].head(2)), pt(2) = tv.dot(polys[2].head(2));

    return pt;
  }
};

/* ========== 障碍物历史数据记录类 ========== */
/**
 * @brief ObjHistory类 - 单个障碍物的历史轨迹记录
 *
 * 订阅障碍物的位置话题，维护其历史位置队列，
 * 用于预测未来运动轨迹。
 */
class ObjHistory {
public:
  /// @brief 静态成员：跳过的消息数量（降采样）
  static int skip_num_;
  
  /// @brief 静态成员：历史队列最大长度
  static int queue_size_;
  
  /// @brief 静态成员：全局起始时间
  static ros::Time global_start_time_;

  ObjHistory() {
  }
  ~ObjHistory() {
  }

  /// @brief 初始化历史记录器
  /// @param id 障碍物ID
  void init(int id);

  /// @brief 位置消息回调
  /// @param msg ROS位姿消息
  /// @details 将位姿消息转换为(x,y,z,t)格式并加入历史队列
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  /// @brief 清空历史记录
  void clear() {
    history_.clear();
  }

  /// @brief 获取历史轨迹
  /// @param his 输出的历史轨迹列表
  void getHistory(list<Eigen::Vector4d>& his) {
    his = history_;
  }

private:
  /// @brief 历史位置队列 [x, y, z, time]
  list<Eigen::Vector4d> history_;
  
  /// @brief 跳过的消息计数
  int skip_;
  
  /// @brief 障碍物索引
  int obj_idx_;
  
  /// @brief 障碍物尺寸
  Eigen::Vector3d scale_;
};

/* ========== 障碍物轨迹预测器 ========== */
/**
 * @brief ObjPredictor类 - 动态障碍物轨迹预测器
 *
 * 功能：
 * 1. 订阅多个障碍物的位置话题
 * 2. 维护每个障碍物的历史轨迹
 * 3. 使用多项式拟合或常速度模型预测未来轨迹
 * 4. 提供预测轨迹和尺寸给规划器使用
 *
 * 预测方法：
 * - 多项式拟合(predictPolyFit): 使用历史数据拟合5次多项式
 * - 常速度(predictConstVel): 假设匀速直线运动
 */
class ObjPredictor {
private:
  ros::NodeHandle node_handle_;

  /// @brief 障碍物数量
  int obj_num_;
  
  /// @brief 轨迹拟合的平滑参数lambda
  double lambda_;
  
  /// @brief 预测更新频率(Hz)
  double predict_rate_;

  /// @brief 位置订阅者列表
  vector<ros::Subscriber> pose_subs_;
  
  /// @brief 障碍物标记订阅者
  ros::Subscriber marker_sub_;
  
  /// @brief 预测定时器
  ros::Timer predict_timer_;
  
  /// @brief 每个障碍物的历史记录器
  vector<shared_ptr<ObjHistory>> obj_histories_;

  /* 与规划器共享的数据 */
  
  /// @brief 预测轨迹列表（输出给规划器）
  ObjPrediction predict_trajs_;
  
  /// @brief 障碍物尺寸列表
  ObjScale obj_scale_;
  
  /// @brief 尺寸初始化标志
  vector<bool> scale_init_;

  /// @brief 障碍物标记回调
  void markerCallback(const visualization_msgs::MarkerConstPtr& msg);

  /// @brief 预测定时器回调
  void predictCallback(const ros::TimerEvent& e);
  
  /// @brief 多项式拟合预测方法
  void predictPolyFit();
  
  /// @brief 常速度预测方法
  void predictConstVel();

public:
  /// @brief 默认构造函数
  ObjPredictor(/* args */);
  
  /// @brief 带节点句柄的构造函数
  ObjPredictor(ros::NodeHandle& node);
  
  /// @brief 析构函数
  ~ObjPredictor();

  /// @brief 初始化预测器
  /// @details 订阅话题，创建定时器
  void init();

  /// @brief 获取预测轨迹
  /// @return 障碍物预测轨迹列表
  ObjPrediction getPredictionTraj();
  
  /// @brief 获取障碍物尺寸
  /// @return 障碍物尺寸列表
  ObjScale getObjScale();

  /// @brief 智能指针类型定义
  typedef shared_ptr<ObjPredictor> Ptr;
};

}  // namespace fast_planner

#endif