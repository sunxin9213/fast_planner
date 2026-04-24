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



#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

/**
 * @brief 符号函数
 * @param x 输入值
 * @return -1, 0, 或 1
 * @details 用于确定光线投射的方向：负数为-1，零为0，正数为1
 */
double signum(double x);

/**
 * @brief 模运算函数
 * @param value 被模运算的值
 * @param modulus 模数
 * @return 标准化到[0, modulus)范围内的值
 * @details 处理负数的模运算，确保结果始终为正数
 */
double mod(double value, double modulus);

/**
 * @brief 计算光线与单位立方体边界的交点时间
 * @param s 起始位置（某个坐标分量）
 * @param ds 方向分量（该坐标的增量）
 * @return 到达下一个整数坐标的最小正时间t
 * @details 算法原理：
 * - 如果方向为负，则取反s和ds后递归处理
 * - 将s标准化到[0,1)区间
 * - 求解s + t*ds = 1，得到t = (1-s)/ds
 * - 这对应光线首次穿过当前体素右边界的时间
 */
double intbound(double s, double ds);

/**
 * @brief 光线投射函数 - 数组版本
 * @param start 光线起点
 * @param end 光线终点
 * @param min 地图最小边界
 * @param max 地图最大边界
 * @param output_points_cnt 输出的体素点数量
 * @param output 输出的体素中心点数组
 * @details 基于"A Fast Voxel Traversal Algorithm for Ray Tracing" (Amanatides & Woo, 1987)
 * 算法核心思想：
 * 1. 计算光线经过的每个体素坐标
 * 2. 使用tMaxX, tMaxY, tMaxZ跟踪到达下一个体素边界的时间
 * 3. 每次选择最小tMax对应的轴方向前进
 * 4. 避免对每个体素进行碰撞检测，大幅提升效率
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);

/**
 * @brief 光线投射函数 - 向量版本
 * @param start 光线起点
 * @param end 光线终点
 * @param min 地图最小边界
 * @param max 地图最大边界
 * @param output 输出的体素中心点动态数组
 * @details 与数组版本功能相同，但使用vector存储结果，更灵活
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

/**
 * @brief RayCaster类 - 体素光线投射器
 *
 * 实现Amanatides & Woo (1987)的快速体素遍历算法，
 * 用于在体素地图中进行光线投射，支持 occupancy grid 更新和ESDF计算。
 *
 * 算法原理：
 * - 将光线参数化为 P(t) = origin + t * direction
 * - 不直接存储t，而是维护tMaxX, tMaxY, tMaxZ表示到达下一个体素边界的时间
 * - 每次选择最小的tMax对应的方向前进
 * - tDelta = 1/|direction| 表示穿过一个体素需要的时间增量
 */
class RayCaster {
private:
  /* 射线参数 */
  Eigen::Vector3d start_;      // 射线起点
  Eigen::Vector3d end_;       // 射线终点
  Eigen::Vector3d direction_; // 射线方向向量（归一化后）
  
  /* 地图边界 */
  Eigen::Vector3d min_;       // 地图最小边界
  Eigen::Vector3d max_;       // 地图最大边界
  
  /* 当前体素坐标 */
  int x_;                     // 当前体素X索引
  int y_;                     // 当前体素Y索引
  int z_;                     // 当前体素Z索引
  
  /* 目标体素坐标 */
  int endX_;                  // 终点体素X索引
  int endY_;                  // 终点体素Y索引
  int endZ_;                  // 终点体素Z索引
  
  /* 距离相关 */
  double maxDist_;           // 射线最大长度（平方）
  double dist_;               // 当前遍历距离
  
  /* 方向分量 */
  double dx_;                // 方向X分量
  double dy_;                // 方向Y分量
  double dz_;                // 方向Z分量
  
  /* 步进方向（+1或-1） */
  int stepX_;                // X方向步进
  int stepY_;                // Y方向步进
  int stepZ_;                // Z方向步进
  
  /* 到达下一个边界的时间 */
  double tMaxX_;             // 到达下一个X边界的时间
  double tMaxY_;             // 到达下一个Y边界的时间
  double tMaxZ_;             // 到达下一个Z边界的时间
  
  /* 时间增量 */
  double tDeltaX_;            // 穿过一个X体素的时间
  double tDeltaY_;           // 穿过一个Y体素的时间
  double tDeltaZ_;           // 穿过一个Z体素的时间
  
  /* 步数计数 */
  int step_num_;             // 已遍历的体素数量

public:
  RayCaster(/* args */) {
  }
  ~RayCaster() {
  }

  /**
   * @brief 设置射线输入参数
   * @param start 射线起点
   * @param end 射线终点
   * @return 是否设置成功
   * @details 执行以下初始化：
   * 1. 计算方向向量并归一化
   * 2. 计算起点和终点的体素索引
   * 3. 初始化tMax和tDelta参数
   * 4. 确定三个方向的步进符号
   */
  bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  /**
   * @brief 执行单步射线遍历
   * @param ray_pt 输出的体素中心点坐标
   * @return 是否还有更多体素需要遍历
   * @details 算法流程：
   * 1. 比较tMaxX, tMaxY, tMaxZ
   * 2. 选择最小的tMax对应的方向前进
   * 3. 更新当前体素坐标
   * 4. 计算新的tMax值
   * 5. 返回下一个体素中心点
   */
  bool step(Eigen::Vector3d& ray_pt);
};

#endif  // RAYCAST_H_