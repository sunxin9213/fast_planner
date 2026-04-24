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



#include <plan_env/edt_environment.h>

namespace fast_planner {

/* ============================== EDT环境类实现 ==============================
 */

/**
 * @brief 初始化EDT环境
 * @details 目前为空实现，预留给未来扩展
 */
void EDTEnvironment::init() {
}

/**
 * @brief 设置SDF地图
 * @param map SDF地图智能指针
 * @details 存储地图引用并预计算分辨率倒数以加速后续计算
 */
void EDTEnvironment::setMap(shared_ptr<SDFMap> map) {
  this->sdf_map_ = map;
  // 预计算分辨率倒数，后续使用乘法代替除法可提升性能
  resolution_inv_ = 1 / sdf_map_->getResolution();
}

/**
 * @brief 设置动态障碍物预测轨迹
 * @param prediction 障碍物预测轨迹列表
 */
void EDTEnvironment::setObjPrediction(ObjPrediction prediction) {
  this->obj_prediction_ = prediction;
}

/**
 * @brief 设置动态障碍物尺寸
 * @param scale 障碍物尺寸列表
 */
void EDTEnvironment::setObjScale(ObjScale scale) {
  this->obj_scale_ = scale;
}

/**
 * @brief 计算点到障碍物包围盒的最小距离
 * @param idx 障碍物索引
 * @param pos 查询点位置
 * @param time 当前时间（用于获取障碍物预测位置）
 * @return 点到障碍物AABB包围盒的最小距离
 * @details 算法：
 * 1. 根据时间获取障碍物当前位置（使用常速度模型预测）
 * 2. 计算障碍物的AABB包围盒（当前位置 ± 0.5*尺寸）
 * 3. 对每个维度：
 *    - 如果点在范围内，该维度距离为0
 *    - 否则取到最近边界的距离
 * 4. 返回3D欧几里得距离
 *
 * 数学公式：
 * dist_i = 0,                           if box_min(i) <= pos(i) <= box_max(i)
 *         min(|pos(i) - box_min(i)|,   otherwise
 *             |pos(i) - box_max(i)|)
 *
 * distance = sqrt(dist_x² + dist_y² + dist_z²)
 */
double EDTEnvironment::distToBox(int idx, const Eigen::Vector3d& pos, const double& time) {
  // 使用常速度模型预测障碍物当前位置（简化计算）
  // 也可以使用evaluate()获得5次多项式精确预测
  // Eigen::Vector3d pos_box = obj_prediction_->at(idx).evaluate(time);
  Eigen::Vector3d pos_box = obj_prediction_->at(idx).evaluateConstVel(time);

  // 计算AABB包围盒边界
  Eigen::Vector3d box_max = pos_box + 0.5 * obj_scale_->at(idx);
  Eigen::Vector3d box_min = pos_box - 0.5 * obj_scale_->at(idx);

  Eigen::Vector3d dist;

  // 对每个维度计算最短距离
  for (int i = 0; i < 3; i++) {
    // 如果点在包围盒内，该维度距离为0
    // 否则取到最近面的距离
    dist(i) = pos(i) >= box_min(i) && pos(i) <= box_max(i) ? 0.0 : min(fabs(pos(i) - box_min(i)),
                                                                       fabs(pos(i) - box_max(i)));
  }

  // 返回3D欧几里得距离
  return dist.norm();
}

/**
 * @brief 计算到所有动态障碍物的最小距离
 * @param pos 查询点位置
 * @param time 当前时间
 * @return 到所有障碍物的最小距离
 * @details 遍历所有障碍物，找到最近的障碍物距离
 */
double EDTEnvironment::minDistToAllBox(const Eigen::Vector3d& pos, const double& time) {
  double dist = 10000000.0;
  for (int i = 0; i < obj_prediction_->size(); i++) {
    double di = distToBox(i, pos, time);
    if (di < dist) dist = di;
  }

  return dist;
}

/**
 * @brief 获取包围某点的8个体素的距离值
 * @param pts 8个体素角点坐标（2x2x2数组）
 * @param dists 对应的距离值输出（2x2x2数组）
 * @details 用于三线性插值前的采样准备
 */
void EDTEnvironment::getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]) {
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        // 从SDF地图获取每个角点的距离值
        dists[x][y][z] = sdf_map_->getDistance(pts[x][y][z]);
      }
    }
  }
}

void EDTEnvironment::interpolateTrilinear(double values[2][2][2],
                                          const Eigen::Vector3d& diff,
                                          double& value,
                                          Eigen::Vector3d& grad) {
  // trilinear interpolation
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

  value = (1 - diff(2)) * v0 + diff(2) * v1;

  grad[2] = (v1 - v0) * resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= resolution_inv_;
}

void EDTEnvironment::evaluateEDTWithGrad(const Eigen::Vector3d& pos,
                                         double time, double& dist,
                                         Eigen::Vector3d& grad) {
  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  sdf_map_->getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinear(dists, diff, dist, grad);
}

double EDTEnvironment::evaluateCoarseEDT(Eigen::Vector3d& pos, double time) {
  double d1 = sdf_map_->getDistance(pos);
  if (time < 0.0) {
    return d1;
  } else {
    double d2 = minDistToAllBox(pos, time);
    return min(d1, d2);
  }
}

// EDTEnvironment::
}  // namespace fast_planner