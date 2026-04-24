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



#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;

/**
 * @brief 体素哈希函数模板
 * @details 用于将Eigen矩阵（体素索引）转换为哈希值，
 * 支持使用unordered_map存储稀疏体素数据
 *
 * 哈希算法：FNV-1a变体
 * seed ^= hash(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2)
 * 0x9e3779b9 是黄金比例常数的移位版本，用于分散哈希值
 */
template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

/**
 * @brief 地图构建参数结构体
 *
 * 存储SDF地图的所有配置参数，包括：
 * - 地图几何参数（分辨率、尺寸、原点）
 * - 相机内参（深度图像投影用）
 * - 深度图像过滤参数
 * - 射线投射参数（occupancy grid概率模型）
 * - 可视化参数
 */
struct MappingParameters {

  /* 地图属性参数 */
  Eigen::Vector3d map_origin_;      // 地图原点（世界坐标）
  Eigen::Vector3d map_size_;        // 地图尺寸（长宽高，米）
  
  /// @brief 地图边界（世界坐标）
  /// @details 最小边界 = map_origin，最大边界 = map_origin + map_size
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;
  
  /// @brief 地图体素数量
  /// @details 计算公式：map_voxel_num = ceil(map_size / resolution)
  Eigen::Vector3i map_voxel_num_;
  
  Eigen::Vector3i map_min_idx_, map_max_idx_;  // 体素索引边界
  
  /// @brief 局部更新范围
  /// @details 以机器人为中心的立方体区域，只更新该区域内的地图
  Eigen::Vector3d local_update_range_;
  
  double resolution_;              // 地图分辨率（米/体素）
  double resolution_inv_;           // 分辨率倒数（用于加速计算）
  double obstacles_inflation_;      // 障碍物膨胀距离（安全裕度）
  string frame_id_;                 // 坐标系ID
  int pose_type_;                   // 相机位姿来源类型
  string map_input_;                // 地图输入类型：1: pose+depth; 2: odom + cloud

  /* 相机内参（用于深度图像投影） */
  /// @brief 相机主点坐标（像素坐标）
  double cx_, cy_;
  /// @brief 相机焦距（像素单位）
  double fx_, fy_;

  /* 深度图像投影过滤参数 */
  /// @brief 深度图像有效距离范围
  double depth_filter_maxdist_, depth_filter_mindist_;
  /// @brief 深度滤波容差（米）
  double depth_filter_tolerance_;
  int depth_filter_margin_;         // 深度图像边缘忽略像素数
  bool use_depth_filter_;           // 是否启用深度过滤
  double k_depth_scaling_factor_;   // 深度值缩放因子
  int skip_pixel_;                  // 深度图像采样跳过像素数（降采样）

  /* 射线投射/Occupancy Grid概率参数 */
  /// @brief 占据概率模型参数（贝叶斯更新）
  /// p_hit_: 击中障碍物的概率（通常~0.7）
  /// p_miss_: 未击中障碍物的概率（通常~0.35）
  /// p_min_/p_max_: 概率截断边界
  /// p_occ_: 占据判定阈值
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;
  
  /// @brief 占据概率的对数几率(logit)形式
  /// @details 使用logit变换将概率映射到实数域，便于加法运算
  /// logit(p) = log(p / (1-p))
  /// hit/miss使用加法，min/max使用截断
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;
  
  /// @brief 射线有效长度范围
  /// @details 小于min_ray_length的射线被忽略，大于max_ray_length的截断
  double min_ray_length_, max_ray_length_;

  /* 局部地图更新参数 */
  double local_bound_inflate_;      // 局部边界膨胀量
  int local_map_margin_;           // 局部地图边缘余量

  /* 可视化和时间显示参数 */
  /// @brief 可视化截断高度
  /// @details 只显示该高度以下的障碍物，用于RViz可视化
  double esdf_slice_height_, visualization_truncate_height_, virtual_ceil_height_, ground_height_;
  bool show_esdf_time_, show_occ_time_;  // 是否显示ESDF/Occupancy计算时间

  /* 主动建图参数 */
  double unknown_flag_;            // 未知区域标记值
};

// 中间映射数据，用于融合和ESDF计算

/**
 * @brief 映射数据结构体
 *
 * 存储地图构建和更新过程中使用的所有运行时数据，
 * 包括地图数据缓冲区、相机状态、标志位等
 */
struct MappingData {
  // 主地图数据：占据概率和欧几里得距离

  /// @brief 占据概率缓冲区（对数形式）
  /// @details 每个体素存储log-odds形式的占据概率正值
  std::vector<double> occupancy_buffer_;
  
  /// @brief 占据概率负值缓冲区
  std::vector<char> occupancy_buffer_neg;
  
  /// @brief 膨胀占据缓冲区（用于碰撞检测）
  std::vector<char> occupancy_buffer_inflate_;
  
  /// @brief ESDF距离场缓冲区（正向）
  std::vector<double> distance_buffer_;
  
  /// @brief ESDF距离场缓冲区（负向，用于未知区域）
  std::vector<double> distance_buffer_neg_;
  
  /// @brief 完整距离场（正负合并）
  std::vector<double> distance_buffer_all_;
  
  /// @brief 临时缓冲区（用于ESDF计算）
  std::vector<double> tmp_buffer1_;
  std::vector<double> tmp_buffer2_;

  // 相机位置和姿态数据

  /// @brief 当前相机位置（世界坐标系）
  Eigen::Vector3d camera_pos_, last_camera_pos_;
  
  /// @brief 当前相机姿态（四元数表示）
  Eigen::Quaterniond camera_q_, last_camera_q_;

  // 深度图像数据

  /// @brief 当前帧和上一帧深度图像
  cv::Mat depth_image_, last_depth_image_;
  
  /// @brief 图像帧计数器（用于双缓冲）
  int image_cnt_;

  // 地图状态标志位

  /// @brief 需要更新标志
  /// occ_need_update_: 需要更新占据地图
  /// local_updated_: 局部地图已更新
  /// esdf_need_update_: 需要更新ESDF
  bool occ_need_update_, local_updated_, esdf_need_update_;
  
  /// @brief 是否已接收第一帧深度图
  bool has_first_depth_;
  
  /// @brief 是否已有里程计/点云数据
  bool has_odom_, has_cloud_;

  // 深度图像投影点云

  /// @brief 投影后的3D点云（用于地图更新）
  vector<Eigen::Vector3d> proj_points_;
  
  /// @brief 有效投影点数量
  int proj_points_cnt;

  // 射线投射加速标志缓冲区

  /// @brief 命中计数缓冲区
  /// count_hit_: 被射线命中的次数
  /// count_hit_and_miss_: 被射线或空白命中的次数
  vector<short> count_hit_, count_hit_and_miss_;
  
  /// @brief 遍历标志和射线终点标志
  vector<char> flag_traverse_, flag_rayend_;
  
  /// @brief 射线投射轮次编号（用于避免重复处理）
  char raycast_num_;
  
  /// @brief 待处理体素队列（用于ESDF更新）
  queue<Eigen::Vector3i> cache_voxel_;

  // ESDF更新范围

  /// @brief 局部ESDF更新边界
  Eigen::Vector3i local_bound_min_, local_bound_max_;

  // 计算时间统计

  /// @brief 融合和ESDF计算时间
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  
  /// @brief 更新次数统计
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief SDFMap类 - 有符号距离场地图
 *
 * 实现基于TSDF(Truncated Signed Distance Function)的地图构建系统。
 * 主要功能：
 * 1. 接收深度图像/点云数据，构建3D占据地图
 * 2. 使用射线投射更新占据概率
 * 3. 计算欧几里得距离场(ESDF)用于轨迹优化
 * 4. 支持局部地图更新以提高效率
 *
 * 坐标系约定：
 * - 世界坐标系：右手系，Z轴向上
 * - 地图索引：X向右，Y向前，Z向上
 *
 * @see paper: "A Volumetric Method for Building Complex Models from Range Images"
 */
class SDFMap {
public:
  SDFMap() {}
  ~SDFMap() {}

  /// @brief 位姿数据类型枚举
  enum {
    POSE_STAMPED = 1,   ///< 使用geometry_msgs::PoseStamped作为位姿源
    ODOMETRY = 2,       ///< 使用nav_msgs::Odometry作为位姿源
    INVALID_IDX = -10000 ///< 无效索引标记
  };

  // ===== 占据地图管理 =====

  /// @brief 重置整个地图缓冲区
  /// @details 将所有占据概率设为未知，并清空ESDF
  void resetBuffer();
  
  /// @brief 重置指定区域的地图缓冲区
  /// @param min 区域最小角点（世界坐标）
  /// @param max 区域最大角点（世界坐标）
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  /// @brief 世界坐标转换为体素索引
  /// @param pos 世界坐标位置
  /// @param id 输出的体素索引
  /// @details 公式: id = floor((pos - origin) / resolution)
  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  
  /// @brief 体素索引转换为世界坐标（返回体素中心）
  /// @param id 体素索引
  /// @param pos 输出的世界坐标（体素中心）
  /// @details 公式: pos = (id + 0.5) * resolution + origin
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  
  /// @brief 体素索引转换为一维数组地址
  /// @param id 三维体素索引
  /// @return 一维数组索引
  /// @details 地址 = id.x + id.y * width + id.z * width * height
  inline int toAddress(const Eigen::Vector3i& id);
  
  /// @brief 三个整数坐标转换为一维地址（重载版本）
  inline int toAddress(int& x, int& y, int& z);
  
  /// @brief 检查世界坐标是否在地图范围内
  inline bool isInMap(const Eigen::Vector3d& pos);
  
  /// @brief 检查体素索引是否在地图范围内
  inline bool isInMap(const Eigen::Vector3i& idx);

  /// @brief 设置指定位置的占据概率
  /// @param pos 世界坐标
  /// @param occ 占据概率值（0-1之间，1表示障碍物）
  /// @details 使用贝叶斯更新：log-odds形式累加
  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  
  /// @brief 设置指定位置为已占据（障碍物）
  inline void setOccupied(Eigen::Vector3d pos);
  
  /// @brief 获取世界坐标位置的占据状态
  /// @return 占据状态：-1未知，0空闲，1占据
  inline int getOccupancy(Eigen::Vector3d pos);
  
  /// @brief 获取体素索引的占据状态
  inline int getOccupancy(Eigen::Vector3i id);
  
  /// @brief 获取膨胀后的占据状态（用于碰撞检测）
  inline int getInflateOccupancy(Eigen::Vector3d pos);

  /// @brief 将体素索引限制在地图范围内
  inline void boundIndex(Eigen::Vector3i& id);
  
  /// @brief 检查体素是否为未知状态
  inline bool isUnknown(const Eigen::Vector3i& id);
  
  /// @brief 检查世界坐标位置是否为未知状态
  inline bool isUnknown(const Eigen::Vector3d& pos);
  
  /// @brief 检查体素是否为已知空闲区域
  inline bool isKnownFree(const Eigen::Vector3i& id);
  
  /// @brief 检查体素是否为已知障碍物
  inline bool isKnownOccupied(const Eigen::Vector3i& id);

  // ===== 距离场管理 =====

  /// @brief 获取世界坐标位置的ESDF距离值
  /// @param pos 世界坐标
  /// @return 到最近障碍物的欧几里得距离
  inline double getDistance(const Eigen::Vector3d& pos);
  
  /// @brief 获取体素索引的ESDF距离值
  inline double getDistance(const Eigen::Vector3i& id);
  
  /// @brief 获取带梯度的距离值（三线性插值）
  /// @param pos 查询位置（世界坐标）
  /// @param grad 输出的梯度向量（指向最近障碍物）
  /// @return 插值后的距离值
  /// @details 使用三线性插值获得平滑的距离场和梯度
  inline double getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad);
  
  /// @brief 获取包围某点的8个体素角点
  /// @param pos 查询位置
  /// @param pts 输出的8个角点坐标
  /// @param diff 相对于最小角点的偏移量（0-1之间）
  /// @details 用于三线性插值准备
  void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d& diff);

  /// @brief 更新整个ESDF距离场
  /// @details 使用BFS（广度优先搜索）从占据体素向外传播距离
  /// 算法：从所有占据体素开始，使用3D BFS更新距离
  void updateESDF3d();
  
  /// @brief 获取指定高度的ESDF切片
  /// @param height 切片高度
  /// @param res 切片分辨率
  /// @param range [x_min, x_max, y_min, y_max]切片范围
  /// @param slice 输出的点坐标列表
  /// @param grad 输出的梯度列表
  /// @param sign 距离场类型：1正向，2负向，3组合
  void getSliceESDF(const double height, const double res, const Eigen::Vector4d& range,
                    vector<Eigen::Vector3d>& slice, vector<Eigen::Vector3d>& grad,
                    int sign = 1);
                    
  /// @brief 初始化地图系统
  /// @param nh ROS节点句柄
  /// @details 从参数服务器加载配置，创建订阅者和发布者
  void initMap(ros::NodeHandle& nh);

  // ===== 可视化发布 =====

  /// @brief 发布占据地图（RViz可视化）
  void publishMap();
  
  /// @brief 发布膨胀后的占据地图
  /// @param all_info 是否发布完整信息
  void publishMapInflate(bool all_info = false);
  
  /// @brief 发布ESDF距离场
  void publishESDF();
  
  /// @brief 发布地图更新区域
  void publishUpdateRange();

  /// @brief 发布未知区域标记
  void publishUnknown();
  
  /// @brief 发布深度图像
  void publishDepth();

  // ===== 状态查询 =====

  /// @brief 检查距离场计算是否正确（调试用）
  void checkDist();
  
  /// @brief 是否已接收深度观测
  bool hasDepthObservation();
  
  /// @brief 里程计数据是否有效
  bool odomValid();
  
  /// @brief 获取地图区域信息
  /// @param ori 地图原点输出
  /// @param size 地图尺寸输出
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  
  /// @brief 获取地图分辨率
  double getResolution();
  
  /// @brief 获取地图原点
  Eigen::Vector3d getOrigin();
  
  /// @brief 获取体素总数
  int getVoxelNum();

  /// @brief 智能指针类型定义
  typedef std::shared_ptr<SDFMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /// @brief 地图参数（配置）
  MappingParameters mp_;
  
  /// @brief 映射数据（运行时）
  MappingData md_;

  /// @brief ESDF填充函数模板
  /// @tparam F_get_val 获取值的函数
  /// @tparam F_set_val 设置值的函数
  /// @param f_get_val 获取值回调
  /// @param f_set_val 设置值回调
  /// @param start 起始索引
  /// @param end 结束索引
  /// @param dim 维度
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

  // ===== 回调函数 =====

  /// @brief 深度图像+位姿同步回调
  void depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                         const geometry_msgs::PoseStampedConstPtr& pose);
                         
  /// @brief 深度图像+里程计同步回调
  void depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
  
  /// @brief 独立深度图像回调
  void depthCallback(const sensor_msgs::ImageConstPtr& img);
  
  /// @brief 点云回调
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img);
  
  /// @brief 独立位姿回调
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose);
  
  /// @brief 独立里程计回调
  void odomCallback(const nav_msgs::OdometryConstPtr& odom);

  // ===== 定时器回调 =====

  /// @brief 占据地图更新定时器回调
  void updateOccupancyCallback(const ros::TimerEvent& /*event*/);
  
  /// @brief ESDF更新定时器回调
  void updateESDFCallback(const ros::TimerEvent& /*event*/);
  
  /// @brief 可视化定时器回调
  void visCallback(const ros::TimerEvent& /*event*/);

  // ===== 核心更新函数 =====

  /// @brief 深度图像投影
  /// @details 将2D深度图像反投影为3D点云
  void projectDepthImage();
  
  /// @brief 射线投射处理
  /// @details 对投影点执行射线投射，更新占据概率
  void raycastProcess();
  
  /// @brief 局部地图清理和膨胀
  /// @details 清除旧的局部区域，膨胀障碍物用于碰撞检测
  void clearAndInflateLocalMap();

  /// @brief 障碍物膨胀函数
  /// @param pt 起点体素索引
  /// @param step 膨胀步数
  /// @param pts 输出的膨胀后体素列表
  inline void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
  
  /// @brief 设置体素占据状态（带缓存）
  /// @param pos 世界坐标
  /// @param occ 占据状态
  /// @return 处理结果
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  
  /// @brief 计算地图中最接近的点
  /// @param pt 查询点
  /// @param camera_pt 相机位置
  /// @return 地图中最接近的点
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);

  // ===== 消息过滤器类型定义 =====

  /// @brief 近似时间同步策略（深度图+里程计）
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;
      
  /// @brief 近似时间同步策略（深度图+位姿）
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
      
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  // ===== ROS通信 =====

  ros::NodeHandle node_;  ///< ROS节点句柄
  
  /// @brief 消息过滤器订阅者
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  
  /// @brief 时间同步器
  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;

  /// @brief 独立订阅者（未使用时间同步）
  ros::Subscriber indep_depth_sub_, indep_odom_sub_, indep_pose_sub_, indep_cloud_sub_;
  
  /// @brief 发布者
  ros::Publisher map_pub_, esdf_pub_, map_inf_pub_, update_range_pub_;
  ros::Publisher unknown_pub_, depth_pub_;
  ros::Timer occ_timer_, esdf_timer_, vis_timer_;

  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
};

/* ============================== definition of inline function
 * ============================== */

inline int SDFMap::toAddress(const Eigen::Vector3i& id) {
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int SDFMap::toAddress(int& x, int& y, int& z) {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void SDFMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
  id = id1;
}

inline double SDFMap::getDistance(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  boundIndex(id);

  return md_.distance_buffer_all_[toAddress(id)];
}

inline double SDFMap::getDistance(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.distance_buffer_all_[toAddress(id1)];
}

inline bool SDFMap::isUnknown(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool SDFMap::isUnknown(const Eigen::Vector3d& pos) {
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool SDFMap::isKnownFree(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool SDFMap::isKnownOccupied(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline double SDFMap::getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad) {
  if (!isInMap(pos)) {
    grad.setZero();
    return 0;
  }

  /* use trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();

  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);

  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);

  diff = (pos - idx_pos) * mp_.resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }
    }
  }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * mp_.resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_.resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

  grad[0] *= mp_.resolution_inv_;

  return dist;
}

inline void SDFMap::setOccupied(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                id(1) * mp_.map_voxel_num_(2) + id(2)] = 1;
}

inline void SDFMap::setOccupancy(Eigen::Vector3d pos, double occ) {
  if (occ != 1 && occ != 0) {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_[toAddress(id)] = occ;
}

inline int SDFMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int SDFMap::getInflateOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

inline int SDFMap::getOccupancy(Eigen::Vector3i id) {
  if (id(0) < 0 || id(0) >= mp_.map_voxel_num_(0) || id(1) < 0 || id(1) >= mp_.map_voxel_num_(1) ||
      id(2) < 0 || id(2) >= mp_.map_voxel_num_(2))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool SDFMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline bool SDFMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
      idx(2) > mp_.map_voxel_num_(2) - 1) {
    return false;
  }
  return true;
}

inline void SDFMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void SDFMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline void SDFMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
  int num = 0;

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z) {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}

#endif