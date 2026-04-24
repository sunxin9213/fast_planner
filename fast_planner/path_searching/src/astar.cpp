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



#include <path_searching/astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace fast_planner {
Astar::~Astar() {
  for (int i = 0; i < allocate_num_; i++) {
    delete path_node_pool_[i];
  }
}

/**
 * @brief A*路径搜索主函数
 *
 * A*算法是一种启发式搜索算法，用于在已知地图中寻找从起点到终点的最优路径
 * 核心公式: f(n) = g(n) + h(n)
 * - g(n): 从起点到当前节点的实际代价
 * - h(n): 从当前节点到终点的启发式估计代价
 * - f(n): 节点的总代价估计，用于优先队列排序
 *
 * @param start_pt 起点位置 [x, y, z]
 * @param end_pt 终点位置 [x, y, z]
 * @param dynamic 是否考虑动态障碍物(时间维度)
 * @param time_start 搜索开始时间(用于动态障碍物)
 * @return int 搜索结果: REACH_END(成功到达), NO_PATH(无路径), 其他
 */
int Astar::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool dynamic, double time_start) {
  /* ---------- 初始化搜索环境 ---------- */
  // 从预分配的节点池中获取起始节点(避免运行时动态内存分配)
  NodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;              // 起始节点无父节点
  cur_node->position = start_pt;        // 设置起点位置
  cur_node->index = posToIndex(start_pt);  // 转换为网格索引
  cur_node->g_score = 0.0;              // 起点到自身的代价为0

  Eigen::Vector3d end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;

  // 计算终点的网格索引
  end_index = posToIndex(end_pt);
  
  // f_score = g_score + h_score (启发式估计)
  // lambda_heu_ 是启发式权重，用于平衡实际代价和估计代价
  // 使用欧氏距离作为启发式函数: h(n) = ||end - current||
  cur_node->f_score = lambda_heu_ * getEuclHeu(cur_node->position, end_pt);
  cur_node->node_state = IN_OPEN_SET;  // 标记为在开放集中

  // 将起始节点加入最小堆优先队列(按f_score升序排列)
  open_set_.push(cur_node);
  use_node_num_ += 1;

  // 动态搜索需要维护时间维度
  if (dynamic) {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    // 插入到扩展节点哈希表中，键为(位置索引, 时间索引)
    // 用于O(1)时间复杂度查找节点是否已被扩展
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
  } else
    // 静态搜索只使用位置索引
    expanded_nodes_.insert(cur_node->index, cur_node);

  // 邻居节点和终止节点指针
  NodePtr neighbor = NULL;
  NodePtr terminate_node = NULL;

  /* ---------- A*搜索主循环 ---------- */
  // 当开放集(优先队列)不为空时持续搜索
  // 每次取出f_score最小的节点进行扩展
  while (!open_set_.empty()) {
    /* ---------- 从开放集中取出f_score最小的节点 ---------- */
    // 优先队列的top()返回最高优先级的元素(即f_score最小的节点)
    cur_node = open_set_.top();

    /* ---------- 判断是否到达终点 ---------- */
    // 使用1-范数距离判断: 检查当前节点是否在终点1个网格范围内
    // 这种宽松的终止条件可以减少搜索时间，同时保证路径质量
    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 && abs(cur_node->index(2) - end_index(2)) <= 1;

    if (reach_end) {
      // 成功到达终点，通过回溯父指针获取完整路径
      terminate_node = cur_node;
      retrievePath(terminate_node);
      has_path_ = true;

      return REACH_END;
    }

    /* ---------- 弹出节点并加入关闭集 ---------- */
    // 当前节点已处理完毕，从开放集移除并加入关闭集
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;  // 统计迭代次数

    /* ---------- 邻居节点扩展初始化 ---------- */
    // 当前节点位置
    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d pro_pos;  // 候选邻居位置
    double pro_t;  // 候选邻居时间

    // 输入集合(候选位移: 26连通域)
    vector<Eigen::Vector3d> inputs;
    Eigen::Vector3d d_pos;  // 位置增量(x,y,z三个方向)

    /* ---------- 扩展循环: 遍历26个方向(3x3x3-1) ---------- */
    // 包括上下左右前后及对角线方向，共26个邻居
    for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
      for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
        for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
          d_pos << dx, dy, dz;

          // 跳过自身位置(零位移)
          if (d_pos.norm() < 1e-3) continue;

          // 候选位置 = 当前节点位置 + 位移增量
          pro_pos = cur_pos + d_pos;

          /* ---------- 可行性检查 ---------- */
          /* 检查1: 是否在地图范围内 */
          // 边界检查，确保不超出地图
          if (pro_pos(0) <= origin_(0) || pro_pos(0) >= map_size_3d_(0) || pro_pos(1) <= origin_(1) ||
              pro_pos(1) >= map_size_3d_(1) || pro_pos(2) <= origin_(2) ||
              pro_pos(2) >= map_size_3d_(2)) {
            continue;
          }

          /* 检查2: 是否已在关闭集中(避免重复扩展) */
          Eigen::Vector3i pro_id = posToIndex(pro_pos);
          int pro_t_id = timeToIndex(pro_t);

          NodePtr pro_node =
              dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);

          if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
            continue;  // 已扩展过的节点，跳过
          }

          /* 检查3: 碰撞检测 - 是否与障碍物安全距离 */
          // 使用欧几里得距离场(EDT)评估与障碍物的距离
          // EDT可以从SDF(有符号距离场)快速查询
          // dist > margin_ 表示在障碍物安全距离之外
          double dist = edt_environment_->evaluateCoarseEDT(pro_pos, -1.0);
          if (dist <= margin_) {
            continue;  // 距离障碍物太近，不安全
          }

          /* ---------- 代价计算 ---------- */
          // g_score: 从起点到该节点的实际代价
          // 使用squaredNorm()计算位移的平方和，相当于欧氏距离的平方
          // 这种计算方式比开方更快
          double time_to_goal, tmp_g_score, tmp_f_score;
          tmp_g_score = d_pos.squaredNorm() + cur_node->g_score;
          // f_score = g_score + 启发式估计
          // 启发式函数使用欧氏距离，估计从当前位置到终点的代价
          tmp_f_score = tmp_g_score + lambda_heu_ * getEuclHeu(pro_pos, end_pt);

          // 根据节点状态进行处理
          if (pro_node == NULL) {
            // 情况1: 新发现的节点，从池中分配
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->position = pro_pos;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->parent = cur_node;  // 记录父节点用于回溯
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic) {
              pro_node->time = cur_node->time + 1.0;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            // 加入优先队列和扩展节点表
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              cout << "节点池耗尽，无可用内存!" << endl;
              return NO_PATH;
            }
          } else if (pro_node->node_state == IN_OPEN_SET) {
            // 情况2: 已在开放集中，检查是否需要更新
            // 如果通过当前路径到达该节点代价更小，则更新
            if (tmp_g_score < pro_node->g_score) {
              pro_node->position = pro_pos;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->parent = cur_node;
              if (dynamic) pro_node->time = cur_node->time + 1.0;
              // 注意: STL优先队列不支持更新操作
              // 实际代码中会重新push，旧节点会被忽略(因为有状态检查)
            }
          } else {
            cout << "搜索状态错误: " << pro_node->node_state << endl;
          }
        }
  }

  /* ---------- 开放集为空，无路径 ---------- */
  // 优先队列为空，说明所有可达节点都已扩展但未到达终点
  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

/**
 * @brief 设置A*算法参数
 *
 * 从ROS参数服务器读取搜索参数，包括:
 * - resolution_: 空间搜索分辨率
 * - time_resolution_: 时间分辨率(动态障碍物)
 * - lambda_heu_: 启发式权重(通常1.0-10.0)
 * - margin_: 障碍物安全距离
 * - allocate_num_: 预分配节点池大小
 *
 * tie_breaker_用于打破启发式函数的平局，确保搜索更倾向于探索新区域
 */
void Astar::setParam(ros::NodeHandle& nh) {
  nh.param("astar/resolution_astar", resolution_, -1.0);      // 网格分辨率
  nh.param("astar/time_resolution", time_resolution_, -1.0);   // 时间分辨率
  nh.param("astar/lambda_heu", lambda_heu_, -1.0);            // 启发式权重
  nh.param("astar/margin", margin_, -1.0);                   // 障碍物安全距离
  nh.param("astar/allocate_num", allocate_num_, -1);        // 预分配节点数
  
  // tie_breaker_用于打破启发式函数的平局
  // 略微增加启发式代价，确保搜索更倾向于探索新区域
  tie_breaker_ = 1.0 + 1.0 / 10000;

  cout << "margin:" << margin_ << endl;
}

/**
 * @brief 回溯获取搜索路径
 *
 * 从终止节点沿着父指针回溯到起点，构建完整路径
 * 这是A*算法的标准路径恢复方法
 */
void Astar::retrievePath(NodePtr end_node) {
  NodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  // 沿着父指针链不断回溯，将节点加入路径
  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  // 反转顺序得到从起点到终点的路径
  reverse(path_nodes_.begin(), path_nodes_.end());
}

/**
 * @brief 获取最终路径点序列
 *
 * 将路径节点转换为位置向量序列，供后续模块使用
 */
std::vector<Eigen::Vector3d> Astar::getPath() {
  vector<Eigen::Vector3d> path;
  for (int i = 0; i < path_nodes_.size(); ++i) {
    path.push_back(path_nodes_[i]->position);
  }
  return path;
}

/**
 * @brief 对角线启发式函数(D*算法)
 *
 * 计算考虑三维对角线移动的最短估计代价
 * 比单纯的曼哈顿距离更接近实际最短路径
 *
 * 算法思想:
 * - 首先尽可能进行对角线移动(距离为sqrt(3))
 * - 剩余轴向移动使用直线或对角线
 *
 * @param x1 起点位置
 * @param x2 终点位置
 * @return double 估计的最短代价
 */
double Astar::getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  double dx = fabs(x1(0) - x2(0));  // x方向距离
  double dy = fabs(x1(1) - x2(1));  // y方向距离
  double dz = fabs(x1(2) - x2(2));  // z方向距离

  double h;
  int diag = min(min(dx, dy), dz);  // 最大对角线移动步数
  dx -= diag;
  dy -= diag;
  dz -= diag;

  // 根据剩余距离计算代价
  if (dx < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  
  // 乘以tie_breaker打破平局
  return tie_breaker_ * h;
}

/**
 * @brief 曼哈顿启发式函数
 *
 * 假设只能沿轴向移动的估计代价
 * 适用于四连通或六连通网格
 *
 * h(n) = |x1-x2| + |y1-y2| + |z1-z2|
 */
double Astar::getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));

  return tie_breaker_ * (dx + dy + dz);
}

/**
 * @brief 欧几里得启发式函数
 *
 * 使用两点间的欧氏距离作为估计代价
 * 这是 admissable (可采纳)的启发式，即不会高估实际代价
 * 对于允许对角线移动的场景，这是最优的启发式
 *
 * h(n) = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
 */
double Astar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  return tie_breaker_ * (x2 - x1).norm();
}

/**
 * @brief 初始化A*搜索器
 *
 * 预分配节点内存池，避免运行时动态内存分配
 * 初始化地图相关参数
 */
void Astar::init() {
  /* ---------- 地图参数 ---------- */
  this->inv_resolution_ = 1.0 / resolution_;        // 分辨率倒数
  inv_time_resolution_ = 1.0 / time_resolution_;    // 时间分辨率倒数
  edt_environment_->getMapRegion(origin_, map_size_3d_);  // 获取地图边界

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  /* ---------- 预分配节点内存池 ---------- */
  // 预先分配大量节点，避免动态分配开销
  // 这是高性能路径规划的关键优化
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new Node;
  }

  use_node_num_ = 0;   // 已使用节点计数
  iter_num_ = 0;       // 迭代次数统计
}

/**
 * @brief 设置环境指针
 *
 * 绑定EDT环境对象，用于地图查询和碰撞检测
 */
void Astar::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

/**
 * @brief 重置搜索器状态
 *
 * 清除所有已使用的节点和路径，准备下一次搜索
 * 节点池中的节点被复用，而不是重新分配
 */
void Astar::reset() {
  expanded_nodes_.clear();  // 清空扩展节点哈希表
  path_nodes_.clear();      // 清空路径节点

  // 清空优先队列(通过swap交换空队列)
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
  open_set_.swap(empty_queue);

  // 重置所有已使用节点的状态
  for (int i = 0; i < use_node_num_; i++) {
    NodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
}

/**
 * @brief 获取所有访问过的节点
 *
 * 用于可视化搜索过程和调试
 * 返回所有曾被加入开放集的节点
 */
std::vector<NodePtr> Astar::getVisitedNodes() {
  vector<NodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

/**
 * @brief 位置转网格索引
 *
 * 将连续世界坐标转换为离散网格索引
 * idx = floor((pt - origin) / resolution)
 *
 * @param pt 世界坐标
 * @return Vector3i 网格索引
 */
Eigen::Vector3i Astar::posToIndex(Eigen::Vector3d pt) {
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

  return idx;
}

/**
 * @brief 时间转时间索引
 *
 * 将连续时间转换为离散时间索引，用于动态障碍物查询
 */
int Astar::timeToIndex(double time) {
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

}  // namespace fast_planner
