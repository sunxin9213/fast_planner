#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

// #include <path_searching/matrix_hash.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include "plan_env/edt_environment.h"

namespace fast_planner {
// #define REACH_HORIZON 1
// #define REACH_END 2
// #define NO_PATH 3
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

class PathNode {
 public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Matrix<double, 6, 1> state;
  double g_score, f_score;
  Eigen::Vector3d input;//当前节点到父节点的控制输入(加速度)
  double duration;//当前节点到父节点的时间间隔或者说输入的控制的持续时间
  double time;  // dyn
  int time_idx;
  PathNode* parent;
  char node_state;

  /* -------------------- */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode* PathNodePtr;

class NodeComparator {
 public:
  bool operator()(PathNodePtr node1, PathNodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
 private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>>
      data_4d_;

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3i idx, PathNodePtr node) {
    data_3d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node) {
    data_4d_.insert(std::make_pair(
        Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter =
        data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }
};

/**
 * @brief 带动力学约束的 A* 搜索器。
 *
 * 这个类用于在状态空间中搜索一条满足速度/加速度约束的可飞行轨迹。
 * 与普通几何 A* 不同，它把搜索对象从“点”扩展为“状态”，即位置 + 速度，
 * 同时结合控制输入（加速度）和时间间隔进行状态传播。
 *
 * 主要职责包括：
 * 1. 通过状态转移模型生成候选状态；
 * 2. 利用地图中的 EDT/SDF 信息做碰撞检测；
 * 3. 用启发式函数估计当前状态到终点的代价；
 * 4. 搜索成功后回溯父节点，得到完整轨迹供后续优化器使用。
 */
class KinodynamicAstar {
 private:
  /* ---------- 搜索节点与数据结构 ---------- */
  /* 节点池：缓存所有已创建的搜索节点，避免频繁分配内存 */
  vector<PathNodePtr> path_node_pool_;
  /* 当前已使用的节点数量与搜索迭代次数 */
  int use_node_num_, iter_num_;
  /* 哈希表：快速判断某个状态/时间索引是否已经被扩展过 */
  NodeHashTable expanded_nodes_;
  /* 优先队列：优先扩展 f = g + h 最小的节点 */
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      open_set_;
  /* 回溯路径时使用的节点链 */
  std::vector<PathNodePtr> path_nodes_;

  /* ---------- 搜索过程中的状态记录 ---------- */
  /* 起点、终点和初始加速度，用于初始化状态传播 */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;
  /* 状态转移矩阵 Phi，用于离散时间下的状态传播 */
  Eigen::Matrix<double, 6, 6> phi_;
  /* 指向地图 / EDT 环境，供碰撞检测使用 */
  EDTEnvironment::Ptr edt_environment_;
  /* 是否已经成功生成 shot trajectory */
  bool is_shot_succ_ = false;
  /* shot trajectory 的多项式系数 */
  Eigen::MatrixXd coef_shot_;
  /* shot trajectory 对应的总时间 */
  double t_shot_;
  /* 是否已经找到有效路径 */
  bool has_path_ = false;

  /* ---------- 搜索参数与地图参数 ---------- */
  /* 搜索相关参数 */
  double max_tau_, init_max_tau_;
  double max_vel_, max_acc_;
  double w_time_, horizon_, lambda_heu_;
  int allocate_num_, check_num_;
  double tie_breaker_;
  bool optimistic_;

  /* 地图与离散化相关参数 */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;

  /* ---------- 辅助函数 ---------- */
  /* 将世界坐标转换为离散网格索引 */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  /* 将连续时间转换为离散时间索引 */
  int timeToIndex(double time);
  /* 回溯父节点，构造完整路径 */
  void retrievePath(PathNodePtr end_node);

  /* ---------- shot trajectory 相关 ---------- */
  /* 求解三次方程 */
  vector<double> cubic(double a, double b, double c, double d);
  /* 求解四次方程 */
  vector<double> quartic(double a, double b, double c, double d, double e);
  /* 生成一条直接连接起点与终点的 shot trajectory */
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                       double time_to_goal);
  /* 估计当前状态到目标状态的启发式代价 */
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                           double& optimal_time);

  /* ---------- 状态传播 ---------- */
  /* 使用常加速度模型进行状态推进 */
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                    Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um,
                    double tau);

 public:
  KinodynamicAstar(){};
  ~KinodynamicAstar();

  /* 搜索结果枚举 */
  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };

  /* ---------- 对外接口 ---------- */
  /* 从 ROS 参数服务器读取搜索参数 */
  void setParam(ros::NodeHandle& nh);
  /* 初始化节点池、地图参数和状态转移矩阵 */
  void init();
  /* 清空上一轮搜索状态，准备下一次搜索 */
  void reset();
  /* 主搜索函数：在动力学约束下从起点搜索到终点 */
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
             Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
             Eigen::Vector3d end_vel, bool init, bool dynamic = false,
             double time_start = -1.0);

  /* 绑定地图 / EDT 环境对象，用于碰撞检测 */
  void setEnvironment(const EDTEnvironment::Ptr& env);

  /* 将搜索结果转成离散轨迹点 */
  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);

  /* 采样关键点并给出起止端导数，用于后续 B 样条参数化 */
  void getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                  vector<Eigen::Vector3d>& start_end_derivatives);

  /* 获取本次搜索访问过的节点，用于调试与可视化 */
  std::vector<PathNodePtr> getVisitedNodes();

  typedef shared_ptr<KinodynamicAstar> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif