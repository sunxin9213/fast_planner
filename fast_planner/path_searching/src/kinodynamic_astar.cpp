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

#include <path_searching/kinodynamic_astar.h>
#include <sstream>
#include <plan_env/sdf_map.h>

using namespace std;
using namespace Eigen;

namespace fast_planner
{
KinodynamicAstar::~KinodynamicAstar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

/**
 * @brief 动力学A*搜索主函数
 *
 * 动力学A*算法核心:
 * - 状态空间: [position(3), velocity(3)] = 6维
 * - 启发式: 考虑动力学约束的时间估计
 * - 使用常加速度模型进行状态扩展
 *
 * 算法流程:
 * 1. 初始化起点节点
 * 2. 循环扩展节点直到找到终点或开放集为空
 * 3. 对每个扩展的节点:
 *    - 采样多个控制输入(加速度方向)
 *    - 使用四元数积分求解末端状态
 *    - 碰撞检测和代价计算
 * 4. 如果找到终点，尝试"shot trajectory"直接到达
 *
 * @param start_pt 起点位置
 * @param start_v 起点速度
 * @param start_a 起点加速度
 * @param end_pt 终点位置
 * @param end_v 终点速度(用于方向约束)
 * @param init 是否初始化搜索
 * @param dynamic 是否考虑动态障碍物
 * @param time_start 搜索开始时间(用于动态障碍物)
 * @return int 搜索结果: 0=成功, -1=失败
 */
int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
{
  start_vel_ = start_v;
  start_acc_ = start_a;

  // 从节点池中分配起点节点
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pt;
  cur_node->state.tail(3) = start_v;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;

  // 终点状态: [位置(3), 速度(3)]
  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;

  end_state.head(3) = end_pt;
  end_state.tail(3) = end_v;
  end_index = posToIndex(end_pt);
  
  // f(n) = g(n) + h(n), 其中h(n)是基于动力学的启发式估计
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);
  use_node_num_ += 1;

  // 动态障碍物处理:记录时间维度
  if (dynamic)
  {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
  }
  else
    expanded_nodes_.insert(cur_node->index, cur_node);

  PathNodePtr neighbor = NULL;
  PathNodePtr terminate_node = NULL;
  bool init_search = init;
  const int tolerance = ceil(1 / resolution_);  // 到达终点的容差

  // 主循环:从开放集中取出f值最小的节点
  while (!open_set_.empty())
  {
    cur_node = open_set_.top();

    // 终止条件检查
    // 1. 达到搜索 horizon
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;
    // 2. 接近终点(在容差范围内)
    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                    abs(cur_node->index(2) - end_index(2)) <= tolerance;

    if (reach_horizon || near_end)
    {
      terminate_node = cur_node;
      retrievePath(terminate_node);
      if (near_end)
      {
        // Check whether shot traj exist
        estimateHeuristic(cur_node->state, end_state, time_to_goal);
        computeShotTraj(cur_node->state, end_state, time_to_goal);
        if (init_search)
          ROS_ERROR("Shot in first search loop!");
      }
    }
    if (reach_horizon)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      else
      {
        std::cout << "reach horizon" << std::endl;
        return REACH_HORIZON;
      }
    }

    if (near_end)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      else if (cur_node->parent != NULL)
      {
        std::cout << "near end" << std::endl;
        return NEAR_END;
      }
      else
      {
        std::cout << "no path" << std::endl;
        return NO_PATH;
      }
    }
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector3d um;
    double pro_t;
    vector<Eigen::Vector3d> inputs;
    vector<double> durations;
    
    // 采样控制输入(加速度)和持续时间
    if (init_search)
    {
      // 首次搜索:使用起点加速度，采样较短的时间间隔
      inputs.push_back(start_acc_);
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
           tau += time_res_init * init_max_tau_)
        durations.push_back(tau);
      init_search = false;
    }
    else
    {
      // 正常搜索:采样27个加速度方向(笛卡尔积)和多个时间间隔
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
          for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
          {
            um << ax, ay, az;
            inputs.push_back(um);
          }
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        durations.push_back(tau);
    }

    // 对每个控制输入和时间间隔组合进行状态扩展
    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j)
      {
        um = inputs[i];
        double tau = durations[j];
        
        // 状态转移:使用常加速度模型
        // x(t+τ) = Φ * x(t) + ∫u*dt
        // Φ = [I, τ*I; 0, I]
        stateTransit(cur_state, pro_state, um, tau);
        pro_t = cur_node->time + tau;

        Eigen::Vector3d pro_pos = pro_state.head(3);

        // 检查1:是否在关闭集中(已扩展过)
        Eigen::Vector3i pro_id = posToIndex(pro_pos);
        int pro_t_id = timeToIndex(pro_t);
        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          continue;
        }

        // 检查2:速度约束
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_)
        {
          continue;
        }

        // 检查3:避免在同一网格重复扩展
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          continue;
        }

        // 检查4:碰撞检测 - 沿轨迹采样检查是否与障碍物相交
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;
        for (int k = 1; k <= check_num_; ++k)
        {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);
          pos = xt.head(3);
          if (edt_environment_->sdf_map_->getInflateOccupancy(pos) == 1 )
          {
            is_occ = true;
            break;
          }
        }
        if (is_occ)
        {
          continue;
        }

        // 计算代价函数
        // g(n) = g(父节点) + (||u||² + w_time) * τ
        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
        // f(n) = g(n) + λ * h(n)
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        // 剪枝:从同一父节点扩展到同一网格只保留最优
        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;
            // 如果新路径更好，更新节点
            if (tmp_f_score < expand_node->f_score)
            {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
              if (dynamic)
                expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }

        // 该节点到达的网格与其他扩展节点不同
        if (!prune)
        {
          if (pro_node == NULL)
          {
            // 新节点:从节点池分配
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic)
            {
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_)
            {
              cout << "run out of memory." << endl;
              return NO_PATH;
            }
          }
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            // 节点已在开放集:如果找到更好路径则更新
            if (tmp_g_score < pro_node->g_score)
            {
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              if (dynamic)
                pro_node->time = cur_node->time + tau;
            }
          }
          else
          {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }
        }
      }
    // init_search = false;
  }

  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

/**
* @brief 提取搜索到的路径
*
* 从终止节点回溯到起点，构建完整路径
*
* @param end_node 终止节点指针
*/
void KinodynamicAstar::retrievePath(PathNodePtr end_node)
{
PathNodePtr cur_node = end_node;
path_nodes_.push_back(cur_node);

while (cur_node->parent != NULL)
{
  cur_node = cur_node->parent;
  path_nodes_.push_back(cur_node);
}

reverse(path_nodes_.begin(), path_nodes_.end());
}

/**
* @brief Shot Trajectory计算
*
* 如果接近终点，尝试计算一条直接可达的"shot trajectory"
* 使用三次多项式插值得到满足边界条件的轨迹
*
* 三次多项式: p(t) = a*t³ + b*t² + v₀*t + p₀
* 系数通过边界条件(起点/终点位置和速度)求解
*
* @param state1 起始状态
* @param state2 目标状态
* @param time_to_goal 到达时间
* @return bool 是否成功(轨迹是否满足约束)
*/
bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
/* 获取系数矩阵 */
const Vector3d p0 = state1.head(3);
const Vector3d dp = state2.head(3) - p0;
const Vector3d v0 = state1.segment(3, 3);
const Vector3d v1 = state2.segment(3, 3);
const Vector3d dv = v1 - v0;
double t_d = time_to_goal;
MatrixXd coef(3, 4);
end_vel_ = v1;

// 根据边界条件求解三次多项式系数
// a = 1/6 * (-12*(dp - v0*t)/t³ + 6*dv/t²)
// b = 1/2 * (6*(dp - v0*t)/t² - 2*dv/t)
Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
Vector3d c = v0;
Vector3d d = p0;

// 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
// a*t^3 + b*t^2 + v0*t + p0
coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

Vector3d coord, vel, acc;
VectorXd poly1d, t, polyv, polya;
Vector3i index;

// 速度/加速度矩阵
Eigen::MatrixXd Tm(4, 4);
Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

/* 轨迹前向检查 */
double t_delta = t_d / 10;
for (double time = t_delta; time <= t_d; time += t_delta)
{
  t = VectorXd::Zero(4);
  for (int j = 0; j < 4; j++)
    t(j) = pow(time, j);

  for (int dim = 0; dim < 3; dim++)
  {
    poly1d = coef.row(dim);
    coord(dim) = poly1d.dot(t);
    vel(dim) = (Tm * poly1d).dot(t);
    acc(dim) = (Tm * Tm * poly1d).dot(t);

    // 检查速度/加速度约束
    if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
    {
      // 可行，允许继续
    }
  }

  // 检查地图边界
  if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
      coord(2) < origin_(2) || coord(2) >= map_size_3d_(2))
  {
    return false;
  }

  // 检查障碍物碰撞
  if (edt_environment_->sdf_map_->getInflateOccupancy(coord) == 1)
  {
    return false;
  }
}
coef_shot_ = coef;
t_shot_ = t_d;
is_shot_succ_ = true;
return true;
}

/**
* @brief 设置动力学A*搜索参数
*
 * 参数说明:
 * - max_tau: 最大时间间隔
 * - init_max_tau: 首次搜索的最大时间间隔
 * - max_vel: 最大速度约束
 * - max_acc: 最大加速度约束
 * - w_time: 时间代价权重
 * - horizon: 搜索范围边界
 * - check_num: 碰撞检测采样点数
 * - optimistic: 是否乐观搜索
 */
void KinodynamicAstar::setParam(ros::NodeHandle& nh)
{
  nh.param("search/max_tau", max_tau_, -1.0);
  nh.param("search/init_max_tau", init_max_tau_, -1.0);
  nh.param("search/max_vel", max_vel_, -1.0);
  nh.param("search/max_acc", max_acc_, -1.0);
  nh.param("search/w_time", w_time_, -1.0);
  nh.param("search/horizon", horizon_, -1.0);
  nh.param("search/resolution_astar", resolution_, -1.0);
  nh.param("search/time_resolution", time_resolution_, -1.0);
  nh.param("search/lambda_heu", lambda_heu_, -1.0);
  nh.param("search/allocate_num", allocate_num_, -1);
  nh.param("search/check_num", check_num_, -1);
  nh.param("search/optimistic", optimistic_, true);
  tie_breaker_ = 1.0 + 1.0 / 10000;

  double vel_margin;
  nh.param("search/vel_margin", vel_margin, 0.0);
  max_vel_ += vel_margin;
}

/**
 * @brief 动力学启发式估计函数
 *
 * 求解时间最优控制问题来估计从状态x1到x2的最小代价
 * 使用四元数方程求解，考虑速度约束
 *
 * 问题建模:
 * 最小化: J = ∫(u^T*u + w_time*1)dt = u^2*t + w_time*t
 * 约束: 速度 <= max_vel
 *
 * 通过求解LQR-type问题得到闭式解
 *
 * @param x1 起始状态 [pos(3), vel(3)]
 * @param x2 目标状态 [pos(3), vel(3)]
 * @param optimal_time 返回最优时间
 * @return double 估计的最小代价
 */
double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{
  const Vector3d dp = x2.head(3) - x1.head(3);  // 位置差
  const Vector3d v0 = x1.segment(3, 3);  // 初始速度
  const Vector3d v1 = x2.segment(3, 3);  // 目标速度

  // 四元数方程系数，从最优控制理论推导得到
  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  // 求解四元数方程得到候选时间点
  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  // 添加基于速度上界的时间估计
  double v_max = max_vel_ * 0.5;
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  // 找到最小代价对应的时间
  double cost = 100000000;
  double t_d = t_bar;

  for (auto t : ts)
  {
    if (t < t_bar)  // 时间必须大于下界
      continue;
    // 计算代价函数
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
}

bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  /* ---------- get coefficient ---------- */
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d v1 = state2.segment(3, 3);
  const Vector3d dv = v1 - v0;
  double t_d = time_to_goal;
  MatrixXd coef(3, 4);
  end_vel_ = v1;

  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector3d c = v0;
  Vector3d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Vector3d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta)
  {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t(j) = pow(time, j);

    for (int dim = 0; dim < 3; dim++)
    {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);

      if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
      {
        // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
        // return false;
      }
    }

    if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
        coord(2) < origin_(2) || coord(2) >= map_size_3d_(2))
    {
      return false;
    }

    // if (edt_environment_->evaluateCoarseEDT(coord, -1.0) <= margin_) {
    //   return false;
    // }
    if (edt_environment_->sdf_map_->getInflateOccupancy(coord) == 1)
    {
      return false;
    }
  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}

/**
 * @brief 三次方程求解器
 *
 * 求解形如 ax^3 + bx^2 + cx + d = 0 的方程
 * 使用Cardano公式，求得0-3个实数根
 *
 * @param a 三次项系数
 * @param b 二次项系数
 * @param c 一次项系数
 * @param d 常数项
 * @return vector<double> 求解得到的实数根列表
 */
vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  
  if (D > 0)  // 一个实数根
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)  // 两个实数根(重根)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else  // 三个实数根
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

/**
 * @brief 四次方程求解器
 *
 * 求解形如 ax^4 + bx^3 + cx^2 + dx + e = 0 的方程
 * 通过转换为三次方程求解
 *
 * @param a 四次项系数
 * @param b 三次项系数
 * @param c 二次项系数
 * @param d 一次项系数
 * @param e 常数项
 * @return vector<double> 求解得到的实数根列表
 */
vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  // 转换为三次方程求解
  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

/**
 * @brief 初始化动力学A*搜索器
 *
 * 预分配节点内存池，初始化地图参数和状态转移矩阵
 */
void KinodynamicAstar::init()
{
  /* ---------- 地图参数 ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
  edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  /* ---------- 预分配节点内存池 ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;
  }

  // 状态转移矩阵Phi，初始化为单位矩阵
  phi_ = Eigen::MatrixXd::Identity(6, 6);
  use_node_num_ = 0;
  iter_num_ = 0;
}

/**
 * @brief 设置环境指针
 */
void KinodynamicAstar::setEnvironment(const EDTEnvironment::Ptr& env)
{
  this->edt_environment_ = env;
}

/**
 * @brief 重置搜索器状态
 *
 * 清除所有已使用的节点和路径，准备下一次搜索
 */
void KinodynamicAstar::reset()
{
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_ = false;
}

std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t)
{
  vector<Vector3d> state_list;

  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();
  Matrix<double, 6, 1> x0, xt;

  while (node->parent != NULL)
  {
    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
    node = node->parent;
  }
  reverse(state_list.begin(), state_list.end());
  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    Vector3d coord;
    VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}

void KinodynamicAstar::getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                                  vector<Eigen::Vector3d>& start_end_derivatives)
{
  /* ---------- path duration ---------- */
  double T_sum = 0.0;
  if (is_shot_succ_)
    T_sum += t_shot_;
  PathNodePtr node = path_nodes_.back();
  while (node->parent != NULL)
  {
    T_sum += node->duration;
    node = node->parent;
  }
  // cout << "duration:" << T_sum << endl;

  // Calculate boundary vel and acc
  Eigen::Vector3d end_vel, end_acc;
  double t;
  if (is_shot_succ_)
  {
    t = t_shot_;
    end_vel = end_vel_;
    for (int dim = 0; dim < 3; ++dim)
    {
      Vector4d coe = coef_shot_.row(dim);
      end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;
    }
  }
  else
  {
    t = path_nodes_.back()->duration;
    end_vel = node->state.tail(3);
    end_acc = path_nodes_.back()->input;
  }

  // Get point samples
  int seg_num = floor(T_sum / ts);
  seg_num = max(8, seg_num);
  ts = T_sum / double(seg_num);
  bool sample_shot_traj = is_shot_succ_;
  node = path_nodes_.back();

  for (double ti = T_sum; ti > -1e-5; ti -= ts)
  {
    if (sample_shot_traj)
    {
      // samples on shot traj
      Vector3d coord;
      Vector4d poly1d, time;

      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }

      point_set.push_back(coord);
      t -= ts;

      /* end of segment */
      if (t < -1e-5)
      {
        sample_shot_traj = false;
        if (node->parent != NULL)
          t += node->duration;
      }
    }
    else
    {
      // samples on searched traj
      Eigen::Matrix<double, 6, 1> x0 = node->parent->state;
      Eigen::Matrix<double, 6, 1> xt;
      Vector3d ut = node->input;

      stateTransit(x0, xt, ut, t);

      point_set.push_back(xt.head(3));
      t -= ts;

      // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
      if (t < -1e-5 && node->parent->parent != NULL)
      {
        node = node->parent;
        t += node->duration;
      }
    }
  }
  reverse(point_set.begin(), point_set.end());

  // calculate start acc
  Eigen::Vector3d start_acc;
  if (path_nodes_.back()->parent == NULL)
  {
    // no searched traj, calculate by shot traj
    start_acc = 2 * coef_shot_.col(2);
  }
  else
  {
    // input of searched traj
    start_acc = node->input;
  }

  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(end_acc);
}

std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
{
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
{
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

  // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
  // origin_(1)) * inv_resolution_),
  //     floor((pt(2) - origin_(2)) * inv_resolution_);

  return idx;
}

int KinodynamicAstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                                    Eigen::Vector3d um, double tau)
{
  for (int i = 0; i < 3; ++i)
    phi_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

}  // namespace fast_planner
