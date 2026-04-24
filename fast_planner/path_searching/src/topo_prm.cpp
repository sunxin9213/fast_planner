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



#include <path_searching/topo_prm.h>
#include <thread>

namespace fast_planner {
TopologyPRM::TopologyPRM(/* args */) {}

TopologyPRM::~TopologyPRM() {}

/**
 * @brief 拓扑PRM初始化函数
 *
 * 初始化拓扑路径规划器的各种参数:
 * - sample_inflate: 采样区域膨胀参数
 * - clearance: 障碍物安全距离
 * - short_cut_num: 捷径迭代次数
 * - reserve_num: 保留路径数量
 * - ratio_to_short: 路径长度阈值比例
 * - max_sample_num: 最大采样点数
 * - max_sample_time: 最大采样时间
 * - max_raw_path: 最大原始路径数
 * - parallel_shortcut: 是否并行捷径
 */
void TopologyPRM::init(ros::NodeHandle& nh) {
  graph_.clear();
  eng_ = default_random_engine(rd_());
  rand_pos_ = uniform_real_distribution<double>(-1.0, 1.0);

  // init parameter
  nh.param("topo_prm/sample_inflate_x", sample_inflate_(0), -1.0);
  nh.param("topo_prm/sample_inflate_y", sample_inflate_(1), -1.0);
  nh.param("topo_prm/sample_inflate_z", sample_inflate_(2), -1.0);
  nh.param("topo_prm/clearance", clearance_, -1.0);
  nh.param("topo_prm/short_cut_num", short_cut_num_, -1);
  nh.param("topo_prm/reserve_num", reserve_num_, -1);
  nh.param("topo_prm/ratio_to_short", ratio_to_short_, -1.0);
  nh.param("topo_prm/max_sample_num", max_sample_num_, -1);
  nh.param("topo_prm/max_sample_time", max_sample_time_, -1.0);
  nh.param("topo_prm/max_raw_path", max_raw_path_, -1);
  nh.param("topo_prm/max_raw_path2", max_raw_path2_, -1);
  nh.param("topo_prm/parallel_shortcut", parallel_shortcut_, false);
  resolution_ = edt_environment_->sdf_map_->getResolution();
  offset_ = Eigen::Vector3d(0.5, 0.5, 0.5) - edt_environment_->sdf_map_->getOrigin() / resolution_;

  // 为每条路径预分配光线投射器
  for (int i = 0; i < max_raw_path_; ++i) {
    casters_.push_back(RayCaster());
  }
}

/**
 * @brief 拓扑路径搜索主函数
 *
 * 完整的拓扑PRM路径规划流程:
 * 1. 创建拓扑图 - 基于可见性采样生成guard和connector节点
 * 2. 图中搜索 - 使用DFS找到所有从起点到终点的路径
 * 3. 路径缩短 - 迭代缩短每条路径
 * 4. 路径剪枝 - 移除拓扑等价的路径
 * 5. 路径选择 - 选择最短的N条路径
 *
 * @param start 起点位置
 * @param end 终点位置
 * @start_pts 起点附近的多个候选点
 * @end_pts 终点附近的多个候选点
 * @graph 返回生成的拓扑图
 * @raw_paths 返回搜索到的原始路径
 * @filtered_paths 返回剪枝后的路径
 * @select_paths 返回最终选择的路径
 */
void TopologyPRM::findTopoPaths(Eigen::Vector3d start, Eigen::Vector3d end,
                                vector<Eigen::Vector3d> start_pts, vector<Eigen::Vector3d> end_pts,
                                list<GraphNode::Ptr>& graph, vector<vector<Eigen::Vector3d>>& raw_paths,
                                vector<vector<Eigen::Vector3d>>& filtered_paths,
                                vector<vector<Eigen::Vector3d>>& select_paths) {
  ros::Time t1, t2;

  double graph_time, search_time, short_time, prune_time, select_time;
  
  /* ---------- 步骤1: 创建拓扑图 ---------- */
  t1 = ros::Time::now();

  start_pts_ = start_pts;
  end_pts_ = end_pts;

  graph = createGraph(start, end);

  graph_time = (ros::Time::now() - t1).toSec();

  /* ---------- 步骤2: 在图中搜索路径 ---------- */
  t1 = ros::Time::now();

  raw_paths = searchPaths();

  search_time = (ros::Time::now() - t1).toSec();

  /* ---------- 步骤3: 路径缩短 ---------- */
  // 使用visibility-based shortcutting迭代缩短路径
  t1 = ros::Time::now();

  shortcutPaths();

  short_time = (ros::Time::now() - t1).toSec();

  /* ---------- 步骤4: 剪枝拓扑等价的路径 ---------- */
  t1 = ros::Time::now();

  filtered_paths = pruneEquivalent(short_paths_);

  prune_time = (ros::Time::now() - t1).toSec();

  /* ---------- 步骤5: 选择N条最短路径 ---------- */
  t1 = ros::Time::now();

  select_paths = selectShortPaths(filtered_paths, 1);

  select_time = (ros::Time::now() - t1).toSec();

  final_paths_ = select_paths;

  // 打印各阶段耗时
  double total_time = graph_time + search_time + short_time + prune_time + select_time;

  std::cout << "\n[Topo]: total time: " << total_time << ", graph: " << graph_time
            << ", search: " << search_time << ", short: " << short_time << ", prune: " << prune_time
            << ", select: " << select_time << std::endl;
}

/**
 * @brief 创建拓扑图
 *
 * 基于可见性采样的拓扑路径规划图构建算法:
 * 1. 在起点和终点创建Guard节点
 * 2. 在起点-终点连线的垂直空间内随机采样
 * 3. 对每个采样点:
 *    - 如果可见的Guard少于2个: 创建新的Guard
 *    - 如果可见的Guard等于2个: 检查是否需要创建Connector
 * 4. 剪枝图中的孤立节点
 *
 * 节点类型:
 * - Guard: 守护节点，表示空间中的关键采样点
 * - Connector: 连接器节点，连接两个可见的Guard
 *
 * @param start 起点位置
 * @param end 终点位置
 * @return list<GraphNode::Ptr> 构建的拓扑图
 */
list<GraphNode::Ptr> TopologyPRM::createGraph(Eigen::Vector3d start, Eigen::Vector3d end) {
  /* 初始化起点、终点和采样区域 */
  graph_.clear();

  // 创建起点和终点的Guard节点
  GraphNode::Ptr start_node = GraphNode::Ptr(new GraphNode(start, GraphNode::Guard, 0));
  GraphNode::Ptr end_node = GraphNode::Ptr(new GraphNode(end, GraphNode::Guard, 1));

  graph_.push_back(start_node);
  graph_.push_back(end_node);

  // 计算采样区域大小(沿起点-终点方向的圆柱体)
  sample_r_(0) = 0.5 * (end - start).norm() + sample_inflate_(0);
  sample_r_(1) = sample_inflate_(1);
  sample_r_(2) = sample_inflate_(2);

  // 计算从世界坐标到采样坐标系的变换矩阵
  translation_ = 0.5 * (start + end);  // 采样坐标系原点

  // 构建局部坐标系
  Eigen::Vector3d xtf, ytf, ztf, downward(0, 0, -1);
  xtf = (end - translation_).normalized();  // x轴指向终点
  ytf = xtf.cross(downward).normalized();     // y轴垂直向下
  ztf = xtf.cross(ytf);                       // z轴

  rotation_.col(0) = xtf;
  rotation_.col(1) = ytf;
  rotation_.col(2) = ztf;

  int node_id = 1;

  /* ---------- 主采样循环 ---------- */
  int sample_num = 0;
  double sample_time = 0.0;
  Eigen::Vector3d pt;
  ros::Time t1, t2;
  
  // 在达到最大采样时间或最大采样数之前持续采样
  while (sample_time < max_sample_time_ && sample_num < max_sample_num_) {
    t1 = ros::Time::now();

    // 在采样区域内生成随机点
    pt = getSample();
    ++sample_num;
    
    // 检查采样点是否在障碍物安全距离之外
    double dist;
    Eigen::Vector3d grad;
    dist = edt_environment_->evaluateCoarseEDT(pt, -1.0);
    if (dist <= clearance_) {
      sample_time += (ros::Time::now() - t1).toSec();
      continue;  // 距离障碍物太近，跳过
    }

    /* 查找可见的Guard节点 */
    vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt);
    
    if (visib_guards.size() == 0) {
      // 没有可见的Guard，创建新的Guard节点
      GraphNode::Ptr guard = GraphNode::Ptr(new GraphNode(pt, GraphNode::Guard, ++node_id));
      graph_.push_back(guard);
    } else if (visib_guards.size() == 2) {
      /* 两个Guard可见，检查是否需要添加新连接 */
      bool need_connect = needConnection(visib_guards[0], visib_guards[1], pt);
      if (!need_connect) {
        sample_time += (ros::Time::now() - t1).toSec();
        continue;  // 不需要新连接
      }
      
      // 需要新连接，创建Connector节点
      GraphNode::Ptr connector = GraphNode::Ptr(new GraphNode(pt, GraphNode::Connector, ++node_id));
      graph_.push_back(connector);

      // 建立双向连接关系
      visib_guards[0]->neighbors_.push_back(connector);
      visib_guards[1]->neighbors_.push_back(connector);

      connector->neighbors_.push_back(visib_guards[0]);
      connector->neighbors_.push_back(visib_guards[1]);
    }

    sample_time += (ros::Time::now() - t1).toSec();
  }

  /* 打印统计信息 */
  std::cout << "[Topo]: sample num: " << sample_num;

  // 剪枝图中没有用的节点
  pruneGraph();

  return graph_;
}

/**
 * @brief 查找从给定点可见的Guard节点
 *
 * 可见性判断: 从pt到Guard节点的连线不与障碍物相交
 * 只返回最多2个可见的Guard节点
 *
 * @param pt 查询点位置
 * @return vector<GraphNode::Ptr> 可见的Guard节点列表
 */
vector<GraphNode::Ptr> TopologyPRM::findVisibGuard(Eigen::Vector3d pt) {
  vector<GraphNode::Ptr> visib_guards;
  Eigen::Vector3d pc;

  int visib_num = 0;

  /* 遍历图中所有节点，查找可见的GUARD */
  for (list<GraphNode::Ptr>::iterator iter = graph_.begin(); iter != graph_.end(); ++iter) {
    if ((*iter)->type_ == GraphNode::Connector) continue;  // 只检查Guard节点

    // 使用线段可见性检测
    if (lineVisib(pt, (*iter)->pos_, resolution_, pc)) {
      visib_guards.push_back((*iter));
      ++visib_num;
      if (visib_num > 2) break;  // 最多返回2个
    }
  }

  return visib_guards;
}

/**
 * @brief 检查是否需要在两个Guard之间创建新连接
 *
 * 判断逻辑:
 * 1. 如果两个Guard之间已经存在通过Connector的连接
 * 2. 检查新路径path1与现有路径path2是否拓扑等价
 * 3. 如果拓扑等价但新路径更短，则更新现有Connector位置
 * 4. 如果不等价，则需要创建新连接
 *
 * @param g1 第一个Guard节点
 * @param g2 第二个Guard节点
 * @param pt 候选的Connector位置
 * @return bool true表示需要创建新连接，false表示不需要
 */
bool TopologyPRM::needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt) {
  // 路径1: g1 -> pt -> g2 (新路径)
  vector<Eigen::Vector3d> path1(3), path2(3);
  path1[0] = g1->pos_;
  path1[1] = pt;
  path1[2] = g2->pos_;

  // 路径2: g1 -> 现有Connector -> g2 (已有路径)
  path2[0] = g1->pos_;
  path2[2] = g2->pos_;

  vector<Eigen::Vector3d> connect_pts;
  bool has_connect = false;
  
  // 遍历g1和g2的邻居，寻找共同的Connector
  for (int i = 0; i < g1->neighbors_.size(); ++i) {
    for (int j = 0; j < g2->neighbors_.size(); ++j) {
      if (g1->neighbors_[i]->id_ == g2->neighbors_[j]->id_) {
        // 找到共同邻居，比较两条路径
        path2[1] = g1->neighbors_[i]->pos_;
        bool same_topo = sameTopoPath(path1, path2, 0.0);
        if (same_topo) {
          // 如果拓扑等价，检查是否能缩短
          if (pathLength(path1) < pathLength(path2)) {
            g1->neighbors_[i]->pos_ = pt;  // 用更短的连接替换
          }
          return false;  // 不需要新连接
        }
      }
    }
  }
  return true;  // 需要创建新连接
}

/**
 * @brief 在采样区域内生成随机采样点
 *
 * 采样方法:
 * 1. 在局部坐标系(rotation_, translation_)内均匀随机采样
 * 2. 将采样点变换到世界坐标系
 *
 * @return Eigen::Vector3d 采样点位置
 */
Eigen::Vector3d TopologyPRM::getSample() {
  /* 在采样区域内随机采样 */
  Eigen::Vector3d pt;
  pt(0) = rand_pos_(eng_) * sample_r_(0);
  pt(1) = rand_pos_(eng_) * sample_r_(1);
  pt(2) = rand_pos_(eng_) * sample_r_(2);

  pt = rotation_ * pt + translation_;

  return pt;
}

/**
 * @brief 检测两点之间的线段是否可见(无障碍物遮挡)
 *
 * 使用射线投射(Raycasting)方法检测:
 * 1. 将线段离散化为多个采样点
 * 2. 检查每个采样点到SDF地图的距离
 * 3. 如果任何点距离障碍物小于阈值thresh，返回不可见
 *
 * @param p1 线段起点
 * @param p2 线段终点
 * @param thresh 障碍物安全距离阈值
 * @param pc 返回碰撞点位置(如果不可见)
 * @param caster_id 射线投射器ID
 * @return bool true表示可见，false表示不可见
 */
bool TopologyPRM::lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh,
                            Eigen::Vector3d& pc, int caster_id) {
  Eigen::Vector3d ray_pt;
  Eigen::Vector3i pt_id;
  double dist;

  // 使用射线投射器检测线段
  casters_[caster_id].setInput(p1 / resolution_, p2 / resolution_);
  while (casters_[caster_id].step(ray_pt)) {
    pt_id(0) = ray_pt(0) + offset_(0);
    pt_id(1) = ray_pt(1) + offset_(1);
    pt_id(2) = ray_pt(2) + offset_(2);
    dist = edt_environment_->sdf_map_->getDistance(pt_id);
    if (dist <= thresh) {
      edt_environment_->sdf_map_->indexToPos(pt_id, pc);
      return false;  // 碰撞，不可见
    }
  }
  return true;  // 可见
}

/**
 * @brief 剪枝图中没有连接的孤立节点
 *
 * 剪枝规则:
 * - 如果节点邻居数量<=1，则该节点为孤立节点
 * - 从图中移除孤立节点及其与其它节点的连接关系
 * - 保留起点(id=0)和终点(id=1)
 */
void TopologyPRM::pruneGraph() {
  /* 剪枝无用的节点 */
  if (graph_.size() > 2) {
    for (list<GraphNode::Ptr>::iterator iter1 = graph_.begin();
         iter1 != graph_.end() && graph_.size() > 2; ++iter1) {
      if ((*iter1)->id_ <= 1) continue;  // 保留起点和终点

      /* 核心: 如果邻居数量<=1，则删除该节点 */
      if ((*iter1)->neighbors_.size() <= 1) {
        // 从其他节点的邻居列表中删除此节点
        for (list<GraphNode::Ptr>::iterator iter2 = graph_.begin(); iter2 != graph_.end(); ++iter2) {
          for (vector<GraphNode::Ptr>::iterator it_nb = (*iter2)->neighbors_.begin();
               it_nb != (*iter2)->neighbors_.end(); ++it_nb) {
            if ((*it_nb)->id_ == (*iter1)->id_) {
              (*iter2)->neighbors_.erase(it_nb);
              break;
            }
          }
        }

        // delete this node from graph, restart checking
        graph_.erase(iter1);
        iter1 = graph_.begin();
      }
    }
  }
}

/**
 * @brief 移除拓扑等价的路径
 *
 * 拓扑等价判断: 两条路径对应的点对之间相互可见
 * 即path1[i]与path2[i]对所有i都是可见的
 *
 * @param paths 待剪枝的路径集合
 * @return vector<vector<Eigen::Vector3d>> 剪枝后的路径集合
 */
vector<vector<Eigen::Vector3d>> TopologyPRM::pruneEquivalent(vector<vector<Eigen::Vector3d>>& paths) {
  vector<vector<Eigen::Vector3d>> pruned_paths;
  if (paths.size() < 1) return pruned_paths;

  /* 剪枝拓扑等价的路径 */
  vector<int> exist_paths_id;
  exist_paths_id.push_back(0);  // 保留第一条路径

  // 逐条比较，保留拓扑不等价的路径
  for (int i = 1; i < paths.size(); ++i) {
    bool new_path = true;

    for (int j = 0; j < exist_paths_id.size(); ++j) {
      // 比较与已存在路径的拓扑等价性
      bool same_topo = sameTopoPath(paths[i], paths[exist_paths_id[j]], 0.0);

      if (same_topo) {
        new_path = false;
        break;
      }
    }

    if (new_path) {
      exist_paths_id.push_back(i);
    }
  }

  // 保存剪枝后的路径
  for (int i = 0; i < exist_paths_id.size(); ++i) {
    pruned_paths.push_back(paths[exist_paths_id[i]]);
  }

  std::cout << ", pruned path num: " << pruned_paths.size();

  return pruned_paths;
}

/**
 * @brief 选择最短的N条路径
 *
 * 选择策略:
 * 1. 按长度排序，依次选择最短的路径
 * 2. 只保留长度小于最短路径ratio_to_short_倍的路径
 * 3. 合并起点和终点附近的路径段
 * 4. 对选择的路径再次进行缩短和剪枝
 *
 * @param paths 候选路径集合
 * @param step 步长参数(未使用)
 * @return vector<vector<Eigen::Vector3d>> 选中的最短路径集合
 */
vector<vector<Eigen::Vector3d>> TopologyPRM::selectShortPaths(vector<vector<Eigen::Vector3d>>& paths,
                                                              int step) {
  /* 保留最短的路径 */
  vector<vector<Eigen::Vector3d>> short_paths;
  vector<Eigen::Vector3d> short_path;
  double min_len;

  for (int i = 0; i < reserve_num_ && paths.size() > 0; ++i) {
    int path_id = shortestPath(paths);
    if (i == 0) {
      short_paths.push_back(paths[path_id]);
      min_len = pathLength(paths[path_id]);
      paths.erase(paths.begin() + path_id);
    } else {
      double rat = pathLength(paths[path_id]) / min_len;
      if (rat < ratio_to_short_) {
        short_paths.push_back(paths[path_id]);
        paths.erase(paths.begin() + path_id);
      } else {
        break;
      }
    }
  }
  std::cout << ", select path num: " << short_paths.size();

  /* 合并起点和终点段 */
  for (int i = 0; i < short_paths.size(); ++i) {
    short_paths[i].insert(short_paths[i].begin(), start_pts_.begin(), start_pts_.end());
    short_paths[i].insert(short_paths[i].end(), end_pts_.begin(), end_pts_.end());
  }
  for (int i = 0; i < short_paths.size(); ++i) {
    shortcutPath(short_paths[i], i, 5);
    short_paths[i] = short_paths_[i];
  }

  short_paths = pruneEquivalent(short_paths);

  return short_paths;
}

/**
 * @brief 判断两条路径是否拓扑等价
 *
 * 拓扑等价判断方法:
 * 1. 将两条路径离散化为相同数量的采样点
 * 2. 检查对应点对之间是否相互可见
 * 4. 如果所有点对都相互可见，则拓扑等价
 *
 * 数学公式:
 * 对i=0,...,pt_num-1，检查 lineVisib(pts1[i], pts2[i]) 是否都为true
 *
 * @param path1 第一条路径
 * @param path2 第二条路径
 * @param thresh 可见性检测阈值
 * @return bool true表示拓扑等价，false表示不等价
 */
bool TopologyPRM::sameTopoPath(const vector<Eigen::Vector3d>& path1,
                               const vector<Eigen::Vector3d>& path2, double thresh) {
  // 计算路径长度
  double len1 = pathLength(path1);
  double len2 = pathLength(path2);

  double max_len = max(len1, len2);

  // 根据长度确定离散化点数
  int pt_num = ceil(max_len / resolution_);

  // 离散化两条路径
  vector<Eigen::Vector3d> pts1 = discretizePath(path1, pt_num);
  vector<Eigen::Vector3d> pts2 = discretizePath(path2, pt_num);

  // 检查对应点对之间的可见性
  Eigen::Vector3d pc;
  for (int i = 0; i < pt_num; ++i) {
    if (!lineVisib(pts1[i], pts2[i], thresh, pc)) {
      return false;  // 有不可见的点对，拓扑不等价
    }
  }

  return true;  // 所有点对都可见，拓扑等价
}

/**
 * @brief 在路径集合中找到最短路径的索引
 *
 * @param paths 路径集合
 * @return int 最短路径的索引
 */
int TopologyPRM::shortestPath(vector<vector<Eigen::Vector3d>>& paths) {
  int short_id = -1;
  double min_len = 100000000;
  for (int i = 0; i < paths.size(); ++i) {
    double len = pathLength(paths[i]);
    if (len < min_len) {
      short_id = i;
      min_len = len;
    }
  }
  return short_id;
}

/**
 * @brief 计算路径的总长度
 *
 * 路径长度公式: L = sum(|path[i+1] - path[i]|) for i=0 to n-2
 *
 * @param path 路径点序列
 * @return double 路径总长度
 */
double TopologyPRM::pathLength(const vector<Eigen::Vector3d>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;

  for (int i = 0; i < path.size() - 1; ++i) {
    length += (path[i + 1] - path[i]).norm();
  }
  return length;
}

/**
 * @brief 将路径离散化为指定数量的采样点
 *
 * 离散化方法(线性插值):
 * 1. 计算路径上每段的长度，累积得到长度列表len_list
 * 2. 等间距采样pt_num个点
 * 3. 对每个采样点，在对应区间内线性插值
 *
 * 插值公式:
 * lambda = (cur_l - len_list[idx]) / (len_list[idx+1] - len_list[idx])
 * pt = (1 - lambda) * path[idx] + lambda * path[idx+1]
 *
 * @param path 原始路径
 * @param pt_num 离散化后的点数
 * @return vector<Eigen::Vector3d> 离散化后的路径点
 */
vector<Eigen::Vector3d> TopologyPRM::discretizePath(const vector<Eigen::Vector3d>& path, int pt_num) {
  vector<double> len_list;
  len_list.push_back(0.0);

  // 计算累积长度列表
  for (int i = 0; i < path.size() - 1; ++i) {
    double inc_l = (path[i + 1] - path[i]).norm();
    len_list.push_back(inc_l + len_list[i]);
  }

  // 计算pt_num个采样点
  double len_total = len_list.back();
  double dl = len_total / double(pt_num - 1);
  double cur_l;

  vector<Eigen::Vector3d> dis_path;
  for (int i = 0; i < pt_num; ++i) {
    cur_l = double(i) * dl;

    // 找到cur_l所在的区间
    int idx = -1;
    for (int j = 0; j < len_list.size() - 1; ++j) {
      if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
        idx = j;
        break;
      }
    }

    // find lambda and interpolate
    double lambda = (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
    Eigen::Vector3d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
    dis_path.push_back(inter_pt);
  }

  return dis_path;
}

/**
 * @brief 将路径转换为引导点(调用离散化函数)
 *
 * @param path 原始路径
 * @param pt_num 离散化后的点数
 * @return vector<Eigen::Vector3d> 离散化后的路径点
 */
vector<Eigen::Vector3d> TopologyPRM::pathToGuidePts(vector<Eigen::Vector3d>& path, int pt_num) {
  return discretizePath(path, pt_num);
}

/**
 * @brief 使用基于可见性的路径缩短算法
 *
 * 算法步骤:
 * 1. 将路径离散化为多个采样点
 * 2. 遍历每个采样点，检查与前一个点是否可见
 * 3. 如果不可见(碰撞)，将碰撞点沿着障碍物梯度方向推开
 * 4. 迭代执行直到路径无法进一步缩短
 *
 * 碰撞处理:
 * - 获取碰撞点处的SDF梯度(指向障碍物)
 * - 计算推力方向: push_dir = grad - (grad·dir)*dir
 *   其中dir是路径前进方向
 * - 将碰撞点沿推力方向移动resolution_距离
 *
 * @param path 待缩短的路径
 * @param path_id 路径ID(用于存储结果)
 * @param iter_num 迭代次数
 */
void TopologyPRM::shortcutPath(vector<Eigen::Vector3d> path, int path_id, int iter_num) {
  vector<Eigen::Vector3d> short_path = path;
  vector<Eigen::Vector3d> last_path;

  for (int k = 0; k < iter_num; ++k) {
    last_path = short_path;

    // 离散化路径
    vector<Eigen::Vector3d> dis_path = discretizePath(short_path);

    if (dis_path.size() < 2) {
      short_paths_[path_id] = dis_path;
      return;
    }

    /* 基于可见性的路径缩短 */
    Eigen::Vector3d colli_pt, grad, dir, push_dir;
    double dist;
    short_path.clear();
    short_path.push_back(dis_path.front());
    
    // 遍历离散点，检测碰撞
    for (int i = 1; i < dis_path.size(); ++i) {
      if (lineVisib(short_path.back(), dis_path[i], resolution_, colli_pt, path_id)) continue;

      // 获取碰撞点处的梯度
      edt_environment_->evaluateEDTWithGrad(colli_pt, -1, dist, grad);
      if (grad.norm() > 1e-3) {
        grad.normalize();
        dir = (dis_path[i] - short_path.back()).normalized();
        // 计算推力方向(垂直于前进方向)
        push_dir = grad - grad.dot(dir) * dir;
        push_dir.normalize();
        // 沿推力方向推开
        colli_pt = colli_pt + resolution_ * push_dir;
      }
      short_path.push_back(colli_pt);
    }
    short_path.push_back(dis_path.back());

    /* 如果没有缩短，则停止迭代 */
    double len1 = pathLength(last_path);
    double len2 = pathLength(short_path);
    if (len2 > len1) {
      short_path = last_path;
      break;
    }
  }

  short_paths_[path_id] = short_path;
}

/**
 * @brief 对所有原始路径进行并行或串行缩短
 *
 * 如果parallel_shortcut_为true，则使用多线程并行处理
 * 否则串行处理每条路径
 */
void TopologyPRM::shortcutPaths() {
  short_paths_.resize(raw_paths_.size());

  if (parallel_shortcut_) {
    vector<thread> short_threads;
    for (int i = 0; i < raw_paths_.size(); ++i) {
      short_threads.push_back(thread(&TopologyPRM::shortcutPath, this, raw_paths_[i], i, 1));
    }
    for (int i = 0; i < raw_paths_.size(); ++i) {
      short_threads[i].join();
    }
  } else {
    for (int i = 0; i < raw_paths_.size(); ++i) shortcutPath(raw_paths_[i], i);
  }
}

/**
 * @brief 将线段离散化为多个采样点
 *
 * @param p1 线段起点
 * @param p2 线段终点
 * @return vector<Eigen::Vector3d> 离散化后的点序列
 */
vector<Eigen::Vector3d> TopologyPRM::discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2) {
  Eigen::Vector3d dir = p2 - p1;
  double len = dir.norm();
  int seg_num = ceil(len / resolution_);

  vector<Eigen::Vector3d> line_pts;
  if (seg_num <= 0) {
    return line_pts;
  }

  // 等间距采样
  for (int i = 0; i <= seg_num; ++i) line_pts.push_back(p1 + dir * double(i) / double(seg_num));

  return line_pts;
}

/**
 * @brief 将路径离散化为按分辨率间隔的点序列
 *
 * @param path 原始路径
 * @return vector<Eigen::Vector3d> 离散化后的路径
 */
vector<Eigen::Vector3d> TopologyPRM::discretizePath(vector<Eigen::Vector3d> path) {
  vector<Eigen::Vector3d> dis_path, segment;

  if (path.size() < 2) {
    ROS_ERROR("what path? ");
    return dis_path;
  }

  // 对每段路径进行离散化
  for (int i = 0; i < path.size() - 1; ++i) {
    segment = discretizeLine(path[i], path[i + 1]);

    if (segment.size() < 1) continue;

    dis_path.insert(dis_path.end(), segment.begin(), segment.end());
    // 避免重复添加连接点
    if (i != path.size() - 2) dis_path.pop_back();
  }
  return dis_path;
}

/**
 * @brief 将多条路径批量离散化
 *
 * @param path 待离散的路径集合
 * @return vector<vector<Eigen::Vector3d>> 离散化后的路径集合
 */
vector<vector<Eigen::Vector3d>> TopologyPRM::discretizePaths(vector<vector<Eigen::Vector3d>>& path) {
  vector<vector<Eigen::Vector3d>> dis_paths;
  vector<Eigen::Vector3d> dis_path;

  for (int i = 0; i < path.size(); ++i) {
    dis_path = discretizePath(path[i]);

    if (dis_path.size() > 0) dis_paths.push_back(dis_path);
  }

  return dis_paths;
}

/**
 * @brief 获取路径上与起点-终点连线方向正交性最强的点
 *
 * 用于找到路径上"最偏离直线"的点，可用于判断路径的弯曲程度
 *
 * @param path 输入路径
 * @return Eigen::Vector3d 正交性最强的点
 */
Eigen::Vector3d TopologyPRM::getOrthoPoint(const vector<Eigen::Vector3d>& path) {
  Eigen::Vector3d x1 = path.front();
  Eigen::Vector3d x2 = path.back();

  Eigen::Vector3d dir = (x2 - x1).normalized();  // 路径主方向
  Eigen::Vector3d mid = 0.5 * (x1 + x2);         // 中点

  double min_cos = 1000.0;
  Eigen::Vector3d pdir;
  Eigen::Vector3d ortho_pt;

  // 遍历路径中间点，找最偏离主方向的点
  for (int i = 1; i < path.size() - 1; ++i) {
    pdir = (path[i] - mid).normalized();
    double cos = fabs(pdir.dot(dir));

    if (cos < min_cos) {
      min_cos = cos;
      ortho_pt = path[i];
    }
  }

  return ortho_pt;
}

/**
 * @brief 在拓扑图中搜索从起点到终点的所有路径
 *
 * 使用深度优先搜索(DFS)算法:
 * 1. 从起点节点开始进行DFS
 * 2. 记录所有到达终点的路径
 * 3. 按节点数排序，优先保留节点数较少的路径
 * 4. 限制返回的路径数量max_raw_path2_
 *
 * @return vector<vector<Eigen::Vector3d>> 搜索到的原始路径集合
 */
// 使用DFS在拓扑图中搜索有效路径
vector<vector<Eigen::Vector3d>> TopologyPRM::searchPaths() {
  raw_paths_.clear();

  // 从图中的第一个节点(起点)开始DFS
  vector<GraphNode::Ptr> visited;
  visited.push_back(graph_.front());

  depthFirstSearch(visited);

  // 按节点数排序路径
  int min_node_num = 100000, max_node_num = 1;
  vector<vector<int>> path_list(100);
  for (int i = 0; i < raw_paths_.size(); ++i) {
    if (int(raw_paths_[i].size()) > max_node_num) max_node_num = raw_paths_[i].size();
    if (int(raw_paths_[i].size()) < min_node_num) min_node_num = raw_paths_[i].size();
    path_list[int(raw_paths_[i].size())].push_back(i);
  }

  // 优先选择节点数较少的路径
  vector<vector<Eigen::Vector3d>> filter_raw_paths;
  for (int i = min_node_num; i <= max_node_num; ++i) {
    bool reach_max = false;
    for (int j = 0; j < path_list[i].size(); ++j) {
      filter_raw_paths.push_back(raw_paths_[path_list[i][j]]);
      if (filter_raw_paths.size() >= max_raw_path2_) {
        reach_max = true;
        break;
      }
    }
    if (reach_max) break;
  }
  std::cout << ", raw path num: " << raw_paths_.size() << ", " << filter_raw_paths.size();

  raw_paths_ = filter_raw_paths;

  return raw_paths_;
}

/**
 * @brief 深度优先搜索递归函数
 *
 * DFS算法:
 * 1. 检查当前节点是否到达终点(id==1)
 * 2. 遍历所有邻居节点
 * 3. 跳过已访问的节点避免环路
 * 4. 递归搜索直到找到所有路径或达到最大数量限制
 *
 * @param vis 当前已访问的节点路径
 */
void TopologyPRM::depthFirstSearch(vector<GraphNode::Ptr>& vis) {
  GraphNode::Ptr cur = vis.back();

  // 首先检查是否直接连接到终点
  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    // 检查是否到达终点
    if (cur->neighbors_[i]->id_ == 1) {
      // 将此路径加入路径集合
      vector<Eigen::Vector3d> path;
      for (int j = 0; j < vis.size(); ++j) {
        path.push_back(vis[j]->pos_);
      }
      path.push_back(cur->neighbors_[i]->pos_);

      raw_paths_.push_back(path);
      if (raw_paths_.size() >= max_raw_path_) return;

      break;
    }
  }

  // 递归遍历所有邻居
  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    // 跳过终点
    if (cur->neighbors_[i]->id_ == 1) continue;

    // 检查是否已访问(避免环路)
    bool revisit = false;
    for (int j = 0; j < vis.size(); ++j) {
      if (cur->neighbors_[i]->id_ == vis[j]->id_) {
        revisit = true;
        break;
      }
    }
    if (revisit) continue;

    // 递归搜索
    vis.push_back(cur->neighbors_[i]);
    depthFirstSearch(vis);
    if (raw_paths_.size() >= max_raw_path_) return;

    // 回溯
    vis.pop_back();
  }
}

/**
 * @brief 设置环境指针
 *
 * @param env 指向EDTEnvironment的智能指针
 */
void TopologyPRM::setEnvironment(const EDTEnvironment::Ptr& env) { this->edt_environment_ = env; }

/**
 * @brief 检测三角形区域的可见性(未完成实现)
 *
 * 原始意图:检测点pt到线段p1-p2的可见性
 * 注意:当前实现有bug(总是返回false)，可能未完成
 *
 * @param pt 测试点
 * @param p1 线段端点1
 * @param p2 线段端点2
 * @return bool 可见性结果
 */
bool TopologyPRM::triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2) {
  // 获取p1-p2沿线的采样点
  vector<Eigen::Vector3d> pts;

  Eigen::Vector3d dir = p2 - p1;
  double length = dir.norm();
  int seg_num = ceil(length / resolution_);

  Eigen::Vector3d pt1;
  for (int i = 1; i < seg_num; ++i) {
    pt1 = p1 + dir * double(i) / double(seg_num);
    pts.push_back(pt1);
  }

  // 测试可见性
  for (int i = 0; i < pts.size(); ++i) {
    {
      return false;
    }
  }

  return true;
}

// 命名空间结束
}  // namespace fast_planner