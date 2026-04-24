/**
 * @file planning_visualization.cpp
 * @brief 规划可视化实现 - RViz可视化工具
 *
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

#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;

namespace fast_planner {

/**
 * @brief 构造函数 - 初始化ROS发布者
 *
 * 创建多个可视化发布者用于显示不同类型的规划信息：
 * - /planning_vis/trajectory: 轨迹
 * - /planning_vis/topo_path: 拓扑路径
 * - /planning_vis/prediction: 障碍物预测
 * - /planning_vis/visib_constraint: 可见性约束
 * - /planning_vis/frontier: 前沿搜索
 * - /planning_vis/yaw: 偏航角
 *
 * @param nh ROS节点句柄
 */
PlanningVisualization::PlanningVisualization(ros::NodeHandle& nh) {
  node = nh;

  // 轨迹可视化
  traj_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 20);
  pubs_.push_back(traj_pub_);

  // 拓扑路径可视化
  topo_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/topo_path", 20);
  pubs_.push_back(topo_pub_);

  // 障碍物预测可视化
  predict_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/prediction", 20);
  pubs_.push_back(predict_pub_);

  // 可见性约束可视化
  visib_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/visib_constraint", 20);
  pubs_.push_back(visib_pub_);

  // 前沿搜索可视化
  frontier_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/frontier", 20);
  pubs_.push_back(frontier_pub_);

  // 偏航角可视化
  yaw_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/yaw", 20);
  pubs_.push_back(yaw_pub_);

  // 初始化状态追踪变量
  last_topo_path1_num_     = 0;
  last_topo_path2_num_     = 0;
  last_bspline_phase1_num_ = 0;
  last_bspline_phase2_num_ = 0;
  last_frontier_num_       = 0;
}

/**
 * @brief 显示球体列表
 *
 * 在RViz中显示一组球体，用于表示点集
 *
 * @param list 3D点向量
 * @param resolution 球体半径
 * @param color RGBA颜色
 * @param id 可视化Marker ID
 * @param pub_id 发布者索引（0=traj, 1=topo, 2=predict, 3=visib, 4=frontier, 5=yaw）
 */
void PlanningVisualization::displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                                              const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);
  ros::Duration(0.001).sleep();
}

/**
 * @brief 显示立方体列表
 *
 * 在RViz中显示一组立方体，用于表示障碍物或空间分区
 *
 * @param list 3D点向量
 * @param resolution 立方体大小
 * @param color RGBA颜色
 * @param id 可视化Marker ID
 * @param pub_id 发布者索引
 */
void PlanningVisualization::displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                                            const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::CUBE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  ros::Duration(0.001).sleep();
}

/**
 * @brief 显示线段列表
 *
 * 在RViz中显示线段列表，用于表示路径、边等
 *
 * @param list1 线段起点列表
 * @param list2 线段终点列表
 * @param line_width 线宽
 * @param color RGBA颜色
 * @param id 可视化Marker ID
 * @param pub_id 发布者索引
 */
void PlanningVisualization::displayLineList(const vector<Eigen::Vector3d>& list1,
                                            const vector<Eigen::Vector3d>& list2, double line_width,
                                            const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::LINE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = line_width;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  ros::Duration(0.001).sleep();
}

/**
 * @brief 绘制第一阶段B样条轨迹集合
 *
 * 绘制由拓扑规划生成的多条候选B样条轨迹
 * 使用不同颜色区分不同轨迹
 *
 * @param bsplines B样条向量
 * @param size 点的大小
 */
void PlanningVisualization::drawBsplinesPhase1(vector<NonUniformBspline>& bsplines, double size) {
  vector<Eigen::Vector3d> empty;

  for (int i = 0; i < last_bspline_phase1_num_; ++i) {
    displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE + i % 100);
    displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE_CTRL_PT + i % 100);
  }
  last_bspline_phase1_num_ = bsplines.size();

  for (int i = 0; i < bsplines.size(); ++i) {
    drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.2), false, 2 * size,
                getColor(double(i) / bsplines.size()), i, i);
  }
}

/**
 * @brief 绘制第二阶段B样条轨迹集合
 *
 * 绘制经过优化或筛选后的B样条轨迹
 *
 * @param bsplines B样条向量
 * @param size 点的大小
 */
void PlanningVisualization::drawBsplinesPhase2(vector<NonUniformBspline>& bsplines, double size) {
  vector<Eigen::Vector3d> empty;

  for (int i = 0; i < last_bspline_phase2_num_; ++i) {
    displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE + (50 + i) % 100);
    displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE_CTRL_PT + (50 + i) % 100);
  }
  last_bspline_phase2_num_ = bsplines.size();

  for (int i = 0; i < bsplines.size(); ++i) {
    drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.3), false, 1.5 * size,
                getColor(double(i) / bsplines.size()), 50 + i, 50 + i);
  }
}

/**
 * @brief 绘制单条B样条轨迹
 *
 * 绘制B样条轨迹及其（可选的）控制点
 *
 * @param bspline B样条对象
 * @param size 轨迹点大小
 * @param color 轨迹颜色
 * @param show_ctrl_pts 是否显示控制点
 * @param size2 控制点大小
 * @param color2 控制点颜色
 * @param id1 轨迹ID
 * @param id2 控制点ID
 */
void PlanningVisualization::drawBspline(NonUniformBspline& bspline, double size,
                                        const Eigen::Vector4d& color, bool show_ctrl_pts, double size2,
                                        const Eigen::Vector4d& color2, int id1, int id2) {
  if (bspline.getControlPoint().size() == 0) return;

  vector<Eigen::Vector3d> traj_pts;
  double                  tm, tmp;
  bspline.getTimeSpan(tm, tmp);

  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
    traj_pts.push_back(pt);
  }
  displaySphereList(traj_pts, size, color, BSPLINE + id1 % 100);

  // draw the control point
  if (!show_ctrl_pts) return;

  Eigen::MatrixXd         ctrl_pts = bspline.getControlPoint();
  vector<Eigen::Vector3d> ctp;

  for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
    Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
    ctp.push_back(pt);
  }

  displaySphereList(ctp, size2, color2, BSPLINE_CTRL_PT + id2 % 100);
}

/**
 * @brief 绘制拓扑图
 *
 * 显示拓扑PRM图结构，包括：
 * - Guard节点（守护节点）- 起点和终点附近
 * - Connector节点（连接节点）- 连接不同区域的节点
 * - 图边（节点间的连接关系）
 *
 * @param graph 图节点列表
 * @param point_size 节点大小
 * @param line_width 边线宽
 * @param color1 Guard节点颜色
 * @param color2 Connector节点颜色
 * @param color3 边颜色
 * @param id 可视化ID
 */
void PlanningVisualization::drawTopoGraph(list<GraphNode::Ptr>& graph, double point_size,
                                          double line_width, const Eigen::Vector4d& color1,
                                          const Eigen::Vector4d& color2, const Eigen::Vector4d& color3,
                                          int id) {
  // clear exsiting node and edge (drawn last time)
  vector<Eigen::Vector3d> empty;
  displaySphereList(empty, point_size, color1, GRAPH_NODE, 1);
  displaySphereList(empty, point_size, color1, GRAPH_NODE + 50, 1);
  displayLineList(empty, empty, line_width, color3, GRAPH_EDGE, 1);

  /* draw graph node */
  vector<Eigen::Vector3d> guards, connectors;
  for (list<GraphNode::Ptr>::iterator iter = graph.begin(); iter != graph.end(); ++iter) {

    if ((*iter)->type_ == GraphNode::Guard) {
      guards.push_back((*iter)->pos_);
    } else if ((*iter)->type_ == GraphNode::Connector) {
      connectors.push_back((*iter)->pos_);
    }
  }
  displaySphereList(guards, point_size, color1, GRAPH_NODE, 1);
  displaySphereList(connectors, point_size, color2, GRAPH_NODE + 50, 1);

  /* draw graph edge */
  vector<Eigen::Vector3d> edge_pt1, edge_pt2;
  for (list<GraphNode::Ptr>::iterator iter = graph.begin(); iter != graph.end(); ++iter) {
    for (int k = 0; k < (*iter)->neighbors_.size(); ++k) {

      edge_pt1.push_back((*iter)->pos_);
      edge_pt2.push_back((*iter)->neighbors_[k]->pos_);
    }
  }
  displayLineList(edge_pt1, edge_pt2, line_width, color3, GRAPH_EDGE, 1);
}

/**
 * @brief 绘制第二阶段拓扑路径
 *
 * 显示经过筛选的最终路径
 *
 * @param paths 路径集合
 * @param line_width 线宽
 */
void PlanningVisualization::drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>>& paths,
                                                double                           line_width) {
  // clear drawn paths
  Eigen::Vector4d color1(1, 1, 1, 1);
  for (int i = 0; i < last_topo_path1_num_; ++i) {
    vector<Eigen::Vector3d> empty;
    displayLineList(empty, empty, line_width, color1, SELECT_PATH + i % 100, 1);
    displaySphereList(empty, line_width, color1, PATH + i % 100, 1);
  }

  last_topo_path1_num_ = paths.size();

  // draw new paths
  for (int i = 0; i < paths.size(); ++i) {
    vector<Eigen::Vector3d> edge_pt1, edge_pt2;

    for (int j = 0; j < paths[i].size() - 1; ++j) {
      edge_pt1.push_back(paths[i][j]);
      edge_pt2.push_back(paths[i][j + 1]);
    }

    displayLineList(edge_pt1, edge_pt2, line_width, getColor(double(i) / (last_topo_path1_num_)),
                    SELECT_PATH + i % 100, 1);
  }
}

/**
 * @brief 绘制第一阶段拓扑路径
 *
 * 显示原始的拓扑路径候选
 *
 * @param paths 路径集合
 * @param size 线宽/点大小
 */
void PlanningVisualization::drawTopoPathsPhase1(vector<vector<Eigen::Vector3d>>& paths, double size) {
  // clear drawn paths
  Eigen::Vector4d color1(1, 1, 1, 1);
  for (int i = 0; i < last_topo_path2_num_; ++i) {
    vector<Eigen::Vector3d> empty;
    displayLineList(empty, empty, size, color1, FILTERED_PATH + i % 100, 1);
  }

  last_topo_path2_num_ = paths.size();

  // draw new paths
  for (int i = 0; i < paths.size(); ++i) {
    vector<Eigen::Vector3d> edge_pt1, edge_pt2;

    for (int j = 0; j < paths[i].size() - 1; ++j) {
      edge_pt1.push_back(paths[i][j]);
      edge_pt2.push_back(paths[i][j + 1]);
    }

    displayLineList(edge_pt1, edge_pt2, size, getColor(double(i) / (last_topo_path2_num_), 0.2),
                    FILTERED_PATH + i % 100, 1);
  }
}

/**
 * @brief 绘制目标点
 *
 * 在RViz中显示规划目标位置
 *
 * @param goal 目标位置
 * @param resolution 目标点大小
 * @param color 颜色
 * @param id 可视化ID
 */
void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution,
                                     const Eigen::Vector4d& color, int id) {
  vector<Eigen::Vector3d> goal_vec = { goal };
  displaySphereList(goal_vec, resolution, color, GOAL + id % 100);
}

/**
 * @brief 绘制几何路径
 *
 * 显示由分段直线组成的路径
 *
 * @param path 路径点列表
 * @param resolution 点的大小
 * @param color 颜色
 * @param id 可视化ID
 */
void PlanningVisualization::drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution,
                                              const Eigen::Vector4d& color, int id) {
  displaySphereList(path, resolution, color, PATH + id % 100);
}

/**
 * @brief 绘制多项式轨迹
 *
 * 显示由分段多项式组成的连续轨迹
 *
 * @param poly_traj 多项式轨迹对象
 * @param resolution 点的大小
 * @param color 颜色
 * @param id 可视化ID
 */
void PlanningVisualization::drawPolynomialTraj(PolynomialTraj poly_traj, double resolution,
                                               const Eigen::Vector4d& color, int id) {
  poly_traj.init();
  vector<Eigen::Vector3d> poly_pts = poly_traj.getTraj();
  displaySphereList(poly_pts, resolution, color, POLY_TRAJ + id % 100);
}

/**
 * @brief 绘制障碍物预测
 *
 * 显示动态障碍物的运动预测轨迹
 *
 * @param pred 障碍物预测对象
 * @param resolution 点的大小
 * @param color 颜色
 * @param id 可视化ID
 */
void PlanningVisualization::drawPrediction(ObjPrediction pred, double resolution,
                                           const Eigen::Vector4d& color, int id) {
  ros::Time    time_now   = ros::Time::now();
  double       start_time = (time_now - ObjHistory::global_start_time_).toSec();
  const double range      = 5.6;

  vector<Eigen::Vector3d> traj;
  for (int i = 0; i < pred->size(); i++) {

    PolynomialPrediction poly = pred->at(i);
    if (!poly.valid()) continue;

    for (double t = start_time; t <= start_time + range; t += 0.8) {
      Eigen::Vector3d pt = poly.evaluateConstVel(t);
      traj.push_back(pt);
    }
  }
  displaySphereList(traj, resolution, color, id % 100, 2);
}

/**
 * @brief 绘制偏航角轨迹
 *
 * 在位置轨迹上显示偏航角方向箭头
 * 用于可视化无人机的朝向变化
 *
 * @param pos 位置B样条
 * @param yaw 偏航角B样条
 * @param dt 时间步长
 */
void PlanningVisualization::drawYawTraj(NonUniformBspline& pos, NonUniformBspline& yaw,
                                        const double& dt) {
  double                  duration = pos.getTimeSum();
  vector<Eigen::Vector3d> pts1, pts2;

  for (double tc = 0.0; tc <= duration + 1e-3; tc += dt) {
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    pc[2] += 0.15;
    double          yc = yaw.evaluateDeBoorT(tc)[0];
    Eigen::Vector3d dir(cos(yc), sin(yc), 0);
    Eigen::Vector3d pdir = pc + 1.0 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(pts1, pts2, 0.04, Eigen::Vector4d(1, 0.5, 0, 1), 0, 5);
}

/**
 * @brief 绘制偏航角路径
 *
 * 在离散位置点显示偏航角方向
 *
 * @param pos 位置B样条
 * @param yaw 偏航角列表
 * @param dt 时间步长
 */
void PlanningVisualization::drawYawPath(NonUniformBspline& pos, const vector<double>& yaw,
                                        const double& dt) {
  vector<Eigen::Vector3d> pts1, pts2;

  for (int i = 0; i < yaw.size(); ++i) {
    Eigen::Vector3d pc = pos.evaluateDeBoorT(i * dt);
    pc[2] += 0.3;
    Eigen::Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
    Eigen::Vector3d pdir = pc + 1.0 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(pts1, pts2, 0.04, Eigen::Vector4d(1, 0, 1, 1), 1, 5);
}

/**
 * @brief HSV到RGBA颜色转换
 *
 * 将HSV颜色空间转换为RGBA格式
 * h值范围[0,1]映射到色相环
 *
 * @param h 色相值 [0,1]
 * @param alpha 透明度 [0,1]
 * @return RGBA颜色向量
 */
Eigen::Vector4d PlanningVisualization::getColor(double h, double alpha) {
  if (h < 0.0 || h > 1.0) {
    std::cout << "h out of range" << std::endl;
    h = 0.0;
  }

  double          lambda;
  Eigen::Vector4d color1, color2;
  if (h >= -1e-4 && h < 1.0 / 6) {
    lambda = (h - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);

  } else if (h >= 1.0 / 6 && h < 2.0 / 6) {
    lambda = (h - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);

  } else if (h >= 2.0 / 6 && h < 3.0 / 6) {
    lambda = (h - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);

  } else if (h >= 3.0 / 6 && h < 4.0 / 6) {
    lambda = (h - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);

  } else if (h >= 4.0 / 6 && h < 5.0 / 6) {
    lambda = (h - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);

  } else if (h >= 5.0 / 6 && h <= 1.0 + 1e-4) {
    lambda = (h - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3)              = alpha;

  return fcolor;
}
// PlanningVisualization::
}  // namespace fast_planner