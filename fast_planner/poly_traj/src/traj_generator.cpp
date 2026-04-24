/**
 * @file traj_generator.cpp
 * @brief 轨迹生成器 - 使用多项式生成轨迹并发布控制命令
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

#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <Eigen/Eigen>
#include <ros/ros.h>

#include <swarmtal_msgs/drone_onboard_command.h>
#include <traj_generator/polynomial_traj.hpp>

using namespace std;

/* ========== ROS发布者 ========== */
ros::Publisher state_pub;      // 状态可视化（速度、加速度箭头）
ros::Publisher pos_cmd_pub;    // 位置命令发布
ros::Publisher traj_pub;        // 轨迹可视化发布

/* ========== 里程计数据 ========== */
nav_msgs::Odometry odom;        // 无人机当前里程计数据
bool have_odom;                 // 是否已收到里程计数据

/**
 * @brief 显示路径（带颜色）
 *
 * 使用ROS可视化消息显示路径点列表
 *
 * @param path 路径点向量
 * @param resolution 点的大小分辨率
 * @param color RGBA颜色
 * @param id 可视化消息ID
 */
void displayPathWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;

  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
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
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

/**
 * @brief 绘制状态箭头
 *
 * 在RViz中绘制从位置出发的向量箭头，用于显示速度或加速度
 *
 * @param pos 起始位置
 * @param vec 向量（速度或加速度）
 * @param id 箭头ID
 * @param color 颜色
 */
void drawState(Eigen::Vector3d pos, Eigen::Vector3d vec, int id, Eigen::Vector4d color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;
  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;
  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);
  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);
  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);
  state_pub.publish(mk_state);
}

/**
 * @brief 里程计回调函数
 *
 * 接收无人机里程计数据
 *
 * @param msg 里程计消息
 */
void odomCallbck(const nav_msgs::Odometry& msg) {
  // 过滤特定帧ID
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;

  odom = msg;
  have_odom = true;
}

/**
 * @brief 主函数 - 轨迹生成与控制命令发布
 *
 * 工作流程：
 * 1. 初始化ROS节点和发布者/订阅者
 * 2. 等待里程计数据就绪
 * 3. 定义航点并生成最小跳跃轨迹
 * 4. 循环发布位置、速度、加速度命令
 */
int main(int argc, char** argv) {
  /* ========== 初始化 ========== */
  ros::init(argc, argv, "traj_generator");
  ros::NodeHandle node;

  // 订阅里程计话题
  ros::Subscriber odom_sub = node.subscribe("/uwb_vicon_odom", 50, odomCallbck);

  // 创建可视化发布者
  traj_pub = node.advertise<visualization_msgs::Marker>("/traj_generator/traj_vis", 10);
  state_pub = node.advertise<visualization_msgs::Marker>("/traj_generator/cmd_vis", 10);

  // 创建控制命令发布者
  pos_cmd_pub =
      node.advertise<swarmtal_msgs::drone_onboard_command>("/drone_commander/onboard_command", 10);

  ros::Duration(1.0).sleep();

  /* ========== 等待里程计就绪 ========== */
  have_odom = false;
  while (!have_odom && ros::ok()) {
    cout << "no odomeetry." << endl;
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }

  /* ========== 生成最小跳跃轨迹 ========== */
  // 定义航点位置（9个点，3列xyz）
  Eigen::MatrixXd pos(9, 3);
  
  // 起点为当前无人机位置
  pos.row(0) =
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  
  // 其他预设航点（形成8字形轨迹）
  pos.row(1) = Eigen::Vector3d(-0.5, 0.5, 1);
  pos.row(2) = Eigen::Vector3d(0, 0, 1);
  pos.row(3) = Eigen::Vector3d(0.5, -0.5, 1);
  pos.row(4) = Eigen::Vector3d(1, 0, 1);
  pos.row(5) = Eigen::Vector3d(0.5, 0.5, 1);
  pos.row(6) = Eigen::Vector3d(0, 0, 1);
  pos.row(7) = Eigen::Vector3d(-0.5, -0.5, 1);
  pos.row(8) = Eigen::Vector3d(-1, 0, 1);

  // 每段时间（8段）
  Eigen::VectorXd time(8);
  time(0) = 2.0;
  time(1) = 1.5;
  time(2) = 1.5;
  time(3) = 1.5;
  time(4) = 1.5;
  time(5) = 1.5;
  time(6) = 1.5;
  time(7) = 2.0;

  // 生成多项式轨迹系数矩阵
  Eigen::MatrixXd poly = generateTraj(pos, time);
  cout << "poly:\n" << poly << endl;
  cout << "pos:\n" << pos << endl;
  cout << "pos 0 1 2: " << pos(0) << ", " << pos(1) << ", " << pos(2) << endl;

  /* ========== 构造多项式轨迹对象 ========== */
  PolynomialTraj poly_traj;
  for (int i = 0; i < poly.rows(); ++i) {
    vector<double> cx(6), cy(6), cz(6);
    for (int j = 0; j < 6; ++j) {
      cx[j] = poly(i, j), cy[j] = poly(i, j + 6), cz[j] = poly(i, j + 12);
    }
    // 系数反转（从高次到低次）
    reverse(cx.begin(), cx.end());
    reverse(cy.begin(), cy.end());
    reverse(cz.begin(), cz.end());
    double ts = time(i);
    poly_traj.addSegment(cx, cy, cz, ts);
  }
  poly_traj.init();
  
  // 获取离散轨迹用于可视化
  vector<Eigen::Vector3d> traj_vis = poly_traj.getTraj();

  // 显示轨迹
  displayPathWithColor(traj_vis, 0.05, Eigen::Vector4d(1, 0, 0, 1), 1);

  /* ========== 循环发布控制命令 ========== */
  ros::Time start_time = ros::Time::now();
  ros::Time time_now;

  ros::Duration(0.1).sleep();

  // 初始化命令消息
  swarmtal_msgs::drone_onboard_command cmd;
  cmd.command_type = swarmtal_msgs::drone_onboard_command::CTRL_POS_COMMAND;
  cmd.param1 = 0;
  cmd.param2 = 0;
  cmd.param3 = 0;
  cmd.param4 = 666666;
  cmd.param5 = 0;
  cmd.param6 = 0;
  cmd.param7 = 0;
  cmd.param8 = 0;
  cmd.param9 = 0;
  cmd.param10 = 0;

  // 主循环：实时计算并发布轨迹上的位置、速度、加速度
  while (ros::ok()) {
    time_now = ros::Time::now();
    double tn = (time_now - start_time).toSec();  // 已运行时间
    
    // 评估轨迹
    Eigen::Vector3d pt = poly_traj.evaluate(tn);      // 位置
    Eigen::Vector3d vel = poly_traj.evaluateVel(tn);    // 速度
    Eigen::Vector3d acc = poly_traj.evaluateAcc(tn);    // 加速度

    // 填充命令参数（缩放10000倍以传递小数）
    cmd.param1 = int(pt(0) * 10000);   // 目标位置x
    cmd.param2 = int(pt(1) * 10000);   // 目标位置y
    cmd.param3 = int(pt(2) * 10000);   // 目标位置z

    cmd.param5 = int(vel(0) * 10000);  // 速度x
    cmd.param6 = int(vel(1) * 10000);  // 速度y

    cmd.param7 = 0;                     // 偏航角
    cmd.param8 = int(acc(0) * 10000);  // 加速度x
    cmd.param9 = int(acc(1) * 10000);  // 加速度y

    // 发布命令
    pos_cmd_pub.publish(cmd);

    // 可视化速度和加速度箭头
    drawState(pt, vel, 0, Eigen::Vector4d(0, 1, 0, 1));  // 绿色：速度
    drawState(pt, acc, 1, Eigen::Vector4d(0, 0, 1, 1));  // 蓝色：加速度
    ros::Duration(0.01).sleep();
  }

  ros::spin();
  return 0;
}
