/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/teb_config.h>

namespace teb_local_planner
{

void TebConfig::loadRosParamFromNodeHandle(const ros::NodeHandle& nh)
{

  nh.param("odom_topic", odom_topic, odom_topic);
  nh.param("map_frame", map_frame, map_frame);

  // <----------------------------  Trajectory  轨迹调整的参数
  // 如果true,自动调整轨迹大小（基于dt_ref）
  nh.param("teb_autosize", trajectory.teb_autosize, trajectory.teb_autosize);
  // 参考时间分辨率（大小一般是1/控制频率）
  nh.param("dt_ref", trajectory.dt_ref, trajectory.dt_ref);
  // 时间迟滞，一般dt_ref的10%
  nh.param("dt_hysteresis", trajectory.dt_hysteresis, trajectory.dt_hysteresis);
  // 轨迹最小样本数
  nh.param("min_samples", trajectory.min_samples, trajectory.min_samples);
  // 轨迹最大样本数
  nh.param("max_samples", trajectory.max_samples, trajectory.max_samples);
  // 有的全局规划器不考虑局部目标点的朝向，需要专门计算覆盖一下
  nh.param("global_plan_overwrite_orientation", trajectory.global_plan_overwrite_orientation, trajectory.global_plan_overwrite_orientation);
  // 如果true, 如果目标点在后面，轨迹可能被初始化成向后的运动（如果机器人有后部的传感器比较推荐）
  nh.param("allow_init_with_backwards_motion", trajectory.allow_init_with_backwards_motion, trajectory.allow_init_with_backwards_motion);
  // 调整viapoint之间的间隔
  nh.getParam("global_plan_via_point_sep", trajectory.global_plan_viapoint_sep); // 弃用了？, see checkDeprecated()
  if (!nh.param("global_plan_viapoint_sep", trajectory.global_plan_viapoint_sep, trajectory.global_plan_viapoint_sep))
    nh.setParam("global_plan_viapoint_sep", trajectory.global_plan_viapoint_sep); // write deprecated value to param server
  // 如果为true,规划器会遵循viapoints在容器中的顺序规划
  nh.param("via_points_ordered", trajectory.via_points_ordered, trajectory.via_points_ordered);
  // 用于优化时考虑的最大路径长度（累加的欧式距离）
  nh.param("max_global_plan_lookahead_dist", trajectory.max_global_plan_lookahead_dist, trajectory.max_global_plan_lookahead_dist);
  // 从当前位置向后开始裁剪路径的距离
  nh.param("global_plan_prune_distance", trajectory.global_plan_prune_distance, trajectory.global_plan_prune_distance);
  // 如果true，用规划器在计算速度，角速度和旋转时用弧长距离
  nh.param("exact_arc_length", trajectory.exact_arc_length, trajectory.exact_arc_length);
  // 如果目标点更新后的距离超过该参数，强制规划器重新初始化轨迹（跳过热启动）
  nh.param("force_reinit_new_goal_dist", trajectory.force_reinit_new_goal_dist, trajectory.force_reinit_new_goal_dist);
  nh.param("force_reinit_new_goal_angular", trajectory.force_reinit_new_goal_angular, trajectory.force_reinit_new_goal_angular);
  // 多少位姿数是有效的
  nh.param("feasibility_check_no_poses", trajectory.feasibility_check_no_poses, trajectory.feasibility_check_no_poses);
  // 发布整个轨迹规划和活跃的障碍物（推荐只在debug时用）
  nh.param("publish_feedback", trajectory.publish_feedback, trajectory.publish_feedback);
  // 旋转碰撞检测最小分辨率
  nh.param("min_resolution_collision_check_angular", trajectory.min_resolution_collision_check_angular, trajectory.min_resolution_collision_check_angular);
  // 用于提取速度命令的位姿索引
  nh.param("control_look_ahead_poses", trajectory.control_look_ahead_poses, trajectory.control_look_ahead_poses);
  // 防止观察点太远
  nh.param("prevent_look_ahead_poses_near_goal", trajectory.prevent_look_ahead_poses_near_goal, trajectory.prevent_look_ahead_poses_near_goal);

  // <--------------------------------------   Robot 机器人相关参数
  // 最大前向线速度
  nh.param("max_vel_x", robot.max_vel_x, robot.max_vel_x);
  // 最大向后线速度
  nh.param("max_vel_x_backwards", robot.max_vel_x_backwards, robot.max_vel_x_backwards);
  // 最大横向线速度
  nh.param("max_vel_y", robot.max_vel_y, robot.max_vel_y);
  // 最大角速度
  nh.param("max_vel_theta", robot.max_vel_theta, robot.max_vel_theta);
  // 最大前向线加速度
  nh.param("acc_lim_x", robot.acc_lim_x, robot.acc_lim_x);
  // 最大横向线加速度
  nh.param("acc_lim_y", robot.acc_lim_y, robot.acc_lim_y);
  // 最大旋转角速度
  nh.param("acc_lim_theta", robot.acc_lim_theta, robot.acc_lim_theta);
  // 最小旋转距离（用于Carlike机器人，差分机器人该参数为零）
  nh.param("min_turning_radius", robot.min_turning_radius, robot.min_turning_radius);
  // 前后轮距离（用于Carlike机器人）
  nh.param("wheelbase", robot.wheelbase, robot.wheelbase);
  // 角速度命令转换为舵角命令
  nh.param("cmd_angle_instead_rotvel", robot.cmd_angle_instead_rotvel, robot.cmd_angle_instead_rotvel);
  // 在轨迹可行性检查之前更新footprint
  nh.param("is_footprint_dynamic", robot.is_footprint_dynamic, robot.is_footprint_dynamic);
  // 按比例减少下发命令的大小
  nh.param("use_proportional_saturation", robot.use_proportional_saturation, robot.use_proportional_saturation);
  // 查询tf时的等待时间
  nh.param("transform_tolerance", robot.transform_tolerance, robot.transform_tolerance);

  // ------------------------------------------  GoalTolerance 目标点误差
  // 与目标点距离在该值内则判断已到达目标点
  nh.param("xy_goal_tolerance", goal_tolerance.xy_goal_tolerance, goal_tolerance.xy_goal_tolerance);
  // 角度误差
  nh.param("yaw_goal_tolerance", goal_tolerance.yaw_goal_tolerance, goal_tolerance.yaw_goal_tolerance);
  // 机器人到达目标点时可以速度不为零
  nh.param("free_goal_vel", goal_tolerance.free_goal_vel, goal_tolerance.free_goal_vel);
  // 低于该值则认为位置移动已经停下来了
  nh.param("trans_stopped_vel", goal_tolerance.trans_stopped_vel, goal_tolerance.trans_stopped_vel);
  // 低于该值则认为旋转已经停下来了
  nh.param("theta_stopped_vel", goal_tolerance.theta_stopped_vel, goal_tolerance.theta_stopped_vel);
  nh.param("complete_global_plan", goal_tolerance.complete_global_plan, goal_tolerance.complete_global_plan);

  // <-------------------------------------------  Obstacles  障碍物参数
  // 距离障碍物最小距离
  nh.param("min_obstacle_dist", obstacles.min_obstacle_dist, obstacles.min_obstacle_dist);
  // 膨胀距离，要大于min_obstacle_dist
  nh.param("inflation_dist", obstacles.inflation_dist, obstacles.inflation_dist);
  // 预测的动态障碍物位置的膨胀距离
  nh.param("dynamic_obstacle_inflation_dist", obstacles.dynamic_obstacle_inflation_dist, obstacles.dynamic_obstacle_inflation_dist);
  // 障碍物运动速度是否设置为恒速模型
  nh.param("include_dynamic_obstacles", obstacles.include_dynamic_obstacles, obstacles.include_dynamic_obstacles);
  // 代价地图的障碍物是否被直接考虑
  nh.param("include_costmap_obstacles", obstacles.include_costmap_obstacles, obstacles.include_costmap_obstacles);
  // 限制机器人后面被考虑障碍物的距离
  nh.param("costmap_obstacles_behind_robot_dist", obstacles.costmap_obstacles_behind_robot_dist, obstacles.costmap_obstacles_behind_robot_dist);
  //
  nh.param("obstacle_poses_affected", obstacles.obstacle_poses_affected, obstacles.obstacle_poses_affected);
  // true，对于每个障碍物找到最近的TEB位姿。false,只找相关的障碍物
  nh.param("legacy_obstacle_association", obstacles.legacy_obstacle_association, obstacles.legacy_obstacle_association);
  // 在legacy_obstacle_association为false时，一定距离内的所有障碍物都会被包括
  nh.param("obstacle_association_force_inclusion_factor", obstacles.obstacle_association_force_inclusion_factor, obstacles.obstacle_association_force_inclusion_factor);
  // [value]*min_obstacle_dist以外的障碍物都会在优化时被忽略，该参数会被优先处理
  nh.param("obstacle_association_cutoff_factor", obstacles.obstacle_association_cutoff_factor, obstacles.obstacle_association_cutoff_factor);
  // 代价地图转换器的名字
  nh.param("costmap_converter_plugin", obstacles.costmap_converter_plugin, obstacles.costmap_converter_plugin);
  // true,代价地图转换器在另外的线程唤醒回调队列
  nh.param("costmap_converter_spin_thread", obstacles.costmap_converter_spin_thread, obstacles.costmap_converter_spin_thread);
  // 最大速度的比率，用于限速
  nh.param("obstacle_proximity_ratio_max_vel",  obstacles.obstacle_proximity_ratio_max_vel, obstacles.obstacle_proximity_ratio_max_vel);
  // 离静态障碍物该减速的距离（下限值）
  nh.param("obstacle_proximity_lower_bound", obstacles.obstacle_proximity_lower_bound, obstacles.obstacle_proximity_lower_bound);
  // 离静态障碍物该减速的距离（上限值）
  nh.param("obstacle_proximity_upper_bound", obstacles.obstacle_proximity_upper_bound, obstacles.obstacle_proximity_upper_bound);

  // <--------------------------------   Optimization
  // 每次外循环迭代时，求解器迭代次数
  nh.param("no_inner_iterations", optim.no_inner_iterations, optim.no_inner_iterations);
  // 每次外循环迭代（改变轨迹尺寸）次数
  nh.param("no_outer_iterations", optim.no_outer_iterations, optim.no_outer_iterations);
  // 激活优化
  nh.param("optimization_activate", optim.optimization_activate, optim.optimization_activate);
  // 打印消息
  nh.param("optimization_verbose", optim.optimization_verbose, optim.optimization_verbose);
  // 惩罚函数时，添加小的余量，用于硬约束的近似
  nh.param("penalty_epsilon", optim.penalty_epsilon, optim.penalty_epsilon);
  // 最大线速度权重
  nh.param("weight_max_vel_x", optim.weight_max_vel_x, optim.weight_max_vel_x);
  // 最大横向线速度权重
  nh.param("weight_max_vel_y", optim.weight_max_vel_y, optim.weight_max_vel_y);
  // 最大角速度权重
  nh.param("weight_max_vel_theta", optim.weight_max_vel_theta, optim.weight_max_vel_theta);
  // 加速度限制权重
  nh.param("weight_acc_lim_x", optim.weight_acc_lim_x, optim.weight_acc_lim_x);
  // 横向线加速度权重
  nh.param("weight_acc_lim_y", optim.weight_acc_lim_y, optim.weight_acc_lim_y);
  // 旋转加速度权重
  nh.param("weight_acc_lim_theta", optim.weight_acc_lim_theta, optim.weight_acc_lim_theta);
  // 非完全约束机器人运动学约束
  nh.param("weight_kinematics_nh", optim.weight_kinematics_nh, optim.weight_kinematics_nh);
  // 前向运动权重
  nh.param("weight_kinematics_forward_drive", optim.weight_kinematics_forward_drive, optim.weight_kinematics_forward_drive);
  // 最小旋转半径权重
  nh.param("weight_kinematics_turning_radius", optim.weight_kinematics_turning_radius, optim.weight_kinematics_turning_radius);
  // 根据运动时间缩短轨迹的权重
  nh.param("weight_optimaltime", optim.weight_optimaltime, optim.weight_optimaltime);
  // 根据路径长度缩短轨迹的权重
  nh.param("weight_shortest_path", optim.weight_shortest_path, optim.weight_shortest_path);
  // 离障碍物距离的权重
  nh.param("weight_obstacle", optim.weight_obstacle, optim.weight_obstacle);
  // 膨胀的权重
  nh.param("weight_inflation", optim.weight_inflation, optim.weight_inflation);
  // 离动态障碍物距离的权重
  nh.param("weight_dynamic_obstacle", optim.weight_dynamic_obstacle, optim.weight_dynamic_obstacle);
  // 动态障碍物膨胀的权重
  nh.param("weight_dynamic_obstacle_inflation", optim.weight_dynamic_obstacle_inflation, optim.weight_dynamic_obstacle_inflation);
  // 根据离静态障碍物距离能达到的最大速度的权重
  nh.param("weight_velocity_obstacle_ratio", optim.weight_velocity_obstacle_ratio, optim.weight_velocity_obstacle_ratio);
  // 最小化viapoint距离的权重
  nh.param("weight_viapoint", optim.weight_viapoint, optim.weight_viapoint);
  //
  nh.param("weight_prefer_rotdir", optim.weight_prefer_rotdir, optim.weight_prefer_rotdir);
  // 迭代权重的因子
  nh.param("weight_adapt_factor", optim.weight_adapt_factor, optim.weight_adapt_factor);
  // 非线性障碍物代价的指数(cost = linear_cost * obstacle_cost_exponent)
  nh.param("obstacle_cost_exponent", optim.obstacle_cost_exponent, optim.obstacle_cost_exponent);

  // <----------------------------------------  Homotopy Class Planner
  // 是否开启同伦
  nh.param("enable_homotopy_class_planning", hcp.enable_homotopy_class_planning, hcp.enable_homotopy_class_planning);
  // true,为同伦开启多线程
  nh.param("enable_multithreading", hcp.enable_multithreading, hcp.enable_multithreading);
  // true,简单的左右障碍物策略产生路径。false，用PRM产生路径
  nh.param("simple_exploration", hcp.simple_exploration, hcp.simple_exploration);
  // 最多开启多少个同伦类
  nh.param("max_number_classes", hcp.max_number_classes, hcp.max_number_classes);
  // 相同同伦内的最大轨迹数，帮助避免局部最小
  nh.param("max_number_plans_in_current_class", hcp.max_number_plans_in_current_class, hcp.max_number_plans_in_current_class);
  // 障碍物代价的尺度因子，为了找到最好的候选路径
  nh.param("selection_obst_cost_scale", hcp.selection_obst_cost_scale, hcp.selection_obst_cost_scale);
  // 选择（0，1）中的值，减小代价
  nh.param("selection_prefer_initial_plan", hcp.selection_prefer_initial_plan, hcp.selection_prefer_initial_plan);
  // viapoint代价的尺度因子
  nh.param("selection_viapoint_cost_scale", hcp.selection_viapoint_cost_scale, hcp.selection_viapoint_cost_scale);
  // 一个新的候选轨迹需要有多少轨迹代价
  nh.param("selection_cost_hysteresis", hcp.selection_cost_hysteresis, hcp.selection_cost_hysteresis);
  // true,时间代价为路径通过时间
  nh.param("selection_alternative_time_cost", hcp.selection_alternative_time_cost, hcp.selection_alternative_time_cost);
  // 在每个规划循环中，TEBs被抛弃的概率
  nh.param("selection_dropping_probability", hcp.selection_dropping_probability, hcp.selection_dropping_probability);
  // 切换到新的同等类上的等待时间
  nh.param("switching_blocking_period", hcp.switching_blocking_period, hcp.switching_blocking_period);
  // roadmap的样本产生数
  nh.param("roadmap_graph_samples", hcp.roadmap_graph_no_samples, hcp.roadmap_graph_no_samples);
  // 采样区域的宽
  nh.param("roadmap_graph_area_width", hcp.roadmap_graph_area_width, hcp.roadmap_graph_area_width);
  // 长方形区域的的长由起终点距离决定，该参数为了几何中心保持不变
  nh.param("roadmap_graph_area_length_scale", hcp.roadmap_graph_area_length_scale, hcp.roadmap_graph_area_length_scale);
  // 改变障碍物值的数量
  nh.param("h_signature_prescaler", hcp.h_signature_prescaler, hcp.h_signature_prescaler);
  nh.param("h_signature_threshold", hcp.h_signature_threshold, hcp.h_signature_threshold);
  nh.param("obstacle_keypoint_offset", hcp.obstacle_keypoint_offset, hcp.obstacle_keypoint_offset);
  nh.param("obstacle_heading_threshold", hcp.obstacle_heading_threshold, hcp.obstacle_heading_threshold);
  nh.param("viapoints_all_candidates", hcp.viapoints_all_candidates, hcp.viapoints_all_candidates);
  nh.param("visualize_hc_graph", hcp.visualize_hc_graph, hcp.visualize_hc_graph);
  nh.param("visualize_with_time_as_z_axis_scale", hcp.visualize_with_time_as_z_axis_scale, hcp.visualize_with_time_as_z_axis_scale);
  nh.param("delete_detours_backwards", hcp.delete_detours_backwards, hcp.delete_detours_backwards);
  nh.param("detours_orientation_tolerance", hcp.detours_orientation_tolerance, hcp.detours_orientation_tolerance);
  nh.param("length_start_orientation_vector", hcp.length_start_orientation_vector, hcp.length_start_orientation_vector);
  nh.param("max_ratio_detours_duration_best_duration", hcp.max_ratio_detours_duration_best_duration, hcp.max_ratio_detours_duration_best_duration);

  // <--------------------------------   Recovery
  // 规划器减小观测距离
  nh.param("shrink_horizon_backup", recovery.shrink_horizon_backup, recovery.shrink_horizon_backup);
  //
  nh.param("shrink_horizon_min_duration", recovery.shrink_horizon_min_duration, recovery.shrink_horizon_min_duration);
  // 尝试发现和解决震荡
  nh.param("oscillation_recovery", recovery.oscillation_recovery, recovery.oscillation_recovery);
  //
  nh.param("oscillation_v_eps", recovery.oscillation_v_eps, recovery.oscillation_v_eps);
  nh.param("oscillation_omega_eps", recovery.oscillation_omega_eps, recovery.oscillation_omega_eps);
  nh.param("oscillation_recovery_min_duration", recovery.oscillation_recovery_min_duration, recovery.oscillation_recovery_min_duration);
  nh.param("oscillation_filter_duration", recovery.oscillation_filter_duration, recovery.oscillation_filter_duration);
  // 开启发散检测
  nh.param("divergence_detection", recovery.divergence_detection_enable, recovery.divergence_detection_enable);
  // 可接受的最大Mahalanobis距离（假设优化没有收敛）
  nh.param("divergence_detection_max_chi_squared", recovery.divergence_detection_max_chi_squared, recovery.divergence_detection_max_chi_squared);

  checkParameters();
  checkDeprecated(nh);
}

void TebConfig::reconfigure(TebLocalPlannerReconfigureConfig& cfg)
{
  boost::mutex::scoped_lock l(config_mutex_);

  // Trajectory
  trajectory.teb_autosize = cfg.teb_autosize;
  trajectory.dt_ref = cfg.dt_ref;
  trajectory.dt_hysteresis = cfg.dt_hysteresis;
  trajectory.global_plan_overwrite_orientation = cfg.global_plan_overwrite_orientation;
  trajectory.allow_init_with_backwards_motion = cfg.allow_init_with_backwards_motion;
  trajectory.global_plan_viapoint_sep = cfg.global_plan_viapoint_sep;
  trajectory.via_points_ordered = cfg.via_points_ordered;
  trajectory.max_global_plan_lookahead_dist = cfg.max_global_plan_lookahead_dist;
  trajectory.exact_arc_length = cfg.exact_arc_length;
  trajectory.force_reinit_new_goal_dist = cfg.force_reinit_new_goal_dist;
  trajectory.force_reinit_new_goal_angular = cfg.force_reinit_new_goal_angular;
  trajectory.feasibility_check_no_poses = cfg.feasibility_check_no_poses;
  trajectory.publish_feedback = cfg.publish_feedback;
  trajectory.control_look_ahead_poses = cfg.control_look_ahead_poses;
  trajectory.prevent_look_ahead_poses_near_goal = cfg.prevent_look_ahead_poses_near_goal;

  // Robot
  robot.max_vel_x = cfg.max_vel_x;
  robot.max_vel_x_backwards = cfg.max_vel_x_backwards;
  robot.max_vel_y = cfg.max_vel_y;
  robot.max_vel_theta = cfg.max_vel_theta;
  robot.acc_lim_x = cfg.acc_lim_x;
  robot.acc_lim_y = cfg.acc_lim_y;
  robot.acc_lim_theta = cfg.acc_lim_theta;
  robot.min_turning_radius = cfg.min_turning_radius;
  robot.wheelbase = cfg.wheelbase;
  robot.cmd_angle_instead_rotvel = cfg.cmd_angle_instead_rotvel;
  robot.use_proportional_saturation = cfg.use_proportional_saturation;

  // GoalTolerance
  goal_tolerance.xy_goal_tolerance = cfg.xy_goal_tolerance;
  goal_tolerance.yaw_goal_tolerance = cfg.yaw_goal_tolerance;
  goal_tolerance.free_goal_vel = cfg.free_goal_vel;
  goal_tolerance.trans_stopped_vel = cfg.trans_stopped_vel;
  goal_tolerance.theta_stopped_vel = cfg.theta_stopped_vel;

  // Obstacles
  obstacles.min_obstacle_dist = cfg.min_obstacle_dist;
  obstacles.inflation_dist = cfg.inflation_dist;
  obstacles.dynamic_obstacle_inflation_dist = cfg.dynamic_obstacle_inflation_dist;
  obstacles.include_dynamic_obstacles = cfg.include_dynamic_obstacles;
  obstacles.include_costmap_obstacles = cfg.include_costmap_obstacles;
  obstacles.legacy_obstacle_association = cfg.legacy_obstacle_association;
  obstacles.obstacle_association_force_inclusion_factor = cfg.obstacle_association_force_inclusion_factor;
  obstacles.obstacle_association_cutoff_factor = cfg.obstacle_association_cutoff_factor;
  obstacles.costmap_obstacles_behind_robot_dist = cfg.costmap_obstacles_behind_robot_dist;
  obstacles.obstacle_poses_affected = cfg.obstacle_poses_affected;
  obstacles.obstacle_proximity_ratio_max_vel = cfg.obstacle_proximity_ratio_max_vel;
  obstacles.obstacle_proximity_lower_bound = cfg.obstacle_proximity_lower_bound;
  obstacles.obstacle_proximity_upper_bound = cfg.obstacle_proximity_upper_bound;

  // Optimization
  optim.no_inner_iterations = cfg.no_inner_iterations;
  optim.no_outer_iterations = cfg.no_outer_iterations;
  optim.optimization_activate = cfg.optimization_activate;
  optim.optimization_verbose = cfg.optimization_verbose;
  optim.penalty_epsilon = cfg.penalty_epsilon;
  optim.weight_max_vel_x = cfg.weight_max_vel_x;
  optim.weight_max_vel_y = cfg.weight_max_vel_y;
  optim.weight_max_vel_theta = cfg.weight_max_vel_theta;
  optim.weight_acc_lim_x = cfg.weight_acc_lim_x;
  optim.weight_acc_lim_y = cfg.weight_acc_lim_y;
  optim.weight_acc_lim_theta = cfg.weight_acc_lim_theta;
  optim.weight_kinematics_nh = cfg.weight_kinematics_nh;
  optim.weight_kinematics_forward_drive = cfg.weight_kinematics_forward_drive;
  optim.weight_kinematics_turning_radius = cfg.weight_kinematics_turning_radius;
  optim.weight_optimaltime = cfg.weight_optimaltime;
  optim.weight_shortest_path = cfg.weight_shortest_path;
  optim.weight_obstacle = cfg.weight_obstacle;
  optim.weight_inflation = cfg.weight_inflation;
  optim.weight_dynamic_obstacle = cfg.weight_dynamic_obstacle;
  optim.weight_dynamic_obstacle_inflation = cfg.weight_dynamic_obstacle_inflation;
  optim.weight_velocity_obstacle_ratio = cfg.weight_velocity_obstacle_ratio;
  optim.weight_viapoint = cfg.weight_viapoint;
  optim.weight_adapt_factor = cfg.weight_adapt_factor;
  optim.obstacle_cost_exponent = cfg.obstacle_cost_exponent;

  // Homotopy Class Planner
  hcp.enable_multithreading = cfg.enable_multithreading;
  hcp.max_number_classes = cfg.max_number_classes;
  hcp.max_number_plans_in_current_class = cfg.max_number_plans_in_current_class;
  hcp.selection_cost_hysteresis = cfg.selection_cost_hysteresis;
  hcp.selection_prefer_initial_plan = cfg.selection_prefer_initial_plan;
  hcp.selection_obst_cost_scale = cfg.selection_obst_cost_scale;
  hcp.selection_viapoint_cost_scale = cfg.selection_viapoint_cost_scale;
  hcp.selection_alternative_time_cost = cfg.selection_alternative_time_cost;
  hcp.selection_dropping_probability = cfg.selection_dropping_probability;
  hcp.switching_blocking_period = cfg.switching_blocking_period;

  hcp.obstacle_heading_threshold = cfg.obstacle_heading_threshold;
  hcp.roadmap_graph_no_samples = cfg.roadmap_graph_no_samples;
  hcp.roadmap_graph_area_width = cfg.roadmap_graph_area_width;
  hcp.roadmap_graph_area_length_scale = cfg.roadmap_graph_area_length_scale;
  hcp.h_signature_prescaler = cfg.h_signature_prescaler;
  hcp.h_signature_threshold = cfg.h_signature_threshold;
  hcp.viapoints_all_candidates = cfg.viapoints_all_candidates;
  hcp.visualize_hc_graph = cfg.visualize_hc_graph;
  hcp.visualize_with_time_as_z_axis_scale = cfg.visualize_with_time_as_z_axis_scale;

  // Recovery
  recovery.shrink_horizon_backup = cfg.shrink_horizon_backup;
  recovery.oscillation_recovery = cfg.oscillation_recovery;
  recovery.divergence_detection_enable = cfg.divergence_detection_enable;
  recovery.divergence_detection_max_chi_squared = cfg.divergence_detection_max_chi_squared;


  checkParameters();
}


void TebConfig::checkParameters() const
{
  // positive backward velocity?
  if (robot.max_vel_x_backwards <= 0)
    ROS_WARN("TebLocalPlannerROS() Param Warning: Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");

  // bounds smaller than penalty epsilon
  if (robot.max_vel_x <= optim.penalty_epsilon)
    ROS_WARN("TebLocalPlannerROS() Param Warning: max_vel_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  if (robot.max_vel_x_backwards <= optim.penalty_epsilon)
    ROS_WARN("TebLocalPlannerROS() Param Warning: max_vel_x_backwards <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  if (robot.max_vel_theta <= optim.penalty_epsilon)
    ROS_WARN("TebLocalPlannerROS() Param Warning: max_vel_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  if (robot.acc_lim_x <= optim.penalty_epsilon)
    ROS_WARN("TebLocalPlannerROS() Param Warning: acc_lim_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  if (robot.acc_lim_theta <= optim.penalty_epsilon)
    ROS_WARN("TebLocalPlannerROS() Param Warning: acc_lim_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");

  // dt_ref and dt_hyst
  if (trajectory.dt_ref <= trajectory.dt_hysteresis)
    ROS_WARN("TebLocalPlannerROS() Param Warning: dt_ref <= dt_hysteresis. The hysteresis is not allowed to be greater or equal!. Undefined behavior... Change at least one of them!");

  // min number of samples
  if (trajectory.min_samples <3)
    ROS_WARN("TebLocalPlannerROS() Param Warning: parameter min_samples is smaller than 3! Sorry, I haven't enough degrees of freedom to plan a trajectory for you. Please increase ...");

  // costmap obstacle behind robot
  if (obstacles.costmap_obstacles_behind_robot_dist < 0)
    ROS_WARN("TebLocalPlannerROS() Param Warning: parameter 'costmap_obstacles_behind_robot_dist' should be positive or zero.");

  // hcp: obstacle heading threshold
  if (hcp.obstacle_keypoint_offset>=1 || hcp.obstacle_keypoint_offset<=0)
    ROS_WARN("TebLocalPlannerROS() Param Warning: parameter obstacle_heading_threshold must be in the interval ]0,1[. 0=0deg opening angle, 1=90deg opening angle.");

  // carlike
  if (robot.cmd_angle_instead_rotvel && robot.wheelbase==0)
    ROS_WARN("TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but wheelbase is set to zero: undesired behavior.");

  if (robot.cmd_angle_instead_rotvel && robot.min_turning_radius==0)
    ROS_WARN("TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but min_turning_radius is set to zero: undesired behavior. You are mixing a carlike and a diffdrive robot");

  // positive weight_adapt_factor
  if (optim.weight_adapt_factor < 1.0)
      ROS_WARN("TebLocalPlannerROS() Param Warning: parameter weight_adapt_factor shoud be >= 1.0");

  if (recovery.oscillation_filter_duration < 0)
      ROS_WARN("TebLocalPlannerROS() Param Warning: parameter oscillation_filter_duration must be >= 0");

  // weights
  if (optim.weight_optimaltime <= 0)
      ROS_WARN("TebLocalPlannerROS() Param Warning: parameter weight_optimaltime shoud be > 0 (even if weight_shortest_path is in use)");

}

void TebConfig::checkDeprecated(const ros::NodeHandle& nh) const
{
  if (nh.hasParam("line_obstacle_poses_affected") || nh.hasParam("polygon_obstacle_poses_affected"))
    ROS_WARN("TebLocalPlannerROS() Param Warning: 'line_obstacle_poses_affected' and 'polygon_obstacle_poses_affected' are deprecated. They share now the common parameter 'obstacle_poses_affected'.");

  if (nh.hasParam("weight_point_obstacle") || nh.hasParam("weight_line_obstacle") || nh.hasParam("weight_poly_obstacle"))
    ROS_WARN("TebLocalPlannerROS() Param Warning: 'weight_point_obstacle', 'weight_line_obstacle' and 'weight_poly_obstacle' are deprecated. They are replaced by the single param 'weight_obstacle'.");

  if (nh.hasParam("costmap_obstacles_front_only"))
    ROS_WARN("TebLocalPlannerROS() Param Warning: 'costmap_obstacles_front_only' is deprecated. It is replaced by 'costmap_obstacles_behind_robot_dist' to define the actual area taken into account.");

  if (nh.hasParam("costmap_emergency_stop_dist"))
    ROS_WARN("TebLocalPlannerROS() Param Warning: 'costmap_emergency_stop_dist' is deprecated. You can safely remove it from your parameter config.");

  if (nh.hasParam("alternative_time_cost"))
    ROS_WARN("TebLocalPlannerROS() Param Warning: 'alternative_time_cost' is deprecated. It has been replaced by 'selection_alternative_time_cost'.");

  if (nh.hasParam("global_plan_via_point_sep"))
    ROS_WARN("TebLocalPlannerROS() Param Warning: 'global_plan_via_point_sep' is deprecated. It has been replaced by 'global_plan_viapoint_sep' due to consistency reasons.");
}

} // namespace teb_local_planner
