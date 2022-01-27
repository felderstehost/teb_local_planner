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

#include <teb_local_planner/teb_local_planner_ros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/algorithm/string.hpp>

// MBF return codes
#include <mbf_msgs/ExePathResult.h>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"


// 注册局部规划器为一个BaseLocalPlanner和MBF's CostmapController插件
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, mbf_costmap_core::CostmapController)

namespace teb_local_planner
{


TebLocalPlannerROS::TebLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), costmap_model_(NULL),
                                           costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                                           dynamic_recfg_(NULL), custom_via_points_active_(false), goal_reached_(false), no_infeasible_plans_(0),
                                           last_preferred_rotdir_(RotType::none), initialized_(false)
{
}


TebLocalPlannerROS::~TebLocalPlannerROS()
{
}

void TebLocalPlannerROS::reconfigureCB(TebLocalPlannerReconfigureConfig& config, uint32_t level)
{
  cfg_.reconfigure(config);
  ros::NodeHandle nh("~/" + name_);
  // 创建机器人的footprint（轮廓）模型，用于优化
  RobotFootprintModelPtr robot_model = getRobotFootprintFromParamServer(nh, cfg_);
  planner_->updateRobotModel(robot_model);
}

void TebLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  // 检查插件是否已经初始化
  if(!initialized_)
  {
    name_ = name;
    // create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle nh("~/" + name);

    // 通过nodehandle获得TebConfig类的参数以及覆盖默认配置
    cfg_.loadRosParamFromNodeHandle(nh);

    // 为障碍物保留一定的存储空间
    obstacles_.reserve(500);

    // 创建可视化的实例
    visualization_ = TebVisualizationPtr(new TebVisualization(nh, cfg_));

    // 创建机器人的footprint（轮廓）模型，用于优化
    RobotFootprintModelPtr robot_model = getRobotFootprintFromParamServer(nh, cfg_);

    // 创建局部规划器实例
    if (cfg_.hcp.enable_homotopy_class_planning)
    {
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_));
      ROS_INFO("Parallel planning in distinctive topologies enabled.");
    }
    else
    {
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_));
      ROS_INFO("Parallel planning in distinctive topologies disabled.");
    }

    // 初始化其他的变量
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.

    costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);

    global_frame_ = costmap_ros_->getGlobalFrameID();
    cfg_.map_frame = global_frame_; // TODO
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    // 初始化用于多边形转换器的代价地图
    if (!cfg_.obstacles.costmap_converter_plugin.empty())
    {
      try
      {
        costmap_converter_ = costmap_converter_loader_.createInstance(cfg_.obstacles.costmap_converter_plugin);
        std::string converter_name = costmap_converter_loader_.getName(cfg_.obstacles.costmap_converter_plugin);
        // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
        boost::replace_all(converter_name, "::", "/");
        costmap_converter_->setOdomTopic(cfg_.odom_topic);
        costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
        costmap_converter_->setCostmap2D(costmap_);

        costmap_converter_->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_, cfg_.obstacles.costmap_converter_spin_thread);
        ROS_INFO_STREAM("Costmap conversion plugin " << cfg_.obstacles.costmap_converter_plugin << " loaded.");
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
        costmap_converter_.reset();
      }
    }
    else
      ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");

    // 获得机器人的footprint以及机器人中心到footprint顶点的最小和最大距离
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);

    // 初始化odom helper,用于从odom消息中接收机器人的速度
    odom_helper_.setOdomTopic(cfg_.odom_topic);

    // 动态配置
    dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(nh);
    dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&TebLocalPlannerROS::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);

    // 校验用于优化footprint和代价的footprint
    validateFootprints(robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_.obstacles.min_obstacle_dist);

    // 设置自定义障碍物的回调函数
    custom_obst_sub_ = nh.subscribe("obstacles", 1, &TebLocalPlannerROS::customObstacleCB, this);

    // 设置自定义via-points(路径点)的回调函数
    via_points_sub_ = nh.subscribe("via_points", 1, &TebLocalPlannerROS::customViaPointsCB, this);

    // 初始化检查失败的机制
    ros::NodeHandle nh_move_base("~");
    double controller_frequency = 5;
    nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);
    failure_detector_.setBufferLength(std::round(cfg_.recovery.oscillation_filter_duration*controller_frequency));

    // 初始化完成，设置初始化的标志位为true
    initialized_ = true;

    ROS_DEBUG("teb_local_planner plugin initialized.");
  }
  else
  {
    ROS_WARN("teb_local_planner has already been initialized, doing nothing.");
  }
}



bool TebLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  // 检查插件是否已经初始化
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // 用于存储全局路径
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
  // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

  // 更新到达终点的标志位为false
  goal_reached_ = false;

  return true;
}


bool TebLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  std::string dummy_message;
  geometry_msgs::PoseStamped dummy_pose;
  geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
  uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
  cmd_vel = cmd_vel_stamped.twist;
  return outcome == mbf_msgs::ExePathResult::SUCCESS;
}

uint32_t TebLocalPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                     const geometry_msgs::TwistStamped& velocity,
                                                     geometry_msgs::TwistStamped &cmd_vel,
                                                     std::string &message)
{
  // 插件是否已经初始化
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
    message = "teb_local_planner has not been initialized";
    return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }

  static uint32_t seq = 0;
  cmd_vel.header.seq = seq++;
  cmd_vel.header.stamp = ros::Time::now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
  goal_reached_ = false;

  // 获得机器人位姿
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  robot_pose_ = PoseSE2(robot_pose.pose);

  // 获取机器人速度
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  robot_vel_.linear.x = robot_vel_tf.pose.position.x;
  robot_vel_.linear.y = robot_vel_tf.pose.position.y;
  robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);

  // 裁剪已经走过的全局路径 (spatially before the robot)
  pruneGlobalPlan(*tf_, robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);

  // 转换全局路径到特定坐标系下(w.r.t. the local costmap)
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  int goal_idx;
  geometry_msgs::TransformStamped tf_plan_to_global;
  if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist,
                           transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    message = "Could not transform the global plan to the frame of the controller";
    return mbf_msgs::ExePathResult::INTERNAL_ERROR;
  }

  // 更新via-points容器
  if (!custom_via_points_active_)
    updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);

  nav_msgs::Odometry base_odom;
  odom_helper_.getOdom(base_odom);

  // 检查是否已经到了目标点
  geometry_msgs::PoseStamped global_goal;
  tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
  double dx = global_goal.pose.position.x - robot_pose_.x();
  double dy = global_goal.pose.position.y - robot_pose_.y();
  double delta_orient = g2o::normalize_theta( tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta() );
  if(fabs(std::sqrt(dx*dx+dy*dy)) < cfg_.goal_tolerance.xy_goal_tolerance
    && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance
    && (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0)
    && (base_local_planner::stopped(base_odom, cfg_.goal_tolerance.theta_stopped_vel, cfg_.goal_tolerance.trans_stopped_vel)
        || cfg_.goal_tolerance.free_goal_vel))
  {
    goal_reached_ = true;
    return mbf_msgs::ExePathResult::SUCCESS;
  }

  // 检查是否进入备份模式并进行相关的设置
  configureBackupModes(transformed_plan, goal_idx);


  // 如果转换后的全局路径为空，返回false
  if (transformed_plan.empty())
  {
    ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
    message = "Transformed plan is empty";
    return mbf_msgs::ExePathResult::INVALID_PATH;
  }

  // 获得当前的目标点，也就是transformed_plan的最后一个点
  robot_goal_.x() = transformed_plan.back().pose.position.x;
  robot_goal_.y() = transformed_plan.back().pose.position.y;
  // 覆盖目标点的朝向，如果有必要的话
  if (cfg_.trajectory.global_plan_overwrite_orientation)
  {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
    //  (enable using the plan as initialization) 用真实的目标点朝向覆盖transformed_plan的目标点朝向
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_goal_.theta());
    tf2::convert(q, transformed_plan.back().pose.orientation);
  }
  else
  {
    robot_goal_.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
  }

  // 用真实的起始点位置覆盖transformed_plan的起始点位置 (allows using the plan as initial trajectory)
  if (transformed_plan.size()==1) // 路径中只有目标点
  {
    transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped()); // 插入起始位姿（还没有初始化）
  }
  transformed_plan.front() = robot_pose; // 更新起始点

  // 清除掉现存的障碍物
  obstacles_.clear();

  // 用代价地图信息或者costmap_converter提供的多边形信息来更新障碍物容器
  if (costmap_converter_)
    updateObstacleContainerWithCostmapConverter();
  else
    updateObstacleContainerWithCostmap();

  // 也考虑自定义障碍物，必须在其他的更新后在被调用，因为该容器没有被清理
  updateObstacleContainerWithCustomObstacles();


  // 加锁，在接下来的优化过程中不允许配置被修改
  boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

  // 准备工作做了这么久，现在开始真正的局部轨迹规划 ╮(╯▽╰)╭
//   bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_.goal_tolerance.free_goal_vel); // straight line init
  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
  if (!success)
  {
    planner_->clearPlanner(); // 强制重新初始化
    ROS_WARN("teb_local_planner was not able to obtain a local plan for the current setting.");

    ++no_infeasible_plans_; // 不可行方案数+1
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner was not able to obtain a local plan";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  // 是否发散
  if (planner_->hasDiverged())
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    // 重置所有变量，再次开始新轨迹的初始化
    planner_->clearPlanner();
    ROS_WARN_THROTTLE(1.0, "TebLocalPlannerROS: the trajectory has diverged. Resetting planner...");

    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  // (but within the first few states only) 检查路径是否可行
  if(cfg_.robot.is_footprint_dynamic)
  {
    // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
  }

  bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses);
  if (!feasible)
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    // 重置所有变量，再次开始新轨迹的初始化
    planner_->clearPlanner();
    ROS_WARN("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...");

    ++no_infeasible_plans_; // 不可行路径的数量+1
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner trajectory is not feasible";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  // 为这个采样区间获取速度命令
  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_.trajectory.control_look_ahead_poses))

  {
    planner_->clearPlanner();
    ROS_WARN("TebLocalPlannerROS: velocity command invalid. Resetting planner...");
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner velocity command invalid";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                   cfg_.robot.max_vel_x, cfg_.robot.max_vel_y, cfg_.robot.max_vel_theta, cfg_.robot.max_vel_x_backwards);

  // convert rot-vel to steering angle if desired (carlike robot).
  // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
  // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
  if (cfg_.robot.cmd_angle_instead_rotvel)
  {
    cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z,
                                                                cfg_.robot.wheelbase, 0.95*cfg_.robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.twist.angular.z))
    {
      cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
      last_cmd_ = cmd_vel.twist;
      planner_->clearPlanner();
      ROS_WARN("TebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...");
      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = ros::Time::now();
      message = "teb_local_planner steering angle is not finite";
      return mbf_msgs::ExePathResult::NO_VALID_CMD;
    }
  }

  // 找到可行的局部轨迹，重置计数器
  no_infeasible_plans_ = 0;

  // 存储上个命令，方便恢复的分析
  last_cmd_ = cmd_vel.twist;

  // 可视化障碍物，路过点，全局路径
  planner_->visualize();
  visualization_->publishObstacles(obstacles_);
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);
  return mbf_msgs::ExePathResult::SUCCESS;
}


bool TebLocalPlannerROS::isGoalReached()
{
  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    planner_->clearPlanner();
    return true;
  }
  return false;
}



void TebLocalPlannerROS::updateObstacleContainerWithCostmap()
{
  // 加进代价地图障碍物
  if (cfg_.obstacles.include_costmap_obstacles)
  {
    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();

    for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
    {
      for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
      {
        if (costmap_->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE)
        {
          Eigen::Vector2d obs;
          costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));

          // 检查该障碍物是否需要被考虑，比如机器人后面太远的障碍物就不考虑了
          Eigen::Vector2d obs_dir = obs-robot_pose_.position();
          if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist  )
            continue;

          obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
        }
      }
    }
  }
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
{
  if (!costmap_converter_)
    return;

  // 从代价地图转换器获取障碍物
  costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
  if (!obstacles)
    return;

  for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  {
    const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    const geometry_msgs::Polygon* polygon = &obstacle->polygon;

    if (polygon->points.size()==1 && obstacle->radius > 0) // 圆形
    {
      obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
    }
    else if (polygon->points.size()==1) // 点
    {
      obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
    }
    else if (polygon->points.size()==2) // 线段
    {
      obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                        polygon->points[1].x, polygon->points[1].y )));
    }
    else if (polygon->points.size()>2) // 真正意义上的多边形
    {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (std::size_t j=0; j<polygon->points.size(); ++j)
        {
            polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
    }

    // 如果障碍物在移动，设置速度
    if(!obstacles_.empty())
      obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
  }
}


void TebLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
{
  // 加入通过消息获得的自定义障碍物
  boost::mutex::scoped_lock l(custom_obst_mutex_);

  if (!custom_obstacle_msg_.obstacles.empty())
  {
    // 用global_frame_设置障碍物的坐标系，而不是随便一个坐标系
    Eigen::Affine3d obstacle_to_map_eig;
    try
    {
      geometry_msgs::TransformStamped obstacle_to_map =  tf_->lookupTransform(global_frame_, ros::Time(0),
                                                                              custom_obstacle_msg_.header.frame_id, ros::Time(0),
                                                                              custom_obstacle_msg_.header.frame_id, ros::Duration(cfg_.robot.transform_tolerance));
      obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      obstacle_to_map_eig.setIdentity();
    }

    for (size_t i=0; i<custom_obstacle_msg_.obstacles.size(); ++i)
    {
      if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0 ) // circle
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new CircularObstacle( (obstacle_to_map_eig * pos).head(2), custom_obstacle_msg_.obstacles.at(i).radius)));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 ) // point
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new PointObstacle( (obstacle_to_map_eig * pos).head(2) )));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2 ) // line
      {
        Eigen::Vector3d line_start( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        Eigen::Vector3d line_end( custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().z );
        obstacles_.push_back(ObstaclePtr(new LineObstacle( (obstacle_to_map_eig * line_start).head(2),
                                                           (obstacle_to_map_eig * line_end).head(2) )));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
      {
        ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
        continue;
      }
      else // 多边形
      {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (size_t j=0; j<custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
        {
          Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].z );
          polyobst->pushBackVertex( (obstacle_to_map_eig * pos).head(2) );
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
      }

      // 如果障碍物在移动，设置速度
      if(!obstacles_.empty())
        obstacles_.back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
    }
  }
}

void TebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation)
{
  via_points_.clear();

  if (min_separation<=0)
    return;

  std::size_t prev_idx = 0;
  for (std::size_t i=1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
  {
    // check separation to the previous via-point inserted
    if (distance_points2d( transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position ) < min_separation)
      continue;

    // add via-point
    via_points_.push_back( Eigen::Vector2d( transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y ) );
    prev_idx = i;
  }

}

Eigen::Vector2d TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
{
  Eigen::Vector2d vel;
  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
  return vel;
}


bool TebLocalPlannerROS::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
    return true;

  try
  {
    // 转换机器人位姿到路径的坐标系 (we do not wait here, since pruning not crucial, if missed a few times)
    geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
    geometry_msgs::PoseStamped robot;
    tf2::doTransform(global_pose, robot, global_to_plan_transform);

    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;

    // 迭代路径点直到找到距离机器人很近的路径点
    std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
         erase_end = it;
         break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;

    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}


bool TebLocalPlannerROS::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                  const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                  std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::TransformStamped* tf_plan_to_global) const
{
  // 该函数是把base_local_planner/goal_functions.h 稍微做了一下修改

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try
  {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // 获取路径坐标系到全局坐标系的转换
    geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp,
                                                                                  plan_pose.header.frame_id, ros::Duration(cfg_.robot.transform_tolerance));

    // 得到路径坐标系下机器人位姿
    geometry_msgs::PoseStamped robot_pose;
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

    // 抛弃掉路径中在local_costmap外面的点
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85; // dist_threshold只取代价地图尺寸的85%, 为了更好处理局部代价地图边缘上的障碍物点


    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;

    // 找到路径中距离机器人最近的点
    bool robot_reached = false;
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

      if (robot_reached && new_sq_dist > sq_dist)
        break;

      if (new_sq_dist < sq_dist) // 找到最近距离
      {
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
          robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
      }
    }

    geometry_msgs::PoseStamped newer_pose;

    double plan_length = 0; // 路径累加的欧拉距离

    // 将特定范围内的路径点进行转换
    while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
    {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;

      // 路径点距离累加
      if (i>0 && max_plan_length>0)
        plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);

      ++i;
    }

    // 机器人距离目标点很近时(<sq_dist_threshold) 但是还没有到比如(orientation error >>0),
    // 这种情况下transformed_plan可能为空，因此显式地填入全局目标点
    if (transformed_plan.empty())
    {
      tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      // 返回当前目标点的索引（在限定距离范围内）
      if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
      // 返回当前目标点的索引（在限定距离范围内）
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }

    // 返回路径坐标系到全局坐标系的转换
    if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
  }
  catch(tf::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex)
  {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex)
  {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }

  return true;
}




double TebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
              int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length) const
{
  int n = (int)global_plan.size();

  // 是否已经到了目标点附近
  if (current_goal_idx > n-moving_average_length-2)
  {
    if (current_goal_idx >= n-1) // 到达目标点
    {
      return tf2::getYaw(local_goal.pose.orientation);
    }
    else
    {
      tf2::Quaternion global_orientation;
      tf2::convert(global_plan.back().pose.orientation, global_orientation);
      tf2::Quaternion rotation;
      tf2::convert(tf_plan_to_global.transform.rotation, rotation);
      // TODO(roesmann): avoid conversion to tf2::Quaternion
      return tf2::getYaw(rotation *  global_orientation);
    }
  }

  // 如果期望的位姿数没有达到，减少被考虑的位姿数
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before

  std::vector<double> candidates;
  geometry_msgs::PoseStamped tf_pose_k = local_goal;
  geometry_msgs::PoseStamped tf_pose_kp1;

  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i)
  {
    // 位姿从全局坐标转换到路径坐标系下
    tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);

    // 计算yaw角度
    candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
        tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );

    if (i<range_end-1)
      tf_pose_k = tf_pose_kp1;
  }
  return average_angles(candidates);
}


void TebLocalPlannerROS::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards) const
{
  double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
  // 限制前向的线速度
  if (vx > max_vel_x)
    ratio_x = max_vel_x / vx;

  // 限制横向线速度
  if (vy > max_vel_y || vy < -max_vel_y)
    ratio_y = std::abs(vy / max_vel_y);

  // 限制角速度
  if (omega > max_vel_theta || omega < -max_vel_theta)
    ratio_omega = std::abs(max_vel_theta / omega);

  // 限制向后的速度
  if (max_vel_x_backwards<=0)
  {
    ROS_WARN_ONCE("TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  }
  else if (vx < -max_vel_x_backwards)
    ratio_x = - max_vel_x_backwards / vx;

  if (cfg_.robot.use_proportional_saturation)
  {
    double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
    vx *= ratio;
    vy *= ratio;
    omega *= ratio;
  }
  else
  {
    vx *= ratio_x;
    vy *= ratio_y;
    omega *= ratio_omega;
  }
}


double TebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
{
  if (omega==0 || v==0)
    return 0;

  double radius = v/omega;

  if (fabs(radius) < min_turning_radius)
    radius = double(g2o::sign(radius)) * min_turning_radius;

  return std::atan(wheelbase / radius);
}


void TebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    ROS_WARN_COND(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}



void TebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::PoseStamped>& transformed_plan,  int& goal_idx)
{
    ros::Time current_time = ros::Time::now();

    // reduced horizon backup mode
    if (cfg_.recovery.shrink_horizon_backup &&
        goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
       (no_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_).toSec() < cfg_.recovery.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
    {
        ROS_INFO_COND(no_infeasible_plans_==1, "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_.recovery.shrink_horizon_min_duration);


        // Shorten horizon if requested
        // reduce to 50 percent:
        int horizon_reduction = goal_idx/2;

        if (no_infeasible_plans_ > 9)
        {
            ROS_INFO_COND(no_infeasible_plans_==10, "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
            horizon_reduction /= 2;
        }

        // we have a small overhead here, since we already transformed 50% more of the trajectory.
        // But that's ok for now, since we do not need to make transformGlobalPlan more complex
        // and a reduced horizon should occur just rarely.
        int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
        goal_idx -= horizon_reduction;
        if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
            transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
        else
            goal_idx += horizon_reduction; // this should not happen, but safety first ;-)
    }


    // detect and resolve oscillations
    if (cfg_.recovery.oscillation_recovery)
    {
        double max_vel_theta;
        double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_.robot.max_vel_x : cfg_.robot.max_vel_x_backwards;
        if (cfg_.robot.min_turning_radius!=0 && max_vel_current>0)
            max_vel_theta = std::max( max_vel_current/std::abs(cfg_.robot.min_turning_radius),  cfg_.robot.max_vel_theta );
        else
            max_vel_theta = cfg_.robot.max_vel_theta;

        failure_detector_.update(last_cmd_, cfg_.robot.max_vel_x, cfg_.robot.max_vel_x_backwards, max_vel_theta,
                               cfg_.recovery.oscillation_v_eps, cfg_.recovery.oscillation_omega_eps);

        bool oscillating = failure_detector_.isOscillating();
        bool recently_oscillated = (ros::Time::now()-time_last_oscillation_).toSec() < cfg_.recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently

        if (oscillating)
        {
            if (!recently_oscillated)
            {
                // save current turning direction
                if (robot_vel_.angular.z > 0)
                    last_preferred_rotdir_ = RotType::left;
                else
                    last_preferred_rotdir_ = RotType::right;
                ROS_WARN("TebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
            }
            time_last_oscillation_ = ros::Time::now();
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
        }
        else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
        {
            last_preferred_rotdir_ = RotType::none;
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
            ROS_INFO("TebLocalPlannerROS: oscillation recovery disabled/expired.");
        }
    }

}

void TebLocalPlannerROS::customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  boost::mutex::scoped_lock l(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;
}

void TebLocalPlannerROS::customViaPointsCB(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  if (cfg_.trajectory.global_plan_viapoint_sep > 0)
  {
    ROS_WARN("Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
             "Ignoring custom via-points.");
    custom_via_points_active_ = false;
    return;
  }

  boost::mutex::scoped_lock l(via_point_mutex_);
  via_points_.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  custom_via_points_active_ = !via_points_.empty();
}

RobotFootprintModelPtr TebLocalPlannerROS::getRobotFootprintFromParamServer(const ros::NodeHandle& nh, const TebConfig& config)
{
  std::string model_name;
  if (!nh.getParam("footprint_model/type", model_name))
  {
    ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
    return boost::make_shared<PointRobotFootprint>();
  }

  // 点状
  if (model_name.compare("point") == 0)
  {
    ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
    return boost::make_shared<PointRobotFootprint>(config.obstacles.min_obstacle_dist);
  }

  // 圆形
  if (model_name.compare("circular") == 0)
  {
    // 获取半径
    double radius;
    if (!nh.getParam("footprint_model/radius", radius))
    {
      ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                       << "/footprint_model/radius' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius <<"m) loaded for trajectory optimization.");
    return boost::make_shared<CircularRobotFootprint>(radius);
  }

  // 线型
  if (model_name.compare("line") == 0)
  {
    // 检查参数
    if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end"))
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                       << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // 获取线段两点的坐标
    std::vector<double> line_start, line_end;
    nh.getParam("footprint_model/line_start", line_start);
    nh.getParam("footprint_model/line_end", line_end);
    if (line_start.size() != 2 || line_end.size() != 2)
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                       << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }

    ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] <<"]m, line_end: ["
                     << line_end[0] << "," << line_end[1] << "]m) loaded for trajectory optimization.");
    return boost::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()), config.obstacles.min_obstacle_dist);
  }

  // 双圆形
  if (model_name.compare("two_circles") == 0)
  {
    // 参数检查
    if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius")
        || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
    {
      ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
                       << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    double front_offset, front_radius, rear_offset, rear_radius;
    nh.getParam("footprint_model/front_offset", front_offset);
    nh.getParam("footprint_model/front_radius", front_radius);
    nh.getParam("footprint_model/rear_offset", rear_offset);
    nh.getParam("footprint_model/rear_radius", rear_radius);
    ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius
                    << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
    return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  }

  // 多边形
  if (model_name.compare("polygon") == 0)
  {

    // 参数检查
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc) )
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                       << "/footprint_model/vertices' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // 获取顶点
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      try
      {
        Point2dContainer polygon = makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
        ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
        return boost::make_shared<PolygonRobotFootprint>(polygon);
      }
      catch(const std::exception& ex)
      {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
    }
    else
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                       << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }

  }

  // otherwise
  ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
  return boost::make_shared<PointRobotFootprint>();
}




Point2dContainer TebLocalPlannerROS::makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name)
{
   // 确保获得一个含有至少3个点的矩阵
   if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       footprint_xmlrpc.size() < 3)
   {
     ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
     throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                              "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
   }

   Point2dContainer footprint;
   Eigen::Vector2d pt;

   for (int i = 0; i < footprint_xmlrpc.size(); ++i)
   {
     // 确保每个点有x,y坐标
     XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
     if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
         point.size() != 2)
     {
       ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                 "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                  full_param_name.c_str());
       throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x() = getNumberFromXMLRPC(point[ 0 ], full_param_name);
    pt.y() = getNumberFromXMLRPC(point[ 1 ], full_param_name);

    footprint.push_back(pt);
  }
  return footprint;
}

double TebLocalPlannerROS::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // 确保value是double或者int
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str());
     throw std::runtime_error("Values in the footprint specification must be numbers");
   }
   return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

} // end namespace teb_local_planner


