
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVROSCONTROLLER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVROSCONTROLLER_H_

#include <cola2_control/low_level_controllers/auv_controller_base.h>
#include <cola2_control/low_level_controllers/request.h>
#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/this_node.h>
#include "cola2_msgs/msg/body_force_req.hpp"
#include "cola2_msgs/msg/body_velocity_req.hpp"
#include "cola2_msgs/msg/nav_sts.hpp"
#include "cola2_msgs/msg/setpoints.hpp"
#include "cola2_msgs/msg/world_waypoint_req.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

class IAUVROSController
{
private:
  void publishThrusterSetpoint(const Eigen::VectorXd setpoint,  const rclcpp::Time now);

  void publishFinSetpoint(const Eigen::VectorXd setpoint,  const rclcpp::Time now);

  void publishMergedPose(const Request pose,  const rclcpp::Time now);

  void publishMergedTwist(const Request twist,  const rclcpp::Time now);

  void publishMergedWrench(const Request response,  const rclcpp::Time now);

  // Node handle
  rclcpp::Node::SharedPtr node_;

  // Name
  std::string name_;

  // Frame id
  std::string frame_id_;

  // Controller frequency
  double frequency_;

  // Diagnostics
  std::shared_ptr<cola2::ros::DiagnosticHelper> diagnostic_;

  // Publisher
  rclcpp::Publisher<cola2_msgs::msg::BodyForceReq>::SharedPtr pub_wrench_;
  rclcpp::Publisher<cola2_msgs::msg::WorldWaypointReq>::SharedPtr pub_merged_pose_;
  rclcpp::Publisher<cola2_msgs::msg::BodyVelocityReq>::SharedPtr pub_merged_twist_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_thrusters_setpoint_;
  rclcpp::Publisher<cola2_msgs::msg::Setpoints>::SharedPtr pub_fins_setpoint_;

  // Subscriber
  rclcpp::Subscription<cola2_msgs::msg::NavSts>::SharedPtr sub_nav_data_;
  rclcpp::Subscription<cola2_msgs::msg::WorldWaypointReq>::SharedPtr sub_ww_req_;
  rclcpp::Subscription<cola2_msgs::msg::BodyVelocityReq>::SharedPtr sub_bv_req_;
  rclcpp::Subscription<cola2_msgs::msg::BodyForceReq>::SharedPtr sub_bf_req_;

  bool _are_thrusters_killed;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr check_diagnostics_;
  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_pose_controller_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_pose_controller_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_velocity_controller_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_velocity_controller_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_thruster_allocator_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_thruster_allocator_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_fin_allocator_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_fin_allocator_srv_;

  // AUV controller ptr.
  std::shared_ptr<IAUVController> auv_controller_;

  // Last navigation time in seconds
  double last_nav_time_;

  // Estimated total altitude
  double last_altitude_;
  double last_altitude_age_;
  double last_depth_;

public:
  IAUVROSController(const std::string &name, const std::string &frame_id);

  void initBase(const rclcpp::Node::SharedPtr &node, std::shared_ptr<IAUVController> auv_controller_ptr, double period);

  bool enablePoseController(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  bool disablePoseController(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  bool enableVelocityController(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  bool disableVelocityController(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  bool enableThrusterAllocator(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  bool disableThrusterAllocator(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  bool enableFinAllocator(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  bool disableFinAllocator(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void checkDiagnostics();

  void timerCallback();

  void updateNav(const cola2_msgs::msg::NavSts::SharedPtr msg);

  void updateWWR(const cola2_msgs::msg::WorldWaypointReq::SharedPtr msg);

  void updateBVR(const cola2_msgs::msg::BodyVelocityReq::SharedPtr msg);

  void updateBFR(const cola2_msgs::msg::BodyForceReq::SharedPtr msg);
};


#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_CONTROL_IAUVROSCONTROLLER_H_
