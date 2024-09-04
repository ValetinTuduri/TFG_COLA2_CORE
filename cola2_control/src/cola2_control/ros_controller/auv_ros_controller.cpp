
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_control/ros_controller/auv_ros_controller_base.h>
#include <cola2_lib_ros/navigation_helper.h>

IAUVROSController::IAUVROSController(const std::string &name, const std::string &frame_id)
    : name_(name),
      frame_id_(frame_id),
      last_nav_time_(0.0),
      last_altitude_(0.5),
      last_altitude_age_(0.0),
      last_depth_(0.0) {}


void IAUVROSController::initBase(const std::shared_ptr<rclcpp::Node> &node, std::shared_ptr<IAUVController> auv_controller_ptr, double period)
{

  node_ = node;
  diagnostic_ = std::make_shared<cola2::ros::DiagnosticHelper>(node_, "soft", cola2::ros::getUnresolvedNodeName(node_));
  // Init pointer to AUV controller
  auv_controller_ = auv_controller_ptr;

  // Save controller frequency
  frequency_ = 1.0 / period;

  // Publishers
  pub_wrench_ = node_->create_publisher<cola2_msgs::msg::BodyForceReq>("merged_body_force_req", 1);
  pub_merged_pose_ = node_->create_publisher<cola2_msgs::msg::WorldWaypointReq>("merged_world_waypoint_req", 1);
  pub_merged_twist_ = node_->create_publisher<cola2_msgs::msg::BodyVelocityReq>("merged_body_velocity_req", 1);
  pub_thrusters_setpoint_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/girona500/controller/thruster_setpoints_sim", 1);

  // Subscribers --> WARNING! The buffer should be at least the size of maximum Request send per iteration/kind

  sub_nav_data_ = node_->create_subscription<cola2_msgs::msg::NavSts>("/navigation", 2, std::bind(&IAUVROSController::updateNav, this, std::placeholders::_1));
  sub_ww_req_ = node_->create_subscription<cola2_msgs::msg::WorldWaypointReq>("/world_waypoint_req", 10, std::bind(&IAUVROSController::updateWWR, this, std::placeholders::_1));
  sub_bv_req_ = node_->create_subscription<cola2_msgs::msg::BodyVelocityReq>("/body_velocity_req", 10, std::bind(&IAUVROSController::updateBVR, this, std::placeholders::_1));
  sub_bf_req_ = node_->create_subscription<cola2_msgs::msg::BodyForceReq>("/body_force_req", 10, std::bind(&IAUVROSController::updateBFR, this, std::placeholders::_1));

  _are_thrusters_killed = false;


  // Services
  enable_pose_controller_srv_ = node_->create_service<std_srvs::srv::Trigger>("enable_pose_controller", std::bind(&IAUVROSController::enablePoseController, this, std::placeholders::_1, std::placeholders::_2));
  disable_pose_controller_srv_ = node_->create_service<std_srvs::srv::Trigger>("disable_pose_controller", std::bind(&IAUVROSController::disablePoseController, this, std::placeholders::_1, std::placeholders::_2));
  enable_velocity_controller_srv_ = node_->create_service<std_srvs::srv::Trigger>("enable_velocity_controller", std::bind(&IAUVROSController::enableVelocityController, this, std::placeholders::_1, std::placeholders::_2));
  disable_velocity_controller_srv_ = node_->create_service<std_srvs::srv::Trigger>("disable_velocity_controller", std::bind(&IAUVROSController::disableVelocityController, this, std::placeholders::_1, std::placeholders::_2));
  enable_thruster_allocator_srv_ = node_->create_service<std_srvs::srv::Trigger>("enable_thrusters", std::bind(&IAUVROSController::enableThrusterAllocator, this, std::placeholders::_1, std::placeholders::_2));
  disable_thruster_allocator_srv_ = node_->create_service<std_srvs::srv::Trigger>("disable_thrusters", std::bind(&IAUVROSController::disableThrusterAllocator, this, std::placeholders::_1, std::placeholders::_2));

  // Timers
  timer_ = node_->create_wall_timer(std::chrono::duration<double>(period), std::bind(&IAUVROSController::timerCallback, this));
  check_diagnostics_ = node_->create_wall_timer(std::chrono::seconds(1), std::bind(&IAUVROSController::checkDiagnostics, this));

  diagnostic_->setEnabled(true);
}

bool IAUVROSController::enablePoseController(std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    std::cout << "[Controller] Enable pose controller\n";
    auv_controller_->setPoseController(true);
    res->success = true;
    return true;
}

bool IAUVROSController::disablePoseController(std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    std::cout << "[Controller] Disable pose controller\n";
    auv_controller_->setPoseController(false);
    res->success = true;
    return true;
}

bool IAUVROSController::enableVelocityController(std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    std::cout << "[Controller] Enable velocity controller\n";
    auv_controller_->setVelocityController(true);
    res->success = true;
    return true;
}

bool IAUVROSController::disableVelocityController(std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    std::cout << "[Controller] Disable velocity controller\n";
    auv_controller_->setVelocityController(false);
    res->success = true;
    return true;
}

bool IAUVROSController::enableThrusterAllocator(std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    std::cout << "[Controller] Enable thruster allocator\n";
    auv_controller_->setThrusterAllocator(true);
    _are_thrusters_killed = false;
    res->success = true;
    return true;
}

bool IAUVROSController::disableThrusterAllocator(std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    std::cout << "[Controller] Disable thruster allocator\n";

    // Send last setpoint to zero
    Eigen::VectorXd setpoint(auv_controller_->getNumberofThrusters());
    for (unsigned int i = 0; i < auv_controller_->getNumberofThrusters(); i++)
    {
        setpoint(i) = 0.0;
    }
    publishThrusterSetpoint(setpoint, node_->now());

    // Disable thrusters
    auv_controller_->setThrusterAllocator(false);
    res->success = true;
    return true;
}

void IAUVROSController::checkDiagnostics()
{
    diagnostic_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::OK);
    if (!auv_controller_->isPoseControllerEnable())
    {
        diagnostic_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::WARN, "pose controller disabled");
    }
    if (!auv_controller_->isVelocityControllerEnable())
    {
        diagnostic_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::WARN, "velocity controller disabled");
    }
    if (!auv_controller_->isThrusterAllocatorEnable())
    {
        diagnostic_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::WARN, "thruster allocator disabled");
    }
    diagnostic_->addKeyValue("pose_controller_enabled", auv_controller_->isPoseControllerEnable());
    diagnostic_->addKeyValue("velocity_controller_enabled", auv_controller_->isVelocityControllerEnable());
    diagnostic_->addKeyValue("thruster_allocator_enabled", auv_controller_->isThrusterAllocatorEnable());
    diagnostic_->publish(node_);
}

void IAUVROSController::timerCallback()
{
  // Get current time
  rclcpp::Time now = node_->get_clock()->now();

  // Check last navigation time
  if (std::fabs(now.seconds() - last_nav_time_) > 1.0)
  {
    RCLCPP_INFO(node_->get_logger(), "Waiting for navigation");
    diagnostic_->reportData(now);

    // Send zeros
    Eigen::VectorXd setpoint(auv_controller_->getNumberofThrusters());
    for (unsigned int i = 0; i < auv_controller_->getNumberofThrusters(); i++)
    {
      setpoint(i) = 0.0;
    }
    publishThrusterSetpoint(setpoint, now);
    return;
  }

  // Iterate controller
  auv_controller_->iteration(now.seconds());

  // Compute thruster setpoints
  auv_controller_->computeThrusterAllocator();

  // Check period for diagnostics
  diagnostic_->reportValidData(now);

  // Publish data
  publishMergedPose(auv_controller_->getMergedPose(), now);
  publishMergedTwist(auv_controller_->getMergedTwist(), now);
  publishMergedWrench(auv_controller_->getMergedWrench(), now);

  // Publish thruster setpoint if enabled
  if (auv_controller_->isThrusterAllocatorEnable())
  {
    Eigen::VectorXd setpoint = auv_controller_->getThrusterSetpoints();
    publishThrusterSetpoint(setpoint, now);
  }
  else
  {
    // Send zeros
    Eigen::VectorXd setpoint(auv_controller_->getNumberofThrusters());
    for (unsigned int i = 0; i < auv_controller_->getNumberofThrusters(); i++)
    {
      setpoint(i) = 0.0;
    }
    publishThrusterSetpoint(setpoint, now);
    RCLCPP_INFO(node_->get_logger(), "setpoints from 192");
  }
}


void IAUVROSController::updateNav(const cola2_msgs::msg::NavSts::SharedPtr msg)
{
  // Check for valid navigation
  if (!cola2::ros::navigationIsValid(*msg))
  {
    return;
  }

  // Store last valid navigation time
  last_nav_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  // Update pose feedback
  std::vector<double> pose_feedback;
  pose_feedback.push_back(msg->position.north);
  pose_feedback.push_back(msg->position.east);
  pose_feedback.push_back(msg->position.depth);
  pose_feedback.push_back(msg->orientation.roll);
  pose_feedback.push_back(msg->orientation.pitch);
  pose_feedback.push_back(msg->orientation.yaw);
  auv_controller_->updatePoseFeedback(pose_feedback);

  // Update twist feedback
  std::vector<double> twist_feedback;
  twist_feedback.push_back(msg->body_velocity.x);
  twist_feedback.push_back(msg->body_velocity.y);
  twist_feedback.push_back(msg->body_velocity.z);
  twist_feedback.push_back(msg->orientation_rate.roll);
  twist_feedback.push_back(msg->orientation_rate.pitch);
  twist_feedback.push_back(msg->orientation_rate.yaw);
  auv_controller_->updateTwistFeedback(twist_feedback);

  // Stores last altitude. If altitude is invalid, during 5 seconds estimate it wrt last altitude and delta depth.
  // If more than 5 seconds put it at 0.5.
  if (msg->altitude > 0.0)
  {
    last_altitude_ = msg->altitude;
    last_altitude_age_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    last_depth_ = msg->position.depth;
  }
  else
  {
    if ((rclcpp::Clock().now().seconds() - last_altitude_age_) > 5.0)
    {
      last_altitude_ = 0.5;
    }
    else
    {
      last_altitude_ = last_altitude_ - (msg->position.depth - last_depth_);
      last_depth_ = msg->position.depth;
    }
  }
}

void IAUVROSController::updateWWR(const cola2_msgs::msg::WorldWaypointReq::SharedPtr msg)
{
  // Init request
  Request req(msg->goal.requester, msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9, msg->goal.priority, 6);

  // Set disable axis
  std::vector<bool> disabled_axis;
  disabled_axis.push_back(msg->disable_axis.x);
  disabled_axis.push_back(msg->disable_axis.y);
  disabled_axis.push_back(msg->disable_axis.z);
  disabled_axis.push_back(msg->disable_axis.roll);
  disabled_axis.push_back(msg->disable_axis.pitch);
  disabled_axis.push_back(msg->disable_axis.yaw);
  req.setDisabledAxis(disabled_axis);

  // Set values
  std::vector<double> values;
  values.push_back(msg->position.north);
  values.push_back(msg->position.east);

  // If desired Z is in altitude, transform it into depth
  if (msg->altitude_mode)
  {
    double altitude_to_depth = (last_depth_ + last_altitude_) - msg->altitude;
    if (altitude_to_depth < 0.0)
      altitude_to_depth = 0.0;
    values.push_back(altitude_to_depth);
  }
  else
  {
    values.push_back(msg->position.depth);
  }

  values.push_back(msg->orientation.roll);
  values.push_back(msg->orientation.pitch);
  values.push_back(msg->orientation.yaw);
  req.setValues(values);

  // Add request to controller ptr.
  auv_controller_->updatePoseRequest(req);
}

#include <rclcpp/rclcpp.hpp>
#include <cola2_msgs/msg/body_velocity_req.hpp>
#include <cola2_msgs/msg/body_force_req.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>

void IAUVROSController::updateBVR(const cola2_msgs::msg::BodyVelocityReq::SharedPtr msg)
{
  // Init request
  Request req(msg->goal.requester, msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9, msg->goal.priority, 6);

  // Set disable axis
  std::vector<bool> disabled_axis;
  disabled_axis.push_back(msg->disable_axis.x);
  disabled_axis.push_back(msg->disable_axis.y);
  disabled_axis.push_back(msg->disable_axis.z);
  disabled_axis.push_back(msg->disable_axis.roll);
  disabled_axis.push_back(msg->disable_axis.pitch);
  disabled_axis.push_back(msg->disable_axis.yaw);
  req.setDisabledAxis(disabled_axis);

  // Set values
  std::vector<double> values;
  values.push_back(msg->twist.linear.x);
  values.push_back(msg->twist.linear.y);
  values.push_back(msg->twist.linear.z);
  values.push_back(msg->twist.angular.x);
  values.push_back(msg->twist.angular.y);
  values.push_back(msg->twist.angular.z);
  req.setValues(values);



  // Add request to controller ptr.
  auv_controller_->updateTwistRequest(req);
}

void IAUVROSController::updateBFR(const cola2_msgs::msg::BodyForceReq::SharedPtr msg)
{
  // Init request
  Request req(msg->goal.requester, msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9, msg->goal.priority, 6);

  // Set disable axis
  std::vector<bool> disabled_axis;
  disabled_axis.push_back(msg->disable_axis.x);
  disabled_axis.push_back(msg->disable_axis.y);
  disabled_axis.push_back(msg->disable_axis.z);
  disabled_axis.push_back(msg->disable_axis.roll);
  disabled_axis.push_back(msg->disable_axis.pitch);
  disabled_axis.push_back(msg->disable_axis.yaw);
  req.setDisabledAxis(disabled_axis);

  // Set values
  std::vector<double> values;
  values.push_back(msg->wrench.force.x);
  values.push_back(msg->wrench.force.y);
  values.push_back(msg->wrench.force.z);
  values.push_back(msg->wrench.torque.x);
  values.push_back(msg->wrench.torque.y);
  values.push_back(msg->wrench.torque.z);
  req.setValues(values);

  // Add request to controller ptr.
  auv_controller_->updateWrenchRequest(req);
}
void IAUVROSController::publishThrusterSetpoint(const Eigen::VectorXd setpoint, const rclcpp::Time now)
{
  // Create ROS thruster setpoint msg
  std_msgs::msg::Float64MultiArray output;

  // Fill header
  //output.header.frame_id = frame_id_;
  //output.header.stamp = now;

  for (unsigned int i = 0; i < setpoint.size(); i++)
  {
    output.data.push_back(setpoint[i]);
  }

  // Publish message
  pub_thrusters_setpoint_->publish(output);
}

void IAUVROSController::publishMergedPose(const Request pose, const rclcpp::Time now)
{
  // Create ROS output
  cola2_msgs::msg::WorldWaypointReq output;

  // Fill header
  output.header.frame_id = "world_ned";
  output.header.stamp = now;

  // Fill goal
  output.goal.priority = pose.getPriority();
  output.goal.requester = pose.getRequester();

  // Fill disable axis
  std::vector<bool> disable_axis = pose.getDisabledAxis();
  assert(disable_axis.size() == 6);
  output.disable_axis.x = disable_axis.at(0);
  output.disable_axis.y = disable_axis.at(1);
  output.disable_axis.z = disable_axis.at(2);
  output.disable_axis.roll = disable_axis.at(3);
  output.disable_axis.pitch = disable_axis.at(4);
  output.disable_axis.yaw = disable_axis.at(5);

  // Fill output values
  std::vector<double> values = pose.getValues();
  assert(values.size() == 6);
  output.position.north = values.at(0);
  output.position.east = values.at(1);
  output.position.depth = values.at(2);
  output.orientation.roll = values.at(3);
  output.orientation.pitch = values.at(4);
  output.orientation.yaw = values.at(5);

  // Publish output
  pub_merged_pose_->publish(output);
}

void IAUVROSController::publishMergedTwist(const Request twist, const rclcpp::Time now)
{
  // Create ROS output
  cola2_msgs::msg::BodyVelocityReq output;

  // Fill header
  output.header.frame_id = frame_id_;
  output.header.stamp = now;

  // Fill goal
  output.goal.priority = twist.getPriority();
  output.goal.requester = twist.getRequester();

  // Fill disable axis
  std::vector<bool> disable_axis = twist.getDisabledAxis();
  assert(disable_axis.size() == 6);
  output.disable_axis.x = disable_axis.at(0);
  output.disable_axis.y = disable_axis.at(1);
  output.disable_axis.z = disable_axis.at(2);
  output.disable_axis.roll = disable_axis.at(3);
  output.disable_axis.pitch = disable_axis.at(4);
  output.disable_axis.yaw = disable_axis.at(5);

  // Fill output values
  std::vector<double> values = twist.getValues();
  assert(values.size() == 6);
  output.twist.linear.x = values.at(0);
  output.twist.linear.y = values.at(1);
  output.twist.linear.z = values.at(2);
  output.twist.angular.x = values.at(3);
  output.twist.angular.y = values.at(4);
  output.twist.angular.z = values.at(5);

  // Publish output
  pub_merged_twist_->publish(output);
}

void IAUVROSController::publishMergedWrench(const Request response, const rclcpp::Time now)
{
  // Create ROS output

  cola2_msgs::msg::BodyForceReq output;

  // Fill header
  output.header.frame_id = frame_id_;
  output.header.stamp = now;

  // Fill goal
  output.goal.priority = response.getPriority();
  output.goal.requester = response.getRequester();

  // Fill disable axis
  std::vector<bool> disable_axis = response.getDisabledAxis();
  assert(disable_axis.size() == 6);
  output.disable_axis.x = disable_axis.at(0);
  output.disable_axis.y = disable_axis.at(1);
  output.disable_axis.z = disable_axis.at(2);
  output.disable_axis.roll = disable_axis.at(3);
  output.disable_axis.pitch = disable_axis.at(4);
  output.disable_axis.yaw = disable_axis.at(5);

  // Fill output values
  std::vector<double> values = response.getValues();
  assert(values.size() == 6);
  output.wrench.force.x = values.at(0);
  output.wrench.force.y = values.at(1);
  output.wrench.force.z = values.at(2);
  output.wrench.torque.x = values.at(3);
  output.wrench.torque.y = values.at(4);
  output.wrench.torque.z = values.at(5);

  // Publish output
  pub_wrench_->publish(output);
}
