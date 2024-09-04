/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */


#include <cola2_control/controllers/anchor.h>
#include <cola2_control/controllers/holonomic_keep_position.h>
#include <cola2_control/controllers/section.h>
#include <cola2_control/controllers/types.h>
#include <cola2_lib/utils/ned.h>
#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/navigation_helper.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/msg/body_velocity_req.hpp>
#include <cola2_msgs/msg/nav_sts.hpp>
#include <cola2_msgs/action/pilot.hpp>  
#include <cola2_msgs/msg/world_waypoint_req.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>



#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <thread>

class Pilot : public rclcpp::Node
{
protected:


  // ROS variables
  rclcpp::Subscription<cola2_msgs::msg::NavSts>::SharedPtr sub_nav_;

  
  rclcpp::Publisher<cola2_msgs::msg::WorldWaypointReq>::SharedPtr pub_wwr_;
  rclcpp::Publisher<cola2_msgs::msg::BodyVelocityReq>::SharedPtr pub_bvr_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_goal_;
 


  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reload_params_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr srv_publish_params_;


  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::shared_ptr<cola2::ros::DiagnosticHelper> diagnostic_;


  // Action server
  using pilot_action = cola2_msgs::action::Pilot;
  using GoalHandlePilot = rclcpp_action::ServerGoalHandle<pilot_action>;
  rclcpp_action::Server<pilot_action>::SharedPtr pilot_server_;
  

  // Current state
  control::Request request;
  control::State current_state_;
  double last_nav_received_;

  // Controllers
  std::shared_ptr<SectionController> section_controller_;
  std::shared_ptr<HolonomicKeepPositionController> holonomic_keep_position_controller_;
  std::shared_ptr<AnchorController> anchor_controller_;

  // Config
  struct
  {
    SectionControllerConfig section_config;
    HolonomicKeepPositionControllerConfig holonomic_keep_position_config;
    AnchorControllerConfig anchor_config;
  } config_;

  // Methods
  /**
   * \brief Diagnostics timer
   */
  void diagnosticsTimer();

  /**
   * Callback to topic VEHICLE_NAMESPACE/navigator/navigation
   */
  void navCallback(const cola2_msgs::msg::NavSts&);

  /**
   * Callbacks for action Pilot
   */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,std::shared_ptr<const pilot_action::Goal>);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePilot>);
  void handle_accepted(const std::shared_ptr<GoalHandlePilot> );
 


  /**
   * Helper method to publish control commands: WorldWaypointReq and BodyForceReq
   */
  void publishControlCommands(const control::State&, const std::uint64_t, const rclcpp::Time&);

  /**
   * Publish feedback for actionlibs
   */
  void publishFeedback(const std::shared_ptr<GoalHandlePilot> , const control::Feedback&);

  /**
   * Publish an RViz marker to the direction that the AUV is going
   */
  void publishMarker(const double, const double, const double);

  /**
   * Publishes a Section RViz marker
   */
  void publishMarkerSections(const control::PointsList);

  /**
   * Load parameters from ROS param server
   */
  void getConfig();

  /**
   * Service to reload parameters from ROS param server
   */
  bool reloadParamsCallback(const std::shared_ptr<rmw_request_id_t> , const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response>);

  /**
   * Publishes the waypoint that the AUV is going to as a geometry_msgs::PointStamped
   * REDUNDANT WITH publishMarker??
   */
  void publishGoal(const double, const double, const double);

  
  void execute(const std::shared_ptr<GoalHandlePilot> goal_handle);
public:
  /**
   * Class constructor
   */

  Pilot();

  void init();
};

Pilot::Pilot(): Node("pilot"),  last_nav_received_(0.0)
{
  // Wait for time
  while (this->get_clock()->now().seconds() == 0.0)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for valid time source");
  }

  // Get config
  getConfig();

  // Initialize controllers
  section_controller_ = std::make_shared<SectionController>(config_.section_config);
  holonomic_keep_position_controller_ = std::make_shared<HolonomicKeepPositionController>(config_.holonomic_keep_position_config);
  anchor_controller_ = std::make_shared<AnchorController>(config_.anchor_config);

  // Reload parameters service
  auto reload_params_callback_func= std::bind(&Pilot::reloadParamsCallback, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3);
  srv_reload_params_ = this->create_service<std_srvs::srv::Trigger>("reload_params",reload_params_callback_func);

  // Publishers
  pub_wwr_ = this->create_publisher<cola2_msgs::msg::WorldWaypointReq>("/world_waypoint_req", 1);
  pub_bvr_ = this->create_publisher<cola2_msgs::msg::BodyVelocityReq>("/body_velocity_req", 1);
  pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", 1);
  pub_goal_ = this->create_publisher<geometry_msgs::msg::PointStamped>("goal", 1);

  // Subscriber
  sub_nav_ = this->create_subscription<cola2_msgs::msg::NavSts>("navigation", 1,std::bind(&Pilot::navCallback, this, std::placeholders::_1));


  this->pilot_server_ = rclcpp_action::create_server<pilot_action>(
      this,
      "pilot",
      std::bind(&Pilot::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Pilot::handle_cancel, this, std::placeholders::_1),
      std::bind(&Pilot::handle_accepted, this, std::placeholders::_1));

  // Diagnostics timer
   diagnostics_timer_ = this->create_wall_timer(std::chrono::duration<double>(0.5), std::bind(&Pilot::diagnosticsTimer, this));

  
}


void Pilot::init()
{
  diagnostic_ = std::make_shared<cola2::ros::DiagnosticHelper>(shared_from_this(), "navigator", cola2::ros::getUnresolvedNodeName(shared_from_this()));
  diagnostic_->setEnabled(true);
  RCLCPP_INFO(this->get_logger(),"Initialized");
}

void Pilot::diagnosticsTimer()
{
  diagnostic_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::OK);
  diagnostic_->publish(shared_from_this());
}

void Pilot::navCallback(const cola2_msgs::msg::NavSts& data)
{
  // Check for valid navigation
  if (!cola2::ros::navigationIsValid(data))
  {
    return;
  }

  // Obtain navigation data
  current_state_.pose.position.ned_origin_latitude = data.origin.latitude;
  current_state_.pose.position.ned_origin_longitude = data.origin.longitude;
  current_state_.pose.position.north = data.position.north;
  current_state_.pose.position.east = data.position.east;
  current_state_.pose.position.depth = data.position.depth;
  current_state_.pose.altitude = data.altitude;
  current_state_.pose.orientation.roll = data.orientation.roll;
  current_state_.pose.orientation.pitch = data.orientation.pitch;
  current_state_.pose.orientation.yaw = data.orientation.yaw;
  current_state_.velocity.linear.x = data.body_velocity.x;
  current_state_.velocity.linear.y = data.body_velocity.y;
  current_state_.velocity.linear.z = data.body_velocity.z;
  last_nav_received_ = data.header.stamp.sec;
}

rclcpp_action::CancelResponse Pilot::handle_cancel(const std::shared_ptr<GoalHandlePilot> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void) goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
rclcpp_action::GoalResponse Pilot::handle_goal(const rclcpp_action::GoalUUID&,std::shared_ptr<const pilot_action::Goal> goal)
{
  // Copy most of the input data to the request data type
  
  request.initial_depth = goal->initial_depth;
  request.final_depth = goal->final_depth;
  request.final_yaw = goal->final_yaw;
  request.final_altitude = goal->final_altitude;
  if (goal->heave_mode == pilot_action::Goal::DEPTH)
    request.heave_mode = control::Request::DEPTH;
  else if (goal->heave_mode == pilot_action::Goal::ALTITUDE)
    request.heave_mode = control::Request::ALTITUDE;
  else if (goal->heave_mode == pilot_action::Goal::BOTH)
    request.heave_mode = control::Request::BOTH;
  else
  {
    RCLCPP_INFO(this->get_logger(), ("Unable to process action request. Unknown heave mode: " + std::to_string(goal->heave_mode)).c_str());
    auto result_msg = std::make_shared<pilot_action::Result>();
    result_msg->state = pilot_action::Result::FAILURE;
    return rclcpp_action::GoalResponse::REJECT;
  }
  request.surge_velocity = goal->surge_velocity;
  request.tolerance_xy = goal->tolerance_xy;
  request.timeout = goal->timeout;
  if (goal->controller_type == pilot_action::Goal::SECTION)
    request.controller_type = control::Request::SECTION;
  else if (goal->controller_type == pilot_action::Goal::ANCHOR)
    request.controller_type = control::Request::ANCHOR;
  else if (goal->controller_type == pilot_action::Goal::HOLONOMIC_KEEP_POSITION)
    request.controller_type = control::Request::HOLONOMIC_KEEP_POSITION;
  else
  {
    RCLCPP_INFO(this->get_logger(),("Unable to process action request. Unknown controller type: " + std::to_string(goal->controller_type)).c_str());
    auto result_msg = std::make_shared<pilot_action::Result>();
    result_msg->state = pilot_action::Result::FAILURE;
    return rclcpp_action::GoalResponse::REJECT;
  }


  request.requester = goal->goal.requester;
  request.priority = goal->goal.priority;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


void Pilot::handle_accepted(const std::shared_ptr<GoalHandlePilot> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&Pilot::execute, this, _1), goal_handle}.detach();
}

void Pilot::execute(const std::shared_ptr<GoalHandlePilot> goal_handle){
  // Main loop
  auto goal = goal_handle->get_goal();
  const double init_time = this->get_clock()->now().seconds();
  rclcpp::Rate r(10);  
  while (rclcpp::ok())  
  {
    // Get iteration time stamp
    const rclcpp::Time iteration_stamp = this->get_clock()->now();

    // Check for preempted. This happens upon user request (by preempting
    // or canceling the goal, or when a new goal is received
    if (goal_handle->is_canceling()) {
        auto result_msg = std::make_shared<pilot_action::Result>();
        result_msg->state = pilot_action::Result::FAILURE;
        goal_handle->canceled(result_msg);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
    }
      // Check timeout
    if (iteration_stamp.seconds() - init_time > goal->timeout)
    {
      RCLCPP_WARN(this->get_logger(),"Timeout");
      auto result_msg = std::make_shared<pilot_action::Result>();
      result_msg->state = pilot_action::Result::TIMEOUT;
      goal_handle->abort(result_msg); 
      return ;
    }
    // Check navigation
    if (iteration_stamp.seconds() - last_nav_received_ > 2.0)
    {
      RCLCPP_ERROR(this->get_logger(),"Pilot action failed due to missing navigation");
      auto result_msg = std::make_shared<pilot_action::Result>();
      result_msg->state = pilot_action::Result::FAILURE;
      goal_handle->abort(result_msg); 
      return;
    }

    // Now that we know that we have recent navigation, convert from latitude and longitude to north and east
    cola2::utils::NED ned(current_state_.pose.position.ned_origin_latitude,
                          current_state_.pose.position.ned_origin_longitude, 0.0);
    double dummy_depth;
    ned.geodetic2Ned(goal->initial_latitude, goal->initial_longitude, 0.0,request.initial_north, request.initial_east,
                     dummy_depth);
    ned.geodetic2Ned(goal->final_latitude, goal->final_longitude, 0.0, request.final_north, request.final_east,
                     dummy_depth);
    request.ned_origin_latitude = current_state_.pose.position.ned_origin_latitude;
    request.ned_origin_longitude = current_state_.pose.position.ned_origin_longitude;

    // Declare the output of the controllers
    control::State controller_output;
    control::Feedback feedback;
    control::PointsList points;

    // Run controller
    try
    {
      if (request.controller_type == control::Request::SECTION)
        section_controller_->compute(current_state_, request, controller_output, feedback, points);
      else if (request.controller_type == control::Request::ANCHOR)
        anchor_controller_->compute(current_state_, request, controller_output, feedback, points);
      else
        holonomic_keep_position_controller_->compute(current_state_, request, controller_output, feedback, points);
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Controller failure: %s", ex.what());
      auto result_msg = std::make_shared<pilot_action::Result>();
      result_msg->state = pilot_action::Result::FAILURE;
      goal_handle->abort(result_msg); 
      return;
    }

    // Publish
    publishControlCommands(controller_output, request.priority, iteration_stamp);
    publishFeedback(goal_handle,feedback);
    publishMarker(request.final_north, request.final_east, request.final_depth);
    publishMarkerSections(points);
    publishGoal(request.final_north, request.final_east, request.final_depth);

    // Check for success
    if (feedback.success)
    {
      RCLCPP_INFO(this->get_logger(),"Success");
      auto result_msg = std::make_shared<pilot_action::Result>();
      result_msg->state = pilot_action::Result::SUCCESS;
      goal_handle->succeed(result_msg);
      break;
    }

    // Diagnostic
    diagnostic_->reportValidData(iteration_stamp);

    // Sleep
    r.sleep();
  }
}

void Pilot::publishGoal(const double x, const double y, const double z) 
{
  
  geometry_msgs::msg::PointStamped goal;
  goal.header.frame_id = "world_ned";
  goal.header.stamp = this->get_clock()->now(); 
  goal.point.x = x;
  goal.point.y = y;
  goal.point.z = z;
  pub_goal_->publish(goal);
}

void Pilot::publishControlCommands(const control::State& controller_output, const std::uint64_t priority,
                                   const rclcpp::Time& now) 
{
  // Create and publish world waypoint request
  cola2_msgs::msg::WorldWaypointReq wwr;
  wwr.header.frame_id = "world_ned";
  wwr.header.stamp = now;
  wwr.goal.priority = priority;
  wwr.goal.requester = this->get_name() + std::string("_pose_req");
  wwr.disable_axis.x = controller_output.pose.disable_axis.x;
  wwr.disable_axis.y = controller_output.pose.disable_axis.y;
  wwr.disable_axis.z = controller_output.pose.disable_axis.z;
  wwr.disable_axis.roll = controller_output.pose.disable_axis.roll;
  wwr.disable_axis.pitch = controller_output.pose.disable_axis.pitch;
  wwr.disable_axis.yaw = controller_output.pose.disable_axis.yaw;
  wwr.position.north = controller_output.pose.position.north;
  wwr.position.east = controller_output.pose.position.east;
  wwr.position.depth = controller_output.pose.position.depth;
  wwr.orientation.roll = controller_output.pose.orientation.roll;
  wwr.orientation.pitch = controller_output.pose.orientation.pitch;
  wwr.orientation.yaw = controller_output.pose.orientation.yaw;
  wwr.altitude_mode = controller_output.pose.altitude_mode;
  wwr.altitude = controller_output.pose.altitude;
  pub_wwr_->publish(wwr);

  // Create and publish body velocity request
  cola2_msgs::msg::BodyVelocityReq bvr;
  bvr.header.frame_id = this->get_name() + std::string("/base_link");
  bvr.header.stamp = now;
  bvr.goal.priority = priority;
  bvr.goal.requester = this->get_name() + std::string("_velocity_req");
  bvr.disable_axis.x = controller_output.velocity.disable_axis.x;
  bvr.disable_axis.y = controller_output.velocity.disable_axis.y;
  bvr.disable_axis.z = controller_output.velocity.disable_axis.z;
  bvr.disable_axis.roll = controller_output.velocity.disable_axis.roll;
  bvr.disable_axis.pitch = controller_output.velocity.disable_axis.pitch;
  bvr.disable_axis.yaw = controller_output.velocity.disable_axis.yaw;
  bvr.twist.linear.x = controller_output.velocity.linear.x;
  bvr.twist.linear.y = controller_output.velocity.linear.y;
  bvr.twist.linear.z = controller_output.velocity.linear.z;
  bvr.twist.angular.x = controller_output.velocity.angular.x;
  bvr.twist.angular.y = controller_output.velocity.angular.y;
  bvr.twist.angular.z = controller_output.velocity.angular.z;
  pub_bvr_->publish(bvr);
}

void Pilot::publishFeedback(const std::shared_ptr<GoalHandlePilot> goal_handle,const control::Feedback& feedback) 
{
  auto msg = std::make_shared<pilot_action::Feedback>();
  msg->distance_to_end = feedback.distance_to_end;
  msg->cross_track_error = feedback.cross_track_error;
  goal_handle->publish_feedback(msg);
}

void Pilot::publishMarker(const double north, const double east, const double depth) 
{
  // Publish marker. Marker is published periodically so that RViz always
  // receives it, even if RViz is started after the ActionGoal arrives
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world_ned";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = this->get_name();
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = north;
  marker.pose.position.y = east;
  marker.pose.position.z = depth;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5;
  marker.lifetime = rclcpp::Duration::from_seconds(2.0);
  marker.frame_locked = false;
  pub_marker_->publish(marker);
}

void Pilot::publishMarkerSections(const control::PointsList points) 
{
  // Create visualization marker
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world_ned";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = this->get_name() ;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;

  // Add points to it
  for (const auto& i : points.points_list)
  {
    geometry_msgs::msg::Point p;
    p.x = i.x;
    p.y = i.y;
    p.z = i.z;
    marker.points.push_back(p);
  }

  marker.scale.x = 0.35;
  marker.color.r = 0.8;
  marker.color.g = 0.8;
  marker.color.b = 0.0;
  marker.color.a = 0.5;
  marker.lifetime = rclcpp::Duration::from_seconds(2.0);
  marker.frame_locked = false;
  pub_marker_->publish(marker);
}

void Pilot::getConfig()
{
  // Load config from param server
  // clang-format off
  // LOS-CTE controller
this->declare_parameter("section.tolerance_z", 1.0);
this->declare_parameter("section.delta", 5.0);
this->declare_parameter("section.distance_to_max_velocity", 5.0);
this->declare_parameter("section.max_surge_velocity", 0.5);
this->declare_parameter("section.min_surge_velocity", 0.2);

// ANCHOR controller
this->declare_parameter("anchor.kp", 0.1);
this->declare_parameter("anchor.radius", 1.0);
this->declare_parameter("anchor.min_surge_velocity", -0.1);
this->declare_parameter("anchor.max_surge_velocity", 0.3);

double tolerance_z, delta, distance_to_max_velocity, max_surge_velocity, min_surge_velocity;
double kp, radius, anchor_min_surge_velocity, anchor_max_surge_velocity;

this->get_parameter("section/tolerance_z", tolerance_z);
this->get_parameter("section/delta", delta);
this->get_parameter("section/distance_to_max_velocity", distance_to_max_velocity);
this->get_parameter("section/max_surge_velocity", max_surge_velocity);
this->get_parameter("section/min_surge_velocity", min_surge_velocity);

this->get_parameter("anchor/kp", kp);
this->get_parameter("anchor/radius", radius);
this->get_parameter("anchor/min_surge_velocity", anchor_min_surge_velocity);
this->get_parameter("anchor/max_surge_velocity", anchor_max_surge_velocity);

    // Assign parameters to configuration structure
config_.section_config.tolerance_z = tolerance_z;
config_.section_config.delta = delta;
config_.section_config.distance_to_max_velocity = distance_to_max_velocity;
config_.section_config.max_surge_velocity = max_surge_velocity;
config_.section_config.min_surge_velocity = min_surge_velocity;

config_.anchor_config.kp = kp;
config_.anchor_config.radius = radius;
config_.anchor_config.min_surge_velocity = anchor_min_surge_velocity;
config_.anchor_config.max_surge_velocity = anchor_max_surge_velocity;


  // clang-format on
}

bool Pilot::reloadParamsCallback(const std::shared_ptr<rmw_request_id_t> , const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  getConfig();
  section_controller_->setConfig(config_.section_config);
  holonomic_keep_position_controller_->setConfig(config_.holonomic_keep_position_config);
  anchor_controller_->setConfig(config_.anchor_config);

  // Publish params after param reload
  srv_publish_params_->async_send_request(req);

  res->success = true;
  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Pilot>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
