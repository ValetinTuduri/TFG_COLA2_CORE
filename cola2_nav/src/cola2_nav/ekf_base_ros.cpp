/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_nav/ekf_base_ros.h"

#include <cola2_lib_ros/navigation_helper.h>

#include <cstdint>
#include <string>

void showInfoOrWarning(const char* text, const bool use_info)
{
  if (use_info)
  {
    RCLCPP_INFO(rclcpp::get_logger("info_logger"),"%s", text);
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("warn_logger"),"%s", text);
  }
}

// *****************************************
// Constructor and destructor
// *****************************************
EKFBaseROS::EKFBaseROS(const unsigned int initial_state_vector_size, const bool online)
  : EKFBase(initial_state_vector_size)
  , ned_(0.0, 0.0, 0.0)  // dummy init 
  , online_(online)
{

}

EKFBaseROS::~EKFBaseROS() noexcept
{
}

void EKFBaseROS::init_base(const std::shared_ptr<rclcpp::Node>& node){

  node_ =  node;
  tf_handler_ = std::make_shared<cola2::ros::TransformHandler>(node_);
  tf_broadcast_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  diagnostic_ = std::make_shared<cola2::ros::DiagnosticHelper>(node_, "navigator", cola2::ros::getUnresolvedNodeName(node_));

  node_->declare_parameter("initialize_filter_from_gps", true);
  node_->declare_parameter("gps_samples_to_init", 10);
  node_->declare_parameter("use_gps_data", true);
  node_->declare_parameter("use_usbl_data", true);
  node_->declare_parameter("use_depth_data", true);
  node_->declare_parameter("use_dvl_data", true);
  node_->declare_parameter("enable_debug", false);
  node_->declare_parameter("ned_latitude", 41.7777);
  node_->declare_parameter("ned_longitude", 3.0333);
  node_->declare_parameter("initialize_depth_sensor_offset", true);
  node_->declare_parameter("surface_to_depth_sensor_distance", 0.05);
  node_->declare_parameter("depth_sensor_offset", 0.0);
  node_->declare_parameter("dvl_max_v", 1.2);
  node_->declare_parameter("water_density", 1030.0);
  node_->declare_parameter("dvl_fallback_delay", 0.0);
  node_->declare_parameter("initial_state_covariance", std::vector<double>{2.0, 2.0, 0.5, 0.02, 0.02, 0.02});
  node_->declare_parameter("prediction_model_covariance", std::vector<double>{0.02, 0.02, 0.02});
  node_->declare_parameter("min_diagnostics_frequency", 25.0);

  // Wait for time
  while (node_->now().seconds() == 0.0)
  {
    rclcpp::spin_some(node_);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Waiting for valid time source");
  }

  // Get correct namespace and vehicle frame
  ns_ = cola2::ros::getNamespace(node_);
  frame_vehicle_ = cola2::ros::getNamespaceNoInitialDash(node_) + std::string("/base_link"); //CAREFUL GIRONA500 HARD CODED!
  RCLCPP_INFO(node_->get_logger(),"vehicle frame: %s", frame_vehicle_.c_str());

  // Load configurations and reset filter
  std_srvs::srv::Trigger_Response res;
  res = getConfig(true);  // get config and show loaded configuration
  if (!res.success)
  {
    while (rclcpp::ok())
    {
      RCLCPP_ERROR(node_->get_logger(),"Problem loading initial config: %s", res.message.c_str());
      rclcpp::sleep_for(std::chrono::seconds(2));
    }
  }
  resetFilter();  // start filter from initial state and configure ned

  // Debug in output
  if (config_.enable_debug_)
  {
    // Current UTC time
    const time_t t = time(nullptr);     // get time now
    const struct tm* now = gmtime(&t);  // UTC time
    char buffer[200];
    strftime(buffer, sizeof(buffer), "/debug_navigator_%Y-%m-%d_%H-%M-%S.txt", now);
    // Create debug file
    ofh_ = std::ofstream(std::string(std::getenv("HOME")) + std::string(buffer));
    ofh_.setf(std::ios::fixed, std::ios::floatfield);
    ofh_.precision(6);
  }

  // Normal online navigator (publishes and calls services)
  if (online_)
  {
    //Publishers
        pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
        pub_nav_ = node_->create_publisher<cola2_msgs::msg::NavSts>("navigation", 1);
        pub_gps_ned_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("gps_ned", 1);
        pub_usbl_ned_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("usbl_ned", 1);

        // Initialize services
       
        auto reload_ned_callback = std::bind(&EKFBaseROS::srvReloadNED, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        auto reload_params_callback =  std::bind(&EKFBaseROS::srvResetNavigation, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        auto reset_navigation_callback = std::bind(&EKFBaseROS::srvResetNavigation, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        auto set_depth_sensor_offset_callback = std::bind(&EKFBaseROS::srvSetDepthSensorOffset, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        srv_reload_ned_ = node_->create_service<std_srvs::srv::Trigger>("reload_ned",reload_ned_callback);
        srv_reload_params_ = node_->create_service<std_srvs::srv::Trigger>("reload_params",reload_params_callback);
        srv_reset_navigation_ = node_->create_service<std_srvs::srv::Trigger>("reset_navigation", reset_navigation_callback);
        srv_set_depth_sensor_offset_ = node_->create_service<std_srvs::srv::Trigger>("set_depth_sensor_offset",set_depth_sensor_offset_callback);
    // clang-format on

    // Service client to tell param_logger to publish parameters
     const std::string publish_params_srv_name = ns_ + "/param_logger/publish_params";
     srv_publish_params_ = node_->create_client<std_srvs::srv::Trigger>(publish_params_srv_name);
    while (rclcpp::ok())
    {
      if (!srv_publish_params_->wait_for_service(std::chrono::seconds(5)))
      {
        break;
      }
      RCLCPP_INFO_STREAM(node_->get_logger(),"Waiting for client to service " << publish_params_srv_name);
    }

    // Init timer to check diagnostics
    timer_ = node_->create_wall_timer(std::chrono::seconds(1), std::bind(&EKFBaseROS::checkDiagnostics, this));
  }
  else
  {
    // Offline navigator
    RCLCPP_WARN(node_->get_logger(),"Using navigator offline...");
  }

  // Diagnostics
  diagnostic_->setEnabled(true);



}

void EKFBaseROS::loadTranformsFromFile(const std::string& fname)
{
  // Load transforms from file
  std::ifstream infile(fname);
  if (!infile.is_open())
  {
    RCLCPP_FATAL(node_->get_logger(),"missing transforms file: %s", fname.c_str());
    rclcpp::shutdown();
  }
  // Process lines
  std::string parent, child;
  double tx, ty, tz, qx, qy, qz, qw;
  while (infile >> parent >> child >> tx >> ty >> tz >> qx >> qy >> qz >> qw)
  {
   tf_handler_->setTransformManually(child, tx, ty, tz, qx, qy, qz, qw);
  }
}

void EKFBaseROS::setPositionXYasGPSinit(const Eigen::Vector2d& xy)
{
  setPositionXY(xy);
  init_gps_ = true;
}

void EKFBaseROS::resetFilter()
{
  // Reset flags
  // general
  init_depth_offset_ = false;
  init_ekf_ = false;
  init_ned_ = false;
  diagnostic_->addKeyValue("filter_init", false);
  diagnostic_->addKeyValue("ned_init", false);
  // sensors
  init_gps_ = false;
  init_depth_ = false;
  init_dvl_ = false;
  init_imu_ = false;
  diagnostic_->addKeyValue("gps_init", false);
  diagnostic_->addKeyValue("depth_init", false);
  diagnostic_->addKeyValue("dvl_init", false);
  diagnostic_->addKeyValue("imu_init", false);

  // start time
  start_time_ = node_->now().seconds();

  // forget saved positions for delayed usbl updates
  last_usbl_positions_.clear();

  // Reset state vector
  x_ = Eigen::VectorXd::Zero(initial_state_vector_size_);
  // Reset covariance
  assert(initial_state_vector_size_ <= config_.initial_state_covariance_.size());
  Eigen::VectorXd p_var = Eigen::VectorXd::Zero(initial_state_vector_size_);
  for (size_t i = 0; i < initial_state_vector_size_; ++i)
  {
    p_var(static_cast<unsigned int>(i)) = config_.initial_state_covariance_[i];
  }
  P_ = Eigen::MatrixXd::Identity(p_var.rows(), p_var.rows());
  P_.diagonal() = p_var;
  // Reset prediction noise
  Eigen::VectorXd q_var = Eigen::VectorXd::Zero(static_cast<unsigned int>(config_.prediction_model_covariance_.size()));
  for (size_t i = 0; i < config_.prediction_model_covariance_.size(); ++i)
  {
    q_var(static_cast<unsigned int>(i)) = config_.prediction_model_covariance_[i];
  }
  Q_ = Eigen::MatrixXd::Identity(q_var.rows(), q_var.rows());
  Q_.diagonal() = q_var;

  // Init ned
  RCLCPP_INFO(node_->get_logger(),"Init NED at: %.8f, %.8f", config_.ned_latitude_, config_.ned_longitude_);
  ned_ = cola2::utils::NED(config_.ned_latitude_, config_.ned_longitude_, 0.0);
  init_ned_ = true;
  diagnostic_->addKeyValue("ned_init", true);
}

Eigen::Vector3d EKFBaseROS::getPositionIncrementFrom(const double time) const
{
  // check oldest kept time
  if (last_usbl_positions_.at(0).time > time)
  {
    // query too old
    return Eigen::Vector3d(-1.0, 0.0, 0.0);
  }
  // find similar time
  for (unsigned int i = 0; i < last_usbl_positions_.size(); ++i)
  {
    if (last_usbl_positions_.at(i).time > time)
    {
      // increment = last position - position at specified time
      const double time_increment =
          last_usbl_positions_.at(last_usbl_positions_.size() - 1).time - last_usbl_positions_.at(i).time;
      const Eigen::Vector3d position_increment =
          last_usbl_positions_.at(last_usbl_positions_.size() - 1).position - last_usbl_positions_.at(i).position;
      return Eigen::Vector3d(time_increment, position_increment(0), position_increment(1));
    }
  }
  // If no position found, return a negative time
  return Eigen::Vector3d(-1.0, 0.0, 0.0);
}

std_srvs::srv::Trigger_Response EKFBaseROS::getConfig(const bool show)
{
  // Load params from ROS param server
  Config temp_config;
  // Flags
  bool ok = true;
  ok &= cola2::ros::getParam("initialize_filter_from_gps", temp_config.initialize_filter_from_gps_, node_);
  ok &= cola2::ros::getParam("gps_samples_to_init", temp_config.gps_samples_to_init_, node_);
  ok &= cola2::ros::getParam("use_gps_data", temp_config.use_gps_data_, node_);
  ok &= cola2::ros::getParam("use_usbl_data", temp_config.use_usbl_data_, node_);
  ok &= cola2::ros::getParam("use_depth_data", temp_config.use_depth_data_, node_);
  ok &= cola2::ros::getParam("use_dvl_data", temp_config.use_dvl_data_, node_);
  ok &= cola2::ros::getParam("enable_debug", temp_config.enable_debug_, node_);
  // NED
  ok &= cola2::ros::getParam("ned_latitude", temp_config.ned_latitude_, 0.0, node_);
  ok &= cola2::ros::getParam("ned_longitude", temp_config.ned_longitude_, 0.0, node_);
  // Depth offset
  ok &= cola2::ros::getParam("initialize_depth_sensor_offset", temp_config.initialize_depth_sensor_offset_, false, node_);
  ok &= cola2::ros::getParam("surface_to_depth_sensor_distance", temp_config.surface2depth_sensor_distance_, 0.0, node_);
  ok &= cola2::ros::getParam("depth_sensor_offset", temp_config.depth_sensor_offset_, 0.0, node_);
  // Sensors
  ok &= cola2::ros::getParam("dvl_max_v", temp_config.dvl_max_v_, 1.5, node_);
  ok &= cola2::ros::getParam("water_density", temp_config.water_density_, 1030.0, node_);
  // DVL fallback
  ok &= cola2::ros::getParam("dvl_fallback_delay", temp_config.dvl_fallback_delay_, 0.0, node_);
  // Covariances
  ok &= cola2::ros::getParam("initial_state_covariance", temp_config.initial_state_covariance_, node_);
  ok &= cola2::ros::getParam("prediction_model_covariance", temp_config.prediction_model_covariance_, node_);
  // Diagnostics
  ok &= cola2::ros::getParam("min_diagnostics_frequency", temp_config.min_diagnostics_frequency_, 25.0, node_);

  // Show
  if (show)
  {
    RCLCPP_INFO(node_->get_logger(),"Loaded config from param server");
    RCLCPP_INFO(node_->get_logger(),"===============================");
    RCLCPP_INFO(node_->get_logger(),"init filter from gps: %d", temp_config.initialize_filter_from_gps_);
    RCLCPP_INFO(node_->get_logger()," gps samples to init: %d", temp_config.gps_samples_to_init_);
    RCLCPP_INFO(node_->get_logger(),"        use gps data: %d", temp_config.use_gps_data_);
    RCLCPP_INFO(node_->get_logger(),"       use usbl data: %d", temp_config.use_usbl_data_);
    RCLCPP_INFO(node_->get_logger(),"      use depth data: %d", temp_config.use_depth_data_);
    RCLCPP_INFO(node_->get_logger(),"        use dvl data: %d", temp_config.use_dvl_data_);
    RCLCPP_INFO(node_->get_logger(),"        enable debug: %d\n", temp_config.enable_debug_);
    RCLCPP_INFO(node_->get_logger(),"       ned latitude: %3.6f", temp_config.ned_latitude_);
    RCLCPP_INFO(node_->get_logger(),"      ned longitude: %3.6f\n", temp_config.ned_longitude_);
    RCLCPP_INFO(node_->get_logger(),"init depth sensor offset: %d", temp_config.initialize_depth_sensor_offset_);
    RCLCPP_INFO(node_->get_logger(),"    surface2depth sensor: %.3f", temp_config.surface2depth_sensor_distance_);
    RCLCPP_INFO(node_->get_logger(),"     depth sensor offset: %.3f\n", temp_config.depth_sensor_offset_);
    RCLCPP_INFO(node_->get_logger(),"  dvl max velocity: %.3f", temp_config.dvl_max_v_);
    RCLCPP_INFO(node_->get_logger(),"dvl fallback delay: %.3f", temp_config.dvl_fallback_delay_);
    RCLCPP_INFO(node_->get_logger(),"     water density: %.3f\n", temp_config.water_density_);
    RCLCPP_INFO(node_->get_logger(),"min diagnostics frequency: %.3f\n", temp_config.min_diagnostics_frequency_);
    // vectors
    std::stringstream ss;
    ss << "   initial state covariance: ";
    for (const double v : temp_config.initial_state_covariance_)
    {
      ss << v << ' ';
    }
    RCLCPP_INFO_STREAM(node_->get_logger(),ss.str());
    ss.str(std::string());  // empty it
    ss << "prediction model covariance: ";
    for (const double v : temp_config.prediction_model_covariance_)
    {
      ss << v << ' ';
    }
    RCLCPP_INFO_STREAM(node_->get_logger(),ss.str());
    ss.str(std::string());
  }

  // Default answer
  std_srvs::srv::Trigger_Response res;
  res.success = true;
  // Check params loaded correctly
  if (!ok)
  {
    res.success = false;
    res.message = "Some params where not found in param server. Params not updated.";
    return res;
  }
  // Check NED and GPS configuration
  if (temp_config.initialize_filter_from_gps_ && !temp_config.use_gps_data_)
  {
    res.success = false;
    res.message = "Impossible to (initialize_filter_from_gps == true) if (use_gps_data_ == false). Params not updated.";
    return res;
  }
  // TODO: no init and yes gps?
  // copy valid config to navigator
  config_ = temp_config;
  // Default return
  return res;
}

void EKFBaseROS::checkDiagnostics()
{
   
  // See if we are in init time
  const bool init_time = (!init_ekf_) && ((node_->now().seconds() - start_time_) < INIT_TIME_NO_WARNINGS);

  // *****************************************
  // Check sensors
  // *****************************************
  const double now = node_->now().seconds();
  bool is_nav_data_ok = true;
  // Check IMU data
  diagnostic_->addKeyValue("last_imu_data", now - last_imu_time_);
  if (now - last_imu_time_ > 1.0)
  {
    is_nav_data_ok = false;
    showInfoOrWarning("IMU too old", init_time);
  }
  // Check DVL data
  if (config_.use_dvl_data_)
  {
    diagnostic_->addKeyValue("last_dvl_data", now - last_dvl_time_);
    if (now - last_dvl_time_ > 2.0)
    {
      is_nav_data_ok = false;
      showInfoOrWarning("DVL too old", init_time);
    }
  }
  // Check altitude data (if it was ever received)
  diagnostic_->addKeyValue("last_altitude_data", now - last_altitude_time_);
  if ((last_altitude_time_ != 0.0) && (now - last_altitude_time_ > 5.0))
  {
    altitude_ = 0.0;
    is_nav_data_ok = false;
    showInfoOrWarning("Altitude too old", init_time);
  }
  // Check depth data
  if (config_.use_depth_data_)
  {
    diagnostic_->addKeyValue("last_depth_data", now - last_depth_time_);
    if (now - last_depth_time_ > 2.0)
    {
      is_nav_data_ok = false;
      showInfoOrWarning("Depth too old", init_time);
    }
  }
  // Check gps data
  diagnostic_->addKeyValue("last_gps_data", 0.0);
  if (config_.use_gps_data_)
  {
    diagnostic_->addKeyValue("last_gps_data", now - last_gps_time_);
    if (now - last_gps_time_ > 3.0)
    {
      if (getPosition()(2) < 1.0)
      {
        is_nav_data_ok = false;
        showInfoOrWarning("GPS too old", init_time);
      }
    }
  }
  // *****************************************
  // Check other
  // *****************************************
  // Check current freq
  /*const double freq = diagnostic_->getCurrentFreq();
  diagnostic_->addKeyValue("freq", freq);
  // Only check when EKF is init and a bit later
  if (init_ekf_ && (e.current_real.seconds() - last_ekf_init_time_ > 10.0))
  {
    if (freq < config_.min_diagnostics_frequency_)
    {
      is_nav_data_ok = false;
      ROS_WARN_STREAM("Diagnostics frequency too low (" << freq << " lower than " << config_.min_diagnostics_frequency_
                                                        << ")");
    }
  }*/
  // If filter or NED not initialized set to Warning
  if (!init_ekf_)
  {
    is_nav_data_ok = false;
  }
  else
  {
    diagnostic_->addKeyValue("filter_init", true);
  }
  // NED init
  if (!init_ned_)
  {
    is_nav_data_ok = false;
  }
  else
  {
    diagnostic_->addKeyValue("ned_init", true);
  }
  // If all nav data is ok set navigator to Ok
  if (is_nav_data_ok && !ned_error_)
  {
    diagnostic_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::OK);
  }
  else
  {
    diagnostic_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::WARN);
  }

  // *****************************************
  // Sensor inits to diagnostics
  // *****************************************
  diagnostic_->addKeyValue("gps_init", !(config_.use_gps_data_ && !init_gps_));
  diagnostic_->addKeyValue("depth_init", init_depth_);
  diagnostic_->addKeyValue("dvl_init", init_dvl_);
  diagnostic_->addKeyValue("imu_init", init_imu_);

  // *****************************************
  // Init console output
  // *****************************************
  if (!init_ekf_)
  {
    showInfoOrWarning("EKF not initialized", init_time);
  }
  if (config_.use_dvl_data_ && !init_dvl_)
  {
    showInfoOrWarning("DVL not initialized", init_time);
  }
  if (config_.use_depth_data_ && !init_depth_)
  {
    showInfoOrWarning("Depth not initialized", init_time);
  }
  if (config_.use_gps_data_ && !init_gps_)
  {
    showInfoOrWarning("GPS not initialized", init_time);
  }
  if (!init_imu_)
  {
    showInfoOrWarning("IMU not initialized", init_time);
  }
  if (!init_ned_)
  {
    showInfoOrWarning("NED not initialized", init_time);
  }

  // Nav data ok console output
  if (!is_nav_data_ok)
  {
    if (init_ekf_)
    {
      RCLCPP_FATAL(node_->get_logger(),"Missing NAV data");
    }
    else
    {
      showInfoOrWarning("Missing NAV data", init_time);
    }
  }

  // Publish diagnostics
  diagnostic_->publish(node_);

}

void EKFBaseROS::updatePositionGPSMsg(const sensor_msgs::msg::NavSatFix& msg)
{
 
  
  updatePositionGPSMsgImpl(msg);
}

bool EKFBaseROS::updatePositionGPSMsgImpl(const sensor_msgs::msg::NavSatFix& msg)
{
  // initial checks
  bool process = false;
  if ((msg.status.status >= msg.status.STATUS_FIX) && (msg.position_covariance[0] < 10.0))
  {
    process = true;
  }
  else if (!init_ekf_ && config_.initialize_filter_from_gps_)
  {
    // invalid measurement before init
    ++gps_samples_wrong_before_init_;
    if (gps_samples_wrong_before_init_ > 60)
    {
      diagnostic_->addKeyValue("filter_init", "false");
      diagnostic_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      diagnostic_->reportData();
      diagnostic_->publish(node_);
      RCLCPP_FATAL(node_->get_logger(),"Impossible to initialize filter with GPS");
    }
  }
  else
  {
    // invalid measurement
    return false;
  }

  // Always publish measurements
  const Eigen::Vector3d latlonh(msg.latitude, msg.longitude, 0.0);
  Eigen::Vector3d ned = ned_.geodetic2Ned(latlonh);
  Eigen::Isometry3d trans;
  if (!tf_handler_->getTransform(msg.header.frame_id, trans))
  {
    RCLCPP_WARN(node_->get_logger(),"GPS update error. No tf for frame %s", msg.header.frame_id.c_str());
    return false;  // not possible to transform
  }
  ned = transforms::position(ned, getOrientation(), trans.translation());
  ned(2) = 0.0;  // water surface

  // Check usage or continue with processing
  if (!config_.use_gps_data_ || !process)
  {
    publishGPSNED(msg.header.stamp, ned, false);
    return false;
  }

  // ===================
  // Process update
  // ===================
  // Diagnostics
  diagnostic_->reportData();

  // Increase number of received messages
  ++gps_samples_;
  last_gps_sample_time_ = rclcpp::Time(msg.header.stamp).seconds();

  // Not enough samples yet
  if (gps_samples_ < static_cast<size_t>(config_.gps_samples_to_init_))
  {
    // Not enough samples yet
    RCLCPP_INFO(node_->get_logger(),"gps samples to init %zu/%d", gps_samples_, config_.gps_samples_to_init_);
    publishGPSNED(msg.header.stamp, ned, false);
    return false;
  }

  // Init depth offset
  if (!init_depth_offset_ && config_.initialize_depth_sensor_offset_)
  {
    std::shared_ptr<rmw_request_id_t> request_header_ptr = std::make_shared<rmw_request_id_t>(); 
        std::shared_ptr<std_srvs::srv::Trigger::Request> req = std::make_shared<std_srvs::srv::Trigger::Request>();
    std::shared_ptr<std_srvs::srv::Trigger::Response> res = std::make_shared<std_srvs::srv::Trigger::Response>();
    srvSetDepthSensorOffset(request_header_ptr, req, res);
    init_depth_offset_ = true;
  }

  // Covariance
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  cov(0, 0) = msg.position_covariance[0];
  cov(1, 1) = msg.position_covariance[4];
  cov = transforms::positionCovariance(cov, getOrientationUncertainty(), getOrientation(), trans.translation());

  // Predict and update
  const double tim = rclcpp::Time(msg.header.stamp).seconds();
  if (makePrediction(tim) || !init_ekf_)
  {
    // Debug
    if (config_.enable_debug_)
    {
      ofh_ << "#gps " << tim << ' ' << ned(0) << ' ' << ned(1) << ' ' << cov(0, 0) << ' ' << cov(0, 1) << ' '
           << cov(1, 0) << ' ' << cov(1, 1) << '\n';
    }
    // Update and publish
    const bool success = updatePositionXY(rclcpp::Time(msg.header.stamp).seconds(), ned.head(2), cov.topLeftCorner(2, 2));
    if (success)
    {
      last_gps_time_ = tim;
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "GPS update rejected.");
    }
    publishGPSNED(msg.header.stamp, ned, success);
    publishNavigation(msg.header.stamp);
    return success;
  }
  // Did not predict
  publishGPSNED(msg.header.stamp, ned, false);
  return false;
}

void EKFBaseROS::updatePositionUSBLMsg(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{

  
  updatePositionUSBLMsgImpl(msg);
}

bool EKFBaseROS::updatePositionUSBLMsgImpl(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
  // Valid measurement
  if (!init_ned_)
  {
    return false;
  }

  // USBL in NED frame
  const Eigen::Vector3d latlonh(msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0);
  Eigen::Vector3d ned = ned_.geodetic2Ned(latlonh);

  // IQUAview sends invalid
  if (!cola2::ros::usblIsValid(msg))
  {
    publishUSBLNED(node_->now(), { ned.x(), ned.y(), getPosition().z() }, USBLStatus::IQUAviewInvalid);
    return false;
  }

  // Get delayed position increment [dt dx dy]
  const Eigen::Vector3d position_increment = getPositionIncrementFrom(rclcpp::Time(msg.header.stamp).seconds());
  if (!init_ekf_ || position_increment(0) < 0.0)
  {
    // Cannot find it in buffer, publish raw
    publishUSBLNED(node_->now(), { ned.x(), ned.y(), getPosition().z() }, USBLStatus::OutOfBuffer);
    return false;
  }

  // Current time measurement
  const rclcpp::Time current_time(rclcpp::Time(msg.header.stamp).seconds() + position_increment(0));

  // Transform to vehicle frame
  Eigen::Isometry3d trans;
  if (!tf_handler_->getTransform(msg.header.frame_id, trans))
  {
    RCLCPP_WARN(node_->get_logger(),"USBL update error. No tf for frame %s", msg.header.frame_id.c_str());
    return false;  // not possible to transform
  }
  ned.head(2) += position_increment.tail(2);  // increment the same we increased
  ned = transforms::position(ned, getOrientation(), trans.translation());

  // Check usage
  if (!config_.use_usbl_data_)
  {
    publishUSBLNED(current_time, { ned.x(), ned.y(), getPosition().z() }, USBLStatus::NotUpdated);
    return false;
  }

  // Diagnostics
  diagnostic_->reportData();

  // Covariance
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (unsigned int i = 0; i < 3; ++i)
  {
    for (unsigned int j = 0; j < 3; ++j)
    {
      cov(i, j) = msg.pose.covariance[6 * i + j];  // from 6x6 matrix
    }
  }
  cov(0, 0) = 10.0;  // TODO: correct covariance from iquaview
  cov(1, 1) = 10.0;
  cov(2, 2) = 10.0;
  cov = transforms::positionCovariance(cov, getOrientationUncertainty(), getOrientation(), trans.translation());

  // Predict and update
  const double tim = current_time.seconds();
  if (makePrediction(tim))
  {
    // Debug
    if (config_.enable_debug_)
    {
      ofh_ << "#usbl " << tim << ' ' << ned(0) << ' ' << ned(1) << ' ' << cov(0, 0) << ' ' << cov(0, 1) << ' '
           << cov(1, 0) << ' ' << cov(1, 1) << '\n';
    }
    // Update and publish
    const bool success = updatePositionXY(tim, ned.head(2), cov.topLeftCorner(2, 2));
    if (success)
    {
      last_usbl_positions_.clear();  // clear history
      last_usbl_time_ = tim;
      publishUSBLNED(current_time, { ned.x(), ned.y(), getPosition().z() }, USBLStatus::Updated);
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(),"USBL position update rejected.");
      publishUSBLNED(current_time, { ned.x(), ned.y(), getPosition().z() }, USBLStatus::NotUpdated);
    }
    publishNavigation(current_time);
    return success;
  }
  // Did not predict
  publishUSBLNED(current_time, { ned.x(), ned.y(), getPosition().z() }, USBLStatus::NotUpdated);
  return false;
}

void EKFBaseROS::updatePositionDepthMsg(const sensor_msgs::msg::FluidPressure& msg)
{
 
  
  // Measurement units
  const double meters = msg.fluid_pressure / (config_.water_density_ * 9.80665);  // pascals to meters
  // Save pressure message for setDepthSensorOffset
  pressure_meters_ = meters;
  // Correct meters using depth_sensor_offset_
  const double corrected_meters = meters + config_.depth_sensor_offset_;
  // Valid measurement
  if (corrected_meters > -1.0)
  {
    // Diagnostics
    diagnostic_->reportData();
    // Construct measurement
    Eigen::Vector3d xyz(0.0, 0.0, corrected_meters);
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    cov(2, 2) = msg.variance / std::pow(config_.water_density_ * 9.80665, 2);  // variance pascals to meters
    // Transform to vehicle frame
    Eigen::Isometry3d trans;
    if (!tf_handler_->getTransform(msg.header.frame_id, trans))
    {
      RCLCPP_WARN(node_->get_logger(),"Depth update error. No tf for frame %s", msg.header.frame_id.c_str());
      return;  // not possible to transform
    }
    xyz = transforms::position(xyz, getOrientation(), trans.translation());
    cov = transforms::positionCovariance(cov, getOrientationUncertainty(), getOrientation(), trans.translation());
    // Predict and update
    const double tim = rclcpp::Time(msg.header.stamp).seconds();
    if (makePrediction(tim) || !init_ekf_)
    {
      // Debug
      if (config_.enable_debug_)
      {
        ofh_ << "#depth " << tim << ' ' << xyz.tail(1) << ' ' << cov(2, 2) << '\n';
      }
      // Update and publish
      if (updatePositionZ(rclcpp::Time(msg.header.stamp).seconds(), xyz.tail(1), cov.bottomRightCorner(1, 1)))
      {
        last_depth_time_ = tim;
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(),"Depth update rejected.");
      }
      publishNavigation(msg.header.stamp);
    }
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(),"Corrected pressure in meters is smaller than -1.0. Call service /set_depth_sensor_offset");
  }
}

void EKFBaseROS::updateVelocityDVLMsgImpl(const stonefish_ros2::msg::DVL& msg, const bool from_dvl)
{
  
  
  // Valid measurement
  if ((msg.velocity_covariance[0] > 0.0) && (std::abs(msg.velocity.x) < config_.dvl_max_v_) &&
      (std::abs(msg.velocity.y) < config_.dvl_max_v_) && (std::abs(msg.velocity.z) < config_.dvl_max_v_))
  {
    // Diagnostics
    if (from_dvl)
    {
      diagnostic_->reportData();  // not increasing if not real sensor
    }
    // Construct measurement
    Eigen::Vector3d vel(msg.velocity.x, msg.velocity.y, msg.velocity.z);
    Eigen::Matrix3d cov;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        cov(i, j) = msg.velocity_covariance[static_cast<size_t>(i * 3 + j)];
      }
    }
    // Transform to vehicle frame
    Eigen::Isometry3d trans;
    if (!tf_handler_->getTransform(msg.header.frame_id, trans))
    {
      RCLCPP_WARN(node_->get_logger(),"DVL update error. No tf for frame %s", msg.header.frame_id.c_str());
      return;  // not possible to transform
    }
    const Eigen::Quaterniond quat(trans.rotation());
    vel = transforms::linearVelocity(vel, getAngularVelocity(), quat, trans.translation());
    cov = transforms::linearVelocityCovariance(cov, getAngularVelocityUncertainty(), quat, trans.translation());
    // Predict and update
    const double tim = rclcpp::Time(msg.header.stamp).seconds();
   
    if (makePrediction(tim) || !init_ekf_)
    {
      // Debug
      if (config_.enable_debug_)
      {
        if (from_dvl)
        {
          ofh_ << "#dvl " << tim << ' ' << vel(0) << ' ' << vel(1) << ' ' << vel(2) << ' ' << cov(0, 0) << ' '
               << cov(0, 1) << ' ' << cov(0, 2) << ' ' << cov(1, 0) << ' ' << cov(1, 1) << ' ' << cov(1, 2) << ' '
               << cov(2, 0) << ' ' << cov(2, 1) << ' ' << cov(2, 2) << '\n';
        }
        else
        {
          ofh_ << "#dvl_fallback " << tim << ' ' << vel(0) << ' ' << vel(1) << ' ' << vel(2) << ' ' << cov(0, 0) << ' '
               << cov(0, 1) << ' ' << cov(0, 2) << ' ' << cov(1, 0) << ' ' << cov(1, 1) << ' ' << cov(1, 2) << ' '
               << cov(2, 0) << ' ' << cov(2, 1) << ' ' << cov(2, 2) << '\n';
        }
      }
      // Update and publish
      
      bool updated = updateVelocity(rclcpp::Time(msg.header.stamp).seconds(), vel, cov,from_dvl);
      if (from_dvl && updated)
      {
        last_dvl_time_ = tim;
      }
      if (!updated)
      {
        RCLCPP_WARN(node_->get_logger(),"%s update rejected.", (from_dvl ? "DVL" : "Velocity"));
      }
      publishNavigation(msg.header.stamp);
    }
  }
}

void EKFBaseROS::updateVelocityDVLMsg(const stonefish_ros2::msg::DVL& msg)
{
  // Make update as DVL sensor
  const bool from_dvl = true;
  updateVelocityDVLMsgImpl(msg, from_dvl);
}

void EKFBaseROS::updateVelocityDVLFallbackMsg(const stonefish_ros2::msg::DVL& msg)
{
  
  // Check that no DVL messages have been received for the specified delay
  if ((rclcpp::Time(msg.header.stamp).seconds() - last_dvl_time_) > config_.dvl_fallback_delay_)
  {
    // Make update not being a DVL sensor
    const bool from_dvl = false;
    updateVelocityDVLMsgImpl(msg, from_dvl);
  }
}

void EKFBaseROS::updateIMUMsg(const sensor_msgs::msg::Imu& msg)
{
 
  
  // Diagnostics
  diagnostic_->reportData();
  // Construct measurement
  Eigen::Quaterniond ori(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  Eigen::Vector3d ang_vel(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
  Eigen::Matrix3d rpy_cov;
  Eigen::Matrix3d ang_vel_cov;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      rpy_cov(i, j) = msg.orientation_covariance[static_cast<size_t>(i * 3 + j)];
      ang_vel_cov(i, j) = msg.angular_velocity_covariance[static_cast<size_t>(i * 3 + j)];
    }
  }
  // Transform to vehicle frame
  Eigen::Isometry3d trans;
  if (!tf_handler_->getTransform(msg.header.frame_id, trans))
  {
    RCLCPP_WARN(node_->get_logger(),"IMU update error. No tf for frame %s", msg.header.frame_id.c_str());
    return;  // not possible to transform
  }
  Eigen::Quaterniond quat(trans.rotation());
  ori = transforms::orientation(ori, quat);  // transform orientation
  const Eigen::Vector3d rpy = cola2::utils::quaternion2euler(ori);
  ang_vel = transforms::angularVelocity(ang_vel, quat);  // transform angular velocity
  // Predict and update
  const double tim = rclcpp::Time(msg.header.stamp).seconds();
  if (makePrediction(tim) || !init_ekf_)
  {
    // Debug
    if (config_.enable_debug_)
    {
      ofh_ << "#imu " << tim << ' ' << rpy(0) << ' ' << rpy(1) << ' ' << rpy(2) << ' ' << rpy_cov(0, 0) << ' '
           << rpy_cov(0, 1) << ' ' << rpy_cov(0, 2) << ' ' << rpy_cov(1, 0) << ' ' << rpy_cov(1, 1) << ' '
           << rpy_cov(1, 2) << ' ' << rpy_cov(2, 0) << ' ' << rpy_cov(2, 1) << ' ' << rpy_cov(2, 2) << '\n';
      ofh_ << "#rate " << tim << ' ' << ang_vel(0) << ' ' << ang_vel(1) << ' ' << ang_vel(2) << ' ' << ang_vel_cov(0, 0)
           << ' ' << ang_vel_cov(0, 1) << ' ' << ang_vel_cov(0, 2) << ' ' << ang_vel_cov(1, 0) << ' '
           << ang_vel_cov(1, 1) << ' ' << ang_vel_cov(1, 2) << ' ' << ang_vel_cov(2, 0) << ' ' << ang_vel_cov(2, 1)
           << ' ' << ang_vel_cov(2, 2) << '\n';
    }
    // Update and publish
    if (updateOrientation(rclcpp::Time(msg.header.stamp).seconds(), rpy, rpy_cov))
    {
      last_imu_time_ = tim;
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(),"IMU update rejected.");
    }
    if (!updateOrientationRate(rclcpp::Time(msg.header.stamp).seconds(), ang_vel, ang_vel_cov))
    {
      RCLCPP_WARN(node_->get_logger(),"IMU orientation rate update rejected.");
    }
    publishNavigation(msg.header.stamp);
  }
}

void EKFBaseROS::updateSoundVelocityMsg(const cola2_msgs::msg::Float32Stamped& msg)
{
  sound_velocity_ = static_cast<double>(msg.data);
}

void EKFBaseROS::updateAltitudeMsg(const sensor_msgs::msg::Range& msg)
{
  // Check valid
  if (msg.range > 0.0f)
  {
    // Measurement
    const Eigen::Vector3d sensor_xyz(static_cast<double>(msg.range), 0.0, 0.0);
    // Transform to vehicle frame
    Eigen::Isometry3d trans;
    if (!tf_handler_->getTransform(msg.header.frame_id, trans))
    {
      return;  // not possible to transform
    }
    const Eigen::Vector3d vehicle_xyz = trans * sensor_xyz;
    // Transform to world_ned oriented in vehicle position
    const Eigen::Vector3d world_oriented_xyz = getOrientation() * vehicle_xyz;
    // Take the z coordinate as altitude
    altitude_ = world_oriented_xyz(2);
    last_altitude_time_ = rclcpp::Time(msg.header.stamp).seconds();
    publishNavigation(msg.header.stamp);
  }
}

void EKFBaseROS::publishNavigation(const rclcpp::Time stamp)
{
  // Position
  const Eigen::Vector3d pos = getPosition();
  const Eigen::Vector3d rpy = getEuler();

  // Save last 10s of positions for the USBL delayed data TODO: make more efficient
  last_usbl_positions_.emplace_back(stamp.seconds(), pos, rpy);
  bool done = false;
  while (!done)
  {
    if (last_usbl_positions_.size() > 0 && (stamp.seconds() - last_usbl_positions_.at(0).time) > USBL_KEEP_TIME)
    {
      last_usbl_positions_.erase(last_usbl_positions_.begin());  // delete first if it is too old
    }
    else
    {
      done = true;
    }
  }

  // Check if gps samples need to be reset when filter already init and been diving for a while
  if ((gps_samples_ != 0) && ((stamp.seconds() - last_gps_sample_time_) > 10.0))
  {
    RCLCPP_INFO(node_->get_logger(),"reset gps samples");
    gps_samples_ = 0;
  }

  // Don't do anything else if offline
  if (!online_)
  {
    return;
  }

  // Rest of state
  const Eigen::Vector3d vel = getVelocity();
  const Eigen::Quaterniond quat = getOrientation();
  const Eigen::Vector3d ang_vel = getAngularVelocity();
  // Covariance
  const Eigen::Matrix3d pos_cov = getPositionUncertainty();
  const Eigen::Matrix3d vel_cov = getVelocityUncertainty();
  const Eigen::Matrix3d rpy_cov = getOrientationUncertainty();
  const Eigen::Matrix3d ang_vel_cov = getAngularVelocityUncertainty();

  // Debug
  if (config_.enable_debug_)
  {
    ofh_ << stamp.seconds() << ' ';
    const unsigned int size = x_.rows();
    // State
    for (size_t i = 0; i < size; ++i)
    {
      ofh_ << x_(static_cast<int32_t>(i)) << ' ';
    }
    // Covariance
    for (size_t i = 0; i < size; ++i)
    {
      for (size_t j = 0; j < size; ++j)
      {
        ofh_ << P_(static_cast<int32_t>(i), static_cast<int32_t>(j)) << ' ';
      }
    }
    ofh_ << '\n';
  }

  // Check that enough time has passed
  if (stamp.seconds() - last_publication_time_ < TIME_BETWEEN_PUBLISHING)
  {
    return;
  }
  last_publication_time_ = stamp.seconds();  // now is the last time

  // If not init, just publish invalid navigation
  if (!init_ekf_)
  {
    pub_nav_->publish(cola2::ros::createInvalidNavigation());
    return;  // nothing else to do
  }

  // Publish Odometry message
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = frame_world_;
  odom.header.stamp = stamp;
  odom.pose.pose.position.x = pos(0);
  odom.pose.pose.position.y = pos(1);
  odom.pose.pose.position.z = pos(2);
  odom.pose.pose.orientation.w = quat.w();
  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();
  odom.twist.twist.linear.x = vel(0);
  odom.twist.twist.linear.y = vel(1);
  odom.twist.twist.linear.z = vel(2);
  odom.twist.twist.angular.x = ang_vel(0);
  odom.twist.twist.angular.y = ang_vel(1);
  odom.twist.twist.angular.z = ang_vel(2);
  for (unsigned int i = 0; i < 3; ++i)
  {
    for (unsigned int j = 0; j < 3; ++j)
    {
      // pose and orientation
      odom.pose.covariance.at(i * 6 + j) = pos_cov(i, j);
      odom.pose.covariance.at((i + 3) * 6 + (j + 3)) = rpy_cov(i, j);
      // velocity and rate
      odom.twist.covariance.at(i * 6 + j) = vel_cov(i, j);
      odom.twist.covariance.at((i + 3) * 6 + (j + 3)) = ang_vel_cov(i, j);
    }
  }
  pub_odom_->publish(odom);

  // Publish Nav Status
  const Eigen::Vector3d latlonh = ned_.ned2Geodetic(pos);
  cola2_msgs::msg::NavSts nav_sts;
  nav_sts.header.frame_id = frame_vehicle_;
  nav_sts.header.stamp = stamp;
  nav_sts.altitude = static_cast<float>(altitude_);
  nav_sts.body_velocity.x = vel(0);
  nav_sts.body_velocity.y = vel(1);
  nav_sts.body_velocity.z = vel(2);
  nav_sts.global_position.latitude = latlonh(0);
  nav_sts.global_position.longitude = latlonh(1);
  nav_sts.orientation.roll = static_cast<float>(rpy(0));
  nav_sts.orientation.pitch = static_cast<float>(rpy(1));
  nav_sts.orientation.yaw = static_cast<float>(rpy(2));
  nav_sts.orientation_rate.roll = static_cast<float>(ang_vel(0));
  nav_sts.orientation_rate.pitch = static_cast<float>(ang_vel(1));
  nav_sts.orientation_rate.yaw = static_cast<float>(ang_vel(2));
  nav_sts.origin.latitude = ned_.getInitLatitude();
  nav_sts.origin.longitude = ned_.getInitLongitude();
  nav_sts.position.north = pos(0);
  nav_sts.position.east = pos(1);
  nav_sts.position.depth = pos(2);
  nav_sts.position_variance.north = pos_cov(0, 0);
  nav_sts.position_variance.east = pos_cov(1, 1);
  nav_sts.position_variance.depth = pos_cov(2, 2);
  nav_sts.orientation_variance.roll = static_cast<float>(rpy_cov(0, 0));
  nav_sts.orientation_variance.pitch = static_cast<float>(rpy_cov(1, 1));
  nav_sts.orientation_variance.yaw = static_cast<float>(rpy_cov(2, 2));
  pub_nav_->publish(nav_sts);

  // Publish vehicle TF
  geometry_msgs::msg::TransformStamped transformStamped;

  // Fill in the transform data
  transformStamped.header.stamp = stamp;
  transformStamped.header.frame_id = frame_world_;
  transformStamped.child_frame_id = frame_vehicle_;

  // Example transform data
  transformStamped.transform.translation.x = odom.pose.pose.position.x;
  transformStamped.transform.translation.y = odom.pose.pose.position.y;
  transformStamped.transform.translation.z = odom.pose.pose.position.z;

  tf2::Quaternion q = tf2::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  // Broadcast the transform
  tf_broadcast_->sendTransform(transformStamped);

}

void EKFBaseROS::publishGPSNED(const rclcpp::Time stamp, const Eigen::Vector3d& ned, const bool& valid) const
{
  // Don't do anything if offline
  if (!online_)
  {
    return;
  }
  // Create and publish message
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_world_;
  msg.pose.position.x = ned(0);
  msg.pose.position.y = ned(1);
  msg.pose.position.z = 0.0;
  if (valid)
  {
    // pointing like the vehicle
    const Eigen::Quaterniond q = getOrientation();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
  }
  else
  {
    // pointing upwards
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.70710678;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.70710678;
  }
  pub_gps_ned_->publish(msg);
}

void EKFBaseROS::publishUSBLNED(const rclcpp::Time stamp, const Eigen::Vector3d& ned, const USBLStatus& status) const
{
  // Don't do anything if offline
  if (!online_)
  {
    return;
  }
  // Create and publish message
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_world_;
  msg.pose.position.x = ned(0);
  msg.pose.position.y = ned(1);
  msg.pose.position.z = ned(2);
  switch (status)
  {
    case USBLStatus::OutOfBuffer:
      [[fallthrough]];
    case USBLStatus::IQUAviewInvalid:
      // pointing downwards
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.70710678;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = -0.70710678;
      break;
    case USBLStatus::NotUpdated:
      // pointing upwards
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.70710678;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 0.70710678;
      break;
    case USBLStatus::Updated:
      // pointing like the vehicle
      const Eigen::Quaterniond q = getOrientation();
      msg.pose.orientation.x = q.x();
      msg.pose.orientation.y = q.y();
      msg.pose.orientation.z = q.z();
      msg.pose.orientation.w = q.w();
      break;
  }
  pub_usbl_ned_->publish(msg);
}

// *****************************************
// Services
// *****************************************
bool EKFBaseROS::srvReloadNED(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{

  (void)request_header;
  (void)req;


  RCLCPP_INFO(node_->get_logger(),"Reload NED service called");

  // forget saved positions for delayed usbl updates
  last_usbl_positions_.clear();

  // Load NED config
  cola2::ros::getParam("~ned_latitude", config_.ned_latitude_, 0.0, node_);
  cola2::ros::getParam("~ned_longitude", config_.ned_longitude_, 0.0, node_);

  // EKF init => reproject position to new ned
  if (init_ekf_)
  {
    // Current position in current ned
    const Eigen::Vector3d current = getPosition();
    RCLCPP_INFO(node_->get_logger(),"srvReloadNED: current position old NED: [%.3f, %.3f, %.3f] in ned (%.8f, %.8f)", current(0), current(1),
             current(2), ned_.getInitLatitude(), ned_.getInitLongitude());
    Eigen::Vector3d latlonh = ned_.ned2Geodetic(current);
    latlonh(2) = 0.0;
    // Current position in new ned
    ned_ = cola2::utils::NED(config_.ned_latitude_, config_.ned_longitude_, 0.0);
    const Eigen::Vector3d xyz = ned_.geodetic2Ned(latlonh);
    setPositionXY(xyz.head(2));
    const Eigen::Vector3d new_current = getPosition();
    RCLCPP_INFO(node_->get_logger(),"srvReloadNED: current position new NED: [%.3f, %.3f, %.3f] in ned (%.8f, %.8f)", new_current(0),
             new_current(1), new_current(2), ned_.getInitLatitude(), ned_.getInitLongitude());
  }
  else
  {
    // EKF not init => restart filter initialization and NED
    resetFilter();
  }

  // Publish params after param reload
  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result = srv_publish_params_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
  {
         RCLCPP_WARN(node_->get_logger(), "Publish params did not succeed ");
         res->message = "Service publish params did not succeed.";
         res->success = false;
  } 
  else
  {
         res->message = "Service publish params succeed.";
         res->success = true;
  }


  return true;
}

bool EKFBaseROS::srvResetNavigation(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{

  (void)request_header;
  (void)req;

  RCLCPP_INFO(node_->get_logger(),"Reset navigation or reload params service called");
  *res = getConfig();
  if (!res->success)
  {
    // invalid config, do not continue
    return true;
  }
  resetFilter();  // reset filter and NED

  // Publish params after param reload

  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result = srv_publish_params_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
  {
         RCLCPP_WARN(node_->get_logger(), "Publish params did not succeed ");
         res->message = "Service publish params did not succeed.";
         res->success = false;
  } 
  else
  {
         res->message = "Service publish params succeed.";
         res->success = true;
  }
  return true;
}

bool EKFBaseROS::srvSetDepthSensorOffset(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{

  (void)request_header;
  (void)req;

  RCLCPP_INFO(node_->get_logger(),"Set depth sensor offset service called");
  config_.depth_sensor_offset_ = config_.surface2depth_sensor_distance_ - pressure_meters_;
  char temp[200];
  std::snprintf(temp, sizeof(temp), "New depth sensor offset at %.3f", config_.depth_sensor_offset_);
  res->success = true;
  res->message = temp;
  RCLCPP_INFO(node_->get_logger(),"%s", res->message.c_str());
  
  return true;
}

double EKFBaseROS::getAltitude() const
{
  return altitude_;
}
