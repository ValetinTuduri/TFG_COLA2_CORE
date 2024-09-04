/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/io/serial_port.h>
#include <cola2_lib/io/tcp_socket.h>
#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/msg/gps_data.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "nmea_gps/nmea_gps.h"
#include <rclcpp/rclcpp.hpp>

/**
 *  \brief COLA2 ROS wrapper for the NMEA GPS driver.
 *  Publishes NavSatFix and custom GpsData messages.
 */
class GPS : public rclcpp::Node
{
private:

  // Diagnostics
  std::shared_ptr<cola2::ros::DiagnosticHelper> diagnostic_;

  // Pointer to the driver that does the work
  std::unique_ptr<NMEAGPS> nmea_gps_;

  // Config struct
  struct
  {
    cola2::io::SPConfig sp_config;
    std::string frame_id;
  } config_;

  // Publishers
  rclcpp::Publisher<cola2_msgs::msg::GPSData>::SharedPtr pub_raw_data_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_nav_sat_fix_;

public:
  /**
   * Starts de rclcpp node.
   */
  GPS();

  /**
   * Gets the config parameters, initializes the connection and sets the
   * publishers
   */
  void init();

  /**
   * Calls the NMEA GPS driver to get the data, fills the NavSatFix and GpsData
   * msgs and publishes them
   */
  void iterate();

  /**
   * Gets the config parameters from the param server
   */
  void getConfig();
};

GPS::GPS() : Node("nmea_gps_node")
{}

void GPS::init()
{
  diagnostic_ = std::make_shared<cola2::ros::DiagnosticHelper>(shared_from_this(), "gps", cola2::ros::getUnresolvedNodeName(shared_from_this()));
   
  getConfig();

  // Define publishers
  pub_raw_data_ = this->create_publisher<cola2_msgs::msg::GPSData>("data", 1);
  pub_nav_sat_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("nav_sat_fix", 1);

  // Init GPS connection and send configuration
  bool done = false;
  while (!done && rclcpp::ok())
  {
    try
    {
      nmea_gps_ = std::unique_ptr<NMEAGPS>(new NMEAGPS());
      nmea_gps_->init(config_.sp_config);
      RCLCPP_INFO(this->get_logger(), "configuring gps device...");
      // send initial configuration commands:
      // always_allocate_off
      nmea_gps_->configure("$PMTK225,0*2B\r\n");
      // easy_mode_off
      nmea_gps_->configure("$PMTK869,1,0*34\r\n");
      // static_navigation_off
      nmea_gps_->configure("$PMTK386,0*23\r\n");
      // done
      done = true;
    }
    catch (const std::exception& ex)
    {
      diagnostic_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Error setting up the serial port: " + std::string(ex.what()));
      diagnostic_->publish(shared_from_this());
      RCLCPP_ERROR(this->get_logger(), "Error setting up the serial port: %s", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }

  // Show message
  diagnostic_->setEnabled(true);
  RCLCPP_INFO(this->get_logger(), "initialized.");
}

void GPS::iterate()
{
  try
  {
    // Get data from nmea driver
    GPSData gps_raw = nmea_gps_->getData();

    // Create and fill custom message
    cola2_msgs::msg::GPSData gps_data;
    gps_data.header.stamp = this->now();
    gps_data.header.frame_id = config_.frame_id;
    gps_data.time_utc = gps_raw.time_utc;
    gps_data.latitude = gps_raw.latitude;
    gps_data.longitude = gps_raw.longitude;
    gps_data.fix_quality = gps_raw.fix_quality;
    gps_data.number_of_satellites = gps_raw.number_of_satellites;
    gps_data.hdop = gps_raw.hdop;
    gps_data.altitude = gps_raw.altitude;
    gps_data.geoidal_separation = gps_raw.geoidal_separation;
    gps_data.heading = gps_raw.heading;
    gps_data.latitude_stdev = gps_raw.latitude_stdev;
    gps_data.longitude_stdev = gps_raw.longitude_stdev;
    gps_data.altitude_stdev = gps_raw.altitude_stdev;
    gps_data.raw_gpgga = gps_raw.raw_gpgga;
    gps_data.raw_gphdt = gps_raw.raw_gphdt;
    gps_data.raw_gpgst = gps_raw.raw_gpgst;

    diagnostic_->addKeyValue("data_quality", static_cast<int>(gps_raw.fix_quality));
    if (gps_raw.fix_quality == 1 || gps_raw.fix_quality == 2)
      diagnostic_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::OK);
    else
      diagnostic_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::OK, "Bad quality data");
    diagnostic_->reportValidData();

    // Create Nav Sat FIX message
    sensor_msgs::msg::NavSatFix nav_sat_fix;
    nav_sat_fix.header.stamp = gps_data.header.stamp;
    nav_sat_fix.header.frame_id = config_.frame_id;
    nav_sat_fix.status.status = gps_raw.fix_quality - 1;
    nav_sat_fix.status.service = nav_sat_fix.status.SERVICE_GPS;
    nav_sat_fix.latitude = gps_raw.latitude;
    nav_sat_fix.longitude = gps_raw.longitude;
    nav_sat_fix.altitude = gps_raw.altitude + gps_raw.geoidal_separation;

    // Correct nav_sat_fix.status.status from raw.fix_quality
    // 0 = invalid
    // 1 = GPS fix (SPS)
    // 2 = DGPS fix
    // 3 = PPS fix
    // 4 = Real Time Kinematic
    // 5 = Float RTK
    // 6 = estimated (dead reckoning) (2.3 feature)
    // 7 = Manual input mode
    // 8 = Simulation mode
    if (gps_raw.fix_quality > 5)
    {
      nav_sat_fix.status.status = nav_sat_fix.status.STATUS_NO_FIX;
    }

    // If stdevs are available, fill diagonal with variances
    // if (gps_raw.latitude_stdev && gps_raw.longitude_stdev && gps_raw.altitude_stdev)
    //{
    //  nav_sat_fix.position_covariance[0] = pow(gps_raw.latitude_stdev, 2.0);
    //  nav_sat_fix.position_covariance[4] = pow(gps_raw.longitude_stdev, 2.0);
    //  nav_sat_fix.position_covariance[8] = pow(gps_raw.altitude_stdev, 2.0);
    //  nav_sat_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    //}
    // else
    //{
    // Fill with hdop (as nmea_navsat_driver)
    nav_sat_fix.position_covariance[0] = pow(gps_raw.hdop * 1.5, 2.0);
    nav_sat_fix.position_covariance[4] = pow(gps_raw.hdop * 1.5, 2.0);
    nav_sat_fix.position_covariance[8] = pow(gps_raw.hdop * 1.5, 2.0);
    nav_sat_fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    //}

    // Publish NavSatFix and custom Gps msg
    pub_nav_sat_fix_->publish(nav_sat_fix);
    pub_raw_data_->publish(gps_data);
  }
  catch (const std::exception& ex)
  {
    if (!rclcpp::ok())
    {
      diagnostic_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::WARN, "I/O device reading error");
      RCLCPP_FATAL(this->get_logger(), "I/O device reading error: %s", ex.what());
    }
  }

  diagnostic_->publish(shared_from_this());
}

void GPS::getConfig()
{

  this->declare_parameter<std::string>("serial_port.path", "/dev/ttyACM0");
  this->declare_parameter<int>("serial_port.baud_rate", 38400);
  this->declare_parameter<int>("serial_port.char_size", 8);
  this->declare_parameter<int>("serial_port.stop_bits", 1);
  this->declare_parameter<std::string>("serial_port.parity", "NONE");
  this->declare_parameter<std::string>("serial_port.flow_control", "NONE");
  this->declare_parameter<int>("serial_port.timeout", 10000);
  this->declare_parameter<std::string>("frame_id", "gps");

  // Retrieve the parameters
  this->get_parameter("serial_port.path", config_.sp_config.sp_path);
  this->get_parameter("serial_port.baud_rate", config_.sp_config.sp_baud_rate);
  this->get_parameter("serial_port.char_size", config_.sp_config.sp_char_size);
  this->get_parameter("serial_port.stop_bits", config_.sp_config.sp_stop_bits);
  this->get_parameter("serial_port.parity", config_.sp_config.sp_parity);
  this->get_parameter("serial_port.flow_control", config_.sp_config.sp_flow_control);
  this->get_parameter("serial_port.timeout", config_.sp_config.sp_timeout);
  this->get_parameter("frame_id", config_.frame_id);


  // Append robot name to frame id to meet the TF tree
  config_.frame_id = cola2::ros::getNamespaceNoInitialDash(shared_from_this()) + std::string("/") + config_.frame_id;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<GPS> node = std::make_shared<GPS>();
  node->init();
  rclcpp::Rate rate(1);
  while (rclcpp::ok())
  {
    node->iterate();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
