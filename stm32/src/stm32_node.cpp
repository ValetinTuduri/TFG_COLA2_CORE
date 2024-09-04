/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/io/serial_port.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/msg/setpoints.hpp>
#include <cola2_msgs/srv/cam_trigger.hpp>
#include "stm32/stm32.h"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>


class STM32node : public rclcpp::Node
{
private:

 

  // Pointer to the driver that does the work
  std::unique_ptr<STM32> stm32_;

  // Config struct
  struct
  {
    cola2::io::SPConfig sp_config;
    std::string frame_id;
  } config_;

  // Publishers
  rclcpp::Subscription<cola2_msgs::msg::Setpoints>::SharedPtr sub_setpoints_;
  rclcpp::Service<cola2_msgs::srv::CamTrigger>::SharedPtr service_camTrigger_;

  std::string fillWithZeros(int number, int width);

  uint8_t normalizeTo255(double value) ;
  
  std::string boolToString(bool value);

public:
  /**
   * Starts de rclcpp node.
   */
  STM32node();

  /**
   * Gets the config parameters, initializes the connection and sets the
   * publishers
   */
  void init();

  void handle_cam_trigger(const std::shared_ptr<cola2_msgs::srv::CamTrigger::Request> request,std::shared_ptr<cola2_msgs::srv::CamTrigger::Response> response);

  void setpoints_callback(const cola2_msgs::msg::Setpoints & msg);

  /**
   * Gets the config parameters from the param server
   */
  void getConfig();
};

STM32node::STM32node() : Node("stm32_node")
{}

void STM32node::init()
{
 
  getConfig();

  // Define publishers
  sub_setpoints_ = this->create_subscription<cola2_msgs::msg::Setpoints>("stm32/Setpoints", 10, std::bind(&STM32node::setpoints_callback, this, std::placeholders::_1));


  service_camTrigger_ = this->create_service<cola2_msgs::srv::CamTrigger>("cam_trigger_service", std::bind(&STM32node::handle_cam_trigger, this,std::placeholders::_1, std::placeholders::_2));
  


  // Init stm32 connection and send configuration
  bool done = false;
  while (!done && rclcpp::ok())
  {
    try
    {
      stm32_ = std::unique_ptr<STM32>(new STM32());
      stm32_->init(config_.sp_config);
      RCLCPP_INFO(this->get_logger(), "configuring stm32 device...");
     
     //Si es vol establir alguna comanda inicial al stm32
     //stm32_->configure("missatge");
      
      // done
      done = true;
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Error setting up the serial port: %s", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }

  // Show message
  RCLCPP_INFO(this->get_logger(), "initialized.");
}

std::string STM32node::fillWithZeros(int number, int width) {
    std::ostringstream oss;
    oss << std::setw(width) << std::setfill('0') << number;
    return oss.str();
}

uint8_t STM32node::normalizeTo255(double value) {
    // Check if the value is within the valid range
    if (value < 0.0 || value > 1.0) {
        std::cerr << "Error: Value must be between 0 and 1." << std::endl;
        return 0; // Return 0 in case of error
    }
    
    // Normalize the value to the range 0-255
    return static_cast<uint8_t>(value * 255.0);
}


std::string STM32node::boolToString(bool value) {
    return value ? "1" : "0";
}

void  STM32node::handle_cam_trigger(const std::shared_ptr<cola2_msgs::srv::CamTrigger::Request> request,std::shared_ptr<cola2_msgs::srv::CamTrigger::Response> response)
{
    if(request->intensity1 < 0 || request->intensity1 > 1 || request->intensity2 < 0 || request->intensity1 > 1)
    {
        response->success = false;
    }
    else
    {
        std::string checksum = fillWithZeros((request->cam1+request->cam2+normalizeTo255(request->intensity1)+normalizeTo255(request->intensity2)),3);
        std::string msg = "cc:"+boolToString(request->cam1)+":"+boolToString(request->cam2)+":";
        msg = msg + fillWithZeros(normalizeTo255(request->intensity1),3)+":"+fillWithZeros(normalizeTo255(request->intensity2),3)+":"+checksum+":ee";
        stm32_->write(msg);
        RCLCPP_INFO(this->get_logger(), msg.c_str());
        response->success = true;
    }
}

void STM32node::setpoints_callback(const cola2_msgs::msg::Setpoints & msg) 
{
    std::string string_to_send= "aa:";
    int checksum = 0;

    // Process the setpoints array
    for (const auto & setpoint : msg.setpoints) {
      if (setpoint > 0.0 && setpoint < 1.0) {
        int normalized_setpoint = normalizeTo255(setpoint);
        checksum += normalized_setpoint;
        string_to_send = string_to_send+fillWithZeros(normalized_setpoint,3)+":";

      } else {
         RCLCPP_INFO(this->get_logger(), "Receveid Setpoints not valid");
        return;
      }
    }
    string_to_send = string_to_send + fillWithZeros(checksum,4);
    string_to_send = string_to_send + "ee";
    RCLCPP_INFO(this->get_logger(), string_to_send.c_str());
    stm32_->write(string_to_send);

}



void STM32node::getConfig()
{

  this->declare_parameter<std::string>("serial_port.path", "/dev/ttyACM0");
  this->declare_parameter<int>("serial_port.baud_rate", 115200);
  this->declare_parameter<int>("serial_port.char_size", 8);
  this->declare_parameter<int>("serial_port.stop_bits", 1);
  this->declare_parameter<std::string>("serial_port.parity", "NONE");
  this->declare_parameter<std::string>("serial_port.flow_control", "NONE");
  this->declare_parameter<int>("serial_port.timeout", 10000);
  this->declare_parameter<std::string>("frame_id", "STM32");

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
  std::shared_ptr<STM32node> node = std::make_shared<STM32node>();
  node->init();
  rclcpp::Rate rate(1);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
