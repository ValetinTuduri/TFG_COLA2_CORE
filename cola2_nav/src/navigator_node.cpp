/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_nav/ekf_position_velocity.h>
#include "rclcpp/rclcpp.hpp"
#include "stonefish_ros2/msg/dvl.hpp"
#include "stonefish_ros2/msg/ins.hpp"

class NavigatorNode : public rclcpp::Node, public EKFPositionVelocity 
{
private:
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;                              // [x y]
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_usbl_;           // [x y]
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr sub_pressure_;                     // [z]
  rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr sub_dvl_;                                     // [vx vy vz]
  rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr sub_dvl_fallback_;                            // [vx vy vz]
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;                                    // [roll pitch yaw vroll vpitch vyaw]
  rclcpp::Subscription<cola2_msgs::msg::Float32Stamped>::SharedPtr sub_sound_velocity_;               // sound velocity from SVS
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_altitude_;                             // altitude from seafloor


public:
  /**
   * \brief Constructor that relates all sensors to their callbacks.
   */
  NavigatorNode();
  /**
   * \brief Destructor.
   */
  ~NavigatorNode();


  void init_NavigatorNode();
};

NavigatorNode::NavigatorNode() : Node("navigator") 
{}

NavigatorNode::~NavigatorNode()
{}


void NavigatorNode::init_NavigatorNode()
{
  init_pv(shared_from_this());
  sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/girona500/navigator/gps", 2, std::bind(&EKFBaseROS::updatePositionGPSMsg, this, std::placeholders::_1));
  sub_usbl_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("usbl", 2, std::bind(&EKFBaseROS::updatePositionUSBLMsg, this, std::placeholders::_1));
  sub_pressure_ = this->create_subscription<sensor_msgs::msg::FluidPressure>("/girona500/navigator/pressure", 2, std::bind(&EKFBaseROS::updatePositionDepthMsg, this, std::placeholders::_1));
  sub_dvl_ = this->create_subscription<stonefish_ros2::msg::DVL>("/girona500/navigator/dvl_sim", 2, std::bind(&EKFBaseROS::updateVelocityDVLMsg, this, std::placeholders::_1));
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/girona500/navigator/imu", 2, std::bind(&EKFBaseROS::updateIMUMsg, this, std::placeholders::_1));

  if (config_.dvl_fallback_delay_ > 0.0)
  {
    RCLCPP_INFO(this->get_logger(), "Subscribing to 'dvl_fallback'");
    sub_dvl_fallback_ = this->create_subscription<stonefish_ros2::msg::DVL>(
      "dvl_fallback", 2, std::bind(&EKFBaseROS::updateVelocityDVLFallbackMsg, this, std::placeholders::_1));
  }

  // Other data
  sub_sound_velocity_ = this->create_subscription<cola2_msgs::msg::Float32Stamped>("sound_velocity", 2, std::bind(&EKFBaseROS::updateSoundVelocityMsg, this, 
  std::placeholders::_1));
  sub_altitude_ = this->create_subscription<sensor_msgs::msg::Range>("altitude", 2, std::bind(&EKFBaseROS::updateAltitudeMsg, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialized");
}

int main(int argc, char** argv)
{
  
  rclcpp::init(argc, argv);
  auto navigator_node = std::make_shared<NavigatorNode>();
  navigator_node->init_NavigatorNode();
  rclcpp::spin(navigator_node);
  rclcpp::shutdown();
  return 0;
}
