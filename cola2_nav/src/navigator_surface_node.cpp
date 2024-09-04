/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_nav/ekf_surface_2d.h>
#include "rclcpp/rclcpp.hpp"


class NavigatorSurfaceNode : public rclcpp::Node, public EKFSurface2D 
{
private:
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;                              // [x y]
  rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr sub_dvl_;                                     // [vx vy vz]
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;                                    // [roll pitch yaw vroll vpitch vyaw]
  rclcpp::Subscription<cola2_msgs::msg::Float32Stamped>::SharedPtr sub_sound_velocity_;               // sound velocity from SVS
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_altitude_;                             // altitude from seafloor
public:
  /**
   * \brief Constructor that relates all sensors to their callbacks.
   */
  NavigatorSurfaceNode();
  /**
   * \brief Destructor.
   */
  ~NavigatorSurfaceNode();

  //
  void init_NavigatorSurfaceNode()
  {
    init_s2D(shared_from_this());
  }
};

NavigatorSurfaceNode::NavigatorSurfaceNode() : Node("navigator_surface") , EKFSurface2D()
{
  // clang-format off
  // Init subscribers
  sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/girona500/navigator/gps", 2, std::bind(&EKFBaseROS::updatePositionGPSMsg, this, std::placeholders::_1));
  sub_dvl_ = this->create_subscription<stonefish_ros2::msg::DVL>("/girona500/navigator/dvl_sim", 2, std::bind(&EKFBaseROS::updateVelocityDVLMsg, this, std::placeholders::_1));
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/girona500/navigator/imu", 2, std::bind(&EKFBaseROS::updateIMUMsg, this, std::placeholders::_1));
  // Other data
  sub_sound_velocity_ = this->create_subscription<cola2_msgs::msg::Float32Stamped>("sound_velocity", 2, std::bind(&EKFBaseROS::updateSoundVelocityMsg, this, std::placeholders::_1));
  sub_altitude_ = this->create_subscription<sensor_msgs::msg::Range>("/girona500/navigator/altitude", 2, std::bind(&EKFBaseROS::updateAltitudeMsg, this, std::placeholders::_1));

  // clang-format on
}

NavigatorSurfaceNode::~NavigatorSurfaceNode()
{
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto navigator_node = std::make_shared<NavigatorSurfaceNode>();
  navigator_node->init_NavigatorSurfaceNode();
  rclcpp::spin(navigator_node);
  rclcpp::shutdown();
  return 0;
}
