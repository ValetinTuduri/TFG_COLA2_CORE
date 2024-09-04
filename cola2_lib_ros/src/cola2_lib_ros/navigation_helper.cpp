/*
 * Copyright (c) 2023 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib_ros/navigation_helper.h"

#include <rclcpp/rclcpp.hpp>

namespace cola2
{
namespace ros
{
bool navigationIsValid(const cola2_msgs::msg::NavSts& msg)
{
  if ((msg.global_position.latitude == 0.0) && (msg.global_position.longitude == 0.0))
  {
    return false;
  }
  return true;
}

cola2_msgs::msg::NavSts createInvalidNavigation()
{
  cola2_msgs::msg::NavSts msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.global_position.latitude = 0.0;
  msg.global_position.longitude = 0.0;
  return msg;
}

constexpr double TIME_DELAY_NO_UPDATES = 0.99e4;  // iquaview uses 1e4 to disable usbls

bool usblIsValid(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
  return usblIsValid(msg, rclcpp::Clock().now());
}

bool usblIsValid(const geometry_msgs::msg::PoseWithCovarianceStamped& msg,  const rclcpp::Time& stamp)
{
  if ((stamp - msg.header.stamp).seconds() > TIME_DELAY_NO_UPDATES)
  {
    return false;
  }
  return true;
}
}  // namespace ros
}  // namespace cola2
