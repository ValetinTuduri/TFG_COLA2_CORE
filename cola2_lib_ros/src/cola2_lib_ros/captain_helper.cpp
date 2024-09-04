/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib_ros/captain_helper.h"

namespace cola2
{
namespace rosutils
{
void waitForIdleHelper(bool* is_idle, bool* is_safety, std::mutex* mtx, bool* first_call,
                       const std::shared_ptr<const cola2_msgs::msg::CaptainStatus>& captain_status)
{
  std::lock_guard<std::mutex> guard(*mtx);
  if (*first_call)
  {
    (*first_call) = false;
    return;  // Discard first message
  }
  (*is_idle) = (captain_status->state == cola2_msgs::msg::CaptainStatus::IDLE);
  (*is_safety) = (captain_status->state == cola2_msgs::msg::CaptainStatus::SAFETYKEEPPOSITION);
}

bool waitForIdle(std::shared_ptr<rclcpp::Node> node)
{
  bool is_idle(false);
  bool is_safety(false);
  std::mutex mtx;
  bool first_call(true);

  //Funcio lambda per assignar al callback del subscriptor
    std::function<void(const cola2_msgs::msg::CaptainStatus::SharedPtr)> callback =
    [&is_idle, &is_safety, &mtx, &first_call](const cola2_msgs::msg::CaptainStatus::SharedPtr msg)
    {
      waitForIdleHelper(&is_idle, &is_safety, &mtx, &first_call, msg);
    };

  rclcpp::Subscription<cola2_msgs::msg::CaptainStatus>::SharedPtr subscription;
  subscription = node->create_subscription<cola2_msgs::msg::CaptainStatus>("/captain/captain_status", 10, callback);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok())
  {
    executor.spin_some();
    {
      std::lock_guard<std::mutex> guard(mtx);
      if (is_safety)
        return false;
      if (is_idle)
        return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return false;
}
}  // namespace rosutils
}  // namespace cola2
