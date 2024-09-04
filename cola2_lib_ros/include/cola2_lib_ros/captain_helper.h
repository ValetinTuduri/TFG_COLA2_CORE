
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ Captain helper for COLA2.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_ROSUTILS_CAPTAIN_HELPER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_ROSUTILS_CAPTAIN_HELPER_H_

#include <cola2_msgs/msg/captain_status.hpp>
#include <cola2_msgs/msg/goal_descriptor.hpp>
#include <cola2_msgs/srv/goto.hpp>

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <mutex>
#include <condition_variable>

namespace cola2
{
namespace rosutils
{
/**
 * @addtogroup CaptainHelper
 * @{
 */

/**
 * @brief Helper function. Not intended to be used on its own.
 *
 * @param is_idle Is captain status idle?
 * @param is_safety Is captain status in safety keep position?
 * @param mtx Mutex.
 * @param first_call Is the first call?
 * @param captain_status Captain Status.
 */
void waitForIdleHelper(bool* is_idle, bool* is_safety, std::mutex* mtx, bool* first_call,
                       const std::shared_ptr<const cola2_msgs::msg::CaptainStatus>& captain_status);

/**
 * @brief This function blocks until the captain is in Idle state.
 *
 * @param node Node.
 * @return Returns true if it returned back to Idle and false if it switched to something else.
 */
bool waitForIdle(std::shared_ptr<rclcpp::Node> node);

/**
 * @brief  This function calls a service from the captain, such as a goto, mission... and then blocks until the captain
 * is back to idle.
 *
 * @param node Node .
 * @tparam srv Service client.
 * @tparam req Service request.
 * @tparam res Service response.
 * @return Returns true if the call and the waiting was succesful, and false otherwise.
 */

template <typename TSrv, typename TReq, typename TRes>
bool callServiceAndWaitForIdle(std::shared_ptr<rclcpp::Node> node, TSrv& srv, const TReq& req, TRes& res)
{
  auto client = node->create_client<TSrv>(srv);

  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node->get_logger(), "Service %s not available.", srv.c_str());
    return false;
  }

  auto request = std::make_shared<TReq>(req);

  auto result = client->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    res = *(result.get());
    if (res.success) {
      if (waitForIdle(node)) {
        return true;
      } else {
        RCLCPP_ERROR(node->get_logger(), "Captain switched to safety state instead of idle");
        return false;
      }
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed.");
  }

  RCLCPP_ERROR(node->get_logger(), "Error processing request in callServiceAndWaitForIdle(). Req:\n%s\nRes:\n%s", 
               req.to_string().c_str(), res.to_string().c_str());
  return false;
}
/** @} */
}  // namespace rosutils
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_ROSUTILS_CAPTAIN_HELPER_H_
