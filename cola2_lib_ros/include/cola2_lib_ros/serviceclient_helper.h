/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ Service Client helper for COLA2.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_UTILS_SERVICECLIENT_HELPER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_UTILS_SERVICECLIENT_HELPER_H_

#include <cola2_lib/utils/thread_safe_flag.h>
#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <stdexcept>
#include <thread>

namespace cola2
{
namespace ros
{
/**
 * @addtogroup ServiceHelper
 * @{
 */

/**
 * @brief Connect to the specified service name and type.
 *
 * @param nh Node handler that wants to connect to a service.
 * @param srv_name Full service name to connect to.
 * @param wait Time to wait between tries, defaults to 1.0.
 * @param ServiceType Type of service used in the connection.
 * @return A ServiceClient to the specified service.
 */
template <class TSrv>
rclcpp::Client<typename TSrv::Request, typename TSrv::Response>::SharedPtr
connectToService(rclcpp::Node::SharedPtr node, const std::string& srv_name, const double wait = 1.0)
{
  auto client = node->create_client<TSrv>(srv_name);
  while (rclcpp::ok())
  {
    if (client->wait_for_service(std::chrono::duration<double>(wait)))
      break;
    RCLCPP_WARN_STREAM(node->get_logger(), "Waiting for client to service: " << srv_name);
    std::this_thread::sleep_for(std::chrono::duration<double>(wait));
  }
  return client;
}

/**
 * @brief Helper function.
 *
 * @tparam srv Service client.
 * @tparam req Service request.
 * @tparam res Service response.
 * @param success Success.
 * @param done Done flag.
 */
template <typename TSrv, typename TReq, typename TRes>
void callServiceHelper(const std::shared_ptr<rclcpp::Node>& node, const TReq* req, TRes* res, bool* success, cola2::utils::ThreadSafeFlag* done)
{
  auto client =  node->create_client<std_srvs::srv::Trigger>(action_id);
  auto result = client->async_send_request(*req);
  *success = rclcpp::spin_until_future_complete(client->get_node_base_interface(), result) == rclcpp::executor::FutureReturnCode::SUCCESS;
  if (*success)
  {
    *res = result.get();
  }
  done->setState(true);
}
/**
 * @brief This function allows calling a service with a timeout.
 *
 * @tparam srv Service client.
 * @tparam req Service request.
 * @tparam res Service response.
 * @param sucess Sucess.
 * @param timeout Timeout in seconds.
 * @return It returns true if the service returned on time and false otherwise.
 */
template <typename TSrv, typename TReq, typename TRes>
bool callServiceWithTimeout(const std::shared_ptr<rclcpp::Node>& node, const TReq& req, TRes& res, bool& success, double timeout)
{
  timeout = timeout < 0.0 ? 0.0 : timeout;
  cola2::utils::ThreadSafeFlag* done = new cola2::utils::ThreadSafeFlag(false);  // This may need to remain on memory
  std::function<void()> callback_func =std::bind(callServiceHelper<TSrv, TReq, TRes>, node, &req, &res, &success, done);
  std::thread th(callback_func);
  done->timedBlockingWaitFor(true, timeout);

  if (done->getState())
  {
    th.join();
    delete done;
    return true;
  }

  th.detach();
  return false;
}

/**
 * @brief This function allows calling a service with a timeout. It throws if the timeout expires, or if a wait time
 * has been configured and the service is not available after waiting. The template parameter must be provided
 * if the compiler type deduction fails.
 *
 * @param node Node .
 * @tparam req Service request.
 * @tparam res Service response.
 * @param srv_name Service name.
 * @param timeout Timeout in seconds.
 * @param wait_time Time in seconds to wait for the service if it does not exist.
 * @return It returns true if the service returned true and false otherwise.
 */
template <typename TSrv>
bool callServiceWithTimeout(const std::shared_ptr<rclcpp::Node>& node, typename TSrv::Request& req, typename TSrv::Response& res,
                            const std::string& srv_name, const double timeout, const double wait_time = 0.0)
{
  // Create service client
  auto client = node->create_client<TSrv>(srv_name);

  // Check if a wait time has been given
  if ((wait_time > 0.0) && (!client->wait_for_service(std::chrono::duration<double>(wait_time))))
  {
    throw std::runtime_error(std::string("Wait time exceeded for service ") + srv_name);
  }

  // Create a copy of the request and the response, create a success bool and create thread safe flag
  typename TSrv::Request req_copy = req;
  typename TSrv::Response res_copy;
  bool success = false;
  cola2::utils::ThreadSafeFlag done(false);

  // Start thread calling lambda with captures to the pointers
  std::thread th([&client, &req_copy, &res_copy, &success, &done]() {
    auto result_future = client->async_send_request(std::make_shared<typename TSrv::Request>(req_copy));
    success = rclcpp::spin_until_future_complete(client->get_node_base_interface(), result_future) == rclcpp::executor::FutureReturnCode::SUCCESS;
    if (success) {
      res_copy = *result_future.get();
    }
    done.setState(true);
  });

  // Wait for result
  done.timedBlockingWaitFor(true, (timeout < 0.0 ? 0.0 : timeout));

  // Check if the service failed to return on time
  if (!done.getState())
  {
    th.detach();
    throw std::runtime_error(std::string("Timeout exceeded for service ") + srv_name + std::string(". Leaking memory"));
  }

  // The call returned on time. Copy back the result
  req = req_copy;
  res = res_copy;

  // Join thread and clear memory
  th.join();

  // Return what the service has returned
  return success;
}

/** @} */
}  // namespace ros
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_UTILS_SERVICECLIENT_HELPER_H_
