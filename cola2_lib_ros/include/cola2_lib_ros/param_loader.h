
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ Param loader for COLA2.
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_ROS_PARAMLOADER_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_ROS_PARAMLOADER_H_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>

namespace cola2
{
namespace ros
{
/**
 * @addtogroup ParamLoader
 * @{
 */

/**
 * @brief Reads the parameter param_name from the ROS param server into param_var variable.
 * If the parameter is not found a warning is displayed.
 *
 * @param param_name Parameter name.
 * @tparam param_var Variable where the parameter is saved.
 * @return Returns true/false depending on if the parameter has been found in the param server.
 */
template <typename T>
bool getParam(const std::string param_name, T& param_var,const rclcpp::Node::SharedPtr& node)
{
  // Display a message if a parameter is not found in the param server
  if (!node->get_parameter(param_name, param_var))
  {
    RCLCPP_WARN_STREAM(node->get_logger(), "Value for parameter " << param_name << " not found!");
    node->declare_parameter<T>(param_name);
    return false;
  }
  return true;
}

/**
 * @brief Reads the parameter param_name from the ROS param server into param_var variable.
 * If not found in param server, outputs a warning and sets the parameter to default_value.
 *
 * @param param_name Parameter name.
 * @tparam param_var Variable where the parameter is saved.
 * @tparam default_value Default value if parameter is not found in ROS param server.
 * @return Returns true/false depending on if the parameter has been found in the param server.
 */
template <typename T>
bool getParam(const std::string param_name, T& param_var, T default_value,const rclcpp::Node::SharedPtr& node)
{
  // Display a message if a parameter is not found in the param server
  if (!node->get_parameter(param_name, param_var))
  {
    RCLCPP_WARN_STREAM(node->get_logger(),"Value for parameter " << param_name << " not found in param server! Using default value "<< default_value);
    param_var = default_value;
    node->declare_parameter<T>(param_name, default_value);
    return false;
  }
  return true;
}

/**
 * @brief Reads the vector parameter param_name from the ROS param server into data vector.
 * If not found in param server, outputs a fatal message.
 *
 * @param param_name Parameter name.
 * @tparam data Vector wheter the parameter values are saved.
 * @return Returns true/false depending on if the parameter has been found in the param server.
 */
template <typename ParamType>
bool getParamVector(const std::string param_name, std::vector<ParamType>& data,const rclcpp::Node::SharedPtr& node)
{
  // Clear vector
  data.clear();
  // Take the param vector and copy it to a std::vector<ParamType>

  std::shared_ptr<rclcpp::SyncParametersClient> parameter_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  std::vector<rclcpp::Parameter> parameters = parameter_client->get_parameters({param_name});

  if (parameters.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Invalid parameters for %s in parameter server!", param_name.c_str());
    return false;
  }

    // Iterate over each parameter and add it to the vector
  for (const rclcpp::Parameter& parameter : parameters) {
    // Assuming ParamType is implicitly convertible from rclcpp::ParameterValue
    try {
      // Retrieve the parameter value using get_value<ParamType>()
      ParamType value = parameter.get_value<ParamType>();
      data.push_back(value);
    } catch (const rclcpp::ParameterTypeException& ex) {
      RCLCPP_ERROR(node->get_logger(), "Parameter type mismatch for %s: %s", param_name.c_str(), ex.what());
      return false;
    }
  }

  return true;
}
/** @} */
}  // namespace ros
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_ROS_PARAMLOADER_H_
