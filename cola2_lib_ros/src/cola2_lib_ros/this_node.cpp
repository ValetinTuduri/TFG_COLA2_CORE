/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib_ros/this_node.h"
#include "rclcpp/rclcpp.hpp"

std::string cola2::ros::getNamespace(const rclcpp::Node::SharedPtr& node)
{
  std::string ns = node->get_name();

  std::size_t pos = ns.find("//");
  while (pos != std::string::npos)
  {
    ns.replace(pos, 2, "/");
    pos = ns.find("//");
  }
  return ns;
}

std::string cola2::ros::getNamespaceNoInitialDash(const rclcpp::Node::SharedPtr& node)
{
  const std::string ns = getNamespace(node);
  if(ns[0] == '/')
  {
    return ns.substr(1, ns.size() - 1);  // skip first character
  }

  return ns;
  
}

std::string cola2::ros::getUnresolvedNodeName(const rclcpp::Node::SharedPtr& node)
{
  std::string node_name = node->get_name();
  std::size_t pos = node_name.find_last_of("/");
  return node_name.substr(pos + 1);
}
