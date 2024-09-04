/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ Cola2 utilities to retrieve information about node namescapes.
 */

#ifndef THIS_NODE_H
#define THIS_NODE_H

#include <string>
#include "rclcpp/rclcpp.hpp"

namespace cola2
{
namespace ros
{
/**
 * @addtogroup NodeName
 * @{
 */

/**
 * @brief Returns the namespace of the node without double slashes.
 *
 * @return Namespace of the node.
 */
std::string getNamespace(const rclcpp::Node::SharedPtr& node);

/**
 * @brief Returns the namespace of the node without double slashes and without the leading dash (useful for frames).
 *
 * @return Namespace of the node.
 */
std::string getNamespaceNoInitialDash(const rclcpp::Node::SharedPtr& node);

/**
 * @brief  Returns the unresolved name of the node (without the namespace) if the node name is /swarm1/girona500/node
 * returns node.
 *
 * @return Unresolved name of the node.
 */
std::string getUnresolvedNodeName(const rclcpp::Node::SharedPtr& node);
/** @} */
}  // namespace ros
}  // namespace cola2

#endif  // THIS_NODE_H
