/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/this_node.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <algorithm>
#include <thread>

namespace cola2
{
namespace ros
{
std::string
DiagnosticHelper::boolToString(const bool value) const
{
  return std::string(value ? "true" : "false");
}

void
DiagnosticHelper::shrinkFrequencyBuffer(const double now)
{
  while ((!frequency_buffer_.empty()) && (now - frequency_buffer_[0] > frequency_buffer_time_limit_))
    frequency_buffer_.pop_front();
}

double
DiagnosticHelper::computeFrequency(const double now)
{
  shrinkFrequencyBuffer(now);
  if (frequency_buffer_.size() < std::max(static_cast<std::size_t>(2), frequency_buffer_min_data_))  // Not enough data
    return -1.0;
  if (now - frequency_buffer_.back() > frequency_buffer_min_time_)  // Data is getting old
    return -1.0;
  double dt = frequency_buffer_.back() - frequency_buffer_[0];
  if (dt < std::max(1e-3, frequency_buffer_min_time_))  // Not enough data
    return -1.0;
  return static_cast<double>(frequency_buffer_.size() - 1) / dt;
}

void
DiagnosticHelper::removeKeyValueImpl(const std::string& key)
{
  for (auto it = diagnostic_msg_.status[0].values.begin(); it != diagnostic_msg_.status[0].values.end(); ++it)
  {
    if (it->key.compare(key) == 0)
    {
      diagnostic_msg_.status[0].values.erase(it);
      break;
    }
  }
}

void
DiagnosticHelper::addKeyValueImpl(const std::string& key, const std::string& value)
{
  removeKeyValueImpl(key);
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = key;
  key_value.value = value;
  diagnostic_msg_.status[0].values.push_back(key_value);
}

DiagnosticHelper::DiagnosticHelper(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& hardware_id)
  : set_level_called_(false)
  , enabled_(false)
  , last_data_(0.0)
  , last_valid_data_(0.0)
  , frequency_buffer_time_limit_(20.0)
  , frequency_buffer_min_data_(5)
  , frequency_buffer_min_time_(2.0)
{
  diagnostic_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(node->get_namespace() + std::string("diagnostics"), 40);
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = hardware_id;
  diagnostic_msg_.status.push_back(status);
}

void
DiagnosticHelper::setFrequencyBufferTimeLimit(const double frequency_buffer_time_limit)
{
  std::lock_guard<std::mutex> guard(mtx_);
  frequency_buffer_time_limit_ = frequency_buffer_time_limit;
}

void
DiagnosticHelper::setFrequencyBufferMinData(const std::size_t frequency_buffer_min_data)
{
  std::lock_guard<std::mutex> guard(mtx_);
  frequency_buffer_min_data_ = frequency_buffer_min_data;
}

void
DiagnosticHelper::setFrequencyBufferMinTime(const double frequency_buffer_min_time)
{
  std::lock_guard<std::mutex> guard(mtx_);
  frequency_buffer_min_time_ = frequency_buffer_min_time;
}

void
DiagnosticHelper::setEnabled(const bool enabled)
{
  std::lock_guard<std::mutex> guard(mtx_);
  enabled_ = enabled;
}

void
DiagnosticHelper::reportData(const rclcpp::Time stamp)
{
  std::lock_guard<std::mutex> guard(mtx_);
  double now_sec = stamp.seconds();
  if (now_sec != 0)
    last_data_ = now_sec;
  else
    last_data_ = rclcpp::Clock().now().seconds();
  frequency_buffer_.push_back(last_data_);
  shrinkFrequencyBuffer(last_data_);
}

void
DiagnosticHelper::reportValidData(const rclcpp::Time stamp)
{
  std::lock_guard<std::mutex> guard(mtx_);
  double now_sec = stamp.seconds();
  if (now_sec != 0)
    last_valid_data_ = now_sec;
  else
    last_valid_data_ = rclcpp::Clock().now().seconds();
  last_data_ = last_valid_data_;
  frequency_buffer_.push_back(last_valid_data_);
  shrinkFrequencyBuffer(last_valid_data_);
}

void
DiagnosticHelper::addKeyValue(const std::string& key, const bool value)
{
  std::lock_guard<std::mutex> guard(mtx_);
  addKeyValueImpl(key, boolToString(value));
}

void
DiagnosticHelper::addKeyValue(const std::string& key, const double value)
{
  std::lock_guard<std::mutex> guard(mtx_);
  addKeyValueImpl(key, std::to_string(value));
}

void
DiagnosticHelper::addKeyValue(const std::string& key, const int value)
{
  std::lock_guard<std::mutex> guard(mtx_);
  addKeyValueImpl(key, std::to_string(value));
}

void
DiagnosticHelper::addKeyValue(const std::string& key, const char* value)
{
  std::lock_guard<std::mutex> guard(mtx_);
  addKeyValueImpl(key, std::string(value));
}

void
DiagnosticHelper::addKeyValue(const std::string& key, const std::string& value)
{
  std::lock_guard<std::mutex> guard(mtx_);
  addKeyValueImpl(key, value);
}

void
DiagnosticHelper::removeKeyValue(const std::string& key)
{
  std::lock_guard<std::mutex> guard(mtx_);
  removeKeyValueImpl(key);
}

void
DiagnosticHelper::setLevelAndMessage(const std::uint8_t level, const std::string& message)
{
  std::lock_guard<std::mutex> guard(mtx_);
  if (message.empty())
  {
    if (level == diagnostic_msgs::msg::DiagnosticStatus::OK)
    {
      // messages_.insert("Ok");
    }
    else if (level == diagnostic_msgs::msg::DiagnosticStatus::WARN)
      messages_.insert("Warn");
    else if (level == diagnostic_msgs::msg::DiagnosticStatus::ERROR)
      messages_.insert("Error");
    else
      messages_.insert("Unknown level");
  }
  else
  {
    messages_.insert(message);
  }
  diagnostic_msg_.status[0].level = std::max(static_cast<std::uint8_t>(diagnostic_msg_.status[0].level), level);
  set_level_called_ = true;
}

void
DiagnosticHelper::setMessage(const std::string& message)
{
  std::lock_guard<std::mutex> guard(mtx_);
  messages_.insert(message);
}

void
DiagnosticHelper::publish(const rclcpp::Node::SharedPtr node,const rclcpp::Time stamp)
{
  std::lock_guard<std::mutex> guard(mtx_);
  // Check if a level has been set
  if (!set_level_called_)
  {
     RCLCPP_WARN(node->get_logger(), "Diagnostic helper publish() is called without setting a level");
  }

  // Header stamp
  rclcpp::Time now = node->now();
  if (stamp.seconds() != 0)
    diagnostic_msg_.header.stamp = stamp;
  else
    diagnostic_msg_.header.stamp = now;

  // Enabled KeyValue
  addKeyValueImpl("enabled", boolToString(enabled_));

  // Messages
  std::string all_messages;
  for (const auto& message : messages_)
    all_messages += message + ". ";
  if (!all_messages.empty())
    all_messages = all_messages.substr(0, all_messages.size() - 2);
  else if (diagnostic_msg_.status[0].level == diagnostic_msgs::msg::DiagnosticStatus::OK)
    all_messages = "Ok";
  diagnostic_msg_.status[0].message = all_messages;

  // Add extra information if enabled
  if (enabled_)
  {
    if (last_data_ != 0.0)
      addKeyValueImpl("data_age", std::to_string(now.seconds() - last_data_));
    else
      removeKeyValueImpl("data_age");

    if (last_valid_data_ != 0.0)
      addKeyValueImpl("valid_data_age", std::to_string(now.seconds()- last_valid_data_));
    else
      removeKeyValueImpl("valid_data_age");

    const double frequency = computeFrequency(now.seconds());
    if (frequency >= 0.0)
      addKeyValueImpl("frequency", std::to_string(frequency));
    else
      removeKeyValueImpl("frequency");
  }

  // Publish
  diagnostic_pub_->publish(diagnostic_msg_);

  // Reset level and message
  diagnostic_msg_.status[0].level = 0;
  messages_.clear();
  set_level_called_ = false;
}
}  // namespace ros
}  // namespace cola2
