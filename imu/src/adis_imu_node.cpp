/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */


#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_lib/io/serial_port.h>
#include <cola2_msgs/msg/adis_imu.hpp>
#include <cola2_msgs/msg/imu_angle_estimator_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cstdint>
#include <deque>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

class AdisImu : public rclcpp::Node
{
private:
  // ROS
  rclcpp::Publisher<cola2_msgs::msg::AdisImu>::SharedPtr pub_adis_;
  rclcpp::Publisher<cola2_msgs::msg::AdisImu>::SharedPtr pub_adis_fast_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_pressure_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_pressure_fast_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_fluid_pressure_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_fluid_pressure_fast_;


  // Initial time
  double initial_time_;

  // Diagnostics
  std::shared_ptr<cola2::ros::DiagnosticHelper> diagnostic_imu_, diagnostic_pressure_;

  // Serial port
  std::shared_ptr<cola2::io::SerialPort> serial_port_;

  // Data buffer
  std::deque<unsigned char> buffer_;

  // This string stores raw serial port bytes
  std::string raw_;

  // Bytes to string lookup table
  std::vector<std::string> bytes_to_string_;

  // To trigger slow publication
  int counter_;

  // Device configuration
  struct
  {
    double max_pressure;
    double min_pressure;
    int dec_rate;
  } device_config_;

  // Serial port configuration
  cola2::io::SPConfig sp_config_;

  // Methods
  double parseBytes(const std::deque<unsigned char>&, const int, const double);
  std::string uCharToStringBits(const unsigned char);
  bool openSerialPort();
  void sendStartCommand();
  void sendConfiguration();
  void getConfig();

public:
  // Methods
  AdisImu();
  void init();
  void iterate();
};

AdisImu::AdisImu()
  : Node("adis_imu")
  , buffer_(33, 0)  // Length of the line to parse in bytes
  , counter_(0)
{

}

void AdisImu::init()
{
  initial_time_ = this->now().seconds();
  diagnostic_imu_ = std::make_shared<cola2::ros::DiagnosticHelper>(shared_from_this(), "imu", cola2::ros::getUnresolvedNodeName(shared_from_this()));
  diagnostic_pressure_ = std::make_shared<cola2::ros::DiagnosticHelper>(shared_from_this(), "pressure", cola2::ros::getUnresolvedNodeName(shared_from_this()));

  // Get configuration from rosparam server
  getConfig();
  
  // Bytes to string lookup table. This saves a lot of CPU
  for (int i = 0; i < 256; ++i)
  {
    std::stringstream s;
    s << std::setfill(' ') << std::setw(3) << i << " ";
    bytes_to_string_.push_back(s.str());
  }

  // Publishers
  pub_adis_ = this->create_publisher< cola2_msgs::msg::AdisImu>("data", 1);
  pub_adis_fast_ = this->create_publisher<cola2_msgs::msg::AdisImu>("data_fast", 1);
  pub_pressure_ = this->create_publisher<std_msgs::msg::Float32>("pressure", 1);
  pub_pressure_fast_ = this->create_publisher<std_msgs::msg::Float32>("pressure_fast", 1);
  pub_fluid_pressure_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("fluid_pressure", 1);
  pub_fluid_pressure_fast_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("fluid_pressure_fast", 1);

  // Open serial port, send config and start
  openSerialPort();
  sendConfiguration();
  sendStartCommand();

  // Show message
  diagnostic_imu_->setEnabled(true);
  diagnostic_pressure_->setEnabled(true);
  RCLCPP_INFO(this->get_logger(), "Initialized");
}

void AdisImu::iterate()
{
  // Get reading
  unsigned char reading;
  try
  {
    reading = serial_port_->readByte(100);

    // Store reading into raw buffer
    raw_.append(bytes_to_string_[reading]);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Error reading serial port: %s", ex.what());
    reading = 0;

    // Try to recover
    openSerialPort();
    sendConfiguration();
    sendStartCommand();
  }

  // Store reading into the buffer
  buffer_.push_back(reading);
  buffer_.pop_front();

  // Check for overrun
  if ((buffer_[0] == 'O') && (buffer_[1] == 'V') && (buffer_[2] == 'E') && (buffer_[3] == 'R') && (buffer_[4] == 'R') &&
      (buffer_[5] == 'U') && (buffer_[6] == 'N'))
  {
    // Avoid warning when the node is initializing
    if (this->now().seconds() - initial_time_ > 10.0)
    {
      RCLCPP_WARN_STREAM(this->get_logger(),"Overrun detected. Check configuration");
    }
  }

  // Check if ready to parse
  if ((buffer_[0] == 'N') && (buffer_[1] == 'e') && (buffer_[2] == 'w') && (buffer_[3] == ':') && (buffer_[32] == '\n'))
  {
    // Get time
    const rclcpp::Time now = this->now();

    // Diagnostics
    diagnostic_imu_->reportValidData(now);
    diagnostic_imu_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::OK);
    diagnostic_pressure_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::OK);

    // IMU sensor
    // gyro           -> lsb = 0.02 deg/s
    // accelerometers -> lsb = 0.8 mg
    // magnetometers  -> lsb = 0.1 mgauss
    // barometer      -> lsb = 40 ubar
    // temperature    -> lsb = 0.00565 deg celsius, bias = 25 deg celsius
    cola2_msgs::msg::AdisImu imu_msg;
    imu_msg.raw = raw_;
    imu_msg.mx = parseBytes(buffer_, 4, 0.1) * 0.001;
    imu_msg.my = parseBytes(buffer_, 6, 0.1) * 0.001;
    imu_msg.mz = parseBytes(buffer_, 8, 0.1) * 0.001;
    imu_msg.gx = parseBytes(buffer_, 10, 0.02) * M_PI / 180.0;
    imu_msg.gy = parseBytes(buffer_, 12, 0.02) * M_PI / 180.0;
    imu_msg.gz = parseBytes(buffer_, 14, 0.02) * M_PI / 180.0;
    imu_msg.ax = parseBytes(buffer_, 16, 0.8) * 0.001 * 9.80665;
    imu_msg.ay = parseBytes(buffer_, 18, 0.8) * 0.001 * 9.80665;
    imu_msg.az = parseBytes(buffer_, 20, 0.8) * 0.001 * 9.80665;
    imu_msg.t = parseBytes(buffer_, 22, 0.00565) + 25.0;
    imu_msg.b = parseBytes(buffer_, 24, 0.00004);
    imu_msg.f = "Highest -> " + uCharToStringBits(buffer_[26]) + " " + uCharToStringBits(buffer_[27]) + " <- Lowest";
    bool valid_pressure = true;
    if ((buffer_[28] == 0) && (buffer_[29] == 0))
    {
      imu_msg.e = -99.0;
      valid_pressure = false;
      diagnostic_pressure_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Invalid pressure");
    }
    else
    {
      // From pressure sensor datasheet
      const int value = (static_cast<int>(buffer_[28]) << 8) + static_cast<int>(buffer_[29]);
      imu_msg.e = device_config_.min_pressure + (static_cast<double>(value) - 16384.0) *
                                                    (device_config_.max_pressure - device_config_.min_pressure) /
                                                    32768.0;
      diagnostic_pressure_->reportValidData(now);
    }
    if ((buffer_[30] == 0) && (buffer_[31] == 0))
    {
      imu_msg.w = -99.0;
      valid_pressure = false;
      diagnostic_pressure_->setLevelAndMessage(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Invalid pressure");
    }
    else
    {
      // From pressure sensor datasheet
      const int value = (static_cast<int>(buffer_[30]) << 8) + static_cast<int>(buffer_[31]);
      imu_msg.w = 0.05 * static_cast<double>((value >> 4) - 24) - 50.0;
    }
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = cola2::ros::getNamespaceNoInitialDash(shared_from_this()) + std::string("/adis_imu");  // TODO: move frame id to yaml
    pub_adis_fast_->publish(imu_msg);

    // Depth sensor (old pressure message without tf!)
    std_msgs::msg::Float32 press_msg;
    sensor_msgs::msg::FluidPressure fluid_press_msg;
    if (valid_pressure)
    {
      press_msg.data = imu_msg.e;
      pub_pressure_fast_->publish(press_msg);

      // Depth sensor (new fluid pressure message)
      fluid_press_msg.header.stamp = now;
      fluid_press_msg.header.frame_id = cola2::ros::getNamespaceNoInitialDash(shared_from_this()) + std::string("/pressure");  // TODO: move frame id to yaml
      fluid_press_msg.fluid_pressure = imu_msg.e * 1e5;
      fluid_press_msg.variance = 1e6;  // TODO: move variance to yaml
      pub_fluid_pressure_fast_->publish(fluid_press_msg);
    }

    // Publish at lower rate
    if (++counter_ >= 6)
    {  // Hardcoded subsampling
      counter_ = 0;
      pub_adis_->publish(imu_msg);
      if (valid_pressure)
      {
        pub_pressure_->publish(press_msg);
        pub_fluid_pressure_->publish(fluid_press_msg);
      }
      diagnostic_imu_->publish(shared_from_this());
      diagnostic_pressure_->publish(shared_from_this());
    }

    // Clear raw buffer
    raw_.clear();
    raw_.reserve(150);
  }
}

double AdisImu::parseBytes(const std::deque<unsigned char>& buffer, const int index, const double lsb)
{
  std::uint16_t bytes;
  bytes = static_cast<std::uint16_t>(buffer[index]) << 8;
  bytes += static_cast<std::uint16_t>(buffer[index + 1]);

  // Two's complement parsing
  if ((bytes & 0x8000) == 0)
  {  // Positive
    return static_cast<double>(bytes) * lsb;
  }
  else
  {  // Negative
    std::uint16_t tmp;
    tmp = bytes ^= 0xFFFF;
    return -static_cast<double>(++tmp) * lsb;
  }
}

std::string AdisImu::uCharToStringBits(const unsigned char input)
{
  // Converts unsigned char to string containing corresponding bits
  std::string output;
  for (int i = 7; i >= 0; --i)
    output += ((((input >> i) & 0x01) == 1) ? "1" : "0");
  return output;
}

bool AdisImu::openSerialPort()
{
  serial_port_ = std::make_shared<cola2::io::SerialPort>(sp_config_);
  try
  {
    serial_port_->open();
    serial_port_->configure();
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Error setting up the serial port: %s", ex.what());
  }
  return serial_port_->isOpen();
}

void AdisImu::sendStartCommand()
{
  try
  {
    serial_port_->write("C0\r");  // Start
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Error sending start command: %s", ex.what());
  }
  rclcpp::sleep_for(std::chrono::milliseconds(200));
}

void AdisImu::sendConfiguration()
{
  try
  {
    serial_port_->write("C1\r");  // Stop
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    std::stringstream s;
    s << std::setfill('0') << std::setw(4) << device_config_.dec_rate;
    serial_port_->write("F" + s.str() + "\r");  // Stop
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    serial_port_->write("MB\r");  // Binary mode
    RCLCPP_INFO(this->get_logger(), "Frequency set to %f hz", 2460.0 / (device_config_.dec_rate + 1));
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Error sending configuration: %s", ex.what());
  }
  rclcpp::sleep_for(std::chrono::milliseconds(200));
}

void AdisImu::getConfig()
{
  this->declare_parameter<double>("max_pressure", 100.0);
  this->declare_parameter<double>("min_pressure", 0.0);
  this->declare_parameter<int>("dec_rate", 1);
  this->declare_parameter<std::string>("serial_port.path", "/dev/ttyACM0");
  this->declare_parameter<int>("serial_port.baud_rate", 9600);
  this->declare_parameter<int>("serial_port.char_size", 8);
  this->declare_parameter<int>("serial_port.stop_bits", 1);
  this->declare_parameter<std::string>("serial_port.parity", "NONE");
  this->declare_parameter<std::string>("serial_port.flow_control", "NONE");
  this->declare_parameter<int>("serial_port.timeout", 10000);

  // Retrieve the parameters
  bool ok = true;

  ok &= this->get_parameter("max_pressure", device_config_.max_pressure);
  ok &= this->get_parameter("min_pressure", device_config_.min_pressure);
  ok &= this->get_parameter("dec_rate", device_config_.dec_rate);
  ok &= this->get_parameter("serial_port.path", sp_config_.sp_path);
  ok &= this->get_parameter("serial_port.baud_rate", sp_config_.sp_baud_rate);
  ok &= this->get_parameter("serial_port.char_size", sp_config_.sp_char_size);
  ok &= this->get_parameter("serial_port.stop_bits", sp_config_.sp_stop_bits);
  ok &= this->get_parameter("serial_port.parity", sp_config_.sp_parity);
  ok &= this->get_parameter("serial_port.flow_control", sp_config_.sp_flow_control);
  ok &= this->get_parameter("serial_port.timeout", sp_config_.sp_timeout);


  // Shutdown if not valid
  if (!ok)
  {
    RCLCPP_FATAL_STREAM(this->get_logger(),"Shutdown due to invalid config parameters!");
    rclcpp::shutdown();
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<AdisImu> node = std::make_shared<AdisImu>();
  node->init();
  rclcpp::Rate rate(50);
  while (rclcpp::ok())
  {
    node->iterate();
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
