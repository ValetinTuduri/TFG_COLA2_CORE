/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef _STM32_H_
#define _STM32_H_

#include <cola2_lib/io/nmea_parser.h>
#include <cola2_lib/io/serial_port.h>
#include <cola2_lib/io/tcp_socket.h>
#include <unistd.h>
#include <string>




/**
 * \brief Class that parses data coming from a NMEA GPS
 *
 * Can handle GPS devices connected through serial or TCP/IP
 */
class STM32
{
public:
  STM32();

  /**
   * Initializer for serial connection
   */
  void init(const cola2::io::SPConfig& sp);

  /**
   * Initializer for ethernet connection
   */
  void init(const cola2::io::TCPConfig& tcp);

  /**
   * Sends a configuration string to the GPS
   */
  void configure(const std::string& s);

  void write(const std::string& s);

private:
  /**
   * Pointer to transparently do I/O operations regardless of the connection type
   */
  std::unique_ptr<cola2::io::IOBase> driver_io_;

  /**
   * To flag when data is ready to be sent (at least a GGPGA sentence must have arrived)
   */
  bool data_ready_;
};

#endif  //_STM32_H_
