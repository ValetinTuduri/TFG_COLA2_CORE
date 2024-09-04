/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "stm32/stm32.h"
#include <cola2_lib/utils/angles.h>

STM32::STM32()
{
}


void STM32::init(const cola2::io::SPConfig& config)
{
  // Setup Serial port connection
  driver_io_ = std::unique_ptr<cola2::io::IOBase>(new cola2::io::SerialPort(config));
  driver_io_->open();
}

void STM32::init(const cola2::io::TCPConfig& config)
{
  driver_io_ = std::unique_ptr<cola2::io::IOBase>(new cola2::io::TcpSocket(config));
  driver_io_->open();
}

void STM32::configure(const std::string& command)
{
  driver_io_->write(command);
  sleep(2.0);
}

void STM32::write(const std::string& s)
{
  driver_io_->write(s);
  sleep(2.0);
}


