/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#ifndef _NMEA_GPS_H_
#define _NMEA_GPS_H_

#include <cola2_lib/io/nmea_parser.h>
#include <cola2_lib/io/serial_port.h>
#include <cola2_lib/io/tcp_socket.h>
#include <unistd.h>
#include <string>

//! Struct to store gps information gathered from different NMEA sentences
class GPSData
{
public:
  std::string time_utc;               // from GPGGA
  double latitude;                    // from GPGGA
  double longitude;                   // from GPGGA
  unsigned int fix_quality;           // from GPGGA
  unsigned int number_of_satellites;  // from GPGGA
  double hdop;                        // from GPGGA
  double altitude;                    // from GPGGA
  double geoidal_separation;          // from GPGGA
  double heading;                     // from GPHDT
  double latitude_stdev;              // from GPGST
  double longitude_stdev;             // from GPGST
  double altitude_stdev;              // from GPGST
  std::string raw_gpgga;              // from GPGGA
  std::string raw_gphdt;              // from GPHDT
  std::string raw_gpgst;              // from GPGST
  GPSData();
};

/**
 * \brief Class that parses data coming from a NMEA GPS
 *
 * Can handle GPS devices connected through serial or TCP/IP
 */
class NMEAGPS
{
public:
  NMEAGPS();

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

  /**
   * Reads all the sentences that are received in a burst from
   * the GPS device and returns a filled GPSData structure
   */
  GPSData getData();

  /**
   *  Parses a NMEA sentence and fills corresponding data on
   *  the GPSData structure according to the sentence header
   */
  bool parseReading(const std::string& s);

private:
  /**
   * Pointer to transparently do I/O operations regardless of the connection type
   */
  std::unique_ptr<cola2::io::IOBase> driver_io_;

  /**
   * Parser for NMEA-type sentences
   */
  std::unique_ptr<NMEAParser> parser_;

  /**
   * To store the GPS information that's being read
   */
  GPSData gps_data_;

  /**
   * To flag when data is ready to be sent (at least a GGPGA sentence must have arrived)
   */
  bool data_ready_;
};

#endif  //_NMEA_GPS_H_
