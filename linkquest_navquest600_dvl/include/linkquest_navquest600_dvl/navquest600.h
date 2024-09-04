/*
*   Copyright (c) 2021 CIRS UdG
*
*   Author: Patryk Cieslak
*/

#ifndef NAVQUEST600_H_
#define NAVQUEST600_H_

#include <cola2_lib/io/serial_port.h>

struct NavQuest600Config
{
    cola2::io::SPConfig sp_config;   
    float output_period;
    int salinity;
    int vos;
    bool flip_up_down;
};

struct NQ1
{  
    uint16_t error_code;
    bool good[4];
    float v_altitude[4];
    float velo_rad[4];
    float wvelo_rad[4];
    uint8_t wvelo_credit[4];
    float velo_instrument[3];
    uint8_t velo_instrument_flag;
    float velo_earth[3];
    uint8_t velo_earth_flag;
    float water_velo_instrument[3];
    uint8_t water_velo_instrument_flag;
    float water_velo_earth[3];
    uint8_t water_velo_earth_flag;
    float rph[3];
    float altitude_estimate;
    float temperature;
    float pressure; // INVALID for this model
    float salinity;
    uint16_t sound_speed;
    uint16_t checksum;
    bool valid;
};

//Commands
#define SET_OUTPUT_PERIOD   1
#define GET_OUTPUT_PERIOD   2
#define SET_OUTPUT_FORMAT   5
#define GET_OUTPUT_FORMAT   6
#define FLIP_UP_DOWN        10
#define RESET_OUTPUT_FORMAT 16
#define SET_BAUDRATE        23
#define RESET               24 //?
#define NQ_START            25
#define SET_SALINITY        26
#define GET_SALINITY        27 //?
#define NQ_STOP             28
#define SET_WORKING_MODE    29
#define GET_WORKING_MODE    30
#define SET_VOS             55
#define GET_VOS             56
#define STEP_WORKING_CMD    59
#define SET_ENABLE_VOS      65
#define GET_ENABLE_VOS      66
#define SET_DEPTH           73

//Class implementing the driver
class NavQuest600
{
public:
    NavQuest600();


    void openSerialPort(const cola2::io::SPConfig&);
    void flushSerialPort();
    void configure();
    std::string listen();
    const NQ1& getStatus();
    void invalidateStatus();
    NavQuest600Config& getConfig();

private:
    void setCommand(uint8_t code, const std::string& data = "");
    uint8_t checksum(const std::string& data);
     std::unique_ptr<cola2::io::IOBase> driver_io_;
    NavQuest600Config config_;
    NQ1 status_;
};

#endif
