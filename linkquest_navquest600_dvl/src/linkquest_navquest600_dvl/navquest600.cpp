#include <linkquest_navquest600_dvl/navquest600.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

NavQuest600::NavQuest600()
{
    config_.sp_config.sp_char_size = 8;
    config_.sp_config.sp_stop_bits = 1;
    config_.sp_config.sp_parity = "NONE";
    config_.sp_config.sp_flow_control = "NONE";
    config_.sp_config.sp_timeout = 1000;
    config_.sp_config.sp_baud_rate = 9600;
    config_.output_period = -1.f;
    config_.vos = 0;
    config_.salinity = 35;
    config_.flip_up_down = false;
    status_.valid = false;
}



NavQuest600Config& NavQuest600::getConfig()
{
    return config_;
}

const NQ1& NavQuest600::getStatus()
{
    return status_;
}

void NavQuest600::invalidateStatus()
{
    status_.valid = false;
}

void NavQuest600::openSerialPort(const cola2::io::SPConfig& config)
{
    driver_io_ = std::unique_ptr<cola2::io::IOBase>(new cola2::io::SerialPort(config));
    driver_io_->open();
}

void NavQuest600::flushSerialPort()
{
    try
    {
        do { 
            driver_io_->readByte(10);
        } while(1);
    }
    catch(const std::exception& e) {}
}

uint8_t NavQuest600::checksum(const std::string& data)
{
    uint8_t chksum = 0;
    for(auto i = data.begin(); i != data.end(); ++i)
        chksum ^= static_cast<uint8_t>(*i);
    return chksum;
}

void NavQuest600::setCommand(uint8_t code, const std::string& data)
{
    std::stringstream command;
    command << "#&!LQNQ.COMD" << std::setfill('0') << std::setw(2) << (int)code << std::setw(2) << (int)code;
    if(data.size() > 0)
        command << " " << data << " " << std::to_string((int)checksum(data + " "));
    command << "\r\n";
    driver_io_->write(command.str());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    
    //std::cout << "[DRIVER] " << command.str();
}

void NavQuest600::configure()
{
    //Flush
    flushSerialPort();
    //Stop output
    setCommand(NQ_STOP);
    //Output mode and period
    if(config_.output_period > 0.f) //Continuous?
    {
        setCommand(SET_WORKING_MODE, "1"); //Continuous command mode
        std::stringstream period;
        period << std::setprecision(2) << std::fixed << config_.output_period;
        setCommand(SET_OUTPUT_PERIOD, period.str()); //Set period
    }
    else
    {
        setCommand(SET_WORKING_MODE, "2"); //Step mode
    }
    //Output format
    //setCommand(RESET_OUTPUT_FORMAT, "00"); //Reset all formats
    //setCommand(SET_OUTPUT_FORMAT, "01"); //Enable NQ1 output format
    //Velocity of sound
    if(config_.vos > 0)
    {
        setCommand(SET_VOS, std::to_string(config_.vos));// Set sound velocity
        setCommand(SET_ENABLE_VOS, "1"); //Enable using user defined sound velocity
    }
    else
    {
        setCommand(SET_ENABLE_VOS, "0"); //Disable using user defined sound velocity
        if(config_.salinity >= 0)
            setCommand(SET_SALINITY, std::to_string(config_.salinity)); // Set salinity
    }
    //Flip z axis ?
    setCommand(FLIP_UP_DOWN, std::to_string((int)config_.flip_up_down));
    //Restart output
    setCommand(NQ_START);
}

std::string NavQuest600::listen()
{
    std::string line = "";
    try
    {
        line = driver_io_->readLine(50);
    }
    catch(const std::exception& e)
    {
        ; //Handle errors other than timeout
    }

    //if(line.size() > 0) std::cout << "[DRIVER] " << line;

    if(line.size() == 0 || line.compare(0, 9, "$#NQ.RES ") != 0) // Skip broken messages
    {	
    	line = "No data received"+std::to_string(line.size());
        return line;
    }

    return line;
    std::stringstream msg(line.substr(9)); //Stripped header
    msg >> std::hex >> status_.error_code >> std::dec
        >> status_.good[0]
        >> status_.good[1]
        >> status_.good[2]
        >> status_.good[3]
        >> status_.v_altitude[0]
        >> status_.v_altitude[1]
        >> status_.v_altitude[2]
        >> status_.v_altitude[3]
        >> status_.velo_rad[0]
        >> status_.velo_rad[1]
        >> status_.velo_rad[2]
        >> status_.velo_rad[3]
        >> status_.wvelo_rad[0]
        >> status_.wvelo_rad[1]
        >> status_.wvelo_rad[2]
        >> status_.wvelo_rad[3]
        >> status_.wvelo_credit[0]
        >> status_.wvelo_credit[1]
        >> status_.wvelo_credit[2]
        >> status_.wvelo_credit[3]
        >> status_.velo_instrument[0]
        >> status_.velo_instrument[1]
        >> status_.velo_instrument[2]
        >> status_.velo_instrument_flag
        >> status_.velo_earth[0]
        >> status_.velo_earth[1]
        >> status_.velo_earth[2]
        >> status_.velo_earth_flag
        >> status_.water_velo_instrument[0]
        >> status_.water_velo_instrument[1]
        >> status_.water_velo_instrument[2]
        >> status_.water_velo_instrument_flag
        >> status_.water_velo_earth[0]
        >> status_.water_velo_earth[1]
        >> status_.water_velo_earth[2]
        >> status_.water_velo_earth_flag
        >> status_.rph[0]
        >> status_.rph[1]
        >> status_.rph[2]
        >> status_.altitude_estimate
        >> status_.temperature
        >> status_.pressure
        >> status_.salinity
        >> status_.sound_speed
        >> status_.checksum;

    uint8_t chksum = checksum(msg.str().substr(0, msg.str().size()-2-std::to_string(status_.checksum).size()));
    status_.valid = status_.checksum == chksum;
    
    return line;
}
