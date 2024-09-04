#include <rclcpp/rclcpp.hpp>
#include <cola2_lib_ros/diagnostic_helper.h>
#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/this_node.h>
#include <linkquest_navquest600_dvl/navquest600.h>
#include <cola2_msgs/msg/dvl.hpp>

class NavQuest600ROS : public rclcpp::Node
{
public:
    NavQuest600ROS();

    void getParams();
    void openDriverSerialPort(NavQuest600Config config);
    void configureDriver();
    void iterate();

private:
    rclcpp::Publisher<cola2_msgs::msg::DVL>::SharedPtr pub_dvl_;
    rclcpp::Publisher<cola2_msgs::msg::DVL>::SharedPtr pub_dvl_water_;
    std::string frame_id_;
    NavQuest600 dev_;
    NavQuest600Config config;
};

NavQuest600ROS::NavQuest600ROS() : Node("linkquest_navquest600_dvl")
{

    // Get config from param server
    getParams();

    // Open serial port and configure driver
    openDriverSerialPort(config);
    configureDriver();


    pub_dvl_ = this->create_publisher<cola2_msgs::msg::DVL>("bottom_velocity", 1);
    pub_dvl_water_ = this->create_publisher<cola2_msgs::msg::DVL>("water_velocity", 1);

    RCLCPP_INFO(this->get_logger(), "Initialized.");
}

void NavQuest600ROS::getParams()
{
    config = dev_.getConfig();
   
    // clang-format off
    this->get_parameter_or("~frame_id", frame_id_, std::string("linkquest_navquest600_dvl"));
    this->get_parameter_or("~sp_path", config.sp_config.sp_path, std::string("/dev/ttyACM0"));
    this->get_parameter_or("~sp_baud_rate", config.sp_config.sp_baud_rate, 9600);
    this->get_parameter_or("~output_period", config.output_period, -1.f);
    this->get_parameter_or("~velocity_of_sound", config.vos, 0);
    this->get_parameter_or("~salinity", config.salinity, 35);
    this->get_parameter_or("~flip_up_down", config.flip_up_down, false);
    // clang-format on
}

void NavQuest600ROS::openDriverSerialPort(NavQuest600Config config)
{
    try
    {
        dev_.openSerialPort(config.sp_config);
    }
    catch (const std::exception& ex)
    {
        RCLCPP_FATAL(this->get_logger(), "Unable to open serial port!");
        exit(0);
    }
}

void NavQuest600ROS::configureDriver()
{
    try
    {
        dev_.configure();
    }
    catch (const std::runtime_error& ex)
    {
        RCLCPP_FATAL(this->get_logger(), "Unable to configure driver: %s", ex.what());
        exit(0);
    }
    catch (const std::logic_error& ex)
    {
        RCLCPP_WARN(this->get_logger(), ex.what());
    }
}

void NavQuest600ROS::iterate()
{
    std::string line = dev_.listen();
    RCLCPP_INFO(this->get_logger(), "I got %s", line.c_str());
    const NQ1& status = dev_.getStatus();
    
    if(status.valid)
    {
        cola2_msgs::msg::DVL msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_;
        msg.altitude = status.altitude_estimate;
        msg.velocity.x = status.velo_instrument[0];
        msg.velocity.y = status.velo_instrument[1];
        msg.velocity.z = status.velo_instrument[2];
        msg.beams.resize(4);
        msg.beams[0].range = status.v_altitude[0];
        msg.beams[0].velocity = status.velo_rad[0];
        msg.beams[1].range = status.v_altitude[1];
        msg.beams[1].velocity = status.velo_rad[1];
        msg.beams[2].range = status.v_altitude[2];
        msg.beams[2].velocity = status.velo_rad[2];
        msg.beams[3].range = status.v_altitude[3];
        msg.beams[3].velocity = status.velo_rad[3];
        for(int i=0; i<4; ++i)
        {
            if(!status.good[i])
            {
                msg.beams[i].range_covariance = -1;
                msg.beams[i].velocity_covariance = -1;
            }
        }
        pub_dvl_->publish(msg);

        msg.velocity.x = status.water_velo_instrument[0];
        msg.velocity.y = status.water_velo_instrument[1];
        msg.velocity.z = status.water_velo_instrument[2];
        msg.beams[0].velocity = status.wvelo_rad[0];
        msg.beams[1].velocity = status.wvelo_rad[1];
        msg.beams[2].velocity = status.wvelo_rad[2];
        msg.beams[3].velocity = status.wvelo_rad[3];
        for(int i=0; i<4; ++i)
            if(status.good[i])
            {
                if(status.wvelo_credit[i] == 0)
                    msg.beams[i].velocity_covariance = -1;
                else
                    msg.beams[i].velocity_covariance = 50-status.wvelo_credit[i];
            }
        pub_dvl_water_->publish(msg);

        dev_.invalidateStatus();
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<NavQuest600ROS> dvl = std::make_shared<NavQuest600ROS>();

    rclcpp::executors::SingleThreadedExecutor executor;
    
    while(rclcpp::ok())
    {
        dvl->iterate();
        executor.spin_some();
    }


    rclcpp::shutdown();
    return 0;
}
