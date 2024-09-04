/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */


#include <cola2_control/low_level_controllers/only_thrusters_controller.h>
#include <cola2_control/ros_controller/auv_ros_controller_base.h>
#include <cola2_lib_ros/param_loader.h>
#include "cola2_msgs/msg/goal_descriptor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>

class OnlyThrustersROSController : public rclcpp::Node, public IAUVROSController 
{
private:
  // AUV controller ptr.
  std::shared_ptr<OnlyThrustersController> auv_controller_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // Helper function to declare PID controller parameters
  void declare_pid_parameters(const std::string &name, double kp, double ti, double td, double i_limit, double fff)
  {
        this->declare_parameter<double>(name + "_kp", kp);
        this->declare_parameter<double>(name + "_ti", ti);
        this->declare_parameter<double>(name + "_td", td);
        this->declare_parameter<double>(name + "_i_limit", i_limit);
        this->declare_parameter<double>(name + "_fff", fff);
  }

  // Helper function to declare polynomial model parameters
  void declare_poly_parameters(const std::string &name, double A, double B, double C)
  {
        this->declare_parameter<double>(name + "_A", A);
        this->declare_parameter<double>(name + "_B", B);
        this->declare_parameter<double>(name + "_C", C);
  }


public:
  /**
   * Class constructor
   * @param name Node name
   * @param frame_id Frame id in which messages must be published
   */
  OnlyThrustersROSController(const std::string name, const std::string frame_id) : Node("controller"),IAUVROSController(name, frame_id)
  {
    // Declare basic parameters
     this->declare_parameter<double>("period", 0.1);
     this->declare_parameter<int>("n_thrusters", 5);

     // Declare the Thruster Control Matrix (TCM)
     this->declare_parameter<std::vector<double>>("TCM", {-1.0, -1.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5587, -0.5587, 0.0, -0.2432, 0.2432, 0.0, 0.0, 0.0});
     for (int i = 1; i <= 5; ++i)
     {
        this->declare_parameter<std::vector<double>>("thruster_" + std::to_string(i) + "_poly_positive",{0.0473235, 0.063145, -0.00256629, 0.000052432528, -0.00000050454768, 0.000000001842152});
        this->declare_parameter<std::vector<double>>("thruster_" + std::to_string(i) + "_poly_negative",{0.04997604, 0.05975017, -0.0022952, 0.0000439588388, -0.00000039543457, 0.00000000134487});
        this->declare_parameter<double>("thruster_" + std::to_string(i) + "_max_force_positive", 95.122);
        this->declare_parameter<double>("thruster_" + std::to_string(i) + "_max_force_negative", 104.659);
     }

     // Enable thrusters
     this->declare_parameter<bool>("enable_thrusters", false);

     // Maximum velocity for each DOF
     this->declare_parameter<double>("max_velocity_x", 0.5);
     this->declare_parameter<double>("max_velocity_y", 0.25);
     this->declare_parameter<double>("max_velocity_z", 0.4);
     this->declare_parameter<double>("max_velocity_roll", 0.0);
     this->declare_parameter<double>("max_velocity_pitch", 0.0);
     this->declare_parameter<double>("max_velocity_yaw", 0.3);

     this->declare_parameter<double>("set_zero_velocity_depth", 1.0);
     this->declare_parameter<std::vector<bool>>("set_zero_velocity_axes", {true, false, true, false, false, true});

     // Pose PID controller parameters
     declare_pid_parameters("p_surge", 0.6, 0.0, 0.0, 0.0, 0.0);
     declare_pid_parameters("p_sway", 1.2, 0.0, 0.0, 0.0, 0.0);
     declare_pid_parameters("p_heave", 0.8, 0.0, 0.0, 0.0, 0.0);
     declare_pid_parameters("p_roll", 0.0, 0.0, 0.0, 0.0, 0.0);
     declare_pid_parameters("p_pitch", 0.0, 0.0, 0.0, 0.0, 0.0);
     declare_pid_parameters("p_yaw", 1.5, 0.0, 0.5, 0.0, 0.0);

     // Twist (velocity) PID controller parameters
     declare_pid_parameters("t_surge", 1.2, 10.0, 0.0, 0.15, 0.0);
     declare_pid_parameters("t_sway", 4.0, 10.0, 0.0, 0.1, 0.0);
     declare_pid_parameters("t_heave", 2.0, 10.0, 0.0, 0.1, 0.05);
     declare_pid_parameters("t_roll", 0.0, 0.0, 0.0, 0.0, 0.0);
     declare_pid_parameters("t_pitch", 0.0, 0.0, 0.0, 0.0, 0.0);
     declare_pid_parameters("t_yaw", 1.2, 10.0, 0.0, 0.1, 0.0);

     // Feed-forward polynomial model parameters
     declare_poly_parameters("poly_surge", 0.0, 40.0, 163.0);
     declare_poly_parameters("poly_sway", 0.0, 40.0, 600.0);
     declare_poly_parameters("poly_heave", 0.0, 40.0, 600.0);
     declare_poly_parameters("poly_roll", 0.0, 0.0, 0.0);
     declare_poly_parameters("poly_pitch", 0.0, 0.0, 0.0);
     declare_poly_parameters("poly_yaw", 0.0, 10.0, 80.0);

     // Maximum wrench (force) for each DOF
     this->declare_parameter<double>("max_wrench_X", 210.0);
     this->declare_parameter<double>("max_wrench_Y", 105.0);
     this->declare_parameter<double>("max_wrench_Z", 210.0);
     this->declare_parameter<double>("max_wrench_Roll", 0.0);
     this->declare_parameter<double>("max_wrench_Pitch", 0.0);
     this->declare_parameter<double>("max_wrench_Yaw", 46.267);

  


     parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&OnlyThrustersROSController::setParams, this, std::placeholders::_1));
  }

  /**
   * Initialize the class OnlyThrustersROSController witha a std::shared_ptr<OnlyThrustersController>
   * @param auv_controller_ptr real C++ OnlyThrusters controller
   * @param period period time in seconds (e.g., 10Hz -> period = 0.1)
   */
  void init(std::shared_ptr<OnlyThrustersController> auv_controller_ptr, const double& period)
  {
    initBase(shared_from_this(),auv_controller_ptr, period);

    // Init pointer to AUV controller
    auv_controller_ = auv_controller_ptr;

  }

  /**
   * Callback for the dynamic reconfigure function to change the controller parameters.
   * @param config
   * @param level
   */
  rcl_interfaces::msg::SetParametersResult setParams(const std::vector<rclcpp::Parameter>)
  {

    rcl_interfaces::msg::SetParametersResult result;
        
    result.successful = true;//TODO: Check if results are succcessful
    
    std::vector<std::map<std::string, double>> p_params;
    std::vector<std::string> keys = {"kp", "ti", "td", "i_limit", "fff"};
  
    double p_surge_kp, p_surge_ti, p_surge_td, p_surge_i_limit, p_surge_fff;

    this->get_parameter("p_surge_kp", p_surge_kp);
    this->get_parameter("p_surge_ti", p_surge_ti);
    this->get_parameter("p_surge_td", p_surge_td);
    this->get_parameter("p_surge_i_limit", p_surge_i_limit);
    this->get_parameter("p_surge_fff", p_surge_fff);

    std::vector<double> values1 = {p_surge_kp, p_surge_ti, p_surge_td, p_surge_i_limit, p_surge_fff};

    auv_controller_->addPIDParamToVector(keys, values1, p_params);

    // Retrieve and store parameters for values2
    double p_sway_kp, p_sway_ti, p_sway_td, p_sway_i_limit, p_sway_fff;
    this->get_parameter("p_sway_kp", p_sway_kp);
    this->get_parameter("p_sway_ti", p_sway_ti);
    this->get_parameter("p_sway_td", p_sway_td);
    this->get_parameter("p_sway_i_limit", p_sway_i_limit);
    this->get_parameter("p_sway_fff", p_sway_fff);
    std::vector<double> values2 = {p_sway_kp, p_sway_ti, p_sway_td, p_sway_i_limit, p_sway_fff};
    auv_controller_->addPIDParamToVector(keys, values2, p_params);

    // Retrieve and store parameters for values3
    double p_heave_kp, p_heave_ti, p_heave_td, p_heave_i_limit, p_heave_fff;
    this->get_parameter("p_heave_kp", p_heave_kp);
    this->get_parameter("p_heave_ti", p_heave_ti);
    this->get_parameter("p_heave_td", p_heave_td);
    this->get_parameter("p_heave_i_limit", p_heave_i_limit);
    this->get_parameter("p_heave_fff", p_heave_fff);
    std::vector<double> values3 = {p_heave_kp, p_heave_ti, p_heave_td, p_heave_i_limit, p_heave_fff};
    auv_controller_->addPIDParamToVector(keys, values3, p_params);

    // Retrieve and store parameters for values4
    double p_roll_kp, p_roll_ti, p_roll_td, p_roll_i_limit, p_roll_fff;
    this->get_parameter("p_roll_kp", p_roll_kp);
    this->get_parameter("p_roll_ti", p_roll_ti);
    this->get_parameter("p_roll_td", p_roll_td);
    this->get_parameter("p_roll_i_limit", p_roll_i_limit);
    this->get_parameter("p_roll_fff", p_roll_fff);
    std::vector<double> values4 = {p_roll_kp, p_roll_ti, p_roll_td, p_roll_i_limit, p_roll_fff};
    auv_controller_->addPIDParamToVector(keys, values4, p_params);

    // Retrieve and store parameters for values5
    double p_pitch_kp, p_pitch_ti, p_pitch_td, p_pitch_i_limit, p_pitch_fff;
    this->get_parameter("p_pitch_kp", p_pitch_kp);
    this->get_parameter("p_pitch_ti", p_pitch_ti);
    this->get_parameter("p_pitch_td", p_pitch_td);
    this->get_parameter("p_pitch_i_limit", p_pitch_i_limit);
    this->get_parameter("p_pitch_fff", p_pitch_fff);
    std::vector<double> values5 = {p_pitch_kp, p_pitch_ti, p_pitch_td, p_pitch_i_limit, p_pitch_fff};
    auv_controller_->addPIDParamToVector(keys, values5, p_params);

    // Retrieve and store parameters for values6
    double p_yaw_kp, p_yaw_ti, p_yaw_td, p_yaw_i_limit, p_yaw_fff;
    this->get_parameter("p_yaw_kp", p_yaw_kp);
    this->get_parameter("p_yaw_ti", p_yaw_ti);
    this->get_parameter("p_yaw_td", p_yaw_td);
    this->get_parameter("p_yaw_i_limit", p_yaw_i_limit);
    this->get_parameter("p_yaw_fff", p_yaw_fff);
    std::vector<double> values6 = {p_yaw_kp, p_yaw_ti, p_yaw_td, p_yaw_i_limit, p_yaw_fff};
    auv_controller_->addPIDParamToVector(keys, values6, p_params);


    std::vector<std::map<std::string, double> > t_params;
    // Retrieve and store parameters for tvalues1
    double t_surge_kp, t_surge_ti, t_surge_td, t_surge_i_limit, t_surge_fff;
    this->get_parameter("t_surge_kp", t_surge_kp);
    this->get_parameter("t_surge_ti", t_surge_ti);
    this->get_parameter("t_surge_td", t_surge_td);
    this->get_parameter("t_surge_i_limit", t_surge_i_limit);
    this->get_parameter("t_surge_fff", t_surge_fff);
    std::vector<double> tvalues1 = {t_surge_kp, t_surge_ti, t_surge_td, t_surge_i_limit, t_surge_fff};
    auv_controller_->addPIDParamToVector(keys, tvalues1, t_params);

    // Retrieve and store parameters for tvalues2
    double t_sway_kp, t_sway_ti, t_sway_td, t_sway_i_limit, t_sway_fff;
    this->get_parameter("t_sway_kp", t_sway_kp);
    this->get_parameter("t_sway_ti", t_sway_ti);
    this->get_parameter("t_sway_td", t_sway_td);
    this->get_parameter("t_sway_i_limit", t_sway_i_limit);
    this->get_parameter("t_sway_fff", t_sway_fff);
    std::vector<double> tvalues2 = {t_sway_kp, t_sway_ti, t_sway_td, t_sway_i_limit, t_sway_fff};
    auv_controller_->addPIDParamToVector(keys, tvalues2, t_params);

    // Retrieve and store parameters for tvalues3
    double t_heave_kp, t_heave_ti, t_heave_td, t_heave_i_limit, t_heave_fff;
    this->get_parameter("t_heave_kp", t_heave_kp);
    this->get_parameter("t_heave_ti", t_heave_ti);
    this->get_parameter("t_heave_td", t_heave_td);
    this->get_parameter("t_heave_i_limit", t_heave_i_limit);
    this->get_parameter("t_heave_fff", t_heave_fff);
    std::vector<double> tvalues3 = {t_heave_kp, t_heave_ti, t_heave_td, t_heave_i_limit, t_heave_fff};
    auv_controller_->addPIDParamToVector(keys, tvalues3, t_params);

    // Retrieve and store parameters for tvalues4
    double t_roll_kp, t_roll_ti, t_roll_td, t_roll_i_limit, t_roll_fff;
    this->get_parameter("t_roll_kp", t_roll_kp);
    this->get_parameter("t_roll_ti", t_roll_ti);
    this->get_parameter("t_roll_td", t_roll_td);
    this->get_parameter("t_roll_i_limit", t_roll_i_limit);
    this->get_parameter("t_roll_fff", t_roll_fff);
    std::vector<double> tvalues4 = {t_roll_kp, t_roll_ti, t_roll_td, t_roll_i_limit, t_roll_fff};
    auv_controller_->addPIDParamToVector(keys, tvalues4, t_params);

    // Retrieve and store parameters for tvalues5
    double t_pitch_kp, t_pitch_ti, t_pitch_td, t_pitch_i_limit, t_pitch_fff;
    this->get_parameter("t_pitch_kp", t_pitch_kp);
    this->get_parameter("t_pitch_ti", t_pitch_ti);
    this->get_parameter("t_pitch_td", t_pitch_td);
    this->get_parameter("t_pitch_i_limit", t_pitch_i_limit);
    this->get_parameter("t_pitch_fff", t_pitch_fff);
    std::vector<double> tvalues5 = {t_pitch_kp, t_pitch_ti, t_pitch_td, t_pitch_i_limit, t_pitch_fff};
    auv_controller_->addPIDParamToVector(keys, tvalues5, t_params);

    // Retrieve and store parameters for tvalues6
    double t_yaw_kp, t_yaw_ti, t_yaw_td, t_yaw_i_limit, t_yaw_fff;
    this->get_parameter("t_yaw_kp", t_yaw_kp);
    this->get_parameter("t_yaw_ti", t_yaw_ti);
    this->get_parameter("t_yaw_td", t_yaw_td);
    this->get_parameter("t_yaw_i_limit", t_yaw_i_limit);
    this->get_parameter("t_yaw_fff", t_yaw_fff);
    std::vector<double> tvalues6 = {t_yaw_kp, t_yaw_ti, t_yaw_td, t_yaw_i_limit, t_yaw_fff};
    auv_controller_->addPIDParamToVector(keys, tvalues6, t_params);

    std::vector<std::map<std::string, double> > poly_params;
    // Retrieve and store parameters for polynomial params (poly_params)
    double poly_surge_A, poly_surge_B, poly_surge_C;
    this->get_parameter("poly_surge_A", poly_surge_A);
    this->get_parameter("poly_surge_B", poly_surge_B);
    this->get_parameter("poly_surge_C", poly_surge_C);
    std::vector<double> pvalues1 = {poly_surge_A, poly_surge_B, poly_surge_C};
    auv_controller_->addPolyParamToVector(pvalues1, poly_params);

    double poly_sway_A, poly_sway_B, poly_sway_C;
    this->get_parameter("poly_sway_A", poly_sway_A);
    this->get_parameter("poly_sway_B", poly_sway_B);
    this->get_parameter("poly_sway_C", poly_sway_C);
    std::vector<double> pvalues2 = {poly_sway_A, poly_sway_B, poly_sway_C};
    auv_controller_->addPolyParamToVector(pvalues2, poly_params);

    double poly_heave_A, poly_heave_B, poly_heave_C;
    this->get_parameter("poly_heave_A", poly_heave_A);
    this->get_parameter("poly_heave_B", poly_heave_B);
    this->get_parameter("poly_heave_C", poly_heave_C);
    std::vector<double> pvalues3 = {poly_heave_A, poly_heave_B, poly_heave_C};
    auv_controller_->addPolyParamToVector(pvalues3, poly_params);

    double poly_roll_A, poly_roll_B, poly_roll_C;
    this->get_parameter("poly_roll_A", poly_roll_A);
    this->get_parameter("poly_roll_B", poly_roll_B);
    this->get_parameter("poly_roll_C", poly_roll_C);
    std::vector<double> pvalues4 = {poly_roll_A, poly_roll_B, poly_roll_C};
    auv_controller_->addPolyParamToVector(pvalues4, poly_params);

    double poly_pitch_A, poly_pitch_B, poly_pitch_C;
    this->get_parameter("poly_pitch_A", poly_pitch_A);
    this->get_parameter("poly_pitch_B", poly_pitch_B);
    this->get_parameter("poly_pitch_C", poly_pitch_C);
    std::vector<double> pvalues5 = {poly_pitch_A, poly_pitch_B, poly_pitch_C};
    auv_controller_->addPolyParamToVector(pvalues5, poly_params);

    double poly_yaw_A, poly_yaw_B, poly_yaw_C;
    this->get_parameter("poly_yaw_A", poly_yaw_A);
    this->get_parameter("poly_yaw_B", poly_yaw_B);
    this->get_parameter("poly_yaw_C", poly_yaw_C);
    std::vector<double> pvalues6 = {poly_yaw_A, poly_yaw_B, poly_yaw_C};
    auv_controller_->addPolyParamToVector(pvalues6, poly_params);



    // Retrieve and store max_wrench parameters
    double max_wrench_X, max_wrench_Y, max_wrench_Z, max_wrench_Roll, max_wrench_Pitch, max_wrench_Yaw;
    this->get_parameter("max_wrench_X", max_wrench_X);
    this->get_parameter("max_wrench_Y", max_wrench_Y);
    this->get_parameter("max_wrench_Z", max_wrench_Z);
    this->get_parameter("max_wrench_Roll", max_wrench_Roll);
    this->get_parameter("max_wrench_Pitch", max_wrench_Pitch);
    this->get_parameter("max_wrench_Yaw", max_wrench_Yaw);
    std::vector<double> max_wrench = {max_wrench_X, max_wrench_Y, max_wrench_Z, max_wrench_Roll, max_wrench_Pitch, max_wrench_Yaw};

    // Retrieve and store max_velocity parameters
    double max_velocity_x, max_velocity_y, max_velocity_z, max_velocity_roll, max_velocity_pitch, max_velocity_yaw;
    this->get_parameter("max_velocity_x", max_velocity_x);
    this->get_parameter("max_velocity_y", max_velocity_y);
    this->get_parameter("max_velocity_z", max_velocity_z);
    this->get_parameter("max_velocity_roll", max_velocity_roll);
    this->get_parameter("max_velocity_pitch", max_velocity_pitch);
    this->get_parameter("max_velocity_yaw", max_velocity_yaw);
    std::vector<double> max_velocity = {max_velocity_x, max_velocity_y, max_velocity_z, max_velocity_roll, max_velocity_pitch, max_velocity_yaw};


    // Update the controller with all gathered parameters
    auv_controller_->setControllerParams(p_params, t_params, poly_params);

    // Set the max velocity and max wrench parameters in the controller
    auv_controller_->setMaxVelocity(max_velocity);
    auv_controller_->setMaxWrench(max_wrench);


    
    

    double set_zero_velocity_depth = 1.0;
    std::vector<bool> set_zero_velocity_axes(6, true);
    cola2::ros::getParam("set_zero_velocity_depth", set_zero_velocity_depth,shared_from_this());
    cola2::ros::getParam("set_zero_velocity_axes", set_zero_velocity_axes,shared_from_this());
    if (set_zero_velocity_axes.size() != 6)
    {
      RCLCPP_WARN(this->get_logger(),"The set zero velocity axes parameter must be of size 6");
    }
    set_zero_velocity_axes.resize(6, true);
    auv_controller_->setSetZeroVelocityDepth(set_zero_velocity_depth);
    auv_controller_->setSetZeroVelocityPriority(cola2_msgs::msg::GoalDescriptor::PRIORITY_SAFETY_LOW);
    auv_controller_->setSetZeroVelocityAxes(set_zero_velocity_axes);

    // Thruster allocator
    std::vector<std::vector<double> > poly_positive_v, poly_negative_v;
    std::vector<double> max_force_thruster_positive_v, max_force_thruster_negative_v;
    for (std::size_t i = 0; i < auv_controller_->getNumberofThrusters(); ++i)
    {
      std::vector<double> thruster_poly_positive, thruster_poly_negative;
      cola2::ros::getParamVector(std::string("thruster_") + std::to_string(i + 1) + std::string("_poly_positive"),thruster_poly_positive,shared_from_this());
      cola2::ros::getParamVector(std::string("thruster_") + std::to_string(i + 1) + std::string("_poly_negative"),thruster_poly_negative,shared_from_this());
      poly_positive_v.push_back(thruster_poly_positive);
      poly_negative_v.push_back(thruster_poly_negative);

      double max_force_positive, max_force_negative;
      cola2::ros::getParam(std::string("thruster_") + std::to_string(i + 1) + std::string("_max_force_positive"), max_force_positive,shared_from_this());
      cola2::ros::getParam(std::string("thruster_") + std::to_string(i + 1) + std::string("_max_force_negative"), max_force_negative,shared_from_this());
      max_force_thruster_positive_v.push_back(max_force_positive);
      max_force_thruster_negative_v.push_back(max_force_negative);
    }
    std::vector<double> tcm;
    cola2::ros::getParamVector("TCM", tcm,shared_from_this());
    auv_controller_->thruster_allocator_.setParams(max_force_thruster_positive_v, max_force_thruster_negative_v,
                                                   poly_positive_v, poly_negative_v, tcm);

    auv_controller_->setControllerParams(p_params, t_params, poly_params);


    bool enable_thrusters;
    this->get_parameter("enable_thrusters",enable_thrusters);

    RCLCPP_INFO(this->get_logger(), "enable_thrusters: %s", enable_thrusters ? "true" : "false");
    if (enable_thrusters)
    {
      RCLCPP_INFO(this->get_logger(),"Thruster enabled\n");
      auv_controller_->setThrusterAllocator(true);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(),"Thruster disabled\n");
      auv_controller_->setThrusterAllocator(false);
    }

    std::cout << "Parameters changed!" << std::endl;
    return result;
  }
};


int main(int argc, char** argv)
{
  // Init ROS 2
  rclcpp::init(argc, argv);
 

  double period = 0.1;
  int n_thrusters = 5;

  auto auv_ctrl_ptr = std::make_shared<OnlyThrustersController>(period, n_thrusters);

  auto ros_controller = std::make_shared<OnlyThrustersROSController>("controller", "controller/base_link");
  ros_controller->init(auv_ctrl_ptr, period);

  // Spin until architecture stops
  rclcpp::spin(ros_controller);

  // Shut down ROS 2
  rclcpp::shutdown();

  return 0;
}
