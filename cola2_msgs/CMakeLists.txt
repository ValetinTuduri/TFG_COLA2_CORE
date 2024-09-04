cmake_minimum_required(VERSION 3.5)
project(cola2_msgs)

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)


# Declare the messages, services, and actions to be built
set(msg_files
    "msg/BodyForceReq.msg"
    "msg/BodyVelocityReq.msg"
    "msg/CaptainStateFeedback.msg"
    "msg/CaptainStatus.msg"
    "msg/DVLBeam.msg"
    "msg/DVL.msg"
    "msg/DecimalLatLon.msg"
    "msg/FlsConfig.msg"
    "msg/Float32Stamped.msg"
    "msg/GoalDescriptor.msg"
    "msg/ImageAcquisitionSettings.msg"
    "msg/KeyValueArray.msg"
    "msg/KeyValue.msg"
    "msg/MissionState.msg"
    "msg/NED.msg"
    "msg/NavSts.msg"
    "msg/RPY.msg"
    "msg/RecoveryAction.msg"
    "msg/SafetySupervisorStatus.msg"
    "msg/Setpoints.msg"
    "msg/SSSConfig.msg"
    "msg/WorldWaypointReq.msg"
    "msg/Bool6Axis.msg"
    "msg/GPSData.msg"
    "msg/AdisImu.msg"
    "msg/ImuAngleEstimatorInfo.msg"
)

set(srv_files
    "srv/DigitalOutput.srv"
    "srv/Goto.srv"
    "srv/MaxJoyVelocity.srv"
    "srv/Mission.srv"
    "srv/Recovery.srv"
    "srv/Section.srv"
    "srv/String.srv"
    "srv/CamTrigger.srv"
)

set(action_files
    "action/Pilot.action" 
)

# Generate ROS 2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    ${action_files}
    DEPENDENCIES std_msgs sensor_msgs actionlib_msgs geometry_msgs nav_msgs 
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

