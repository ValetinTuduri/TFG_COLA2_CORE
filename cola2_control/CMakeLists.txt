cmake_minimum_required(VERSION 3.5)
project(cola2_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies

find_package(COLA2_LIB REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cola2_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cola2_lib_ros REQUIRED)



# System dependencies
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# Declare things to be passed to dependent projects


# Specify locations of header files. Your package locations should be listed
# before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Declare C++ libraries
add_library(cola2_control
  src/${PROJECT_NAME}/controllers/anchor.cpp
  src/${PROJECT_NAME}/controllers/holonomic_keep_position.cpp
  src/${PROJECT_NAME}/controllers/section.cpp
  src/${PROJECT_NAME}/low_level_controllers/auv_controller_base.cpp
  src/${PROJECT_NAME}/low_level_controllers/controller_base.cpp
  src/${PROJECT_NAME}/low_level_controllers/merge.cpp
  src/${PROJECT_NAME}/low_level_controllers/ndof_controller.cpp
  src/${PROJECT_NAME}/low_level_controllers/only_thruster_allocator.cpp
  src/${PROJECT_NAME}/low_level_controllers/only_thrusters_controller.cpp
  src/${PROJECT_NAME}/low_level_controllers/pid.cpp
  src/${PROJECT_NAME}/low_level_controllers/poly.cpp
  src/${PROJECT_NAME}/low_level_controllers/request.cpp
  src/${PROJECT_NAME}/ros_controller/auv_ros_controller.cpp
)

# Link the libraries with ament dependencies
ament_target_dependencies(cola2_control 
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_msgs
  nav_msgs
  cola2_msgs
  cola2_lib_ros
  Eigen3
  rclcpp_action
  rclcpp_components
)

# Control nodes
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_msgs
  nav_msgs
  cola2_msgs
  cola2_lib_ros
  Eigen3
  
)
target_link_libraries(controller_node cola2_control)

add_executable(pilot_node src/pilot_node.cpp)
ament_target_dependencies(pilot_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_msgs
  nav_msgs
  cola2_msgs
  cola2_lib_ros
  Eigen3
  rclcpp_action
  rclcpp_components
  
)
target_link_libraries(pilot_node cola2_control)




# Install targets
install(TARGETS
  cola2_control
  controller_node
  pilot_node
  DESTINATION lib/${PROJECT_NAME})

# Install include directories
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})
  
#install(
#  DIRECTORY launch/
#  DESTINATION share/${PROJECT_NAME}/
#)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
  


# Ament package macros
ament_package()
