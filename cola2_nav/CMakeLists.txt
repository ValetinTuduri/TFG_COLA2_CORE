cmake_minimum_required(VERSION 3.5)
project(cola2_nav)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



find_package(COLA2_LIB REQUIRED)

# Find dependencies
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
find_package(stonefish_ros2 REQUIRED)


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
add_library(cola2_nav
  # Bases
  src/${PROJECT_NAME}/ekf_base.cpp
  src/${PROJECT_NAME}/ekf_base_ros.cpp
  src/${PROJECT_NAME}/transformations.cpp
  # Complete filters
  src/${PROJECT_NAME}/ekf_position.cpp
  src/${PROJECT_NAME}/ekf_position_velocity.cpp
  src/${PROJECT_NAME}/ekf_surface_2d.cpp
)

# Link the libraries with ament dependencies
ament_target_dependencies(cola2_nav 
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
  stonefish_ros2
)

# Navigator nodes
add_executable(navigator_node src/navigator_node.cpp)
ament_target_dependencies(navigator_node 
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
  stonefish_ros2
  
)
target_link_libraries(navigator_node cola2_nav)

add_executable(navigator_position_node src/navigator_position_node.cpp)
ament_target_dependencies(navigator_position_node 
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
  stonefish_ros2
)
target_link_libraries(navigator_position_node cola2_nav)



add_executable(navigator_surface_node src/navigator_surface_node.cpp)
ament_target_dependencies(navigator_surface_node 
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
  stonefish_ros2
)
target_link_libraries(navigator_surface_node cola2_nav)


# Install targets
install(TARGETS
  cola2_nav
  navigator_node
  navigator_position_node
  navigator_surface_node
  DESTINATION lib/${PROJECT_NAME})

# Install include directories
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})
  
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
  


# Ament package macros
ament_package()
