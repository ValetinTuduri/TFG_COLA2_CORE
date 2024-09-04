cmake_minimum_required(VERSION 3.5)
project(cola2_lib_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Add compile options
add_compile_options(-Wall -Wextra)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cola2_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(COLA2_LIB REQUIRED)

# Specify include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${COLA2_LIB_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(cola2_ros SHARED
   src/${PROJECT_NAME}/captain_helper.cpp
   src/${PROJECT_NAME}/diagnostic_helper.cpp
   src/${PROJECT_NAME}/navigation_helper.cpp
   src/${PROJECT_NAME}/setpoints_selector.cpp
   src/${PROJECT_NAME}/this_node.cpp
   src/${PROJECT_NAME}/transform_handler.cpp
   src/${PROJECT_NAME}/types.cpp
)

# Specify include directories and link dependencies
target_include_directories(cola2_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${COLA2_LIB_INCLUDE_DIRS}
)


ament_target_dependencies(cola2_ros
  rclcpp
  tf2
  tf2_ros
  std_msgs
  sensor_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  cola2_msgs
  OpenCV
  COLA2_LIB
)


target_link_libraries(cola2_ros ${COLA2_LIB_LIBRARIES})


# Install targets and headers
install(TARGETS cola2_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)


install(FILES src/cola2_lib_ros/urdf_macros.xacro
  DESTINATION share/${PROJECT_NAME}/src/${PROJECT_NAME}
)

# Export package dependencies
ament_export_dependencies(rclcpp tf2 tf2_ros std_msgs sensor_msgs diagnostic_msgs geometry_msgs nav_msgs cola2_msgs OpenCV COLA2_LIB )

# Export include directories and libraries for other packages
ament_export_include_directories(include)
ament_export_libraries(cola2_ros)

ament_package()

