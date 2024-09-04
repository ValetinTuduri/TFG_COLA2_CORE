cmake_minimum_required(VERSION 3.5)

project(nmea_gps)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable warnings
add_compile_options(-Wall -Wextra)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(COLA2_LIB REQUIRED)
find_package(cola2_msgs REQUIRED)
find_package(cola2_lib_ros REQUIRED)

find_package(rosidl_default_generators REQUIRED)


# Include directories
include_directories(
  include
  ${cola2_lib_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(${PROJECT_NAME}_lib SHARED src/${PROJECT_NAME}/nmea_gps.cpp)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp sensor_msgs std_msgs COLA2_LIB cola2_msgs cola2_lib_ros)

# Add executables
add_executable(${PROJECT_NAME}_node src/nmea_gps_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs std_msgs COLA2_LIB cola2_msgs cola2_lib_ros)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

# Install targets
install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export package dependencies
ament_export_dependencies(rclcpp sensor_msgs std_msgs COLA2_LIB cola2_msgs cola2_lib_ros)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)

# Package setup
ament_package()

