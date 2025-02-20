cmake_minimum_required(VERSION 3.5)

if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1")
  message(STATUS "ROS1 detected. Skipping package dbw_mkz_msgs_ros2")
  return()
endif()

project(dbw_mkz_msgs_ros2)
set(CMAKE_CXX_STANDARD 14)

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Get all message files
file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs geometry_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)

# Install rule.yml file
install(FILES rule.yml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
