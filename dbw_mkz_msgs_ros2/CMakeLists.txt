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
file(GLOB msg_files msg/*.msg)
set(msgs)
foreach(filePath ${msg_files})
  get_filename_component(name ${filePath} NAME)
  set(msgs ${msgs} msg/${name})
endforeach(filePath)

# Generate interfaces
rosidl_generate_interfaces(
    ${PROJECT_NAME}
        ${msgs}
    DEPENDENCIES
        geometry_msgs
        std_msgs
)
# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_package()

# Install rule.yml file
install(FILES rule.yml
  DESTINATION share/${PROJECT_NAME}
)
