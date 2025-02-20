cmake_minimum_required(VERSION 3.5)
project(dbw_mkz_msgs_ros2)
set(CMAKE_CXX_STANDARD 14)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

# ROS 2
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)


ament_package()

install(FILES rule.yml
  DESTINATION share/${PROJECT_NAME}
)
