cmake_minimum_required(VERSION 3.5)
project(dbw_mkz_msgs)

# First check if ROS_VERSION is set (colcon sets this)
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION_MAJOR $ENV{ROS_VERSION})
else()
  # If ROS_VERSION is not set, try to find catkin (ROS1)
  find_package(catkin QUIET)
  if(catkin_FOUND)
    set(ROS_VERSION_MAJOR 1)
  endif()
endif()

if(NOT DEFINED ROS_VERSION_MAJOR)
  message(FATAL_ERROR "Could not determine ROS version")
endif()

if(DEFINED ENV{ROS_VERSION} AND $ENV{ROS_VERSION} EQUAL 1)
  # ROS 1 uses a different, incompatible CMake workflow than ROS 2. Until CARMA
  # fully transitions to ROS 2, we need to support ROS 1 environments too.
  message(STATUS "Configuring `dbw_mkz_msgs` for ROS 1")
  include(${PROJECT_SOURCE_DIR}/cmake/configure_for_ros1.cmake)

  return()

endif()

# ROS 2
carma_check_ros_version(2)

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


ament_auto_package()

include(GNUInstallDirs)

install(FILES rule.yml
  DESTINATION ${CMAKE_INSTALL_DATADIR}/carma_cooperative_perception_interfaces
)
