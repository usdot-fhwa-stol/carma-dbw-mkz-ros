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

if(${ROS_VERSION_MAJOR} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
  )

  add_message_files(DIRECTORY msg FILES
    AmbientLight.msg
    BrakeCmd.msg
    BrakeInfoReport.msg
    BrakeReport.msg
    DriverAssistReport.msg
    FuelLevelReport.msg
    Gear.msg
    GearCmd.msg
    GearNum.msg
    GearReject.msg
    GearReport.msg
    HillStartAssist.msg
    Misc1Report.msg
    ParkingBrake.msg
    QualityFactor.msg
    SteeringCmd.msg
    SteeringReport.msg
    SurroundReport.msg
    ThrottleCmd.msg
    ThrottleInfoReport.msg
    ThrottleReport.msg
    TirePressureReport.msg
    TurnSignal.msg
    TurnSignalCmd.msg
    TwistCmd.msg
    WatchdogCounter.msg
    WheelPositionReport.msg
    WheelSpeedReport.msg
    Wiper.msg
  )

  generate_messages(DEPENDENCIES
    std_msgs
    geometry_msgs
  )

  catkin_package(CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
  )

  # Make sure cmake knows about the messages we generate
  if(DEFINED CMAKE_STAGING_PREFIX)
    # colcon support
    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_STAGING_PREFIX})
  endif()

else()
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
endif()

# Common installation rules
if(${ROS_VERSION_MAJOR} EQUAL 1)
  install(DIRECTORY bmr
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
          FILES_MATCHING PATTERN "*.bmr"
  )
else()
  install(DIRECTORY bmr
          DESTINATION share/${PROJECT_NAME}
          FILES_MATCHING PATTERN "*.bmr"
  )
endif()

if(${ROS_VERSION_MAJOR} EQUAL 2)
  ament_package()
endif()
