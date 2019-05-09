cmake_minimum_required(VERSION 2.8.3)
project(dataspeed_rostest)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(CATKIN_DEPENDS roscpp
               INCLUDE_DIRS include)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
