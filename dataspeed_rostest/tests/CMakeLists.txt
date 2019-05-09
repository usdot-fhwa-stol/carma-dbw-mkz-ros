# Tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

# These dependencies are only needed for unit testing
find_package(roslaunch REQUIRED)
find_package(rostest REQUIRED)

if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_STANDARD 11)
endif()

add_executable(${PROJECT_NAME}_topics topics_node.cpp)
add_dependencies(${PROJECT_NAME}_topics ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_topics ${catkin_LIBRARIES})

# rostest
add_rostest_gtest(${PROJECT_NAME}_test_wait_for_topics topics.test system/test_wait_for_topics.cpp)
target_link_libraries(${PROJECT_NAME}_test_wait_for_topics ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_test_wait_for_topics PUBLIC ../include)
