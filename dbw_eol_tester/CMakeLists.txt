cmake_minimum_required(VERSION 3.3)
project(dbw_eol_tester)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  dbw_mkz_can
  dataspeed_rostest
)

if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_STANDARD 11)
endif()

add_compile_options(${TARGET_NAME} -Wall -Wextra -Werror -Wno-reorder -Wno-unused-function) #-pedantic fails in rosconsole https://github.com/ros/rosconsole/issues/9

# Find gtest and build into target from source
find_package(GTest)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gtest)
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${gtest_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/gtest/src)

catkin_package()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/gtest
  ${GTEST_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/tester.cpp gtest/src/gtest-all.cc)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME dbw_eol_tester PREFIX "")

function(copy_and_prune_files files_to_copy dest)
  set(dest "${CMAKE_CURRENT_SOURCE_DIR}/${dest}") # make sure we make this folder in source tree
  file(GLOB existing_files "${dest}/*")
  file(MAKE_DIRECTORY "${dest}")
  
  # make a list of all of the filenames of the files that will be copied
  foreach(file ${files_to_copy})
    get_filename_component(name "${file}" NAME)
    list(APPEND filenames_to_copy "${name}")
  endforeach(file)

  # remove all files that are only in existing_files
  foreach(file ${existing_files})
    get_filename_component(name ${file} NAME)
    if(${name} IN_LIST filenames_to_copy)
    else()
      message("upgrading file: ${file}")
      file(REMOVE ${file})
    endif()
  endforeach(file)

  # copy all files
  file(INSTALL ${files_to_copy} DESTINATION "${dest}"
    FILES_MATCHING PATTERN "*_BOO_*" PATTERN "*_*PEC_*") # only copy brake and throttle firmware
endfunction(copy_and_prune_files)

# maintain a local copy of dev firmware and released firmware for simple testing of both
file(GLOB DEV_HEX_FILES "../../../bin/*/app/*.hex")
copy_and_prune_files("${DEV_HEX_FILES}" dev_firmware)
file(GLOB RELEASED_HEX_FILES "../../../release/*/Firmware/*/*.hex")
copy_and_prune_files("${RELEASED_HEX_FILES}" firmware)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY firmware launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
