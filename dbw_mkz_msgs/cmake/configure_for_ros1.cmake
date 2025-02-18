# Copyright 2023 Leidos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# We want the ROS 1 support to be self-contained as much as possible so it can be
# easily removed in the (hopefully) near future. Therefore, we call find_package(...)
# here instead of in `dependencies.cmake`.
find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
)

file(GLOB dbw_mkz_msgs_MSG_FILES RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg)

# catkin assumes message description files are in the `msg/` directory, so we need to
# strip the `msg/` prefix from dbw_mkz_msgs_MSG_FILES
set(dbw_mkz_msgs_MSG_FILENAMES "")
foreach(msg_file_path ${dbw_mkz_msgs_MSG_FILES})
  get_filename_component(msg_filename ${msg_file_path} NAME)
  LIST(APPEND dbw_mkz_msgs_MSG_FILENAMES ${msg_filename})
endforeach()

add_message_files(
  FILES
    ${dbw_mkz_msgs_MSG_FILENAMES}
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
)
