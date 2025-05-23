# Copyright 2015 TIER IV, Inc. All rights reserved.
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
#
# Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

cmake_minimum_required(VERSION 3.5)
project(random_test_runner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -DBOOST_ALLOW_DEPRECATED_HEADERS)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
include(FindProtobuf REQUIRED)

# TODO(HansRobo): Remove this workaround once https://github.com/autowarefoundation/autoware_universe/issues/10410 is fixed
find_package(TinyXML2 CONFIG QUIET)
if(NOT TinyXML2_FOUND)
  find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h)
  find_library(TINYXML2_LIBRARY tinyxml2)
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(TinyXML2 DEFAULT_MSG TINYXML2_LIBRARY TINYXML2_INCLUDE_DIR)
  mark_as_advanced(TINYXML2_INCLUDE_DIR TINYXML2_LIBRARY)
  if(NOT TARGET tinyxml2::tinyxml2)
    add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
    set_property(TARGET tinyxml2::tinyxml2 PROPERTY IMPORTED_LOCATION ${TINYXML2_LIBRARY})
    set_property(TARGET tinyxml2::tinyxml2 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${TINYXML2_INCLUDE_DIR})
    list(APPEND TinyXML2_TARGETS tinyxml2::tinyxml2)
  endif()
endif()

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/data_types.cpp
  src/test_randomizer.cpp
  src/lanelet_utils.cpp
  src/random_test_runner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROTOBUF_LIBRARY}
  ${YAML_CPP_LIBRARIES}
  pthread
  sodium
  zmq
  fmt
)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

install(
  DIRECTORY launch rviz include param test/map
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package()
