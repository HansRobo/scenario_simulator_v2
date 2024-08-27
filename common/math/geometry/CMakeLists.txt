cmake_minimum_required(VERSION 3.8)
project(geometry)

# set(CMAKE_CXX_COMPILER /usr/bin/clang++)
# set(CMAKE_C_COMPILER /usr/bin/clang)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_options(-O1)

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(geometry SHARED DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)

rclcpp_components_register_node(geometry
  PLUGIN "main_geometry::main_geometry_node"
  EXECUTABLE main_geometry_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_auto_package()
