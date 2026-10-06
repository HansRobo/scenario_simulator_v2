cmake_minimum_required(VERSION 3.16)
project(openscenario_python)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# The headless pybind facade only makes sense in SSV2_HEADLESS_EGO builds (it reaches the
# in-process ego injection / read seam that exists only then). Passed by the validator build as a
# global colcon cmake-arg (-DSSV2_HEADLESS_EGO=ON); when OFF this package builds nothing.
option(SSV2_HEADLESS_EGO "Build the headless Diffusion-Planner OpenSCENARIO Python module" OFF)

if(SSV2_HEADLESS_EGO)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
  find_package(pybind11_vendor REQUIRED)
  find_package(pybind11 REQUIRED)

  # The module import name must match PYBIND11_MODULE(openscenario_python, ...).
  pybind11_add_module(openscenario_python src/openscenario_python.cpp)

  # Defined transitively via traffic_simulator's PUBLIC compile definition, but set explicitly too
  # so this TU is guaranteed consistent with the headers it includes (ODR safety).
  target_compile_definitions(openscenario_python PRIVATE SSV2_HEADLESS_EGO)

  ament_target_dependencies(openscenario_python PUBLIC
    openscenario_interpreter
    traffic_simulator
    traffic_simulator_msgs
    autoware_planning_msgs
    autoware_vehicle_msgs
    geometry_msgs
    lifecycle_msgs
    rclcpp)

  # MlPlannerObserver builds autoware_ml_planner's own preprocessing from its sources: the package
  # itself refuses to build without CUDA and TensorRT, which the preprocessing does not use.
  set(ML_PLANNER_SOURCE_DIR "$ENV{ML_PLANNER_SOURCE_DIR}" CACHE PATH
    "autoware_ml_planner package directory; MlPlannerObserver is built only when it is set")
  if(EXISTS "${ML_PLANNER_SOURCE_DIR}/src/preprocessing/input_builder.cpp")
    set(ml_planner_deps
      autoware_lanelet2_extension
      autoware_object_recognition_utils
      autoware_perception_msgs
      autoware_traffic_light_utils
      autoware_trajectory
      autoware_utils_geometry
      autoware_utils_math
      autoware_vehicle_info_utils
      lanelet2_traffic_rules
      nav_msgs
      tl_expected)
    foreach(dep IN LISTS ml_planner_deps)
      find_package(${dep} REQUIRED)
    endforeach()
    find_package(Eigen3 REQUIRED)
    find_package(xtensor REQUIRED)
    set(ml_planner_src "${ML_PLANNER_SOURCE_DIR}/src")
    target_sources(openscenario_python PRIVATE
      src/ml_planner_observer.cpp
      "${ml_planner_src}/preprocessing/input_builder.cpp"
      "${ml_planner_src}/preprocessing/items/agent.cpp"
      "${ml_planner_src}/preprocessing/items/ego_history.cpp"
      "${ml_planner_src}/preprocessing/items/ego_shape.cpp"
      "${ml_planner_src}/preprocessing/items/goal_pose.cpp"
      "${ml_planner_src}/preprocessing/items/map.cpp"
      "${ml_planner_src}/preprocessing/items/traffic_signals.cpp"
      "${ml_planner_src}/preprocessing/items/turn_indicators.cpp"
      "${ml_planner_src}/preprocessing/preprocessing_utils.cpp"
      "${ml_planner_src}/utils/utils.cpp")
    target_include_directories(openscenario_python PRIVATE "${ML_PLANNER_SOURCE_DIR}/include")
    target_compile_definitions(openscenario_python PRIVATE OPENSCENARIO_PYTHON_ML_PLANNER)
    ament_target_dependencies(openscenario_python PUBLIC ${ml_planner_deps})
    target_link_libraries(openscenario_python PRIVATE Eigen3::Eigen xtensor)

    # The autoware libraries the simulator's runtime install does not carry are shipped next to
    # the module. Its own copy of the lanelet2 extension and every message library stay the host's.
    set_target_properties(openscenario_python PROPERTIES INSTALL_RPATH "$ORIGIN/../..")
    install(CODE "
      string(REPLACE \":\" \";\" search_dirs \"\$ENV{LD_LIBRARY_PATH}\")
      file(GET_RUNTIME_DEPENDENCIES
        LIBRARIES \"$<TARGET_FILE:openscenario_python>\"
        RESOLVED_DEPENDENCIES_VAR resolved
        DIRECTORIES \${search_dirs}
        PRE_INCLUDE_REGEXES \"^libautoware_\" \"^libvehicle_info_utils\"
        PRE_EXCLUDE_REGEXES \".*\"
        POST_EXCLUDE_REGEXES \"^/opt/\" \"^/usr/\" \"lanelet2_extension\" \"_msgs/\")
      foreach(lib IN LISTS resolved)
        file(INSTALL \"\${lib}\" DESTINATION \"\${CMAKE_INSTALL_PREFIX}/lib\" FOLLOW_SYMLINK_CHAIN)
      endforeach()")
  else()
    message(WARNING "ML_PLANNER_SOURCE_DIR does not point at autoware_ml_planner; MlPlannerObserver is left out")
  endif()

  # Install so the module is importable from the integrated .venv once the overlay setup.bash is
  # sourced (colcon adds <prefix>/lib/python3.10/site-packages to PYTHONPATH).
  install(TARGETS openscenario_python
    DESTINATION "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
endif()

ament_auto_package()
