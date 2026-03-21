cmake_minimum_required(VERSION 3.5)
project(task_priority_controllers)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(taskspace_controllers REQUIRED)

set(THIS_PACKAGE_DEPS rclcpp taskspace_controllers)

######################################################
# Parameter libraries
generate_parameter_library(
  maximize_manipulability_parameters
  src/objectives/maximize_manipulability_parameters.yaml
)

generate_parameter_library(
  avoid_joint_limits_parameters
  src/objectives/avoid_joint_limits_parameters.yaml
)

generate_parameter_library(
  match_configuration_parameters
  src/objectives/match_configuration_parameters.yaml
)

generate_parameter_library(
  task_priority_controller_parameters
  src/task_priority_controller_parameters.yaml
)

######################################################
# Build executables and libraries

## Redundancy resolution objectives
add_library(rr_objectives SHARED
  src/objectives/objective_plugin.cpp
  src/objectives/minimize_velocity.cpp
  src/objectives/maximize_manipulability.cpp
  src/objectives/avoid_joint_limits.cpp
  src/objectives/match_configuration.cpp
)
target_include_directories(rr_objectives PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rr_objectives
  maximize_manipulability_parameters
  avoid_joint_limits_parameters
  match_configuration_parameters
)
ament_target_dependencies(rr_objectives ${THIS_PACKAGE_DEPS})
pluginlib_export_plugin_description_file(
  task_priority_controllers
  objective_plugins.xml
)

## Task priority controller
add_library(${PROJECT_NAME} SHARED
  src/task_priority_controller.cpp
  src/pose_controller.cpp
  src/objectives/objective_plugin.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  task_priority_controller_parameters
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})
pluginlib_export_plugin_description_file(
  controller_interface
  task_priority_controller_plugins.xml
)

######################################################
# Install executables and libraries
install(
  TARGETS
    ${PROJECT_NAME}
    rr_objectives
    maximize_manipulability_parameters
    avoid_joint_limits_parameters
    match_configuration_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  FILES
    task_priority_controller_plugins.xml
    objective_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export interfaces for downstream packages
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} rr_objectives)
ament_export_dependencies(${THIS_PACKAGE_DEPS})

ament_package()
