cmake_minimum_required(VERSION 3.5)
project(taskspace_controllers)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if ($ENV{ROS_DISTRO} STREQUAL "jazzy")
  set(TASKSPACE_CONTROLLERS_JAZZY TRUE)
elseif ($ENV{ROS_DISTRO} STREQUAL "iron")
  set(TASKSPACE_CONTROLLERS_IRON TRUE)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
  set(TASKSPACE_CONTROLLERS_HUMBLE TRUE)
else()
  message(WARNING "ROS2 version must be {jazzy|iron|humble}")
endif()
configure_file(include/taskspace_controllers/ros2_version_config.h.in ros2_version_config.h)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(taskspace_control_msgs REQUIRED)

generate_parameter_library(
  pose_controller_parameters
  src/pose_controller_parameters.yaml
)

generate_parameter_library(
  taskspace_controller_base_parameters
  src/taskspace_controller_base_parameters.yaml
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR} # ros2_version_config.h
)

add_library(${PROJECT_NAME} SHARED
  src/utility.cpp
  src/taskspace_controller_base.cpp
  src/pose_controller.cpp
)
target_link_libraries(${PROJECT_NAME}
  taskspace_controller_base_parameters
  pose_controller_parameters
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  urdf
  kdl_parser
  pluginlib
  controller_interface
  realtime_tools
  taskspace_control_msgs
)

pluginlib_export_plugin_description_file(
  controller_interface
  taskspace_controller_plugins.xml
)

# Installation
install(
  TARGETS ${PROJECT_NAME} pose_controller_parameters
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
  FILES taskspace_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export interfaces for downstream packages
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  urdf
  kdl_parser
  pluginlib
  controller_interface
  generate_parameter_library
  realtime_tools
  taskspace_control_msgs
)

ament_package()
