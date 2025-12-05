cmake_minimum_required(VERSION 3.5)
project(axially_symmetric_controllers)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

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
find_package(taskspace_controllers REQUIRED)
find_package(task_priority_controllers REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/utility.cpp
  src/nullspace_controller.cpp
  src/twist_decomposition_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  urdf
  kdl_parser
  pluginlib
  controller_interface
  realtime_tools
  taskspace_controllers
  task_priority_controllers
  taskspace_control_msgs
)

pluginlib_export_plugin_description_file(
  controller_interface
  axially_symmetric_controller_plugins.xml
)

# Install targets and headers
install(
  TARGETS ${PROJECT_NAME}
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
  FILES axially_symmetric_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  urdf
  kdl_parser
  pluginlib
  controller_interface
  realtime_tools
  taskspace_controllers
  task_priority_controllers
  taskspace_control_msgs
)

ament_package()
