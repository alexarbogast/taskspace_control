cmake_minimum_required(VERSION 3.5)
project(axially_symmetric_controllers)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(taskspace_controllers REQUIRED)
find_package(task_priority_controllers REQUIRED)

set(THIS_PACKAGE_DEPS
  rclcpp
  taskspace_controllers
  task_priority_controllers
)

######################################################
# Parameter libraries
generate_parameter_library(
  axially_symmetric_controller_parameters
  src/axially_symmetric_controller_parameters.yaml
)

######################################################
# Build executables and libraries
add_library(${PROJECT_NAME} SHARED
  src/utility.cpp
  src/axially_symmetric_controller_base.cpp
  src/nullspace_controller.cpp
  src/twist_decomposition_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  axially_symmetric_controller_parameters
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})

pluginlib_export_plugin_description_file(
  controller_interface
  axially_symmetric_controller_plugins.xml
)

######################################################
# Install executables and libraries
install(
  TARGETS
    ${PROJECT_NAME}
    axially_symmetric_controller_parameters
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
ament_export_dependencies(${THIS_PACKAGE_DEPS})

ament_package()
