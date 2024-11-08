cmake_minimum_required(VERSION 3.0.2)
project(task_priority_controllers)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  kdl_parser
  taskspace_controllers
  taskspace_control_msgs
  hardware_interface
  controller_interface
  realtime_tools
  dynamic_reconfigure
  pluginlib
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/ManipulabilityObjective.cfg
  cfg/JointLimitsObjective.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} rr_objectives
  CATKIN_DEPENDS
    roscpp
    urdf
    kdl_parser
    taskspace_controllers
    taskspace_control_msgs
    hardware_interface
    controller_interface
    realtime_tools
    dynamic_reconfigure
    pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## redundancy resolution objectives
add_library(rr_objectives
  src/objectives/objective_plugin.cpp
  src/objectives/minimize_velocity.cpp
  src/objectives/maximize_manipulability.cpp
  src/objectives/avoid_joint_limits.cpp
)
target_link_libraries(rr_objectives ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(rr_objectives
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

## task_priority_controllers
add_library(${PROJECT_NAME}
  src/task_priority_controller.cpp
  src/pose_controller.cpp
  src/objectives/objective_plugin.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

## Installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} rr_objectives
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    task_priority_controller_plugins.xml
    objective_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
