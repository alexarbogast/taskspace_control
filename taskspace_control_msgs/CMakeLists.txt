cmake_minimum_required(VERSION 3.0.2)
project(taskspace_control_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
    PoseTwistSetpoint.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    QueryPose.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
)
