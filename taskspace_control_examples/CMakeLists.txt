cmake_minimum_required(VERSION 3.0.2)
project(taskspace_control_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
  taskspace_controllers
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  rospy
  taskspace_controllers
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/pose_control_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
