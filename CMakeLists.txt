cmake_minimum_required(VERSION 3.0.2)
project(ta2_farhan)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  actionlib
  smach
  smach_ros
)

catkin_package()

# catkin_install_python(PROGRAMS
#   scripts/simple_state_machine.py
#   scripts/position_database_server.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
