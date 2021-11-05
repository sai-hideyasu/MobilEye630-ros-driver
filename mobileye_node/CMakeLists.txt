cmake_minimum_required(VERSION 3.0.2)
project(mobileye_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_install_python(PROGRAMS
  scripts/mobileye_node.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mobileye_node
)

install(FILES
  scripts/mobileye_node.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mobileye_node
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
