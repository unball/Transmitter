cmake_minimum_required(VERSION 2.8.3)
project(communication_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  rospy
  rosserial_arduino
  rosserial_client
)


catkin_package(
)


catkin_install_python(PROGRAMS dummy_pub/dummy_pub.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})