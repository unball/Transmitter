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

add_message_files (DIRECTORY msg FILES
  comm_msg.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
)

catkin_install_python(PROGRAMS dummy_publisher/dummy_pub.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
