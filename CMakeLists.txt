cmake_minimum_required(VERSION 2.8.3)
project(communication)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  rospy
  rosserial_arduino
  rosserial_client
  joy
)

add_message_files (DIRECTORY msg FILES
  comm_msg.msg
  target_positions_msg.msg
  wheels_speeds_msg.msg
  robots_speeds_msg.msg
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
)
add_executable(nodetest
               src/nodetest.cpp)
add_dependencies(nodetest communication_node_gencpp)
target_link_libraries(nodetest ${catkin_LIBRARIES})

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
)

catkin_install_python(PROGRAMS dummy_publisher/dummy_pub.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
