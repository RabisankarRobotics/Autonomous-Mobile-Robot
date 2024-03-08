cmake_minimum_required(VERSION 3.0.2)
project(four_wheeler_odom)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

generate_messages(DEPENDENCIES  
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  )


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    tf2_ros
    tf2
    geometry_msgs
)




include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  nodes/simple_publisher.py
  nodes/odom_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

