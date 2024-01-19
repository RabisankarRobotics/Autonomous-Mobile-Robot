  cmake_minimum_required(VERSION 3.0.2)
  project(bumperbot_examples)


  # find dependencies
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf2_ros
    geometry_msgs
    message_generation
    nav_msgs
    sensor_msgs
    tf2
  )

  catkin_python_setup() 

  generate_messages(  
    DEPENDENCIES
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
  
  )


  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
  )




  include_directories(
  # include
    ${catkin_INCLUDE_DIRS}
  )


  # add_executable(amr_odom src/amr_odom.cpp)


  # Install
  # install(TARGETS
  #   amr_odom
  #   DESTINATION lib/${PROJECT_NAME}
  # )

  catkin_install_python(PROGRAMS 
    nodes/simple_publisher.py
    nodes/simple_subscriber.py
    nodes/tf_examples_node.py
    nodes/bumperbot_odom_node.py
    nodes/imu_node.py
    nodes/tf_broadcaster_imu.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
