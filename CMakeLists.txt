cmake_minimum_required(VERSION 2.8.3)
project(wmra_ros)


find_package(catkin REQUIRED COMPONENTS
  urdf
  kdl_parser
  robot_state_publisher
  roscpp
  rospy
  std_msgs
  message_generation
  control_toolbox
  controller_manager
  hardware_interface 
  transmission_interface
  pluginlib
  joint_limits_interface
  tf
)
  
catkin_package(
  CATKIN_DEPENDS
)

add_executable(wmra_hardware src/wmra_hardware.cpp src/MotorController.cpp)
target_link_libraries(wmra_hardware ${catkin_LIBRARIES})
add_dependencies(wmra_hardware wmra_hardware_gencpp)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
