cmake_minimum_required(VERSION 3.0.2)
project(piper_camera)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge
)

catkin_install_python(PROGRAMS
  scripts/piper_orbbec_usb.py
  scripts/orbbec_wrist.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
