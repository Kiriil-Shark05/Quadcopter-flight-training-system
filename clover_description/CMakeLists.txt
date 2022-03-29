cmake_minimum_required(VERSION 3.0)
project(clover_description)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  rospy
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/main.py scripts/range2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

