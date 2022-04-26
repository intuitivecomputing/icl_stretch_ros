cmake_minimum_required(VERSION 3.0.2)
project(sp_analysis)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rosbag
  std_msgs
  geometry_msgs
  kalman_filter
  people_msgs
)

catkin_package(CATKIN_DEPENDS 
  rosbag 
  rospy 
  std_msgs 
  geometry_msgs 
  kalman_filter
  people_msgs
)


if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_python(scripts)
  roslint_add_test()
endif()

# catkin_install_python(PROGRAMS scripts/analysis.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})