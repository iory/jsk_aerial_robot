cmake_minimum_required(VERSION 3.0.2)
project(gimbalrotor_remote)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  aerial_robot_msgs
  control_msgs
  nav_msgs
  pluginlib
  roscpp
  rviz
  sensor_msgs
  spinal
  std_msgs
  std_srvs
  urdf
  )

set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
# avoid the clash between the Qt "signals"/"slots" macros and boost
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS actionlib aerial_robot_msgs control_msgs nav_msgs pluginlib roscpp rviz sensor_msgs spinal std_msgs std_srvs urdf
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/arm_control_panel.cpp
  src/arm_control_panel.h
  src/flight_teleop_panel.cpp
  src/flight_teleop_panel.h
  )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(PROGRAMS scripts/remote_rviz.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
