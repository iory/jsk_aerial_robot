cmake_minimum_required(VERSION 3.0.2)
project(gimbalrotor_remote)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rviz
  spinal
  std_srvs
  )

set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
# avoid the clash between the Qt "signals"/"slots" macros and boost
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS pluginlib roscpp rviz spinal std_srvs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/arm_torque_panel.cpp
  src/arm_torque_panel.h
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
