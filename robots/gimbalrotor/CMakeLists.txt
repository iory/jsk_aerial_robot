cmake_minimum_required(VERSION 3.0.2)
project(gimbalrotor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  catkin_virtualenv
  controller_manager
  hardware_interface
  spinal
  pluginlib
  roscpp
  sensor_msgs
  )

find_package(Eigen3 REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs controller_manager hardware_interface pluginlib roscpp sensor_msgs spinal
)

# python dependencies (scikit-robot) go into a virtualenv in the devel/install space, locked in requirements.txt;
# system packages stay visible for rospy and the other ROS python modules.
# scikit-robot needs python >= 3.10, so the skrobot interface is not built on older ROS (e.g. noetic, python 3.8).
# The lock is universal from 3.10 so the same requirements.txt installs on every python that builds it.
set(GIMBALROTOR_SKROBOT_MIN_PYTHON 3.10)
if(PYTHON_VERSION_STRING VERSION_LESS GIMBALROTOR_SKROBOT_MIN_PYTHON)
  message(WARNING "python ${PYTHON_VERSION_STRING} < ${GIMBALROTOR_SKROBOT_MIN_PYTHON}: "
    "the scikit-robot interface of gimbalrotor (python/gimbalrotor/skrobot_interface.py, "
    "script/skrobot_go_and_return.py) is not installed")
  set(GIMBALROTOR_BUILD_SKROBOT_INTERFACE FALSE)
else()
  set(GIMBALROTOR_BUILD_SKROBOT_INTERFACE TRUE)
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in
    EXTRA_UV_COMPILE_ARGS --universal --python-version ${GIMBALROTOR_SKROBOT_MIN_PYTHON}
    )
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(gimbalrotor_controller src/control/gimbalrotor_controller.cpp)
target_link_libraries(gimbalrotor_controller ${catkin_LIBRARIES})

add_library(gimbalrotor_robot_model src/model/gimbalrotor_robot_model.cpp)
target_link_libraries(gimbalrotor_robot_model ${catkin_LIBRARIES})

add_library(gimbalrotor_navigation src/gimbalrotor_navigation.cpp)
target_link_libraries(gimbalrotor_navigation ${catkin_LIBRARIES})

add_executable(arm_hardware_interface_node src/arm/arm_hardware_interface_node.cpp src/arm/arm_hardware_interface.cpp)
target_link_libraries(arm_hardware_interface_node ${catkin_LIBRARIES})
add_dependencies(arm_hardware_interface_node ${catkin_EXPORTED_TARGETS})

install(TARGETS arm_hardware_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

if(GIMBALROTOR_BUILD_SKROBOT_INTERFACE)
  catkin_install_python(PROGRAMS
    script/grape_harvest_sim_grasp.py
    script/make_grape_vineyard_world.py
    script/skrobot_go_and_return.py
    script/skrobot_grape_harvest_demo.py
    script/estimate_cog_from_flight.py
    script/thin_room_map.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()

install(DIRECTORY config plugins launch maps models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
