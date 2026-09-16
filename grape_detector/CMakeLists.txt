cmake_minimum_required(VERSION 3.0.2)
project(grape_detector)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
  )

catkin_package()

# ultralytics (YOLO-World) and torch go into a virtualenv in the devel/install space, locked in
# requirements.txt, so that no python package has to be installed on the machine by hand.
# The lock is universal from 3.10 so the same requirements.txt installs on every python that builds it.
set(GRAPE_DETECTOR_MIN_PYTHON 3.10)
if(PYTHON_VERSION_STRING VERSION_LESS GRAPE_DETECTOR_MIN_PYTHON)
  message(WARNING "python ${PYTHON_VERSION_STRING} < ${GRAPE_DETECTOR_MIN_PYTHON}: "
    "grape_detector is not installed")
else()
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in
    EXTRA_UV_COMPILE_ARGS --universal --python-version ${GRAPE_DETECTOR_MIN_PYTHON}
    )
  catkin_install_python(PROGRAMS
    scripts/grape_detector_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
