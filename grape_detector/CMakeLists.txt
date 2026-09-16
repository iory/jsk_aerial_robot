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
  # torch: the cuda build (requirements.txt, several GB of nvidia libraries) only on a machine with a gpu,
  # the cpu build from the pytorch index (requirements-cpu.txt) otherwise. -DCATKIN_VIRTUALENV_CUDA=ON/OFF
  # overrides the detection.
  catkin_virtualenv_detect_cuda(GRAPE_DETECTOR_CUDA)
  if(GRAPE_DETECTOR_CUDA)
    set(torch_args)
  else()
    set(torch_args REQUIREMENTS_VARIANT cpu EXTRA_UV_ARGS --torch-backend cpu)
  endif()
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in
    ${torch_args}
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

install(FILES requirements.txt requirements-cpu.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
