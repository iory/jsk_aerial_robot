cmake_minimum_required(VERSION 3.0.2)
project(grape_detector)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
  )

catkin_python_setup()

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
    scripts/export_adla_model.py
    scripts/replay_video.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()

# The models of backend:=adla (the NPU of a Khadas VIM4, made by scripts/export_adla_model.py) are
# too large for git and derived from the AGPL-3.0 weights of Ultralytics, so they are published as a
# release of iory/jsk_aerial_robot and downloaded into models/ on a machine with the NPU runtime.
# -DGRAPE_DETECTOR_DOWNLOAD_ADLA_MODELS=ON/OFF overrides the detection.
find_library(GRAPE_DETECTOR_NNSDK nnsdk)
if(GRAPE_DETECTOR_NNSDK)
  set(download_adla_default ON)
else()
  set(download_adla_default OFF)
endif()
option(GRAPE_DETECTOR_DOWNLOAD_ADLA_MODELS
  "download the .adla models for the NPU of a Khadas VIM4" ${download_adla_default})
set(GRAPE_DETECTOR_ADLA_MODELS_URL
  https://github.com/iory/jsk_aerial_robot/releases/download/grape_detector-adla-models-1)
# name and sha256 of each file of the release
set(GRAPE_DETECTOR_ADLA_MODELS
  yoloe-v8l_640_int8.adla 0faa726926a7528c1a2f494fd9a9b8a36f966af704d2d291c0799b2ad723a3e4
  yoloe-v8l_640_int8.yaml f081e38c49afdc2d974357deea4932beeae227d2f8538731e578e659aa0acccf
  yoloe-v8s_640_int8.adla 71e5861348c058b3053bad0dbcf9276bdf672d1dd5baf297b03a8678218d6fee
  yoloe-v8s_640_int8.yaml 8dcbf938c7423356e81b1d02bf8b41d06c01bb999094e445035e0f5ce437e8fc
  )

function(grape_detector_download_model name sha256)
  set(path ${PROJECT_SOURCE_DIR}/models/${name})
  if(EXISTS ${path})
    file(SHA256 ${path} actual)
    if(actual STREQUAL sha256)
      return()
    endif()
  endif()
  message(STATUS "Downloading ${GRAPE_DETECTOR_ADLA_MODELS_URL}/${name}")
  file(DOWNLOAD ${GRAPE_DETECTOR_ADLA_MODELS_URL}/${name} ${path}
    EXPECTED_HASH SHA256=${sha256} STATUS status)
  list(GET status 0 code)
  if(NOT code EQUAL 0)
    list(GET status 1 reason)
    message(FATAL_ERROR "can not download ${name}: ${reason}")
  endif()
endfunction()

if(GRAPE_DETECTOR_DOWNLOAD_ADLA_MODELS)
  list(LENGTH GRAPE_DETECTOR_ADLA_MODELS count)
  math(EXPR last "${count} - 1")
  foreach(i RANGE 0 ${last} 2)
    math(EXPR j "${i} + 1")
    list(GET GRAPE_DETECTOR_ADLA_MODELS ${i} name)
    list(GET GRAPE_DETECTOR_ADLA_MODELS ${j} sha256)
    grape_detector_download_model(${name} ${sha256})
  endforeach()
  install(DIRECTORY models
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
endif()

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(FILES requirements.txt requirements-cpu.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
