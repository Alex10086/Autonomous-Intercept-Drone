cmake_minimum_required(VERSION 3.16)
project(uav_vision_dectect)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(uav_common_msg REQUIRED)
find_package(OpenCV REQUIRED)

# The SDK may be a release archive (include/) or a system-style installation.
set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH "ONNX Runtime C++ SDK root")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS "${ONNXRUNTIME_ROOT}/include"
  PATH_SUFFIXES onnxruntime onnxruntime/core/session
  REQUIRED)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  HINTS "${ONNXRUNTIME_ROOT}/lib" "${ONNXRUNTIME_ROOT}/lib64"
  REQUIRED)
get_filename_component(ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
message(STATUS "ONNX Runtime SDK: ${ONNXRUNTIME_INCLUDE_DIR}; ${ONNXRUNTIME_LIBRARY}")

add_executable(uav_vision_dectect
  src/main.cpp
  src/uav_topic_subscrib.cpp
  src/LightTrack.cpp
  src/yolo_detector.cpp)
target_include_directories(uav_vision_dectect PRIVATE
  include/uav_vision_dectect
  include/light_track
  "${ONNXRUNTIME_INCLUDE_DIR}"
  ${OpenCV_INCLUDE_DIRS})
# CUDA/shared provider libraries are loaded by ORT, not linked directly.
target_link_libraries(uav_vision_dectect ${OpenCV_LIBS} "${ONNXRUNTIME_LIBRARY}")
set_target_properties(uav_vision_dectect PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
  INSTALL_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE)
ament_target_dependencies(uav_vision_dectect
  ament_index_cpp rclcpp cv_bridge sensor_msgs uav_common_msg)

install(TARGETS uav_vision_dectect DESTINATION lib/${PROJECT_NAME})
# ament --symlink-install links resources, including the external ONNX data.
install(DIRECTORY model DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_tracking_math test/test_tracking_math.cpp)
  target_include_directories(test_tracking_math PRIVATE include/light_track)
  add_test(NAME tracking_math COMMAND test_tracking_math)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
