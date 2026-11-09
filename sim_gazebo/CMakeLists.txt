cmake_minimum_required(VERSION 3.10)
project(gst_camera_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gazebo REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS} ${GST_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(GstCameraPlugin SHARED gst_camera_plugin.cc)
target_link_libraries(GstCameraPlugin
  ${GAZEBO_LIBRARIES}
  CameraPlugin
  ${GST_LIBRARIES}
)
