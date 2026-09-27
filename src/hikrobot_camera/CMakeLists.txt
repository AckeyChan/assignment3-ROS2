cmake_minimum_required(VERSION 3.11)
project(hikrobot_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(camera_node
  src/main.cpp
  src/camera_node.cpp
)
target_compile_features(camera_node PRIVATE cxx_std_17)
target_include_directories(camera_node PRIVATE include)
ament_target_dependencies(camera_node rclcpp)



if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(camera_node PRIVATE -Wall -Wextra -Wpedantic)

#检测MVS
endif()
if (NOT DEFINED MVS_ROOT)
  set(MVS_ROOT "/opt/MVS" CACHE PATH "Path to MVS SDK")
endif()
 find_path(MVS_INCLUDE_DIR NAMES MvCameraControl.h HINTS ${MVS_ROOT}/include)
 find_library(MVS_LIBRARY NAMES MvCameraControl HINTS ${MVS_ROOT}/lib/64 ${MVS_ROOT}/lib)
if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
  message(FATAL_ERROR "Could not find MVS SDK. Please set MVS_ROOT to the path of the MVS SDK.")
else()
  target_include_directories(camera_node PRIVATE ${MVS_INCLUDE_DIR})
  target_link_libraries(camera_node ${MVS_LIBRARY})
  message(STATUS "Found MVS SDK: ${MVS_INCLUDE_DIR}")
  message(STATUS "Found MVS Library: ${MVS_LIBRARY}")
  
 endif()




install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
