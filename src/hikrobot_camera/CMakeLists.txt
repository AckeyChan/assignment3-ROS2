cmake_minimum_required(VERSION 3.10)
project(hikrobot_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

# 厂商 SDK（海康机器人 MVS）：不在 rosdep 中，需单独安装。
# 默认安装路径为 /opt/MVS，可通过 -DMVS_ROOT=<path> 指定其他位置。

if(NOT DEFINED MVS_ROOT)
  set(MVS_ROOT "/opt/MVS" CACHE PATH "Path to the HIKROBOT MVS SDK")
endif()

find_path(MVS_INCLUDE_DIR NAMES MvCameraControl.h HINTS "${MVS_ROOT}/include")
find_library(MVS_LIBRARY NAMES MvCameraControl HINTS "${MVS_ROOT}/lib/64" "${MVS_ROOT}/lib")

if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
  message(FATAL_ERROR
    "未找到 MVS SDK。请先安装海康机器人 MVS（默认路径 /opt/MVS），"
    "或使用 -DMVS_ROOT=<MVS 安装路径> 指定 SDK 位置。")
endif()
message(STATUS "MVS SDK include: ${MVS_INCLUDE_DIR}")
message(STATUS "MVS SDK library: ${MVS_LIBRARY}")

add_executable(camera_node
  src/main.cpp
  src/camera_node.cpp
)
target_compile_features(camera_node PRIVATE cxx_std_17)
target_include_directories(camera_node PRIVATE include ${MVS_INCLUDE_DIR})
target_link_libraries(camera_node
  rclcpp::rclcpp
  rcl_interfaces::rcl_interfaces
  ${sensor_msgs_TARGETS}
  ${MVS_LIBRARY}
  Threads::Threads
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(camera_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
