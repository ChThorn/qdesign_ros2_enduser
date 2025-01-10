cmake_minimum_required(VERSION 3.10)
project(qdesign_ros2_enduser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# Qt setup
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${realsense2_INCLUDE_DIRS}
)

# Specify header files
set(HEADER_FILES
  include/${PROJECT_NAME}/DialogController.h
  include/${PROJECT_NAME}/DialogTest.h
)

# Specify source files
set(SOURCE_FILES
  src/main.cpp
  src/DialogController.cpp
)

# Create Qt viewer executable
add_executable(qt_realsense_node
  ${SOURCE_FILES}
  ${HEADER_FILES}
)

ament_target_dependencies(qt_realsense_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(qt_realsense_node
  Qt5::Core
  Qt5::Widgets
  ${OpenCV_LIBS}
)

# Create camera launch executable
add_executable(realsense_camera_node
  launch/launch_camera.cpp
)

ament_target_dependencies(realsense_camera_node
  rclcpp
  sensor_msgs
)

target_link_libraries(realsense_camera_node
  ${realsense2_LIBRARY}
  realsense2  # Add explicit linking to realsense2
)

# Install executables
install(TARGETS
  qt_realsense_node
  realsense_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN "*.cpp"
)

ament_package()
