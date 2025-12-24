cmake_minimum_required(VERSION 3.16)
project(stereo_camera_publisher)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
)

# Add executable
add_executable(stereo_camera_node
  src/stereo_camera_node.cpp
  src/stereo_camera_publisher.cpp
)

target_include_directories(stereo_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link libraries
ament_target_dependencies(stereo_camera_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  camera_info_manager
)

# Link V4L2 and OpenCV libraries
target_link_libraries(stereo_camera_node
  v4l2
  ${OpenCV_LIBS}
)

# Install targets
install(TARGETS stereo_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
