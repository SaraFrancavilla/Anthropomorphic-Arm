cmake_minimum_required(VERSION 3.8)
project(ros2_ur5_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(LIBTORCH_URL "https://download.pytorch.org/libtorch/cpu/libtorch-shared-with-deps-2.2.0%2Bcpu.zip")
set(LIBTORCH_DIR ${CMAKE_BINARY_DIR}/libtorch)
set(LIBTORCH_ZIP ${CMAKE_BINARY_DIR}/libtorch.zip)

if(NOT EXISTS ${LIBTORCH_DIR})
  message(STATUS "Downloading libTorch...")
  file(DOWNLOAD ${LIBTORCH_URL} ${LIBTORCH_ZIP} SHOW_PROGRESS)
  execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf ${LIBTORCH_ZIP}
                  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${LIBTORCH_DIR}")
find_package(Torch REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
  include
)

add_executable(publish_trajectory_node src/publish_trajectory_node.cpp)
add_executable(gripper_service src/gripper_service.cpp)
add_executable(yolo_detection_node src/detection.cc)
add_executable(pose_from_camera_node src/position.cc)

ament_target_dependencies(publish_trajectory_node
  rclcpp
  trajectory_msgs
  sensor_msgs
  control_msgs
  rclcpp_action
  std_srvs
)

ament_target_dependencies(gripper_service
  rclcpp
  ament_index_cpp
  trajectory_msgs
  sensor_msgs
  control_msgs
  rclcpp_action
  std_srvs
)

target_link_libraries(yolo_detection_node
  ${OpenCV_LIBS}
  ${TORCH_LIBRARIES}
)
target_compile_definitions(yolo_detection_node PRIVATE -D_GLIBCXX_USE_CXX11_ABI=1)
ament_target_dependencies(yolo_detection_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

ament_target_dependencies(pose_from_camera_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  vision_msgs
)

install(TARGETS
  publish_trajectory_node
  gripper_service
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory
install(
  DIRECTORY 
  launch
  config
  params
  gripper
  worlds
  models
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
