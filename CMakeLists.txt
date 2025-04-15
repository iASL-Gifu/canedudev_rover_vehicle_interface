cmake_minimum_required(VERSION 3.8)
project(canedudev_rover_vehicle_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)

include_directories(
  include
)

add_executable(vehicle_command_sender src/vehicle_command_sender.cpp)
ament_target_dependencies(vehicle_command_sender
  rclcpp
  std_msgs
  sensor_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
)

add_executable(vehicle_state_receiver src/vehicle_state_receiver.cpp)
ament_target_dependencies(vehicle_state_receiver
  rclcpp
  std_msgs
  geometry_msgs
  autoware_vehicle_msgs
  tier4_vehicle_msgs
)

install(TARGETS
  vehicle_command_sender
  vehicle_state_receiver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
