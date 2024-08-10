cmake_minimum_required(VERSION 3.8)
project(rover_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(tier4_vehicle_msgs)

#sender
add_executable(vehicle_sender src/vehicle_sender.cpp)

target_link_libraries(vehicle_sender
  ${CANLIB_LIBRARY}
)

ament_target_dependencies(vehicle_sender
  rclcpp
  std_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
)

#receiver
add_executable(vehicle_receiver src/vehicle_receiver.cpp)

target_link_libraries(vehicle_receiver
  ${CANLIB_LIBRARY}
)

ament_target_dependencies(vehicle_receiver
  rclcpp
  std_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
  tier4_vehicle_msgs
)

#bridge
add_executable(sender_bridge src/sender_bridge.cpp)

target_link_libraries(sender_bridge
  ${CANLIB_LIBRARY}
)

ament_target_dependencies(sender_bridge
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS sender_bridge
  vehicle_sender
  vehicle_receiver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
