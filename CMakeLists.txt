cmake_minimum_required(VERSION 3.8)
project(canedudev_interface)
find_package(ament_cmake_auto REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_auto_find_build_dependencies()

ament_auto_add_executable(canedudev_interface_receiver
src/can_receiver.cpp)

ament_auto_add_executable(canedudev_interface_sender
src/can_sender.cpp
)

ament_auto_package(INSTALL_TO_SHARE
launch
params)


