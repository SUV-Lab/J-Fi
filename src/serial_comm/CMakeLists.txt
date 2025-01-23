cmake_minimum_required(VERSION 3.8)
project(serial_comm)

include(ExternalProject)
ExternalProject_Add(
    mavlink
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/mavlink
    BINARY_DIR ${CMAKE_BINARY_DIR}/mavlink
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}/mavlink
               -DMAVLINK_DIALECT=common
               -DMAVLINK_VERSION=2.0
    INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/mavlink
)
include_directories(${CMAKE_INSTALL_PREFIX}/mavlink/include)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include headers
include_directories(include)

# Add compiler options for disable Suppress Warnings, This code prevents the build from failing due to warnings about alignment issues caused by auto-generated mavlink packed structures.
add_compile_options(-Wno-pedantic -Wno-address-of-packed-member)

# Build library
add_library(mavlink_serial_comm_lib
  src/mavlink_serial_comm.cpp
)
add_dependencies(mavlink_serial_comm_lib
  mavlink
)
ament_target_dependencies(mavlink_serial_comm_lib
  rclcpp
  std_msgs
)
target_include_directories(mavlink_serial_comm_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Build executable
add_executable(serial_comm_node
  src/serial_comm_node.cpp
)
ament_target_dependencies(serial_comm_node
  rclcpp
  std_msgs
)
target_link_libraries(serial_comm_node
  mavlink_serial_comm_lib
)

install(TARGETS
  mavlink_serial_comm_lib
  serial_comm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()