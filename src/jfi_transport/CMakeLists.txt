cmake_minimum_required(VERSION 3.8)
project(jfi_transport)

include(ExternalProject)
ExternalProject_Add(mavlink
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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# Include headers
include_directories(include)

# Add compiler options for disable Suppress Warnings, This code prevents the build from failing due to warnings about alignment issues caused by auto-generated mavlink packed structures.
add_compile_options(-Wno-pedantic -Wno-address-of-packed-member)

add_executable(jfi_transport src/jfi_transport.cpp)
add_dependencies(jfi_transport mavlink)
ament_target_dependencies(jfi_transport rclcpp std_msgs px4_msgs)

install(TARGETS
  jfi_transport
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()