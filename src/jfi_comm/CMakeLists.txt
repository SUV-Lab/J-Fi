cmake_minimum_required(VERSION 3.8)
project(jfi_comm)

configure_file(
  ${CMAKE_SOURCE_DIR}/config/jfi.xml
  ${CMAKE_SOURCE_DIR}/mavlink/message_definitions/v1.0/jfi.xml
  @ONLY
)

include(ExternalProject)
ExternalProject_Add(
    mavlink
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/mavlink
    BINARY_DIR ${CMAKE_BINARY_DIR}/mavlink
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}/mavlink
                -DMAVLINK_DIALECT=jfi
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
find_package(rosidl_default_generators REQUIRED)
find_package(px4_msgs REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolyTraj.msg"
  DEPENDENCIES std_msgs
)

# Include headers
include_directories(include)

# Add compiler options for disable Suppress Warnings
add_compile_options(-Wno-pedantic -Wno-address-of-packed-member)

# Build library
add_library(jfi_comm_lib
  src/jfi_comm.cpp
)
add_dependencies(jfi_comm_lib
  mavlink
)
ament_target_dependencies(jfi_comm_lib
  rclcpp
  std_msgs
  px4_msgs
)
target_include_directories(jfi_comm_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(serial_comm_node_lib
  src/serial_comm_node.cpp
)
ament_target_dependencies(serial_comm_node_lib
  rclcpp
  std_msgs
  px4_msgs
)
# Link against generated message interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(serial_comm_node_lib
  jfi_comm_lib
  ${cpp_typesupport_target}
)

# Build executable
add_executable(serial_comm_node
  src/main.cpp
)
ament_target_dependencies(serial_comm_node
  rclcpp
  std_msgs
  px4_msgs
)
target_link_libraries(serial_comm_node
  serial_comm_node_lib
  ${cpp_typesupport_target}
)

install(TARGETS
  jfi_comm_lib
  serial_comm_node_lib
  serial_comm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()