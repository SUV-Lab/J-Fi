cmake_minimum_required(VERSION 3.8)
project(jfi_comm)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(mavlink_vendor REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SwarmComm.msg"
  DEPENDENCIES std_msgs
)

# Include headers
include_directories(include)

# suppress warnings from auto-generated packed MAVLink structs
add_compile_options(-Wno-pedantic -Wno-address-of-packed-member)

# Build library
add_library(jfi_comm_lib
  src/jfi_comm.cpp
)
ament_target_dependencies(jfi_comm_lib
  rclcpp
  mavlink_vendor
)
target_include_directories(jfi_comm_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${mavlink_vendor_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:${mavlink_vendor_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/vendor>
)

add_library(serial_comm_node_lib
  src/serial_comm_node.cpp
)
add_dependencies(serial_comm_node_lib ${PROJECT_NAME}_rosidl_get_typesupport_targets)
ament_target_dependencies(serial_comm_node_lib
  rclcpp
  std_msgs
  ${PROJECT_NAME}_rosidl_typesupport_cpp
)
target_link_libraries(serial_comm_node_lib
  jfi_comm_lib
)

# Build executable
add_executable(serial_comm_node
  src/main.cpp
)
ament_target_dependencies(serial_comm_node
  rclcpp
)
target_link_libraries(serial_comm_node
  serial_comm_node_lib
)

add_executable(evaluator_node
  src/evaluator_node.cpp
)
add_dependencies(evaluator_node ${PROJECT_NAME}_rosidl_get_typesupport_targets)
ament_target_dependencies(evaluator_node
  rclcpp
  trajectory_msgs
  ${PROJECT_NAME}_rosidl_typesupport_cpp
)

# Install
install(TARGETS
  jfi_comm_lib
  serial_comm_node_lib
  serial_comm_node
  evaluator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()