cmake_minimum_required(VERSION 3.8)
project(serial_relay)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/SerialRawData.msg"
    DEPENDENCIES std_msgs
)


# Include headers
include_directories(include)

# Add compiler options for disable Suppress Warnings, This code prevents the build from failing due to warnings about alignment issues caused by auto-generated mavlink packed structures.
add_compile_options(-Wno-pedantic -Wno-address-of-packed-member)



# Build library
add_library(serial_comm_lib
    src/serial_comm.cpp
)

ament_target_dependencies(serial_comm_lib
    rclcpp
    std_msgs
)
target_include_directories(serial_comm_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


# Build executable
add_executable(relay_node
    src/relay_node.cpp
    src/main.cpp
)

rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

ament_target_dependencies(relay_node
    rclcpp
    std_msgs
)
target_link_libraries(relay_node
    serial_comm_lib
    ${typesupport_target}
)

set_target_properties(relay_node PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN"
)

install(TARGETS
    serial_comm_lib
    relay_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
