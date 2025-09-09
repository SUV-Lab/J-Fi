cmake_minimum_required(VERSION 3.8)
project(mavlink_vendor)

# find dependencies
find_package(ament_cmake REQUIRED)

configure_file(
  ${CMAKE_SOURCE_DIR}/config/jfi.xml
  ${CMAKE_SOURCE_DIR}/mavlink/message_definitions/v1.0/jfi.xml
  @ONLY
)

include(ExternalProject)
ExternalProject_Add(
    mavlink_build
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mavlink
    BINARY_DIR ${CMAKE_BINARY_DIR}/mavlink_build
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/install
               -DMAVLINK_DIALECT=jfi
               -DMAVLINK_VERSION=2.0
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install
)

install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/install/include/mavlink/
    DESTINATION include/mavlink
)

ament_export_include_directories(include)

ament_package()
