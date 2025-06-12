cmake_minimum_required(VERSION 3.8)
project(protocol)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/gmcs_proto.cpp
    src/convert_msg.cpp
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    geometry_msgs
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Export library
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


# ==========================
# Unit Test Configuration
# ==========================
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_gmcs_proto test/test_gmcs_proto.cpp)
    target_link_libraries(test_gmcs_proto protocol)
    install(TARGETS test_gmcs_proto DESTINATION lib/${PROJECT_NAME})
endif()


ament_package()
