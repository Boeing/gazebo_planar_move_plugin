cmake_minimum_required(VERSION 3.8)
project(gazebo_planar_move_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)

target_include_directories(
    ${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} gazebo_dev tf2_geometry_msgs gazebo_ros nav_msgs sensor_msgs)
ament_export_dependencies(gazebo_dev)

ament_export_targets(PlanarMove HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(DIRECTORY include DESTINATION include)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT PlanarMove
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES
    DESTINATION include
)

# Add Tests
if(BUILD_TESTING)
    install(DIRECTORY test DESTINATION share/${PROJECT_NAME})
    find_package(launch_testing_ament_cmake REQUIRED)

    add_launch_test(test/test_plugin.py)

endif()

ament_package()
