cmake_minimum_required(VERSION 3.8)
project(gazebo_planar_move_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED)

# ── Gazebo Harmonic via vendor packages ──────────────────────
# gz_sim_vendor ships gz-sim 8 (Harmonic)
# gz_plugin_vendor ships gz-plugin
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)

target_include_directories(
    ${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} rclcpp tf2_ros tf2_geometry_msgs nav_msgs sensor_msgs geometry_msgs)

target_link_libraries(${PROJECT_NAME} gz-sim::gz-sim gz-plugin::gz-plugin)

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

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

# Add Tests
if(BUILD_TESTING)
    install(DIRECTORY test DESTINATION share/${PROJECT_NAME})
    find_package(launch_testing_ament_cmake REQUIRED)

    add_launch_test(test/test_plugin.py)

endif()

ament_package()
