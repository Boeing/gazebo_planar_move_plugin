cmake_minimum_required(VERSION 3.0.2)
project(gazebo_planar_move_plugin)

add_definitions(-std=c++11)

find_package(
    catkin REQUIRED
    COMPONENTS
        gazebo_ros
        geometry_msgs
        nav_msgs
        roscpp
        sensor_msgs
        tf2_geometry_msgs
        tf2_ros
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
    CATKIN_DEPENDS
    gazebo_ros
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    tf2_geometry_msgs
    tf2_ros
    INCLUDE_DIRS
    include
    LIBRARIES
    ${PROJECT_NAME}
    DEPENDS
    Boost
    gazebo
)

include_directories(include)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} include/${PROJECT_NAME}/${PROJECT_NAME}.h src/${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING
    PATTERN "*.h"
)

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_plugin.test)
endif()
