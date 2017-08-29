cmake_minimum_required(VERSION 2.8.3)
project(planar_move)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
    DEPENDS gazebo
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    include/planar_move/planar_move.h
    src/planar_move.cpp
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
