##
# Copyright (C) 2019 by AutoSense Organization. All rights reserved.
# Gary Chan <chenshj35@mail2.sysu.edu.cn>
##

cmake_minimum_required(VERSION 2.8.3)
project(erp42_object_tracker_node)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    # ROS core
    roscpp
    # ROS messages
    std_msgs
    sensor_msgs
    geometry_msgs
    # ROS PCL
    pcl_conversions
    pcl_ros
    # 3rd modules for ROS, like Eigen
    cmake_modules
    # local packages
    common_lib
    segmenters_lib
    # needed for detection_node
    roi_filters_lib
    tracking_lib
    feature_extractors_lib
)


## System dependencies are found with CMake's conventions
### PCL
find_package(PCL 1.8 REQUIRED COMPONENTS
    common
    io
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

catkin_package(
    CATKIN_DEPENDS object_builders_lib feature_extractors_lib
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########


### Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

### Declare a C++ executable
add_executable(${PROJECT_NAME}_detection
    src/detection_node.cpp
)
add_executable(${PROJECT_NAME}_tracking
    src/tracking_node.cpp
    src/ihgp/Matern32model.cpp 
    src/ihgp/InfiniteHorizonGP.cpp 
)

### Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_detection
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_tracking
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)