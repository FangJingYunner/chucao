cmake_minimum_required(VERSION 3.0.2)
project(objectcluster)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        roscpp#
        pcl_ros#
        pcl_conversions#
#        tf
        eigen_conversions #
        cv_bridge   #可以将sensor/Image 和OpenCV关联起来
        sensor_msgs#
        image_transport#
        nav_msgs#
#        message_generation
#        std_msgs
#        message_runtime
        #        boxmsgs

)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
#include(cmake/glog.cmake)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES objectcluster
#  CATKIN_DEPENDS cv_bridge eigen_conversions image_transport nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

link_directories(
        include
        ${OpenCV_LIBRARY_DIRS}
        ${PCL_LIBRARY_DIRS}
)


add_executable(ObjectSegVoxelGrid1 src/ObjectSeg_VoxelGrid.cc)
add_dependencies(ObjectSegVoxelGrid1 ${catkin_EXPORTED_TARGETS})
target_link_libraries(ObjectSegVoxelGrid1 ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ObjectSegVoxelGridM7 src/ObjectSeg_VoxelGridM7.cc)
add_dependencies(ObjectSegVoxelGridM7 ${catkin_EXPORTED_TARGETS})
target_link_libraries(ObjectSegVoxelGridM7 ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} ${PCL_LIBRARIES})
