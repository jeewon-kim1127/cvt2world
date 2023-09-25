cmake_minimum_required(VERSION 3.0.2)
project(cvt2world)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(PACKAGE_DEPENDENCIES
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
)

# find dependencies
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(PCL REQUIRED)
find_package(Eigen3)

include_directories(include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

#include_directories("~/catkin_ws/src/cvt2world/include")
catkin_package()

add_executable(cvt2world_node src/cvt2world_node.cpp)
target_link_libraries(cvt2world_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cvt2world_multi_node src/cvt2world_multi_node.cpp)
target_link_libraries(cvt2world_multi_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})