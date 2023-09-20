cmake_minimum_required(VERSION 3.5)
project(cvt2world)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PCL REQUIRED )
find_package(pcl_conversions REQUIRED)
find_package(Eigen3)
include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#include_directories("~/catkin_ws/src/cvt2world/include")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_executable(cvt2world_node src/cvt2world_node.cpp)
target_link_libraries(cvt2world_node ${PCL_LIBRARIES})
ament_target_dependencies(cvt2world_node rclcpp rcpputils pcl_conversions std_msgs sensor_msgs nav_msgs)

install(TARGETS cvt2world_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
