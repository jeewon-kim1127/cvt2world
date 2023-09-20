
#pragma once
 
using namespace std;

#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <queue>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_WARN RCUTILS_LOG_WARN
#define ROS_ERROR RCUTILS_LOG_ERROR
