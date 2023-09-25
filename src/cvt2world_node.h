
#pragma once
 
using namespace std;

#include <cstdio>
#include <ros/ros.h>

#include <thread>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

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
