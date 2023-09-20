#include "cvt2world_node.h"

Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud;

std::string POSE_TOPIC="/vins_estimator/camera_pose";
std::string DEPTH_TOPIC="/UAV3/camera/depth/color/points";
std::string PUBLISH_DEPTH_TOPIC="/depth_cloud";

queue<sensor_msgs::msg::PointCloud2::ConstPtr> depth_buf;
queue<nav_msgs::msg::Odometry::ConstPtr> pose_buf;
std::mutex m_buf;

void registerPub(rclcpp::Node::SharedPtr n)
{
    pub_point_cloud= n->create_publisher<sensor_msgs::msg::PointCloud2>(PUBLISH_DEPTH_TOPIC, 100);
}

void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
{
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
}

void depth_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
{
    m_buf.lock();
    depth_buf.push(pcl_msg);
    m_buf.unlock();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr depth_world)
{
    pcl::PassThrough<pcl::PointXYZ> pass_filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr x_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr z_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pass_filter.setInputCloud(depth_world);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(-0.2, 0.2);
    pass_filter.setFilterLimitsNegative(false);
    pass_filter.filter(*z_tmp);

    pass_filter.setInputCloud(z_tmp);
    pass_filter.setFilterFieldName("x");
    pass_filter.setFilterLimits(-15, 15);
    pass_filter.setFilterLimitsNegative(false); //pass points in the range
    pass_filter.filter(*x_tmp);

    pass_filter.setInputCloud(x_tmp);
    pass_filter.setFilterFieldName("y");
    pass_filter.setFilterLimits(-15, 15);
    pass_filter.setFilterLimitsNegative(false); //pass points in the range
    pass_filter.filter(*ptCloud);

    return ptCloud;
}


void sync_process()
{
    while(1)
    {
        nav_msgs::msg::Odometry::ConstPtr pose_msg = NULL;
        sensor_msgs::msg::PointCloud2::ConstPtr depth_msg = NULL;
        double time = 0;

        m_buf.lock();
        if(!depth_buf.empty() and !pose_buf.empty())
        {
            double t_depth = depth_buf.front()->header.stamp.sec + depth_buf.front()->header.stamp.nanosec * (1e-9);
            double t_pose  = pose_buf.front()->header.stamp.sec  + pose_buf.front()->header.stamp.nanosec * (1e-9);

            // 0.003s sync tolerance
            if(t_depth < t_pose - 0.003)
            {
                depth_buf.pop();
                printf("throw depth\n");
            }
            else if(t_depth > t_pose + 0.003)
            {
                pose_buf.pop();
                printf("throw pose\n");
            }
            else
            {
                double totla_begin = rclcpp::Clock().now().seconds();
                time = t_depth;
                depth_msg = depth_buf.front();
                depth_buf.pop();
                pose_msg = pose_buf.front();
                pose_buf.pop();

                std::cout << "---------------------------------" << endl;
                std::cout << "< Synched time >" << endl;
                std::cout << "depth: " << std::fixed << t_depth << " cam pose:" << std::fixed << t_pose << std::endl; 

                // convert depth ROS msg => pcl
                pcl::PCLPointCloud2 pcl_pc2;
                pcl_conversions::toPCL(*depth_msg, pcl_pc2);
                pcl::PointCloud<pcl::PointXYZ>::Ptr depth_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::fromPCLPointCloud2(pcl_pc2, *depth_cloud);

                if (depth_cloud->size() == 0)// && campose_msg!=NULL )
                {
                    ROS_WARN("[IGNORE] depth cloud size is 0");
                    return ;
                }
                Eigen::Vector3d t_wc(pose_msg->pose.pose.position.x, 
                                        pose_msg->pose.pose.position.y, 
                                        pose_msg->pose.pose.position.z);
                Eigen::Quaterniond q_wc(pose_msg->pose.pose.orientation.w, 
                                        pose_msg->pose.pose.orientation.x, 
                                        pose_msg->pose.pose.orientation.y, 
                                        pose_msg->pose.pose.orientation.z);
                Eigen::Matrix3d R_wc = q_wc.toRotationMatrix();
                Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
                T_wc.block<3,3>(0,0) = R_wc;
                T_wc.block<3,1>(0,3) = t_wc;

                pcl::PointCloud<pcl::PointXYZ>::Ptr depth_world(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::transformPointCloud(*depth_cloud, *depth_world, T_wc);

                double begin = rclcpp::Clock().now().seconds();
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_depth_world = filterCloud(depth_world);
                double end = rclcpp::Clock().now().seconds();
                cout<<"filtering time: "<<end-begin<<endl;

                sensor_msgs::msg::PointCloud2 depth_world_msg;

                pcl::toROSMsg(*filtered_depth_world, depth_world_msg);  //filter depth cloud
                //pcl::toROSMsg(*depth_world, depth_world_msg); //do not filter depth cloud, but change coord

                depth_world_msg.header.frame_id = "world";

                pub_point_cloud->publish(depth_world_msg);

                double total_end = rclcpp::Clock().now().seconds();
                cout<<"total time: "<<total_end-totla_begin<<endl;

            }
        }
        m_buf.unlock();

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
	auto n = rclcpp::Node::make_shared("cvt2world");

    auto sub_pose = n->create_subscription<nav_msgs::msg::Odometry>(POSE_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), pose_callback);
    auto sub_depth = n->create_subscription<sensor_msgs::msg::PointCloud2>(DEPTH_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), depth_callback);
    registerPub(n);
    std::thread sync_thread{sync_process};

    
    rclcpp::spin(n);

    return 0;
}
