#include "cvt2world_node.h"

Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();

// std::string POSE_TOPIC1="/UAV3/vins_estimator/camera_pose";
// std::string DEPTH_TOPIC1="/UAV3/camera/depth/color/points";
queue<sensor_msgs::PointCloud2::ConstPtr> depth_buf1;
queue<nav_msgs::Odometry::ConstPtr> pose_buf1;

// std::string POSE_TOPIC2="/vins_estimator/camera_pose";
// std::string DEPTH_TOPIC2="/camera/depth/color/points";
queue<sensor_msgs::PointCloud2::ConstPtr> depth_buf2;
queue<nav_msgs::Odometry::ConstPtr> pose_buf2;

ros::Publisher pub_point_cloud;
std::string PUBLISH_DEPTH_TOPIC="/depth_cloud";
std::mutex m_buf;
double offsets[2]={0.0, 0.8};

void registerPub(ros::NodeHandle n)
{
    pub_point_cloud= n.advertise<sensor_msgs::PointCloud2>(PUBLISH_DEPTH_TOPIC, 100);
}

void UAV1_pose_callback(const nav_msgs::OdometryConstPtr pose_msg)
{
    m_buf.lock();
    pose_buf1.push(pose_msg);
    m_buf.unlock();
}

void UAV1_depth_callback(const sensor_msgs::PointCloud2ConstPtr pcl_msg)
{
    m_buf.lock();
    depth_buf1.push(pcl_msg);
    m_buf.unlock();
}

void UAV2_pose_callback(const nav_msgs::OdometryConstPtr pose_msg)
{
    m_buf.lock();
    pose_buf2.push(pose_msg);
    m_buf.unlock();
}

void UAV2_depth_callback(const sensor_msgs::PointCloud2ConstPtr pcl_msg)
{
    m_buf.lock();
    depth_buf2.push(pcl_msg);
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

sensor_msgs::PointCloud2 is_sync(queue<sensor_msgs::PointCloud2::ConstPtr> &depth_buf, queue<nav_msgs::Odometry::ConstPtr> &pose_buf, double offset)
{
    sensor_msgs::PointCloud2 depth_world_msg;

        double t_depth = depth_buf.front()->header.stamp.sec + depth_buf.front()->header.stamp.nsec * (1e-9);
        double t_pose = pose_buf.front()->header.stamp.sec  + pose_buf.front()->header.stamp.nsec * (1e-9);
        
        // cout<<t_depth-t_pose<<endl;
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
        else {
            auto depth_msg = depth_buf.front();
            depth_buf.pop();
            auto pose_msg = pose_buf.front();
            pose_buf.pop();

            std::cout << "---------------------------------" << endl;
            std::cout << "< Synched time >" << endl;
            std::cout << "depth: " << std::fixed << t_depth << " cam pose:" << std::fixed << t_pose << std::endl; 

            // convert depth ROS msg => pcl
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*depth_msg, pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr depth_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromPCLPointCloud2(pcl_pc2, *depth_cloud);

            Eigen::Vector3d t_wc(pose_msg->pose.pose.position.x + offset, 
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

            pcl::toROSMsg(*depth_world, depth_world_msg); //do not filter depth cloud, but change coord
            depth_world_msg.header.frame_id = "world";
            pub_point_cloud.publish(depth_world_msg);
            
            //depth_world_msg = convert2World(depth_buf, pose_buf);
        }
    
    return depth_world_msg;

}
void sync_process()
{
    while(1)
    {
        sensor_msgs::PointCloud2 UAV1_depth_world_msg;
        sensor_msgs::PointCloud2 UAV2_depth_world_msg;

        m_buf.lock();

        if(!depth_buf1.empty() and !pose_buf1.empty())
        {
            UAV1_depth_world_msg = is_sync(depth_buf1, pose_buf1, offsets[0]);
        }
        if(!depth_buf2.empty() and !pose_buf2.empty())
        {
            UAV2_depth_world_msg = is_sync(depth_buf2, pose_buf2, offsets[1]);
        }

        m_buf.unlock();

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "cvt2world");
	auto n = ros::NodeHandle("~");

    ros::Subscriber sub_UAV1_pose = n.subscribe("/UAV1_pose_cloud_in",100, UAV1_pose_callback);
    ros::Subscriber sub_UAV1_depth = n.subscribe("/UAV1_depth_cloud_in", 100, UAV1_depth_callback);

    ros::Subscriber sub_UAV2_pose = n.subscribe("/UAV2_pose_cloud_in",100, UAV2_pose_callback);
    ros::Subscriber sub_UAV2_depth = n.subscribe("/UAV2_depth_cloud_in", 100, UAV2_depth_callback);

    registerPub(n);
    std::thread sync_thread{sync_process};

    ros::spin();

    return 0;
}
