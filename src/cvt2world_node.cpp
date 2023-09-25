#include "cvt2world_node.h"

Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
ros::Publisher pub_point_cloud;

// std::string POSE_TOPIC="/vins_estimator/camera_pose";
// std::string DEPTH_TOPIC="/camera/depth/color/points";
std::string PUBLISH_DEPTH_TOPIC="/depth_cloud";

queue<sensor_msgs::PointCloud2::ConstPtr> depth_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
std::mutex m_buf;

void registerPub(ros::NodeHandle n)
{
    pub_point_cloud= n.advertise<sensor_msgs::PointCloud2>(PUBLISH_DEPTH_TOPIC, 100);
}

void pose_callback(const nav_msgs::OdometryConstPtr pose_msg)
{
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
}

void depth_callback(const sensor_msgs::PointCloud2ConstPtr pcl_msg)
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
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;
        sensor_msgs::PointCloud2::ConstPtr depth_msg = NULL;
        double time = 0;

        m_buf.lock();
        if(!depth_buf.empty() and !pose_buf.empty())
        {
            double t_depth = depth_buf.front()->header.stamp.sec + depth_buf.front()->header.stamp.nsec * (1e-9);
            double t_pose  = pose_buf.front()->header.stamp.sec  + pose_buf.front()->header.stamp.nsec * (1e-9);

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
                double totla_begin = ros::Time::now().toSec();
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

                double begin = ros::Time::now().toSec();
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_depth_world = filterCloud(depth_world);
                double end = ros::Time::now().toSec();
                cout<<"filtering time: "<<end-begin<<endl;

                sensor_msgs::PointCloud2 depth_world_msg;

                pcl::toROSMsg(*filtered_depth_world, depth_world_msg);  //filter depth cloud
                //pcl::toROSMsg(*depth_world, depth_world_msg); //do not filter depth cloud, but change coord

                depth_world_msg.header.frame_id = "world";

                pub_point_cloud.publish(depth_world_msg);

                double total_end = ros::Time::now().toSec();
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
    ros::init(argc, argv, "cvt2world");
	auto n = ros::NodeHandle("~");

    ros::Subscriber sub_pose = n.subscribe("/pose_cloud_in",1000, pose_callback);
    ros::Subscriber sub_depth = n.subscribe("/depth_cloud_in", 1000, depth_callback);
    registerPub(n);
    std::thread sync_thread{sync_process};

    
    ros::spin();


    return 0;
}
