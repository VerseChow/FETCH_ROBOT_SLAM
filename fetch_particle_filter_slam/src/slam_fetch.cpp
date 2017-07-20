#include "ros/ros.h"
#include "iostream"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "tf2_msgs/TFMessage.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"

#include "boost/make_shared.hpp"

#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "mapping.hpp"
#include "particle_filter.hpp"
#include "utils.h"


Pose_xyt pose_odom = boost::make_shared<pose_xyt>();
Pose_xyt slam_pose = boost::make_shared<pose_xyt>();
float theta_pre;
float theta_cur;
float u_theta;
float u_motion;
sensor_msgs::LaserScan laser_scan;
geometry_msgs::Vector3 translation_info;

Particles samples = boost::make_shared<particles>();

bool first_tf = false;

bool first = false;


void TransReceive(const tf2_msgs::TFMessage::ConstPtr& TF_Message)
{
 
    std::string frame_name_1 = "laser_link";

    for(int i=0;i<TF_Message->transforms.size();i++)
    {
        if (TF_Message->transforms[i].child_frame_id == frame_name_1)
        {
            translation_info = TF_Message->transforms[i].transform.translation; 
        }      
    }
    first_tf = true;
}

void MsgsReceiveCallBack(const sensor_msgs::LaserScan::ConstPtr& laser_msg,\
                            const nav_msgs::Odometry::ConstPtr& Odom_msg)
{
    laser_scan.header = laser_msg->header;
    laser_scan.angle_min = -laser_msg->angle_max;//becase laser scan fram rotate 180 degrees according to the fram of map
    laser_scan.angle_max = -laser_msg->angle_min;
    laser_scan.angle_increment = laser_msg->angle_increment;                                    
    laser_scan.scan_time = laser_msg->scan_time;
    laser_scan.range_min = laser_msg->range_min;
    laser_scan.range_max = laser_msg->range_max;
    laser_scan.ranges = laser_msg->ranges;
    std::reverse(laser_scan.ranges.begin(), laser_scan.ranges.end());
    laser_scan.intensities = laser_msg->intensities;
    
    double roll, pitch, yaw;
    pose_odom->header = Odom_msg->header;
    pose_odom->pose.x = Odom_msg->pose.pose.position.x;
    pose_odom->pose.y = Odom_msg->pose.pose.position.y;

    tf::Quaternion q(Odom_msg->pose.pose.orientation.x, \
                     Odom_msg->pose.pose.orientation.y, \
                     Odom_msg->pose.pose.orientation.z, \
                     Odom_msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);
    pose_odom->pose.theta = wrap_to_pi(yaw);
    theta_cur = wrap_to_pi(yaw);
    u_theta = theta_cur-theta_pre;
    theta_pre = theta_cur;

    first = true;
}


int main(int argc, char **argv)
{

    int count = 1;

    pcl::PointCloud<pcl::PointXYZRGB> rgbdmap;
 

    ros::init(argc, argv, "Slam_Fetch");

    ros::NodeHandle n;
    
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher rgbd_map_raw_pub = n.advertise<sensor_msgs::PointCloud2>("rgbd_map_raw", 10);
    ros::Rate r(30);

    printf("start listening\n");

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(n, "/base_scan", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, "/odom_combined", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), laser_sub, odom_sub);
    sync.registerCallback(boost::bind(&MsgsReceiveCallBack, _1, _2));
    ros::Subscriber tf_sub = n.subscribe("/tf", 1, TransReceive);

    nav_msgs::OccupancyGrid Fetch_map;
    Fetch_map.header.seq = count;
    Fetch_map.header.frame_id = "/odom";
    Fetch_map.header.stamp = ros::Time::now();
    Fetch_map.info.map_load_time = ros::Time::now();
    Fetch_map.info.resolution = 0.05;//m/cell
    Fetch_map.info.width = 10000;
    Fetch_map.info.height = 10000;

    Fetch_map.info.origin.position.x = -250.0;
    Fetch_map.info.origin.position.y = -250.0;
    Fetch_map.info.origin.position.z = 0.0;
    Fetch_map.info.origin.orientation.w = 1.0;
    Fetch_map.data.assign(Fetch_map.info.width*Fetch_map.info.height, 50);

    Mapping O_gmapping(25,10,1);
    
    while(ros::ok())
    {
        if(first && first_tf)
        {
            tf_sub.shutdown();
            break;
        }
        ros::spinOnce();
    }

    ParticleFilter P_Filter(1000, translation_info.x);
    P_Filter.InitializeFilterAtPose(pose_odom->pose);
    slam_pose = pose_odom;

    while (ros::ok())
    {
        Fetch_map.header.seq = count;

        if(fabs(u_theta)<=0.05)
          O_gmapping.UpdateMap(laser_scan, slam_pose->pose, Fetch_map, translation_info.x);//Update 
       
        while(count<=1)
        {
          slam_pose->pose = P_Filter.UpdateFilter(pose_odom->pose, laser_scan, Fetch_map);
          count++;
        }
        count = 1;
        samples = P_Filter.GetParticles();

        //printf("Draw Particles\n");
        map_pub.publish(Fetch_map); 
        DrawParticles(marker_pub, samples);

        ros::spinOnce();
    }
    return 0;
}
