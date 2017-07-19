#include "ros/ros.h"
#include "iostream"

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

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
//struct Pose_xyt;

Pose_xyt pose_odom = boost::make_shared<pose_xyt>();
Pose_xyt slam_pose = boost::make_shared<pose_xyt>();
float theta_pre;
float theta_cur;
float u_theta;
float u_motion;
sensor_msgs::LaserScan laser_scan;
geometry_msgs::Vector3 translation_info;


Particles samples = boost::make_shared<particles>();

pcl::PointCloud<pcl::PointXYZRGB> rgbd_map_raw;

sensor_msgs::PointCloud2 map_pcl2;

tf::Vector3 camera_trans;
tf::Quaternion camera_rot;

bool pcl_updated = false;
bool first_odom = false;
bool first_laser = false;
bool first_tf = false;

void LaserReceive(const sensor_msgs::LaserScan::ConstPtr& laser_msg);

tf::StampedTransform get_rgbd_map();


void pcl2_receive(const sensor_msgs::PointCloud2::ConstPtr& pcl2_msg)
{

    if(first_tf)
    {
        tf::TransformListener listener_camera_odom;
        tf::StampedTransform transform;
        //sensor_msgs::PointCloud2 pcl_out;
        try
        {
            listener_camera_odom.waitForTransform("/odom", "/head_camera_rgb_optical_frame", ros::Time(0), ros::Duration(10.0));
            listener_camera_odom.lookupTransform("/odom", "/head_camera_rgb_optical_frame", ros::Time(0), transform); 
          
            pcl::fromROSMsg(*pcl2_msg, rgbd_map_raw);

            sensor_msgs::PointCloud2 map_pcl2;
            pcl::PointCloud<pcl::PointXYZRGB> pcl_out;
            pcl_ros::transformPointCloud(rgbd_map_raw, pcl_out, transform);

            rgbd_map_raw = pcl_out;
        
            pcl_updated = true;
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
}


void LaserReceive(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
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
    first_laser = true;
}
// %EndTag(CALLBACK)%
void tranform_receive(const tf2_msgs::TFMessage::ConstPtr& TF_Message)
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

void odom_receive(const nav_msgs::Odometry::ConstPtr& Odom_msg)
{
    if(first_odom)
    {
        double roll, pitch, yaw;
        pose_odom->header = Odom_msg->header;
        pose_odom->pose.x = Odom_msg->pose.pose.position.x;
        pose_odom->pose.y = Odom_msg->pose.pose.position.y;

        tf::Quaternion q(Odom_msg->pose.pose.orientation.x, Odom_msg->pose.pose.orientation.y, Odom_msg->pose.pose.orientation.z, Odom_msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);

        m.getRPY(roll, pitch, yaw);
        pose_odom->pose.theta = wrap_to_pi(yaw);
        theta_cur = wrap_to_pi(yaw);
        u_theta = theta_cur-theta_pre;
        theta_pre = theta_cur;
    }
    else
    {
        double roll, pitch, yaw;
        pose_odom->header = Odom_msg->header;
        pose_odom->pose.x = Odom_msg->pose.pose.position.x;
        pose_odom->pose.y = Odom_msg->pose.pose.position.y;

        tf::Quaternion q(Odom_msg->pose.pose.orientation.x, Odom_msg->pose.pose.orientation.y, Odom_msg->pose.pose.orientation.z, Odom_msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);

        m.getRPY(roll, pitch, yaw);
        pose_odom->pose.theta = wrap_to_pi(yaw);
        //printf("1\n");
        theta_pre = pose_odom->pose.theta;
        theta_cur = pose_odom->pose.theta;
        first_odom = true;
    }

}

int main(int argc, char **argv)
{

    int count = 1;

    pcl::PointCloud<pcl::PointXYZRGB> rgbdmap;
 

    ros::init(argc, argv, "Slam_Fetch");
    tf::TransformListener listener_camera_odom;
    tf::StampedTransform transform;    
    //samples->resize(1000);
    ros::NodeHandle n;
    
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher rgbd_map_raw_pub = n.advertise<sensor_msgs::PointCloud2>("rgbd_map_raw", 10);
    ros::Rate r(30);
  // %Tag(SUBSCRIBER)%
    printf("start listening\n");
    ros::Subscriber sub_scan = n.subscribe("/base_scan", 1000, LaserReceive);
    ros::Subscriber sub_tf = n.subscribe("/tf", 1000, tranform_receive);
    ros::Subscriber sub_odom = n.subscribe("/odom_combined", 1000, odom_receive);
    //ros::Subscriber sub_pcl2 = n.subscribe("/head_camera/depth_registered/points", 1000, pcl2_receive);
    //init tranform_receive
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
        if(first_tf && first_laser && first_odom)
          
          break;
        ros::spinOnce();
    }

    try
    {
        listener_camera_odom.waitForTransform("/odom", "/head_camera_rgb_optical_frame", ros::Time(0), ros::Duration(10.0));
        listener_camera_odom.lookupTransform("/odom", "/head_camera_rgb_optical_frame", ros::Time(0), transform);
        camera_trans = transform.getOrigin();
        camera_rot = transform.getRotation();
    }
    catch(tf::TransformException ex)
    { 
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }



    ParticleFilter P_Filter(3000, translation_info.x);
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
        //DrawParticles(marker_pub, samples);
    
        /*
        try
        {
            listener_camera_odom.waitForTransform("/odom", "/head_camera_rgb_optical_frame", ros::Time(0), ros::Duration(10.0));
            listener_camera_odom.lookupTransform("/odom", "/head_camera_rgb_optical_frame", ros::Time(0), transform);
            tf::Vector3 temp_tans = transform.getOrigin();
            tf::Quaternion temp_rot = transform.getRotation();
            printf("%f\t%f\n",fabs((temp_tans-camera_trans).length()), fabs(wrap_to_pi_2((temp_rot-camera_rot).getAngle())));
            if((fabs((temp_tans-camera_trans).length())<0.001)&&(fabs(wrap_to_pi_2((temp_rot-camera_rot).getAngle())) < 0.001))
            {
                transform = get_rgbd_map();
                
                camera_trans = transform.getOrigin();
                camera_rot = transform.getRotation();
                

                pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());

                sensor_msgs::PointCloud2 map_pcl2;

                pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> OutlierFilter;
                pcl::VoxelGrid<pcl::PCLPointCloud2> VoxelFilter;
                rgbdmap +=  rgbd_map_raw;

                pcl::toPCLPointCloud2(rgbdmap, *cloud);
                
                VoxelFilter.setInputCloud (cloud);
                VoxelFilter.setLeafSize (0.01f, 0.01f, 0.01f);
                VoxelFilter.filter (*cloud);
                
                OutlierFilter.setInputCloud (cloud);
                OutlierFilter.setMeanK (10);
                OutlierFilter.setStddevMulThresh (1.0);
                OutlierFilter.filter (*cloud);

                pcl::fromPCLPointCloud2(*cloud, rgbdmap);

                pcl::toROSMsg(rgbdmap,map_pcl2);
                map_pcl2.header.frame_id = "/odom";
                rgbd_map_raw_pub.publish(map_pcl2);
                printf("render rgbd map\n");
            }
            else
            {
                camera_trans = temp_tans;
                camera_rot = temp_rot;
            }
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        /*
        if(pcl_updated)
        { 
            tf::TransformListener listener_camera_odom;
            tf::StampedTransform transform;
            try 
            {
                pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());

                //listener_camera_odom.waitForTransform("/odom", "/head_camera_rgb_optical_frame", ros::Time(0), ros::Duration(10.0));
                //listener_camera_odom.lookupTransform("/odom", "/head_camera_rgb_optical_frame", ros::Time(0), transform);
                sensor_msgs::PointCloud2 map_pcl2;
                //pcl::PointCloud<pcl::PointXYZRGB> pcl_out;
                //pcl_ros::transformPointCloud(rgbd_map_raw, pcl_out, transform);
                pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> OutlierFilter;
                pcl::VoxelGrid<pcl::PCLPointCloud2> VoxelFilter;
                rgbdmap +=  rgbd_map_raw;

                pcl::toPCLPointCloud2(rgbdmap, *cloud);
                
                VoxelFilter.setInputCloud (cloud);
                VoxelFilter.setLeafSize (0.01f, 0.01f, 0.01f);
                VoxelFilter.filter (*cloud);
                
                OutlierFilter.setInputCloud (cloud);
                OutlierFilter.setMeanK (50);
                OutlierFilter.setStddevMulThresh (1.0);
                OutlierFilter.filter (*cloud);

                pcl::fromPCLPointCloud2(*cloud, rgbdmap);

                pcl::toROSMsg(rgbdmap,map_pcl2);
                map_pcl2.header.frame_id = "/odom";
                rgbd_map_raw_pub.publish(map_pcl2);
                
                pcl_updated = false;
            }
            catch(tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

        }
        */

        ros::spinOnce();
    }
    return 0;
}

tf::StampedTransform get_rgbd_map()
{
    
    tf::TransformListener listener_camera_odom;
    tf::StampedTransform transform;
    try
    {
        listener_camera_odom.waitForTransform("/odom", "/head_camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));
        listener_camera_odom.lookupTransform("/odom", "/head_camera_rgb_optical_frame", ros::Time(0), transform); 
        sensor_msgs::PointCloud2::ConstPtr pcl2_msg  = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/head_camera/depth_registered/points", ros::Duration(1));
        pcl::fromROSMsg(*pcl2_msg, rgbd_map_raw);

        sensor_msgs::PointCloud2 map_pcl2;
        pcl::PointCloud<pcl::PointXYZRGB> pcl_out;
        pcl_ros::transformPointCloud(rgbd_map_raw, pcl_out, transform);

        rgbd_map_raw = pcl_out;   
        //pcl_updated = true;
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    return transform;
}