/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY out OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "main.hpp"
#include "mapping.hpp"
#include "particle_filter.hpp"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
//struct Pose_xyt;

Pose_xyt pose_odom;
Pose_xyt slam_pose;
float theta_pre;
float theta_cur;
float u_theta;
float u_motion;
sensor_msgs::LaserScan laser_scan;
geometry_msgs::Vector3 translation_info;


std::vector<Particle> samples;

pcl::PointCloud<pcl::PointXYZRGB> rgbd_map_raw;

sensor_msgs::PointCloud2 map_pcl2;

tf::Vector3 camera_trans;
tf::Quaternion camera_rot;

bool pcl_updated = false;
bool first_odom = false;
bool first_laser = false;
bool first_tf = false;


void draw_laser_scan(const sensor_msgs::LaserScan laser_scan, const ros::Publisher marker_pub, const geometry_msgs::Pose2D& pose_r);

void laser_receive(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
void draw_particles(const ros::Publisher marker_pub, const std::vector<Particle>& sampe_particles);
tf::StampedTransform get_rgbd_map();
/*
void *visualize_marker(void *threadid)
{
    int argc; 
    char **argv;
    ros::init(argc, argv, "marker_visualize");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Subscriber sub_scan = n.subscribe("/base_scan", 1000, laser_receive);
    ros::Rate r(30);

    while(ros::ok())
    {
        ros::spinOnce();

        draw_particles(marker_pub, samples);
        //draw_laser_scan(laser_scan, marker_pub, pose_r);
        r.sleep();
    }
    pthread_exit(NULL);

}*/


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


void laser_receive(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{ 
    //if(pose_odom.header.stamp.sec == laser_msg->header.stamp.sec)
    //{
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
    //}
}
// %EndTag(CALLBACK)%
void tranform_receive(const tf2_msgs::TFMessage::ConstPtr& TF_Message)
{
 
    //tf::StampedTransform transform;
    std::string frame_name_1 = "laser_link";

   
    for(int i=0;i<TF_Message->transforms.size();i++)
    {
        if (TF_Message->transforms[i].child_frame_id == frame_name_1)
        {
            
            translation_info = TF_Message->transforms[i].transform.translation; 

            //printf("%f\t%f\t%f\n", translation_info.x, translation_info.y, translation_info.z);
        }      
          
    }
    first_tf = true;
}

void odom_receive(const nav_msgs::Odometry::ConstPtr& Odom_msg)
{
    if(first_odom)
    {
        double roll, pitch, yaw;
        pose_odom.header = Odom_msg->header;
        pose_odom.pose.x = Odom_msg->pose.pose.position.x;
        pose_odom.pose.y = Odom_msg->pose.pose.position.y;

        tf::Quaternion q(Odom_msg->pose.pose.orientation.x, Odom_msg->pose.pose.orientation.y, Odom_msg->pose.pose.orientation.z, Odom_msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);

        m.getRPY(roll, pitch, yaw);
        pose_odom.pose.theta = wrap_to_pi(yaw);
        theta_cur = wrap_to_pi(yaw);
        u_theta = theta_cur-theta_pre;
        theta_pre = theta_cur;
    }
    else
    {
        double roll, pitch, yaw;
        pose_odom.header = Odom_msg->header;
        pose_odom.pose.x = Odom_msg->pose.pose.position.x;
        pose_odom.pose.y = Odom_msg->pose.pose.position.y;

        tf::Quaternion q(Odom_msg->pose.pose.orientation.x, Odom_msg->pose.pose.orientation.y, Odom_msg->pose.pose.orientation.z, Odom_msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);

        m.getRPY(roll, pitch, yaw);
        pose_odom.pose.theta = wrap_to_pi(yaw);
        //printf("1\n");
        theta_pre = pose_odom.pose.theta;
        theta_cur = pose_odom.pose.theta;
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
    //samples.resize(1000);
    ros::NodeHandle n;
    
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher rgbd_map_raw_pub = n.advertise<sensor_msgs::PointCloud2>("rgbd_map_raw", 10);
    ros::Rate r(30);
  // %Tag(SUBSCRIBER)%
    printf("start listening\n");
    ros::Subscriber sub_scan = n.subscribe("/base_scan", 1000, laser_receive);
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



    ParticleFilter P_Filter(1000, translation_info.x);
    P_Filter.initialize_FilterAtPose(pose_odom.pose);
    slam_pose = pose_odom;
  // %EndTag(SUBSCRIBER)%

    while (ros::ok())
    {

        /*
        Fetch_map.header.seq = count;

        if(fabs(u_theta)<=0.001)
          O_gmapping.updateMap(laser_scan, slam_pose.pose, Fetch_map, translation_info.x);//Update 
       
        while(count<=1)
        {
          slam_pose.pose = P_Filter.update_Filter(pose_odom.pose, laser_scan, Fetch_map);
          count++;
        }
        count = 1;
        samples = P_Filter.particles();

        //printf("Draw Particles\n");
        map_pub.publish(Fetch_map); 
        draw_particles(marker_pub, samples);
      */

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

        //ros::spinOnce();
        
    }
// %EndTag(SPIN)%

    return 0;
}
// %EndTag(FULLTEXT)%
inline void draw_laser_scan(const sensor_msgs::LaserScan laser_scan, const ros::Publisher marker_pub, const geometry_msgs::Pose2D& pose_r)
{
      float robot_angle=pose_r.theta;
      float angle;

      visualization_msgs::Marker line_list;
      line_list.header.frame_id = "/odom";
      line_list.header.stamp = ros::Time::now();
      line_list.ns = "scan_lines";
      line_list.action = visualization_msgs::Marker::ADD;
      line_list.pose.orientation.w = 1.0;
      line_list.color.g = 1.0;
      line_list.color.a = 1.0;

      line_list.type = visualization_msgs::Marker::LINE_LIST;
      line_list.id = 0;
      line_list.scale.x = 0.001;
      line_list.scale.y = 0.001;
      line_list.scale.z = 0.001;

      //send laser data to rviz
      for(uint i=0;i<laser_scan.ranges.size();i++)
      {
          if(angle <= laser_scan.angle_max)
          {
              angle = laser_scan.angle_min+i*laser_scan.angle_increment;
          }
          else
          {
              angle = laser_scan.angle_max;
          }
          
          geometry_msgs::Point p;
          
          p.x = pose_r.x+translation_info.x*cos(pose_r.theta);//laser translation 0.23
          p.y = pose_r.y+translation_info.x*sin(pose_r.theta);
          p.z = translation_info.z;

          line_list.points.push_back(p);

          if (isnan(laser_scan.ranges[i]))
          {
              p.x += laser_scan.range_max*cos(robot_angle+angle);
              p.y = laser_scan.range_max*sin(robot_angle+angle);
          }
          else
          {
              p.x += laser_scan.ranges[i]*cos(robot_angle+angle);
              p.y = laser_scan.ranges[i]*sin(robot_angle+angle);
          }      

          line_list.points.push_back(p);  

      }
      marker_pub.publish(line_list);
}

inline void draw_particles(const ros::Publisher marker_pub, const std::vector<Particle>& sample_particles)
{
    visualization_msgs::Marker Points;
    Points.header.frame_id = "/odom";
    Points.header.stamp = ros::Time::now();
    Points.ns = "sample_particles";
    Points.action = visualization_msgs::Marker::ADD;
    Points.pose.orientation.w = 1.0;
    Points.color.r = 1.0;
    Points.color.a = 1.0;

    Points.type = visualization_msgs::Marker::POINTS;
    Points.id = 0;
    Points.scale.x = 0.05;
    Points.scale.y = 0.05;
    Points.scale.z = 0.05;

    for(int i=0; i<sample_particles.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = sample_particles[i].pose.x;
        p.y = sample_particles[i].pose.y;
        Points.points.push_back(p);
        //printf("drawing: %f\t%f\t\n", p.x, p.y);
    }

    marker_pub.publish(Points);
}

tf::StampedTransform get_rgbd_map()
{
    
    tf::TransformListener listener_camera_odom;
    tf::StampedTransform transform;
        //sensor_msgs::PointCloud2 pcl_out;
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