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


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%

geometry_msgs::Pose2D pose_r;


sensor_msgs::LaserScan laser_scan;
geometry_msgs::Vector3 translation_info;

void draw_laser_scan(const sensor_msgs::LaserScan laser_scan, const ros::Publisher marker_pub, const geometry_msgs::Pose2D& pose_r);

void laser_receive(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{ 
    laser_scan.angle_min = -laser_msg->angle_max;//becase laser scan fram rotate 180 degrees according to the fram of map
    laser_scan.angle_max = -laser_msg->angle_min;
    laser_scan.angle_increment = laser_msg->angle_increment;                                    
    laser_scan.scan_time = laser_msg->scan_time;
    laser_scan.range_min = laser_msg->range_min;
    laser_scan.range_max = laser_msg->range_max;
    laser_scan.ranges = laser_msg->ranges;
    std::reverse(laser_scan.ranges.begin(), laser_scan.ranges.end());
    laser_scan.intensities = laser_msg->intensities;
}
// %EndTag(CALLBACK)%
void tranform_receive(const tf2_msgs::TFMessage::ConstPtr& TF_Message)
{
    std::string frame_name = "laser_link";
    for(int i=0;i<TF_Message->transforms.size();i++)
    {
        if (TF_Message->transforms[i].child_frame_id == frame_name)
        {
            translation_info = TF_Message->transforms[1].transform.translation;
            printf("%f\t%f\t%f\n", translation_info.x, translation_info.y, translation_info.z);
        }
    }
}

void odom_receive(const nav_msgs::Odometry::ConstPtr& Odom_msg)
{
    double roll, pitch, yaw;

    pose_r.x = Odom_msg->pose.pose.position.x;
    pose_r.y = Odom_msg->pose.pose.position.y;

    tf::Quaternion q(Odom_msg->pose.pose.orientation.x, Odom_msg->pose.pose.orientation.y, Odom_msg->pose.pose.orientation.z, Odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);
    pose_r.theta = yaw;

}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "laser_re");
	float count = 0;
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  ros::Rate r(30);
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  printf("start listening\n");
  ros::Subscriber sub_scan = n.subscribe("/base_scan", 1000, laser_receive);
  ros::Subscriber sub_tf = n.subscribe("/tf", 1000, tranform_receive);
  ros::Subscriber sub_odom = n.subscribe("/odom_combined", 1000, odom_receive);
  //init tranform_receive
  nav_msgs::OccupancyGrid Fetch_map;
  Fetch_map.header.seq = count;
  Fetch_map.header.frame_id = "/odom";
  Fetch_map.header.stamp = ros::Time::now();
  Fetch_map.info.map_load_time = ros::Time::now();
  Fetch_map.info.resolution = 0.05;//m/cell
  Fetch_map.info.width = 1000;
  Fetch_map.info.height = 1000;

  Fetch_map.info.origin.position.x = -25.0;
  Fetch_map.info.origin.position.y = -25.0;
  Fetch_map.info.origin.position.z = 0.0;
  Fetch_map.info.origin.orientation.w = 1.0;
  Fetch_map.data.assign(Fetch_map.info.width*Fetch_map.info.height, 50);
  //int data[2] = {100, 100};
  //Fetch_map.data = &data;
  //printf("%d\n", Fetch_map.data[0]);
  Mapping O_gmapping(25, 1, 1);

// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  while (ros::ok())
  {
    // Init occupancy grid
      Fetch_map.header.seq = count;
      //map_info_pub.publish(Fetch_map.info);

      map_pub.publish(Fetch_map);

      //init laser marker rviz
      draw_laser_scan(laser_scan, marker_pub, pose_r);

      ros::spinOnce();
      if(pose_r.theta != 0)
          O_gmapping.updateMap(laser_scan, pose_r, Fetch_map);//Update map
    	r.sleep();
      count++;
  }
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
void draw_laser_scan(const sensor_msgs::LaserScan laser_scan, const ros::Publisher marker_pub, const geometry_msgs::Pose2D& pose_r)
{
      float robot_angle=pose_r.theta;
      float angle;

      visualization_msgs::Marker line_list;
      line_list.header.frame_id = "/odom";
      line_list.header.stamp = ros::Time::now();
      line_list.ns = "points_and_lines";
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
          
          p.x = pose_r.x+0.23*cos(pose_r.theta);//laser translation 0.23
          p.y = pose_r.y+0.23*sin(pose_r.theta);
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