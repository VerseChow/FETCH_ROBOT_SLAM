#include "utils.h"

void DrawLaserScan(const sensor_msgs::LaserScan laser_scan,\
						      const ros::Publisher marker_pub,\
						      const geometry_msgs::Pose2D& pose_r,\
						      const geometry_msgs::Vector3 translation_info)
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

      /*send laser data to rviz*/
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

void DrawParticles(const ros::Publisher marker_pub,\
	              const std::vector<Particle> sample_particles)
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
    }
    marker_pub.publish(Points);
}