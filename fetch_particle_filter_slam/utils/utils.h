#ifndef UTILS_H
#define UTILS_H 

#include "vector"

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "tf/transform_datatypes.h"
#include "datatypes.h"

void DrawLaserScan(const sensor_msgs::LaserScan laser_scan,\
                  const ros::Publisher marker_pub,\
                  const geometry_msgs::Pose2D& pose_r,\
                  const geometry_msgs::Vector3 translation_info);

void DrawParticles(const ros::Publisher marker_pub,\
                  const Particles sample_particles);

void DrawArrow(const ros::Publisher marker_pub,\
              Pose_xyt pose);

void DrawPath(const ros::Publisher marker_pub,\
              Pose_xyt pose, visualization_msgs::Marker& Points);

#endif