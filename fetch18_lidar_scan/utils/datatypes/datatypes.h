#ifndef DATATYPES_H
#define DATATYPES_H
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Header.h"

#define pi 3.1415926

struct Particle
{   
    geometry_msgs::Pose2D pose;
    float weight;
};

struct Pose_xyt
{
    std_msgs::Header header;
    geometry_msgs::Pose2D pose;
};

#endif