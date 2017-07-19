#ifndef DATATYPES_H
#define DATATYPES_H
#include "boost/shared_ptr.hpp"

#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Header.h"

#define pi 3.1415926

struct Particle
{   
    geometry_msgs::Pose2D pose;
    float weight;
};

struct pose_xyt
{
    std_msgs::Header header;
    geometry_msgs::Pose2D pose;
};
typedef boost::shared_ptr<pose_xyt> Pose_xyt;

typedef std::vector<Particle> particles;
typedef boost::shared_ptr<std::vector<Particle>> Particles;

#endif