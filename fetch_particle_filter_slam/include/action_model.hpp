#ifndef ACTION_MODEL_HPP
#define ACTION_MODEL_HPP 

#include "vector"
#include "random"

#include "boost/make_shared.hpp"

#include "geometry_msgs/Pose2D.h"
#include "angle_functions.hpp"

#include "datatypes.h"

class ActionModel
{
	public:	
		ActionModel(float a1, float a2, float a3, float a4, float t,\
					const geometry_msgs::Pose2D& odom_r);
		ActionModel();
		~ActionModel();
		bool UpdateAction(const geometry_msgs::Pose2D& odom_r);
		Particles ApplyAction(Particles& particles);
	private:
		float a1_, a2_, a3_, a4_;
		float u_x_, u_y_, u_theta_;
		float threshold_;
		geometry_msgs::Pose2D pose_pre_, pose_cur_;
		float Sample(float sigma, std::default_random_engine& generator);
};

#endif 
