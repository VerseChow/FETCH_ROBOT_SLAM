#ifndef ACTION_MODEL_HPP
#define ACTION_MODEL_HPP 

#include "main.hpp"

struct Particle;

class ActionModel
{
	public:	
		ActionModel(float a1, float a2, float a3, float a4, float t, const geometry_msgs::Pose2D& odom_r);
		ActionModel();
		bool update_Action(const geometry_msgs::Pose2D& odom_r);
		std::vector<Particle> apply_Action(std::vector<Particle>& particles);
	private:
		float a1, a2, a3, a4;
		float u_x, u_y, u_theta;
		float threshold;
		geometry_msgs::Pose2D pose_pre, pose_cur;
		float sample(float sigma, std::default_random_engine& generator);
};

#endif 
