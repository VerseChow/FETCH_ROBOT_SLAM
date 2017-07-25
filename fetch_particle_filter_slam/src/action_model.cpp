#include "action_model.hpp"

ActionModel::ActionModel(float a1, float a2, float a3, float a4, float t,\
						const geometry_msgs::Pose2D& odom_r):
a1_(a1), 
a2_(a2), 
a3_(a3), 
a4_(a4),
threshold_(t),//0.01
pose_pre_(odom_r)
{
}

ActionModel::ActionModel(){}

ActionModel::~ActionModel(){}

bool ActionModel::UpdateAction(const geometry_msgs::Pose2D& odom_r)
{
	pose_cur_ = odom_r;
	u_x_ = pose_cur_.x-pose_pre_.x;
	u_y_ = pose_cur_.y-pose_pre_.y;
	u_theta_ = wrap_to_pi(pose_cur_.theta-pose_pre_.theta);

	if(fabs(u_x_)<=threshold_ && fabs(u_y_)<=threshold_ && fabs(u_theta_)<=threshold_)
		return	false;
	else
		return	true;
}

Particles ActionModel::ApplyAction(Particles& particles)
{
	std::default_random_engine generator;	
	float d_rot1, d_rot2, d_trans;
	float d_rot1_e, d_rot2_e, d_trans_e;

	d_rot1 = atan2(u_y_, u_x_)-pose_pre_.theta;
	d_trans = sqrt(u_x_*u_x_+u_y_*u_y_);
	d_rot2 = u_theta_-d_rot1;

	for (int i=0; i<particles->size(); i++)
	{	
		d_rot1_e = d_rot1-Sample(a1_*d_rot1+a2_*d_trans, generator);
		d_trans_e = d_trans-Sample(a3_*d_trans+a4_*(d_rot1+d_rot2), generator);
		d_rot2_e = d_rot2-Sample(a1_*d_rot2+a2_*d_trans, generator);

		particles->at(i).pose.x +=  d_trans_e*cos(particles->at(i).pose.theta+d_rot1_e);
		particles->at(i).pose.y +=  d_trans_e*sin(particles->at(i).pose.theta+d_rot1_e);
		particles->at(i).pose.theta += d_rot1_e+d_rot2_e;
		particles->at(i).pose.theta = wrap_to_pi(particles->at(i).pose.theta);
	}

	pose_pre_ = pose_cur_;
	//printf("finish sampling\n");
	return particles;
}

float ActionModel::Sample(float sigma, std::default_random_engine& generator)
{

	std::normal_distribution<double> distribution(0, sigma);
	float number = (float)distribution(generator);
	return number;
}