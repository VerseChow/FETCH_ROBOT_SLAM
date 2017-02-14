#include "action_model.hpp"


ActionModel::ActionModel(float a1, float a2, float a3, float a4, float t, const geometry_msgs::Pose2D& odom_r):
a1(a1), 
a2(a2), 
a3(a3), 
a4(a4),
threshold(t),//0.01
pose_pre(odom_r)
{
}

ActionModel::ActionModel(){}

bool ActionModel::update_Action(const geometry_msgs::Pose2D& odom_r)
{
	printf("begin update_Action\n");
	pose_cur = odom_r;
	u_x = pose_cur.x-pose_pre.x;
	u_y = pose_cur.y-pose_pre.y;
	u_theta = pose_cur.theta-pose_pre.theta;
	//printf("finish update\n");
	if(fabs(u_x)<=threshold && fabs(u_y)<=threshold && fabs(u_theta)<=threshold)
		return	false;
	else
		return	true;
}
std::vector<Particle> ActionModel::apply_Action(std::vector<Particle>& particles)
{
	std::default_random_engine generator;	
	float d_rot1, d_rot2, d_trans;
	float d_rot1_e, d_rot2_e, d_trans_e;

	d_rot1 = atan2(u_y, u_x)-pose_pre.theta;
	d_trans = sqrt(u_x*u_x+u_y*u_y);
	d_rot2 = u_theta-d_rot1;

	for(int i=0; i<particles.size(); i++)
	{	

		printf("%d\t%d\n", i, (int)particles.size());	
		d_rot1_e = d_rot1-sample(a1*d_rot1+a2*d_trans, generator);
		d_trans_e = d_trans-sample(a3*d_trans+a4*(d_rot1+d_rot2), generator);
		d_rot2_e = d_rot2-sample(a1*d_rot2+a2*d_trans, generator);

		particles.at(i).pose.x +=  d_trans_e*cos(particles[i].pose.theta+d_rot1_e);
		particles.at(i).pose.y +=  d_trans_e*sin(particles[i].pose.theta+d_rot1_e);
		particles.at(i).pose.theta += d_rot1_e+d_rot2_e;
		particles.at(i).pose.theta = wrap_to_pi(particles.at(i).pose.theta);

		printf("%f\t%f\t%f\t\n", particles[i].pose.x, particles[i].pose.y, particles[i].pose.theta);
	}

	pose_pre = pose_cur;
	printf("finish sampling\n");
	return particles;
}

float ActionModel::sample(float sigma, std::default_random_engine& generator)
{

	std::normal_distribution<double> distribution(0, sigma);
	float number = (float)distribution(generator);
	return number;
/*
	float x1, x2, w, y1;
	float m=0;
	static float y2;
	static int use_last = 0;

	if (use_last)		        
	{
		y1 = y2;
		use_last = 0;
	}
	else
	{
		do {
			x1 = (float)(rand() % 100)/100;
    		x2 = (float)(rand() % 100)/100;
			w = x1 * x1 + x2 * x2;
		} while ( w >= 1.0 );

		w = sqrt( (-2.0 * log( w ) ) / w );
		y1 = x1 * w;
		y2 = x2 * w;
		use_last = 1;
	}
	return( m + y1 * sigma );
	*/
}