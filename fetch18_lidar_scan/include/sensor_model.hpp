#ifndef SENSOR_MODEL_HPP
#define SENSOR_MODEL_HPP

#include "main.hpp"
#include "mapping.hpp"

struct  Particle;

class SensorModel
{
	public:

	    /**
	    * Constructor for SensorModel.
	    */
	    SensorModel(float maxLaserDistance, float s, float offset);
	    SensorModel();
	    /**
	    * likelihood computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
	    * 
	    * \param    particle            Particle for which the log-likelihood will be calculated
	    * \param    scan                Laser scan to use for estimating log-likelihood
	    * \param    map                 Current map of the environment
	    * \return   Likelihood of the particle given the current map and laser scan.
	    */

	    float likelihood(const Particle& particle, const sensor_msgs::LaserScan& laser_scann, const nav_msgs::OccupancyGrid& O_map);

	private:
    	
    	float	sigma_hit;
		float	kMaxLaserDistance_;
		float 	offset;

    	float gaussian(float m, float sigma, float zk);
    	float max_dit(float zk);
    	geometry_msgs::Point Bresenham(int i0, int j0, int xt, int yt, const nav_msgs::OccupancyGrid& map);
    	bool ifsteep(int x0, int y0, int xl, int yl);
};





#endif