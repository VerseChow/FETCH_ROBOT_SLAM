#ifndef SENSOR_MODEL_HPP
#define SENSOR_MODEL_HPP

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

#include "mapping.hpp"
#include "angle_functions.hpp"

#include "datatypes.h"

struct Particle;

class SensorModel
{
	public:

	    /**
	    * Constructor for SensorModel.
	    */
	    SensorModel(float maxLaserDistance, float s, float offset);
	    SensorModel();
	    ~SensorModel();
	    /**
	    * likelihood computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
	    * 
	    * \param    particle            Particle for which the log-likelihood will be calculated
	    * \param    laser_scann                Laser scan to use for estimating log-likelihood
	    * \param    map                 Current map of the environment
	    * \return   Likelihood of the particle given the current map and laser scan.
	    */
	    float Likelihood(const Particle& particle, const sensor_msgs::LaserScan& laser_scann, const nav_msgs::OccupancyGrid& O_map);

	private:
    	float sigma_hit_;
		float kMaxLaserDistance_;
		float offset_;

    	float Gaussian(float m, float sigma, float zk);
    	float Max_dit(float zk);
    	geometry_msgs::Point Bresenham(int i0, int j0, int xt, int yt, const nav_msgs::OccupancyGrid& map);
    	bool Ifsteep(int x0, int y0, int xl, int yl);
};





#endif