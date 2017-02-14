#include "sensor_model.hpp"


SensorModel::SensorModel(float maxLaserDistance, float s, float offset):
kMaxLaserDistance_(maxLaserDistance),
sigma_hit(s),
offset(offset)
{
}

SensorModel::SensorModel(){}

float SensorModel::likelihood(const Particle& particle, const sensor_msgs::LaserScan& laser_msg, const nav_msgs::OccupancyGrid& O_map)
{
   	int logtemp;
   	float weight=1;

	float dist, theta, z_k;
	float x_t, y_t;//where ray terminates
	int i0, j0, it, jt;
	bool isnan;

	geometry_msgs::Point grid_r, grid_t, grid_obstacle;
	geometry_msgs::Pose2D pose_r, pose_t;
    
	for(int scannum=0; scannum<laser_msg.ranges.size(); scannum++)//determin the line 
	{	
		//printf("%d 1\n", (int)scannum);
	    pose_r.x = particle.pose.x+offset*cos(particle.pose.theta);//laser scan frame translation 0.23 
	    pose_r.y = particle.pose.y+offset*sin(particle.pose.theta);
	    pose_r.theta = particle.pose.theta;
	    
	    theta = pose_r.theta+laser_msg.angle_min+laser_msg.angle_increment*scannum;
	    isnan = std::isnan(laser_msg.ranges[scannum]);

	    if (isnan||laser_msg.ranges[scannum]>=25)
	    {
	    	dist = kMaxLaserDistance_;
	    }
	    else
	    	dist = laser_msg.ranges[scannum];

	    pose_t.x = pose_r.x+dist*cos(theta);
	    pose_t.y = pose_r.y+dist*sin(theta);
	    
	    //printf("%d 2\n", (int)scannum);
	    /*grid index at the original pose and the terminal of the ray*/
	    grid_r = Mapping::global_position_to_grid_cell(pose_r, O_map);
	    grid_t = Mapping::global_position_to_grid_cell(pose_t, O_map);

	    i0 = (int)grid_r.x;
	    j0 = (int)grid_r.y;
	    it = (int)grid_t.x;
	    jt = (int)grid_t.y;

	   	Mapping::check_range(i0, j0, O_map);
	    Mapping::check_range(it, jt, O_map);
	    
	    //printf("%d 3\n", (int)scannum);
		grid_obstacle = Bresenham(i0, j0, it, jt, O_map);
		//printf("%d 4\n", (int)scannum);
		z_k = sqrt(((grid_obstacle.x-grid_r.x)*(grid_obstacle.x-grid_r.x)+
			  (grid_obstacle.y-grid_r.y)*(grid_obstacle.y-grid_r.y))*O_map.info.resolution);
		if(isnan)
			weight += max_dit(z_k);
		else
			weight += gaussian(dist, sigma_hit, z_k);
		//printf("%d 5\n", (int)scannum);
	}
	return weight;
}

float SensorModel::gaussian(float m, float sigma, float zk)
{
	return 1/sqrt(2*pi*sigma*sigma)*exp(-0.5*(((zk-m)*(zk-m))/(sigma*sigma)));
}

float SensorModel::max_dit(float zk)
{
	if((zk>=(kMaxLaserDistance_-1)) && zk<=kMaxLaserDistance_)
		return 1.;
	else
		return 0.;
}



geometry_msgs::Point SensorModel::Bresenham(int x0, int y0, int xl, int yl, const nav_msgs::OccupancyGrid& map)
{
    int logtemp, logmax=0;
    int deltax, deltay, error, jstep, j;
    bool steep = ifsteep(x0, y0, xl, yl);
    geometry_msgs::Point max_grid;

    if(steep)
    {
		std::swap(x0,y0);
		std::swap(xl,yl);
    }
    if(x0 > xl)
    {
		std::swap(x0,xl);
		std::swap(y0,yl);
    }
    deltax = xl-x0;
    deltay = abs(yl-y0);
    error = deltax/2;
    j = y0;
    if(y0 < yl)
		jstep = 1;
    else
		jstep = -1;
    for(int i=x0; i<=xl; i++)
    {
		if(steep)
		{

			logtemp = Mapping::getlog_odds(j,i,map);

		    if(logtemp>logmax)
		    {
			    logmax = logtemp;
			    max_grid.x = j;
			    max_grid.y = i;
		    }

		}
		else
		{
			logtemp = Mapping::getlog_odds(i,j,map);

		    if(logtemp>logmax)
		    {
			    logmax = logtemp;
			    max_grid.x = i;
			    max_grid.y = j;
		    }
			    
		}
		error = error - deltay;
		if(error < 0)
		{
		    j = j+jstep;
		    error = error + deltax;
		}   
    }
    return max_grid;
}


bool SensorModel::ifsteep(int x0, int y0, int xl, int yl)
{
    if(abs(yl-y0)>abs(xl-x0))
    {
      return true;
    }
    else
      return false;  
}

