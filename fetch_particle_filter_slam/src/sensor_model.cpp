#include "sensor_model.hpp"

SensorModel::SensorModel(float maxLaserDistance, float s, float offset):
kMaxLaserDistance_(maxLaserDistance),
sigma_hit_(s),
offset_(offset)
{
}

SensorModel::SensorModel(){}

SensorModel::~SensorModel(){}

float SensorModel::Likelihood(const Particle& particle,\
							const sensor_msgs::LaserScan& laser_msg,\
							const nav_msgs::OccupancyGrid& O_map)
{
   	int logtemp;
   	float weight=1;

	float dist, theta, z_k;
	/*where ray terminates*/
	float x_t, y_t;
	int i0, j0, it, jt;
	bool isnan;

	geometry_msgs::Point grid_r, grid_t, grid_obstacle;
	geometry_msgs::Pose2D pose_r, pose_t;
    /*determin the line*/
	for (int scannum=0; scannum < laser_msg.ranges.size(); scannum++)
	{	
		/*laser scan frame translation 0.23*/
	    pose_r.x = particle.pose.x+offset_*cos(particle.pose.theta);
	    pose_r.y = particle.pose.y+offset_*sin(particle.pose.theta);
	    pose_r.theta = particle.pose.theta;
	    
	    theta = pose_r.theta+laser_msg.angle_min+laser_msg.angle_increment*scannum;
	    isnan = std::isnan(laser_msg.ranges[scannum]);

	    if (isnan || laser_msg.ranges[scannum] >= 25)
	    {
	    	dist = kMaxLaserDistance_;
	    }
	    else
	    	dist = laser_msg.ranges[scannum];

	    pose_t.x = pose_r.x+dist*cos(theta);
	    pose_t.y = pose_r.y+dist*sin(theta);
	    
	    /*grid index at the original pose and the terminal of the ray*/
	    grid_r = Mapping::GlobalPositionToGridCell(pose_r, O_map);
	    grid_t = Mapping::GlobalPositionToGridCell(pose_t, O_map);

	    i0 = (int)grid_r.x;
	    j0 = (int)grid_r.y;
	    it = (int)grid_t.x;
	    jt = (int)grid_t.y;

	   	Mapping::CheckRange(i0, j0, O_map);
	    Mapping::CheckRange(it, jt, O_map);
	    
	    if(Mapping::GetlogOdds(it,jt,O_map) > 80)
	    	weight += Mapping::GetlogOdds(it,jt,O_map);
	    //else
	    //	weight += (Mapping::GetlogOdds(it,jt,O_map)/100);
	    //printf("11111 %f \n",  weight);
	    //printf("%d 3\n", (int)scannum);
	    /*
		grid_obstacle = Bresenham(i0, j0, it, jt, O_map);
		//printf("%d 4\n", (int)scannum);
		z_k = sqrt((grid_obstacle.x-grid_r.x)*(grid_obstacle.x-grid_r.x)+
			  (grid_obstacle.y-grid_r.y)*(grid_obstacle.y-grid_r.y))*O_map.info.resolution;
		if(isnan)
			weight += Max_dit(z_k);
		else
		{
			weight += gaussian(dist, sigma_hit_, z_k);
	
		}
		*/	
	}
	return weight;
}

float SensorModel::Gaussian(float m, float sigma, float zk)
{
	return (1/(sigma*sqrt(2*pi)))*exp(-((zk-m)*(zk-m))/(2*sigma*sigma));
}

float SensorModel::Max_dit(float zk)
{
	if((zk>=(kMaxLaserDistance_-1)) && zk<=kMaxLaserDistance_)
		return 1.;
	else
		return 0.;
}

geometry_msgs::Point SensorModel::Bresenham(int x0, int y0, int xl, int yl, const nav_msgs::OccupancyGrid& map)
{
    int logtemp;
    int deltax, deltay, error, jstep, j;
    bool steep = Ifsteep(x0, y0, xl, yl);
    geometry_msgs::Point obstacle_grid;

    if (steep)
    {
		std::swap(x0, y0);
		std::swap(xl, yl);
    }
    if (x0 > xl)
    {
		std::swap(x0, xl);
		std::swap(y0, yl);
    }
    deltax = xl-x0;
    deltay = abs(yl-y0);
    error = deltax/2;
    j = y0;
    if (y0 < yl)
		jstep = 1;
    else
		jstep = -1;
    for (int i=x0; i<=xl; i++)
    {
		if (steep)
		{

			logtemp = Mapping::GetlogOdds(j,i,map);

		    if (logtemp > 90)
		    {
			    obstacle_grid.x = j;
			    obstacle_grid.y = i;
			    break;
		    }

		}
		else
		{
			logtemp = Mapping::GetlogOdds(i,j,map);

		    if (logtemp > 90)
		    {
			    obstacle_grid.x = i;
			    obstacle_grid.y = j;
			    break;
		    }
			    
		}
		error = error - deltay;
		if (error < 0)
		{
		    j = j+jstep;
		    error = error + deltax;
		}   
    }
    return obstacle_grid;
}

bool SensorModel::Ifsteep(int x0, int y0, int xl, int yl)
{
    if (abs(yl-y0) > abs(xl-x0))
    {
      return true;
    }
    else
      return false;  
}

