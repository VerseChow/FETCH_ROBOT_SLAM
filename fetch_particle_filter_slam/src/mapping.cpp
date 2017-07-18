#include "mapping.hpp"


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}

void Mapping::UpdateMap(const sensor_msgs::LaserScan& laser_msg, const geometry_msgs::Pose2D& pose, nav_msgs::OccupancyGrid& map, float offset)
{
    int logtemp;
	float dist, theta;
	float x_t, y_t;//where ray terminates
	geometry_msgs::Point grid_r, grid_t;

	for (int scannum=0; scannum<laser_msg.ranges.size(); scannum++)//determin the line 
	{	
	    pose_r_.x = pose.x+offset*cos(pose.theta);//laser scan frame translation 0.23 
	    pose_r_.y = pose.y+offset*sin(pose.theta);
	    pose_r_.theta = pose.theta;
	    
	    theta = pose_r_.theta+laser_msg.angle_min+laser_msg.angle_increment*scannum;
	    
	    if (isnan(laser_msg.ranges[scannum]))
	    {
	    	dist = kMaxLaserDistance_;
	    }
	    else
	    	dist = laser_msg.ranges[scannum];

	    pose_t_.x = pose_r_.x+dist*cos(theta);
	    pose_t_.y = pose_r_.y+dist*sin(theta);
	    

	    /*grid index at the original pose and the terminal of the ray*/
	    grid_r = GlobalPositionToGridCell(pose_r_, map);
	    grid_t = GlobalPositionToGridCell(pose_t_, map);

	    i0 = (int)grid_r.x;
	    j0 = (int)grid_r.y;
	    it = (int)grid_t.x;
	    jt = (int)grid_t.y;

	    CheckRange(i0, j0, map);
	    CheckRange(it, jt, map);
	    
	    if (laser_msg.ranges[scannum] < kMaxLaserDistance_)
	    {
		    logtemp = GetlogOdds(it, jt, map);
			if( logtemp <= (100-kHitOdds_))
			{
			    logtemp = logtemp+kHitOdds_;
	    		SetlogOdds(it,jt,logtemp,map);
			}
			
			Bresenham(i0, j0, it, jt, map);
			
			logtemp = GetlogOdds(it, jt, map);
		    
			if (logtemp <= (100-kHitOdds_))
			{
			    logtemp = logtemp+kHitOdds_;
	    		SetlogOdds(it,jt,logtemp,map);	
			}		
	    }
	    else if (laser_msg.ranges[scannum] >= kMaxLaserDistance_)
	    {
			Bresenham(i0, j0, it, jt, map);
	    }
	}
    
}

void Mapping::CheckRange(int& i, int& j, const nav_msgs::OccupancyGrid& map)
{
	int width = map.info.width;
	int height = map.info.height;

	if (i > width)
		i = width;
	if (j>height)
		j = height;
}

void Mapping::Bresenham(int x0, int y0, int xl, int yl, nav_msgs::OccupancyGrid& map)
{
    int logtemp;
    int deltax, deltay, error, jstep, j;
    bool steep = Ifsteep(x0, y0, xl, yl);

    if (steep)
    {
		std::swap(x0,y0);
		std::swap(xl,yl);
    }
    if (x0 > xl)
    {
		std::swap(x0,xl);
		std::swap(y0,yl);
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
			logtemp = GetlogOdds(j,i,map);

		    if (logtemp >= kMissOdds_)
		    {
			    logtemp = GetlogOdds(j,i,map)-kMissOdds_;
	    	    SetlogOdds(j,i,logtemp,map);
		    }

		}
		else
		{
			logtemp = GetlogOdds(i,j,map);
		    
		    if (logtemp >= kMissOdds_)
		    {
		    	logtemp = GetlogOdds(i,j,map)-kMissOdds_;
			    SetlogOdds(i,j,logtemp,map);
		    }
			    
		}
		error = error - deltay;
		if (error < 0)
		{
		    j = j+jstep;
		    error = error + deltax;
		}   
    }  
}


bool Mapping::Ifsteep(int x0, int y0, int xl, int yl)
{
    if (abs(yl-y0) > abs(xl-x0))
    {
      return true;
    }
    else
      return false;  
}

bool Mapping::SetlogOdds(int xi, int yi, int logvalue, nav_msgs::OccupancyGrid& map)
{
	if ((xi <= map.info.width) && (yi <= map.info.height))
	{
		int index = (yi-1)*(map.info.height) + (xi-1);
		map.data[index] = logvalue;
		return true;
	}
	else
		return false;
}

int Mapping::GetlogOdds(int xi, int yi, const nav_msgs::OccupancyGrid& map)
{
	if ((xi<=map.info.width) && (yi<=map.info.height))
	{
		int index = (yi-1)*(map.info.height) + (xi-1);
		return map.data[index];
	}
	else
		return -2; //wrong number
}

geometry_msgs::Point Mapping::GlobalPositionToGridCell(const geometry_msgs::Pose2D pose, const nav_msgs::OccupancyGrid& map)
{
	geometry_msgs::Point Grid;
	float offsetx = -map.info.origin.position.x;
	float offsety = -map.info.origin.position.y;
    //printf("%f\t%f\t%f\n",pose.x+offsetx,pose.y+offsety,pose.theta);
	Grid.x = floor((pose.x+offsetx)/(map.info.resolution));
	Grid.y = floor((pose.y+offsety)/(map.info.resolution));

	return Grid;
}






