#ifndef MAPPING_HPP
#define MAPPING_HPP

#include "main.hpp"



/**
* Mapping implements the occupancy grid mapping algorithm.  On each map update, the updateMap method is called. The
* provided laser scan should be used to update the provided OccupancyGrid.
*/
class Mapping
{
    public:
        
        /**
        * Constructor for Mapping.
        * 
        * \param    maxLaserDistance    Maximum distance for the rays to be traced
        * \param    hitOdds             Increase in occupied odds for cells hit by a laser ray
        * \param    missOdds            Decrease in occupied odds for cells passed through by a laser ray
        */
        Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds);
        
        /**
        * updateMap incorporates information from a new laser scan into an existing OccupancyGrid.
        * 
        * \param    scan            Laser scan to use for updating the occupancy grid
        * \param    pose            Pose of the robot at the time when the last ray was measured
        * \param    map             OccupancyGrid instance to be updated
        */
        void updateMap(const sensor_msgs::LaserScan& laser_msg, const geometry_msgs::Pose2D& pose, nav_msgs::OccupancyGrid& map);


    private:
        
        const float  kMaxLaserDistance_;
        const int8_t kHitOdds_;
        const int8_t kMissOdds_;
        //////////////////// TODO: Add any private members needed for your occupancy grid mapping algorithm ///////////////
        float A, B ,C;
        geometry_msgs::Pose2D pose_r, pose_t;//pose of robot
        /*grid index at the original pose and the terminal of the ray*/
        size_t i0, j0, it, jt;//

        bool l_determine(size_t i, size_t j, float length);
        
        bool ifsteep(int x0, int y0, int xt, int yt);
        void Bresenham(int i0, int j0, int xt, int yt, nav_msgs::OccupancyGrid& map);
        
        bool setlog_odds(size_t xi, size_t yt, int logvalue, nav_msgs::OccupancyGrid& map);
        int getlog_odds(size_t xi, size_t yi, const nav_msgs::OccupancyGrid& map);
        geometry_msgs::Point global_position_to_grid_cell(const geometry_msgs::Pose2D pose, const nav_msgs::OccupancyGrid& map);


};

#endif // SLAM_MAPPING_HPP
