#ifndef MAPPING_HPP
#define MAPPING_HPP

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"

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
        void UpdateMap(const sensor_msgs::LaserScan& laser_msg,\
                        const geometry_msgs::Pose2D& pose, nav_msgs::OccupancyGrid& map,\
                        float offset);
        static int GetlogOdds(int xi, int yi, const nav_msgs::OccupancyGrid& map);
        static geometry_msgs::Point GlobalPositionToGridCell(const geometry_msgs::Pose2D pose,\
                                                            const nav_msgs::OccupancyGrid& map);
        static void CheckRange(int& i, int& j, const nav_msgs::OccupancyGrid& map);

    private:
        
        const float  kMaxLaserDistance_;
        const int8_t kHitOdds_;
        const int8_t kMissOdds_;
        /*pose of robot*/
        geometry_msgs::Pose2D pose_r_, pose_t_;
        /*grid index at the original pose and the terminal of the ray*/
        int i0, j0, it, jt;
        bool Ifsteep(int x0, int y0, int xt, int yt);
        void Bresenham(int i0, int j0, int xt, int yt, nav_msgs::OccupancyGrid& map);     
        bool SetlogOdds(int xi, int yt, int logvalue, nav_msgs::OccupancyGrid& map);
};

#endif // SLAM_MAPPING_HPP
