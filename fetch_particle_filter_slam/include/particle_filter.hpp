#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include "vector"
#include "cmath"
#include "algorithm"
#include "cfloat"

#include "boost/make_shared.hpp"

#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#include "datatypes.h"
#include "action_model.hpp"
#include "sensor_model.hpp"

class SensorModel;
class ActionModel;

/**
* ParticleFilter implements a standard SIR-based particle filter. The set of particles is initialized at some pose. Then
* on subsequent calls to updateFilter, a new pose estimate is computed using the latest odometry and laser measurements
* along with the current map of the environment.
* 
* This implementation of the particle filter uses a fixed number of particles for each iteration. Each filter update is
* a simple set of operations:
* 
*   1) Draw N particles from current set of weighted particles.
*   2) Sample an action from the ActionModel and apply it to each of these particles.
*   3) Compute a weight for each particle using the SensorModel.
*   4) Normalize the weights.
*   5) Use the max-weight or mean-weight pose as the estimated pose for this update.
*/
class ParticleFilter
{
    struct CustomGreater
    {
        inline bool operator() (const Particle& lft, const Particle& rht)
        {   
            return lft.weight > rht.weight;
        }   
    } MyCompare;

    public:
        /**
        * Constructor for ParticleFilter.
        *
        * \param    numParticles        Number of particles to use
        */
        ParticleFilter(int numParticles, float offset);
        
        ~ParticleFilter();
        /**
        * initializeFilterAtPose initializes the particle filter with the samples distributed according
        * to the provided pose estimate.
        *
        * \param    pose            Initial pose of the robot
        */
        void InitializeFilterAtPose(const geometry_msgs::Pose2D& pose);
        
        /**
        * updateFilter increments the state estimated by the particle filter. The filter update uses the most recent
        * odometry estimate and laser scan along with the occupancy grid map to estimate the new pose of the robot.
        *
        * \param    odometry        Calculated odometry at the time of the final ray in the laser scan
        * \param    laser           Most recent laser scan of the environment
        * \param    map             Map built from the maximum likelihood pose estimate
        * \return   Estimated robot pose.
        */
        geometry_msgs::Pose2D UpdateFilter(const geometry_msgs::Pose2D& odometry,
                                            const sensor_msgs::LaserScan& laser,
                                            const nav_msgs::OccupancyGrid& map);
        /**
        * poseEstimate retrieves the current pose estimate computed by the filter.
        */
        geometry_msgs::Pose2D EstimatePosteriorPose(Particles& posterior);   
        Particles GetParticles();
    private:
        /*the posterior distribution of particles at the end of the previous update*/
        Particles posterior_;     
        /*pose estimate associated with the posterior distribution*/
        geometry_msgs::Pose2D posterior_pose_; 
        /*Action model to apply to particles on each update*/
        ActionModel actionModel_;
        /*Sensor model to compute particle weights*/
        SensorModel sensorModel_;
        /*number of particles to use for estimating*/
        int kNumParticles_;
        float laser_frame_offset;
        /*uniform distribution*/
        std::random_device rd;
        std::mt19937 gen;
        std::uniform_int_distribution<> dis;

        Particles ResamplePosteriorDistribution();
        Particles ComputeProposalDistribution(Particles& prior);
        Particles ComputeNormalizedPosterior(Particles& proposal,
                                             const sensor_msgs::LaserScan& laser,
                                             const nav_msgs::OccupancyGrid& map);
};

#endif
