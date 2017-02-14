#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include "main.hpp"
#include "action_model.hpp"
#include "sensor_model.hpp"

struct Particle;

struct 
{
    inline bool operator() (const Particle& lft, const Particle& rht)
    {   
        return lft.weight > rht.weight;
    }   
} customGreater;

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
    public:
        //int utime_last;     //time of parent, previous odometry
        //int utime_now;      //time of current(action model, proposal), given by odometry
        //pose_xyt_t last_esti;   //the last best estimate
        /**
        * Constructor for ParticleFilter.
        *
        * \param    numParticles        Number of particles to use
        * \pre  numParticles > 1
        */
        ParticleFilter(int numParticles, float offset);
        
        /**
        * initializeFilterAtPose initializes the particle filter with the samples distributed according
        * to the provided pose estimate.
        *
        * \param    pose            Initial pose of the robot
        */
        void initialize_FilterAtPose(const geometry_msgs::Pose2D& pose);
        
        /**
        * updateFilter increments the state estimated by the particle filter. The filter update uses the most recent
        * odometry estimate and laser scan along with the occupancy grid map to estimate the new pose of the robot.
        *
        * \param    odometry        Calculated odometry at the time of the final ray in the laser scan
        * \param    laser           Most recent laser scan of the environment
        * \param    map             Map built from the maximum likelihood pose estimate
        * \return   Estimated robot pose.
        */
        geometry_msgs::Pose2D update_Filter(const geometry_msgs::Pose2D& odometry,
                                            const sensor_msgs::LaserScan& laser,
                                            const nav_msgs::OccupancyGrid& map);


         /**
        * poseEstimate retrieves the current pose estimate computed by the filter.
        */
        geometry_msgs::Pose2D estimate_PosteriorPose(std::vector<Particle>& posterior);   
        std::vector<Particle> particles(void);
    private:
        
        std::vector<Particle> posterior_;     // The posterior distribution of particles at the end of the previous update
        geometry_msgs::Pose2D posteriorPose_;              // Pose estimate associated with the posterior distribution
        

        ActionModel actionModel_;  // Action model to apply to particles on each update
        SensorModel sensorModel_;  // Sensor model to compute particle weights
        
        int kNumParticles_;         // Number of particles to use for estimating the 
        float laser_frame_offset;

        std::random_device rd;//uniform distribution
        std::mt19937 gen;
        std::uniform_int_distribution<> dis;

        std::vector<Particle> resample_PosteriorDistribution(void);
        std::vector<Particle> compute_ProposalDistribution(std::vector<Particle>& prior);
        std::vector<Particle> compute_NormalizedPosterior(std::vector<Particle>& proposal,
                                                         const sensor_msgs::LaserScan& laser,
                                                         const nav_msgs::OccupancyGrid& map);

};

#endif // SLAM_PARTICLE_FILTER_HPP
