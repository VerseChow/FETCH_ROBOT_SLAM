#include "particle_filter.hpp"



ParticleFilter::ParticleFilter(int numParticles, float offset):
kNumParticles_ (numParticles), 
laser_frame_offset(offset)
{
    assert(kNumParticles_ > 1);

    gen = std::mt19937(rd());
    dis = std::uniform_int_distribution<>(1, kNumParticles_);

}


void ParticleFilter::initialize_FilterAtPose(const geometry_msgs::Pose2D& odometry)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////

    actionModel_ = ActionModel(0.01, 0.01, 0.01, 0.01, 0.01, odometry);
    sensorModel_ = SensorModel(25, 0.01, laser_frame_offset);

    Particle sampleparticle;

    for(int i=0; i<kNumParticles_; i++)
    {
        sampleparticle.pose = odometry;
        sampleparticle.weight = 1.0; ///NORMLIZE the wieght such their mean equals 1
        posterior_.push_back(sampleparticle);
    }
    posteriorPose_ = odometry;        
}


geometry_msgs::Pose2D ParticleFilter::update_Filter(const geometry_msgs::Pose2D& odometry,
                                                    const sensor_msgs::LaserScan& laser,
                                                    const nav_msgs::OccupancyGrid& map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.

    if(actionModel_.update_Action(odometry))
    {
        std::vector<Particle> prior = resample_PosteriorDistribution();
        std::vector<Particle> proposal = compute_ProposalDistribution(prior);
        printf("begin update weight\n") ;
        //posterior_ = proposal;
        posterior_ = compute_NormalizedPosterior(proposal, laser, map);   
        printf("finish update weight\n") ;
        posteriorPose_ = estimate_PosteriorPose(posterior_); 
        printf("finish update Filter \n");
    }
    //std::cout << "updating\n";

    return posteriorPose_;
}


std::vector<Particle> ParticleFilter::particles(void) 
{
    printf("Tring to fetch Particles\n");
    return posterior_;
}


std::vector<Particle> ParticleFilter::resample_PosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////n";
    std::vector<Particle> prior;

    float u_dis;///random number to draw i with probability of w
    float sum_dist;
    int pick_num;

    for(int i=0;i<posterior_.size();i++)
    {
        u_dis = (float)dis(gen);//generate a random from 0 to M
        for(int j=0; j<posterior_.size(); j++)
        {
            sum_dist += posterior_[j].weight;          
            if( u_dis<sum_dist )
            {
                pick_num = j;                
                break;
            }
        }
        sum_dist = 0;        
        prior.push_back(posterior_[pick_num]);
    }
    return prior;
}


std::vector<Particle> ParticleFilter::compute_ProposalDistribution(std::vector<Particle>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
	//std::cout << "before apply action\n";
    std::vector<Particle> proposal;

    proposal = actionModel_.apply_Action(prior);

    return proposal ;
}


std::vector<Particle> ParticleFilter::compute_NormalizedPosterior(std::vector<Particle>& proposal,
                                                                  const sensor_msgs::LaserScan& laser,
                                                                  const nav_msgs::OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<Particle> posterior;
    //std::cout << "computing norm\n";
    float sum_weight = 0;
    for(int i=0; i<proposal.size(); i++)
    {     
        printf("begin calculate weight of %d particle\n", i);
        proposal[i].weight = sensorModel_.likelihood(proposal[i], laser, map);
        printf("finish calculate weight of %d particle\n", i);
        posterior.push_back( proposal[i] );

        sum_weight += proposal[i].weight;
    }
    printf("normalize weight\n");
    sum_weight = proposal.size()/sum_weight;
    ////Normlize the weight such that their mean is 1
    for(int i=0;i<proposal.size();i++)
    {
        posterior[i].weight = posterior[i].weight*sum_weight;
    }
    //std::cout << "computed norm\n";
    return posterior;
}



geometry_msgs::Pose2D ParticleFilter::estimate_PosteriorPose(std::vector<Particle>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
	//std::cout << "estimating\n";
    //////CHANGE TO COMPUTE the WEIGHTED MEAN of top n 
    int pick_num = 10;
    float weightsum = 0;
    geometry_msgs::Pose2D pose_Estimate;
    ///find the top 10 pose's number with max weight
    //std::cout<<"before loop1\n";


    std::sort(posterior.begin(), posterior.end(), customGreater);


    for(int i = 0; i<pick_num; i++)
    {
        pose_Estimate.x += posterior[i].pose.x*posterior[i].weight;
        pose_Estimate.y += posterior[i].pose.y*posterior[i].weight;
        pose_Estimate.theta += wrap_to_pi(posterior[i].pose.theta)*posterior[i].weight;///CHANGE theta to the standard range
        weightsum += posterior[i].weight;
    }
    //std::cout<<"after loop2\n";

    pose_Estimate.x = pose_Estimate.x/weightsum;
    pose_Estimate.y = pose_Estimate.y/weightsum;
    pose_Estimate.theta = pose_Estimate.theta/weightsum;

    return pose_Estimate;
}

