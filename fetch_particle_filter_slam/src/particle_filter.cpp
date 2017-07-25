#include "particle_filter.hpp"

ParticleFilter::ParticleFilter(int numParticles, float offset):
kNumParticles_ (numParticles), 
laser_frame_offset(offset)
{
    assert(kNumParticles_ > 1);
    gen = std::mt19937(rd());
    dis = std::uniform_int_distribution<>(1, kNumParticles_);

    posterior_ = boost::make_shared<particles>();
}

ParticleFilter::ParticleFilter(){}

ParticleFilter::~ParticleFilter(){}

void ParticleFilter::InitializeFilterAtPose(const geometry_msgs::Pose2D& odometry)
{
    /*Initializing the particles in the particle filter*/
    actionModel_ = ActionModel(0.5*0.5, 0.001, 0.5*0.5, 0.001, 0.001, odometry);
    sensorModel_ = SensorModel(25, 0.02, laser_frame_offset);

    Particle sample_particle;// = boost::make_shared<particle>();

    for(int i=0; i<kNumParticles_; i++)
    {
        sample_particle.pose = odometry;
        /*NORMLIZE the wieght such their mean equals 1*/
        sample_particle.weight = 1.0; 
        posterior_->push_back(sample_particle);
    }
    posterior_pose_ = odometry;        
}


geometry_msgs::Pose2D ParticleFilter::UpdateFilter(const geometry_msgs::Pose2D& odometry,
                                                    const sensor_msgs::LaserScan& laser,
                                                    const nav_msgs::OccupancyGrid& map)
{
    /*Only update the particles if motion was detected. If the robot didn't move, then
    obviously don't do anything.*/

    if(actionModel_.UpdateAction(odometry))
    {
        Particles prior = posterior_;
        Particles proposal = ComputeProposalDistribution(prior);   
        //printf("begin update weight\n");
        //posterior_ = proposal;
        posterior_ = ComputeNormalizedPosterior(proposal, laser, map);
        //printf("finish update weight\n"); 
        posterior_ = ResamplePosteriorDistribution();
        //printf("finish resampling\n");
        posterior_pose_ = EstimatePosteriorPose(posterior_); 
        //printf("finish update Filter \n");
    }
    //std::cout << "updating\n";

    return posterior_pose_;
}


Particles ParticleFilter::GetParticles() 
{
    //printf("Tring to fetch Particles\n");
    return posterior_;
}

Particles ParticleFilter::ResamplePosteriorDistribution()
{
    /*Resampling from the posterior distribution*/;
    Particles prior = boost::make_shared<particles>();
    /*random number to draw i with probability of w*/
    float u_dis;
    float sum_dist;
    float min = FLT_MAX;
    int pick_num = 0, min_index = 0;

    for(int i=0; i<posterior_->size(); i++)
    {
        /*generate a random from 0 to M*/
        u_dis = (float)dis(gen);
        for(int j=0; j<posterior_->size(); j++)
        {
            sum_dist += posterior_->at(j).weight;
            if (posterior_->at(j).weight <= min)
            {
                min = posterior_->at(j).weight;
                min_index = j;
            }
            if (u_dis <= sum_dist)
            {
                pick_num = j;              
                break;
            }
            else if (j==posterior_->size() || u_dis>sum_dist)
                pick_num = min_index;
            //printf("%d\t%d, size:%d\n", i, j, posterior_->size());

        }
        //printf("%f\t%f\t%d\n", sum_dist, u_dis, pick_num);
        sum_dist = 0;    
        prior->push_back(posterior_->at(pick_num));
    }
    return prior;
}

Particles ParticleFilter::ComputeProposalDistribution(Particles& prior)
{
    /*creating the proposal distribution by sampling from the ActionModel*/
    Particles proposal = boost::make_shared<particles>();

    proposal = actionModel_.ApplyAction(prior);

    return proposal ;
}

Particles ParticleFilter::ComputeNormalizedPosterior(Particles& proposal,
                                                    const sensor_msgs::LaserScan& laser,
                                                    const nav_msgs::OccupancyGrid& map)
{
    /*Computing the normalized posterior distribution using the 
    particles in the proposal distribution*/
    Particles posterior = boost::make_shared<particles>();
    //std::cout << "computing norm\n";
    float sum_weight = 0;
    for(int i=0; i<proposal->size(); i++)
    {   
        //printf("1. %f\n", proposal->at(i).weight);
        proposal->at(i).weight = sensorModel_.Likelihood(proposal->at(i), laser, map);
        //printf("2. %f\n", proposal->at(i).weight);
        posterior->push_back(proposal->at(i));
        //printf("3. %f\n", posterior->at(i).weight);
        sum_weight += proposal->at(i).weight;
    }

    sum_weight = proposal->size()/sum_weight;
    //printf("3. %f\n", sum_weight);
    for(int i=0; i<proposal->size(); i++)
    {
        //printf("1. %f\t%f\n", proposal->at(i).weight, sum_weight);
        posterior->at(i).weight = (posterior->at(i).weight)*sum_weight;
        //printf("2. %f\t%f\n", posterior->at(i).weight, sum_weight);
    }

    return posterior;
}


geometry_msgs::Pose2D ParticleFilter::EstimatePosteriorPose(Particles& posterior)
{
    /*Compute the final pose estimate based on the posterior distribution
    Change TO COMPUTE the WEIGHTED MEAN of top n*/
    int pick_num = 1;
    float weightsum = 0;
    geometry_msgs::Pose2D pose_estimate;
    /*find the top 10 pose's number with max weight*/
    std::sort(posterior->begin(), posterior->end(), MyCompare);

    for(int i = 0; i<pick_num; i++)
    {
        pose_estimate.x += posterior->at(i).pose.x*posterior->at(i).weight;
        pose_estimate.y += posterior->at(i).pose.y*posterior->at(i).weight;
        pose_estimate.theta += wrap_to_pi(posterior->at(i).pose.theta)*posterior->at(i).weight;///CHANGE theta to the standard range
        weightsum += posterior->at(i).weight;
        //printf("weight:%f\n", posterior->at(i).weight);
    }

    pose_estimate.x = pose_estimate.x/weightsum;
    pose_estimate.y = pose_estimate.y/weightsum;
    pose_estimate.theta = pose_estimate.theta/weightsum;

    return pose_estimate;
}

