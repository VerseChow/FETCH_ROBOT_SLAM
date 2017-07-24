#include "slam_fetch_ros_wrapper.hpp"

SlamFetch::SlamFetch(ros::NodeHandle& n, uint64_t num,
				float max_dist, int8_t hit_odds, int8_t miss_odds):
n_(n),
particle_num_(num),
max_dist_(max_dist_),
h_odds_(hit_odds),
m_odds_(miss_odds)
{
	pose_odom_ = boost::make_shared<pose_xyt>();
	pose_slam_ = boost::make_shared<pose_xyt>();
	samples_ = boost::make_shared<particles>();

	fetch_map_.header.seq = count_;
    fetch_map_.header.frame_id = "/odom";
    fetch_map_.header.stamp = ros::Time::now();
    fetch_map_.info.map_load_time = ros::Time::now();
    /*m/cell*/
    fetch_map_.info.resolution = 0.05;
    fetch_map_.info.width = 10000;
    fetch_map_.info.height = 10000;

    fetch_map_.info.origin.position.x = -250.0;
    fetch_map_.info.origin.position.y = -250.0;
    fetch_map_.info.origin.position.z = 0.0;
    fetch_map_.info.origin.orientation.w = 1.0;
    fetch_map_.data.assign(fetch_map_.info.width*fetch_map_.info.height, 50);

    map_pub_ = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    marker_pub_ = n.advertise<visualization_msgs::Marker>("particles", 10);
    tf_sub_ = n.subscribe("/tf", 1, &SlamFetch::TransReceiveCallBack, this);

    laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(n, "/base_scan", 1);
    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(n, "/odom_combined", 1);
	
    sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *laser_sub_, *odom_sub_);
    sync_->registerCallback(boost::bind(&SlamFetch::MsgsReceiveCallBack, this, _1, _2));

    state_ = INIT;
}

SlamFetch::~SlamFetch(){}

void SlamFetch::RunStateMachine()
{
	while(ros::ok())
	{
		int trigger_num_ = 1;
		switch(state_)
		{
			case INIT:
				if (tf_first_ && odom_laser_first_)
				{
					state_ = SLAM;
					o_map_ = new Mapping(max_dist_, h_odds_, m_odds_);
					p_filter_ = new ParticleFilter(particle_num_, translation_info_.x);
					p_filter_->InitializeFilterAtPose(pose_odom_->pose);
					pose_slam_ = pose_odom_;
					tf_sub_.shutdown();
				}
				break;
			case SLAM:
				fetch_map_.header.seq = count_;
		        if(fabs(u_theta_) <= 0.05)
	          		o_map_->UpdateMap(laser_scan_, pose_slam_->pose, fetch_map_, translation_info_.x); 
				
				while(trigger_num_ <= 1)				
				{	
					pose_slam_->pose = p_filter_->UpdateFilter(pose_odom_->pose, laser_scan_, fetch_map_);
					trigger_num_++;
				}
		        samples_ = p_filter_->GetParticles();
		        
		        count_++;
		        trigger_num_ = 1;

		        fetch_map_.header.stamp = ros::Time::now();
    			fetch_map_.info.map_load_time = ros::Time::now();
		        map_pub_.publish(fetch_map_); 
		        DrawParticles(marker_pub_, samples_);			
				break;
		}
        ros::spinOnce();
	}
}

void SlamFetch::TransReceiveCallBack(const tf2_msgs::TFMessage::ConstPtr& tf_message)
{
	std::string frame_name = "laser_link";
    for(auto trans: tf_message->transforms)
    {
        if (trans.child_frame_id == frame_name)
        {
            translation_info_ = trans.transform.translation;
            break; 
        }      
    }
    tf_first_ = true;
}

void SlamFetch::MsgsReceiveCallBack(const sensor_msgs::LaserScan::ConstPtr& laser_msg,\
                            		const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    laser_scan_.header = laser_msg->header;
    /*becase laser scan fram rotate 180 degrees according to the frame of map*/
    laser_scan_.angle_min = -laser_msg->angle_max;
    laser_scan_.angle_max = -laser_msg->angle_min;
    laser_scan_.angle_increment = laser_msg->angle_increment;                                    
    laser_scan_.scan_time = laser_msg->scan_time;
    laser_scan_.range_min = laser_msg->range_min;
    laser_scan_.range_max = laser_msg->range_max;
    laser_scan_.ranges = laser_msg->ranges;

    std::reverse(laser_scan_.ranges.begin(), laser_scan_.ranges.end());
    laser_scan_.intensities = laser_msg->intensities;
    
    double roll, pitch, yaw;
    pose_odom_->header = odom_msg->header;
    pose_odom_->pose.x = odom_msg->pose.pose.position.x;
    pose_odom_->pose.y = odom_msg->pose.pose.position.y;

    tf::Quaternion q(odom_msg->pose.pose.orientation.x, \
                     odom_msg->pose.pose.orientation.y, \
                     odom_msg->pose.pose.orientation.z, \
                     odom_msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);
    pose_odom_->pose.theta = wrap_to_pi(yaw);
    theta_cur_ = wrap_to_pi(yaw);
    u_theta_ = theta_cur_-theta_pre_;
    theta_pre_ = theta_cur_;

    odom_laser_first_ = true;
}