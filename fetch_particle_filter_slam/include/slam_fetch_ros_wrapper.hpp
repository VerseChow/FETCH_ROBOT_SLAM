#ifndef SLAM_FETCH_HPP
#define SLAM_FETCH_HPP
#include "ros/ros.h"
#include "iostream"
#include "stdint.h"
#include "mutex"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf2_msgs/TFMessage.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Odometry.h"

#include "boost/make_shared.hpp"

#include "mapping.hpp"
#include "particle_filter.hpp"
#include "utils.h"

class Mapping;
class ParticleFilter;

class SlamFetch
{
    enum STATE {INIT, SLAM};
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,\
            nav_msgs::Odometry> MySyncPolicy;
    public:
        SlamFetch(ros::NodeHandle& n, uint64_t num,\
                float max_dist, int8_t hit_odds, int8_t miss_odds);
        ~SlamFetch();
        void RunStateMachine();
    private:
        ros::NodeHandle n_;

        Pose_xyt pose_odom_;
        Pose_xyt pose_slam_;
        Particles samples_;

        float theta_pre_;
        float theta_cur_;
        float u_theta_;
        float u_motion_;
        double offset_x_;
        
        sensor_msgs::LaserScan laser_scan_;
        geometry_msgs::Vector3 translation_info_;
        nav_msgs::OccupancyGrid fetch_map_;

        STATE state_;
        uint64_t count_ = 1;
        uint64_t particle_num_;
        float max_dist_;
        int8_t h_odds_;
        int8_t m_odds_;

        ros::Publisher map_pub_;
        ros::Publisher samples_marker_pub;
        ros::Publisher pose_marker_pub_;
        ros::Publisher path_marker_pub_;

        visualization_msgs::Marker path_;

        ros::Subscriber tf_sub_;
        message_filters::Subscriber<sensor_msgs::LaserScan> *laser_sub_;
        message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
        message_filters::Synchronizer<MySyncPolicy> *sync_;

        Mapping *o_map_;
        ParticleFilter *p_filter_;

        bool tf_first_ = false;
        bool odom_laser_first_ = false;

        void TransReceiveCallBack(const tf2_msgs::TFMessage::ConstPtr& tf_message);
        void MsgsReceiveCallBack(const sensor_msgs::LaserScan::ConstPtr& laser_msg,\
                                const nav_msgs::Odometry::ConstPtr& odom_msg);
        void TFBraodcaster();
};

#endif
