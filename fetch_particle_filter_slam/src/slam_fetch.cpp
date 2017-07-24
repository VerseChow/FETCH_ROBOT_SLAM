#include "slam_fetch_ros_wrapper.hpp"

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "Slam_Fetch");
    ros::NodeHandle n;

    SlamFetch slam_fetch(n, 1000, 25, 10, 1);
    slam_fetch.RunStateMachine();
}
