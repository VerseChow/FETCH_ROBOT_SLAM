# FETCH ROBOT SLAM
This is a [ROS Indigo](http://www.ros.org/) package used for [Fetch Robot](http://fetchrobotics.com/platforms-research-development/) to do SLAM. The algorithm is based on Particle Filter Lidar-based Simultanously Localization and Mapping.
## Usage
1. Do 
```
catkin_make -j8
``` 
in catkin work space after pull the source code

2. Do 
```
roslaunch fetch_particle_filter_slam`
```
to launch the ros node
## Topic Created
1. Marker: slam_path
           slam_pose
           particles
2. map (Occupancy Grid Map)
3. Create a "world" frame, its child frame is "odom" frame

You could visualize them in RVIZ  
## Topic Subscribed
1. tf
2. base_scan
3. odom_combined
