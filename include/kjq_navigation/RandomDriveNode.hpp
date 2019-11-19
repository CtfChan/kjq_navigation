#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <grid_map_ros/grid_map_ros.hpp>



namespace kjq_navigation
{

class RandomDriveNode {

public:
    RandomDriveNode(ros::NodeHandle &node_handle);

private:
    bool readParameters();

    void laserCallback(const sensor_msgs::LaserScan& msg);

    void publishGridMap();

    void publishCmdVel();

    // ROS nodehandle
    ros::NodeHandle& node_handle_;

    // subscribers and publishers
    ros::Subscriber laser_sub_;

    ros::Publisher grid_map_pub_;
    ros::Publisher safe_cmd_vel_pub_;

    // occupancy grid map
    grid_map::GridMap map_;


};


} 