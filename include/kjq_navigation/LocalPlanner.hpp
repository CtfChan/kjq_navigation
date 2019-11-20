#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <grid_map_ros/grid_map_ros.hpp>


namespace kjq_navigation
{

class LocalPlanner {

public:
    LocalPlanner();

    void updateLocalMap(const sensor_msgs::LaserScan& msg);

    grid_map::GridMap& getLocalMap();

    void publishRandomCmdVel();

private:
    void publishGridMap();

    void publishCmdVel();

    float map_width_ = 1.0f;
    float map_length_ = 1.0f;
    float map_resolution_ = 0.015;
    float robot_radius_ = 0.10f;

    // node handle
    ros::NodeHandle node_handle_;

    // publishers 
    ros::Publisher grid_map_pub_;
    ros::Publisher safe_cmd_vel_pub_;

    // local cost map
    grid_map::GridMap map_;


};


} 