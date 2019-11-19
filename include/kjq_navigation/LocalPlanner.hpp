#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <grid_map_ros/grid_map_ros.hpp>



namespace kjq_navigation
{

class LocalPlanner {

public:
    LocalPlanner();

    void updateLocalMap(const sensor_msgs::LaserScan& msg);

    void publishRandomCmdVel();

private:
    bool readParameters();

    void publishGridMap();

    void publishCmdVel();

    // publishers 
    ros::Publisher grid_map_pub_;
    ros::Publisher safe_cmd_vel_pub_;

    // local cost map
    grid_map::GridMap map_;


};


} 