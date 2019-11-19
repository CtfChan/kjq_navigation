#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "kjq_navigation/AStarPlanner.hpp"
#include "kjq_navigation/LocalPlanner.hpp"


namespace kjq_navigation
{

enum class RobotState{ 
    LOST, 
    WAYPOINT_TRACKING, 
    TO_BLOCK, 
    DROP_OFF,
    FINISH
};


class MovementNode {

public:
    MovementNode(ros::NodeHandle &node_handle);

private:

    void laserCallback(const sensor_msgs::LaserScan& msg);

    void poseCallback();  // AMCL pose


    

    // ROS nodehandle
    ros::NodeHandle& node_handle_;

    // subscribers and publishers
    ros::Subscriber laser_sub_;

    // robot state
    RobotState state_ = RobotState::LOST;

    // local and global planner
    // AStarPlanner global_;
    LocalPlanner local_planner_;


};

} // kjq_navigation