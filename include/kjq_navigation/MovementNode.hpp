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

    void mapCallback(const nav_msgs::OccupancyGrid& msg);

    void publishGlobalMap();

    void publishLocalMap();

    // ROS nodehandle
    ros::NodeHandle& node_handle_;

    // subscribers and publishers
    ros::Subscriber global_map_sub_;
    ros::Subscriber laser_sub_;
    ros::Subscriber pose_sub_;

    ros::Publisher global_map_pub_;
    ros::Publisher local_map_pub_;
    ros::Publisher cmd_vel_pub_;

    // robot state
    RobotState state_ = RobotState::LOST;

    // local and global planner
    AStarPlanner global_planner_;
    LocalPlanner local_planner_;


};

} // kjq_navigation