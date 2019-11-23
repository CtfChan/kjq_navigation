#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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

    void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);

    void goalCallback(const geometry_msgs::PoseStamped& msg);

    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);

    // ROS nodehandle
    ros::NodeHandle& node_handle_;

    // subscribers and publishers
    ros::Subscriber global_map_sub_;
    ros::Subscriber laser_sub_;
    ros::Subscriber init_pose_sub_;
    ros::Subscriber goal_pose_sub_;
    ros::Subscriber amcl_pose_sub_;

    ros::Publisher path_pub_;
    ros::Publisher global_map_pub_;
    ros::Publisher local_map_pub_;
    ros::Publisher cmd_vel_pub_;

    // robot state
    RobotState state_ = RobotState::LOST;

    geometry_msgs::Pose curr_; 
    geometry_msgs::Pose goal_; 

    // local and global planner
    AStarPlanner global_planner_;
    LocalPlanner local_planner_;

    ros::Timer timer_;

    nav_msgs::Path path_;
    nav_msgs::Path goal_points_;


};

} // kjq_navigation