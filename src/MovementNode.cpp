#include "kjq_navigation/MovementNode.hpp"

#include <iostream>

namespace kjq_navigation {

void MovementNode::laserCallback(const sensor_msgs::LaserScan& msg) {
    // update local planner
    local_planner_.updateLocalMap(msg);
    publishLocalMap();
}

void MovementNode::mapCallback(const nav_msgs::OccupancyGrid& msg) {
    global_planner_.setMap(msg);
    publishGlobalMap();
}

void MovementNode::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
    curr_ = msg.pose.pose;
}

void MovementNode::goalCallback(const geometry_msgs::PoseStamped& msg) {
    goal_ = msg.pose;

    path_ = global_planner_.plan(grid_map::Position(curr_.position.x, curr_.position.y), 
                                     grid_map::Position(goal_.position.x, goal_.position.y) ) ;
    // add final orientation
    path_.poses.back().pose.orientation = goal_.orientation;
    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = "map";

    goal_points_.header.stamp = ros::Time::now();
    goal_points_.header.frame_id = "map";
    goal_points_.poses.push_back(path_.poses.front());
    for (size_t i = 1; i < path_.poses.size(); ++i) {
        auto& q1 = path_.poses[i].pose.orientation;
        auto& q2 = path_.poses[i-1].pose.orientation;
        if (std::abs(q1.x-q2.x) < std::numeric_limits<float>::epsilon() &&
            std::abs(q1.y-q2.y) < std::numeric_limits<float>::epsilon() &&
            std::abs(q1.z-q2.z) < std::numeric_limits<float>::epsilon() && 
            std::abs(q1.w-q2.w) < std::numeric_limits<float>::epsilon()) {
                continue;
        }
        goal_points_.poses.push_back(path_.poses[i]);
    }

    path_pub_.publish(goal_points_);

    

}

void MovementNode::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
    // publish cmd_vel based on state
    if (state_ == RobotState::LOST) {
        // local_planner_.publishRandomCmdVel();
    } else if (state_ == RobotState::WAYPOINT_TRACKING) {

    }
}


MovementNode::MovementNode(ros::NodeHandle &node_handle) : node_handle_(node_handle) {  
    laser_sub_ = node_handle_.subscribe("/scan_synced", 1,
                                        &MovementNode::laserCallback, this);
    global_map_sub_ = node_handle_.subscribe("/loc_map", 1,
                                        &MovementNode::mapCallback, this);
    init_pose_sub_ =  node_handle_.subscribe("/initialpose", 1,
                                        &MovementNode::initPoseCallback, this);
    goal_pose_sub_ = node_handle_.subscribe("/move_base_simple/goal", 1,
                                        &MovementNode::goalCallback, this);
    amcl_pose_sub_ = node_handle_.subscribe("/amcl_pose", 1,
                                        &MovementNode::amclPoseCallback, this);

    cmd_vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    local_map_pub_ = node_handle_.advertise<grid_map_msgs::GridMap>("/local_map", 1, true);                
    global_map_pub_ = node_handle_.advertise<grid_map_msgs::GridMap>("/global_map", 1, true);
    path_pub_ =  node_handle_.advertise<nav_msgs::Path>("/global_path", 1, true);
}

void MovementNode::publishGlobalMap() {
    auto map = global_planner_.getGlobalCostMap();
    map.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map, message);
    global_map_pub_.publish(message);
}

void MovementNode::publishLocalMap() {
    auto& map = local_planner_.getLocalMap();
    map.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map, message);
    local_map_pub_.publish(message);
}


} // kjq_navigation


int main(int argc, char **argv)
{
    ros::init(argc, argv, "movement");
    ros::NodeHandle nodeHandle;

    kjq_navigation::MovementNode movement(nodeHandle);

    // TODO use async spinner for more threads
    ros::spin();
    // ros::AsyncSpinner spinner(4); // Use 4 threads
    // spinner.start();

    return 0;
}
