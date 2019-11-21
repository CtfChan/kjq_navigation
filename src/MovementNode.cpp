#include "kjq_navigation/MovementNode.hpp"

#include <iostream>

namespace kjq_navigation {

void MovementNode::laserCallback(const sensor_msgs::LaserScan& msg) {
    // update local planner
    local_planner_.updateLocalMap(msg);
    publishLocalMap();

    // publish cmd_vel based on state
    if (state_ == RobotState::LOST) {
        // local_planner_.publishRandomCmdVel();
    } else if (state_ == RobotState::WAYPOINT_TRACKING) {

    }
}

void MovementNode::mapCallback(const nav_msgs::OccupancyGrid& msg) {
    global_planner_.setMap(msg);
    std::cerr << "GOT GLOBAL MAP" << std::endl;
    publishGlobalMap();
}

void MovementNode::publishGlobalMap() {
    auto map = global_planner_.getGlobalCostMap();
    std::cerr << map.getSize().x() << " " <<  map.getSize().y() << std::endl;
    std::cerr << map.getLength().x() << " " <<  map.getLength().y() << std::endl;

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

MovementNode::MovementNode(ros::NodeHandle &node_handle) : node_handle_(node_handle) {  
    laser_sub_ = node_handle_.subscribe("/scan_synced", 1,
                                        &MovementNode::laserCallback, this);
    global_map_sub_ = node_handle_.subscribe("/loc_map", 1,
                                        &MovementNode::mapCallback, this);

    local_map_pub_ = node_handle_.advertise<grid_map_msgs::GridMap>("/local_map", 1, true);                
    global_map_pub_ = node_handle_.advertise<grid_map_msgs::GridMap>("/global_map", 1, true);
    cmd_vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
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
