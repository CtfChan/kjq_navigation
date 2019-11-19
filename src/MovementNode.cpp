#include "kjq_navigation/MovementNode.hpp"

namespace kjq_navigation {

void MovementNode::laserCallback(const sensor_msgs::LaserScan& msg) {
    // update local planner
    local_planner_.updateLocalMap(msg);

    // publish cmd_vel based on state
    if (state_ == RobotState::LOST) {
        local_planner_.publishRandomCmdVel();
    } else if (state_ == RobotState::WAYPOINT_TRACKING) {

    }


}






MovementNode::MovementNode(ros::NodeHandle &node_handle) : 
    node_handle_(node_handle) {  

    laser_sub_ = node_handle_.subscribe("/scan_synced", 1,
                                        &MovementNode::laserCallback, this);
}

} // kjq_navigation





int main(int argc, char **argv)
{
    ros::init(argc, argv, "movement");
    ros::NodeHandle nodeHandle;

    kjq_navigation::MovementNode movement(nodeHandle);

    ros::spin();
    return 0;
}
