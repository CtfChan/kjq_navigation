
#include "kjq_navigation/BiPridgeNode.hpp"

namespace kjq_navigation {


void PiBridgeNode::scanCallback(sensor_msgs::LaserScan::Ptr msg) {
    msg->header.stamp = ros::Time::now();

    std::vector<float> &ranges = msg->ranges;
    std::vector<float> &intensities = msg->intensities;

    for (int i = 0; i < ranges.size(); ++i) {
        if (intensities[i] < 253.0) {
            ranges[i] = std::numeric_limits<float>::infinity();
        }
    }

    laser_pub_.publish(msg);
}

void PiBridgeNode::odomCallback(geometry_msgs::Pose::Ptr msg) {
    t_.header.stamp = ros::Time::now();
    t_.header.frame_id = "odom";
    t_.child_frame_id = "base_link";

    t_.transform.translation.x = msg->position.x;
    t_.transform.translation.y = msg->position.y;
    t_.transform.translation.z = msg->position.z;
    t_.transform.rotation.x = msg->orientation.x;
    t_.transform.rotation.y= msg->orientation.y;
    t_.transform.rotation.z = msg->orientation.z;
    t_.transform.rotation.w = msg->orientation.w;

    broadcaster_.sendTransform(t_);
}


PiBridgeNode::PiBridgeNode(ros::NodeHandle& nh) : n_(nh) {
    laser_sub_ = n_.subscribe("/scan", 1000, &PiBridgeNode::scanCallback, this);
    odom_sub_ = n_.subscribe("/pose", 1000,  &PiBridgeNode::odomCallback, this);

    laser_pub_ = n_.advertise<sensor_msgs::LaserScan>("/scan_synced", 1000);
}

} //

int main(int argc, char  **argv) {
    ros::init(argc, argv, "pi_bridge_node");
    ros::NodeHandle n;

    auto p = kjq_navigation::PiBridgeNode(n);

    ros::spin();

    return 0;
}






