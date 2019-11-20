#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>


namespace kjq_navigation
{

class PiBridgeNode {
public:
    PiBridgeNode(ros::NodeHandle& nh);

private:
    void scanCallback(sensor_msgs::LaserScan::Ptr msg);

    void odomCallback(geometry_msgs::Pose::Ptr msg);

    ros::NodeHandle n_;
    geometry_msgs::TransformStamped t_;
    tf::TransformBroadcaster broadcaster_;


    ros::Subscriber laser_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher laser_pub_;
    ros::Publisher odom_pub_;


};

}