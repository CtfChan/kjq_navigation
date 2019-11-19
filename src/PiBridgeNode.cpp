#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

ros::Publisher laser_pub, odom_pub;


geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;


void scanCallback(sensor_msgs::LaserScan::Ptr msg) {
    msg->header.stamp = ros::Time::now();
    laser_pub.publish(msg);
}

void odomCallback(nav_msgs::Odometry::Ptr msg) {
    msg->header.stamp = ros::Time::now();
    odom_pub.publish(msg);

    t.header.stamp = msg->header.stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y= msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    broadcaster.sendTransform(t);
}

int main(int argc, char  **argv) {
    ros::init(argc, argv, "pi_bridge_node");
    ros::NodeHandle n;

    ros::Subscriber laser_sub = n.subscribe("/scan", 1000, scanCallback);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1000, odomCallback);

    laser_pub = n.advertise<sensor_msgs::LaserScan>("/scan_synced", 1000);
    odom_pub = n.advertise<nav_msgs::Odometry>("/odom_synced", 1000);


    ros::spin();

    return 0;
}





