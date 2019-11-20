#include <ros/ros.h>
#include "kjq_navigation/AStarPlanner.hpp"

#include <grid_map_ros/grid_map_ros.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "AStarTestNode");
    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("global_path", 1, true);

    float robot_radius = 0.1;

    // Create empty map for now
    grid_map::GridMap map({"global"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(4*0.3048, 8*0.3048), 0.03, grid_map::Position(0.0, 0.0));
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1),
    map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());
    map.get("global") = grid_map::Matrix::Zero(map.getSize()(0), map.getSize()(1));

    // draw simple line
    grid_map::Position start(0.2, -0.2);
    grid_map::Position end(0.2, 0.2);
    for (grid_map::LineIterator it(map, start, end); !it.isPastEnd(); ++it) {
        map.at("global", *it) = 1.0;
    }

    kjq_navigation::AStarPlanner astar(robot_radius, map);

    auto path = astar.plan(grid_map::Position(0.0, 0.0), grid_map::Position(0.4, 0.0) );

    ros::Rate rate(30.0);
    while (nh.ok()) {
        auto& mp = astar.getGlobalCostMap();

        // Publish grid map.
        ros::Time time = ros::Time::now();
        map.setTimestamp(time.toNSec());
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(mp, message);
        map_pub.publish(message);
        // ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
        ROS_INFO("Publishing Grid");

        path.header.stamp = ros::Time::now();
        path_pub.publish(path);

        rate.sleep();

    }

    return 0;
}
