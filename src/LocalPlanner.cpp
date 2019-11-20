#include "kjq_navigation/LocalPlanner.hpp"

#include <iostream>

namespace kjq_navigation
{

grid_map::GridMap& LocalPlanner::getLocalMap() {
    return map_;
}

LocalPlanner::LocalPlanner() : map_({"local"}) {
    // initialize map details
    map_.setGeometry(grid_map::Length(map_width_, map_length_), map_resolution_, 
            grid_map::Position(0.0, 0.0));
    map_.setFrameId("base_link"); 

}

void LocalPlanner::updateLocalMap(const sensor_msgs::LaserScan& msg) {
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_min;
    float angle_increment = msg.angle_increment;
    float range_min = msg.range_min;
    float range_max = msg.range_max;
    std::vector<float> ranges = msg.ranges;

    std::cerr << "UPDATING LOCAL MAP" << std::endl;

    map_.get("local") = grid_map::Matrix::Zero(map_.getSize()(0), map_.getSize()(1));

    for (int i = 0; i < ranges.size(); ++i) {   
        if(ranges[i] < 0.12)
            continue;
     
        // compute 3D point
        float angle = angle_min + angle_increment * i;
        float x = ranges[i] * std::cos(angle);
        float y = ranges[i] * std::sin(angle);
        float z = 0.0f;

        //we assume laser is directly above base link, only upside down
        x = -x;

        // check in map, if so run circle iterator
        grid_map::Position center(x, y);
        if (map_.isInside(center)) {
            for (grid_map::CircleIterator iterator(map_, center, robot_radius_);
                !iterator.isPastEnd(); ++iterator) {
                map_.at("local", *iterator) = 1.0;
            }
        }
    }
}


// void LocalPlanner::publishRandomCmdVel() {
    
// }



// void LocalPlanner::publishGridMap() {
//     map_.setTimestamp(ros::Time::now().toNSec());
//     grid_map_msgs::GridMap message;
//     grid_map::GridMapRosConverter::toMessage(map_, message);
//     grid_map_pub_.publish(message);
//     ROS_DEBUG("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
// }   





    




} // namespace kjq_navigation



