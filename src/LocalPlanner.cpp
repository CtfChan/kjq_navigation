#include "kjq_navigation/LocalPlanner.hpp"



namespace kjq_navigation
{


LocalPlanner::LocalPlanner(ros::NodeHandle &node_handle) : 
        node_handle_(node_handle), map_({"local_costmap"}) {

    //  publisher initialization
    grid_map_pub_ = node_handle_.advertise<grid_map_msgs::GridMap>("/local_planner/local_costmap", 1, true);
    safe_cmd_vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

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

    map_.get("local_costmap") = grid_map::Matrix::Zero(map_.getSize()(0), map_.getSize()(1));

      for (int i = 0; i < ranges.size(); ++i) {        
        // compute 3D point
        float angle = angle_min + angle_increment * i;
        float x = ranges[i] * std::cos(angle);
        float y = ranges[i] * std::sin(angle);
        float z = 0.0f;

        //we assume laser is directly above base link, only upside down
        x = -x;

        // inflate around point
        grid_map::Position center(x, y);
        for (grid_map::CircleIterator iterator(map_, center, robot_radius_); !iterator.isPastEnd(); ++iterator) {
            map_.at("local_costmap", *iterator) = 1.0;
        }     
    }

    publishGridMap();
}

void LocalPlanner::publishRandomCmdVel() {
    
}



void LocalPlanner::publishGridMap() {
    map_.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map_, message);
    grid_map_pub_.publish(message);
    ROS_DEBUG("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
}   





    




} // namespace kjq_navigation



