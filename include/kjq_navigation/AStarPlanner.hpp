#pragma once

// ROS
#include <grid_map_ros/grid_map_ros.hpp>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>


namespace kjq_navigation
{

class AStarPlanner {
public:
    AStarPlanner();

    void setMap(const nav_msgs::OccupancyGrid& msg);

    nav_msgs::Path plan(const grid_map::Position& start, const grid_map::Position& end);

    grid_map::GridMap& getGlobalCostMap() { return map_; }


    // TODO deprecate
    AStarPlanner(const float robot_radius, grid_map::GridMap& map);


private:


    bool inMap(const grid_map::Index& idx) {
        return idx.x() >= 0 && idx.y() >= 0 && 
                idx.x() < map_.getSize()(0) && idx.y() < map_.getSize()(1);
    }

    std::string index2String(const grid_map::Index& idx) {
        return std::to_string(idx.x()) + " " + std::to_string(idx.y());
    }

    grid_map::Index string2Index(const std::string str) {
        int x, y;
        std::istringstream iss(str);
        iss >> x >> y;
        return grid_map::Index(x, y);
    }

    float calculateHeuristic(const grid_map::Index& start, const grid_map::Index& end);


    // private member variables
    float robot_radius_ = 0.1;
    bool map_initialized_ = false;
    grid_map::GridMap map_; // this stores global map and inflated global map

};



} // namespace kjq_navigation