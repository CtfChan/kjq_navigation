#include <kjq_navigation/AStarPlanner.hpp>

#include <queue>
#include <unordered_set>

#include <Eigen/Geometry>


namespace kjq_navigation
{


void AStarPlanner::setMap(const nav_msgs::OccupancyGrid& msg) {
    if (map_initialized_) return;

    map_initialized_=true;
    grid_map::GridMapRosConverter::fromOccupancyGrid(msg, "global", map_);

    map_.add("inflation", grid_map::Matrix::Zero(map_.getSize()(0), map_.getSize()(1)) );

    // apply inflation to map
    grid_map::Position center;
    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
        if (map_.at("global", *it) > 0.0) {
            map_.getPosition(*it, center);
            for (grid_map::CircleIterator iterator(map_, center, robot_radius_); !iterator.isPastEnd(); ++iterator) {
                map_.at("inflation", *iterator) = 1.0;
            }
        }
    }
}


AStarPlanner::AStarPlanner() : map_(grid_map::GridMap({"global", "inflation"})) {
     map_.setFrameId("map");
}


// TODO use std::move for map
AStarPlanner::AStarPlanner(const float robot_radius, grid_map::GridMap& map) : 
    robot_radius_(robot_radius), map_(map), map_initialized_(true)  {

    // create new layers for planning purposes
    map_.add("inflation", grid_map::Matrix::Zero(map_.getSize()(0), map_.getSize()(1)));

    // apply inflation to map
    grid_map::Position center;
    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
        if (map_.at("global", *it) > 0.5) {
            map_.getPosition(*it, center);
            for (grid_map::CircleIterator iterator(map_, center, robot_radius); !iterator.isPastEnd(); ++iterator) {
                map_.at("inflation", *iterator) = 1.0;
            }
        }
    }


}


nav_msgs::Path AStarPlanner::plan(const grid_map::Position& start, const grid_map::Position& end) {
    nav_msgs::Path path;

    // convert to indices
    grid_map::Index start_idx, end_idx;
    if (map_.getIndex(start, start_idx) == false ) {
        ROS_ERROR("INVALID START POSITION FOR PLANNING!!!!");
        return path;
    }
    if ( map_.getIndex(end, end_idx) == false) {
        ROS_ERROR("INVALID END POSITION FOR PLANNING!!!!");
        return path;
    }

    std::string start_key = index2String(start_idx);
    std::string end_key = index2String(end_idx);
    ROS_INFO_STREAM("Planning mission " << start_key << " to " << end_key);

    // initialize data structures for A*
    auto comp = [](const std::pair<float, grid_map::Index>& p1, 
    const std::pair<float, grid_map::Index>& p2) {
        return p1.first > p2.first;
    };
    std::priority_queue<std::pair<float, grid_map::Index>, 
        std::vector<std::pair<float, grid_map::Index>>, decltype(comp)> open(comp);   
    std::unordered_map<std::string, std::string> came_from; 
    std::unordered_map<std::string, float> cost_so_far;

    open.emplace(0.0f, start_idx);
    cost_so_far[start_key] = 0.0f;
    came_from[start_key] = start_key;

    const std::vector<std::pair<int, int>> dir = {{-1, 0}, {0, -1}, {1, 0}, {0, 1},
                                                  {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    while(!open.empty()) {
        auto node = open.top(); open.pop();
        grid_map::Index idx = node.second;
        std::string idx_key = index2String(idx);
      
        if (idx.x() == end_idx.x() && idx.y() == end_idx.y())
            break;

        for (auto d : dir) {
            grid_map::Index neighbour(idx.x()+d.first, idx.y()+d.second);
            if (inMap(neighbour) && map_.isValid(neighbour, "inflation") && map_.at("inflation", neighbour) < 1.0) {
                float new_cost = cost_so_far[idx_key] + std::sqrt(d.first*d.first + d.second*d.second);
                std::string neighbour_key = index2String(neighbour);

                // if we've never been here before or the cost is lower than what we've seen
                if (cost_so_far.find(neighbour_key) == cost_so_far.end() 
                    || new_cost < cost_so_far[neighbour_key]) {
                    cost_so_far[neighbour_key] = new_cost;
                    float priority = new_cost + calculateHeuristic(neighbour, end_idx);
                    open.emplace(priority, neighbour);
                    came_from[neighbour_key] = idx_key;
                }
            }
        }
    }

    
    if (cost_so_far.find(end_key) != cost_so_far.end())
        ROS_INFO_STREAM("Planning complete with cost " << std::to_string(cost_so_far[end_key])) ;

    // reconstruct path
    std::string curr_key = end_key;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = map_.getFrameId();

    int x, y;
    grid_map::Position pos;

    while (curr_key != start_key) {
        std::istringstream iss(curr_key);
        iss >> x >> y;
        map_.getPosition(grid_map::Index(x, y), pos);
        
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = map_.getFrameId();
        pose.pose.position.x = pos.x();
        pose.pose.position.y = pos.y();
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);

        curr_key = came_from[curr_key];
    }

    std::reverse(path.poses.begin(), path.poses.end());

    // generate orientation
    for (size_t i = 0; i < path.poses.size()-1; ++i) {
        float dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
        float dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
        float yaw = std::atan2(dy, dx);
        
       Eigen::Quaternionf q = q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
                            * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
                            * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

        path.poses[i].pose.orientation.x = q.x();
        path.poses[i].pose.orientation.y = q.y();
        path.poses[i].pose.orientation.z = q.z();
        path.poses[i].pose.orientation.w = q.w();
        
        if (i+1 == path.poses.size()-1) {
            path.poses[i+1].pose.orientation.x = q.x();
            path.poses[i+1].pose.orientation.y = q.y();
            path.poses[i+1].pose.orientation.z = q.z();
            path.poses[i+1].pose.orientation.w = q.w();
        }

    }

    return path;


}



float AStarPlanner::calculateHeuristic(const grid_map::Index& start, const grid_map::Index& end) {  
    return (start.matrix()-end.matrix()).norm();
}


    




} // namespace kjq_navigation



