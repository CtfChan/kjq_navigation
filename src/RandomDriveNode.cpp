#include "kjq_navigation/RandomDriveNode.hpp"


namespace kjq_navigation
{

RandomDriveNode::RandomDriveNode(ros::NodeHandle &node_handle) : 
    node_handle_(node_handle), map_({"local_costmap"})
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    laser_sub_ = node_handle_.subscribe("/scan", 1,
                                        &RandomDriveNode::laserCallback, this);

    grid_map_pub_ = node_handle_.advertise<grid_map_msgs::GridMap>("/random_drive/local_costmap", 1, true);
    
    safe_cmd_vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    map_.setGeometry(grid_map::Length(1.0, 1.0), 0.015, grid_map::Position(0.0, 0.0));
    map_.setFrameId("base_link"); 

    ROS_INFO("Successfully launched node.");
}

bool RandomDriveNode::readParameters()
{
    //   if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
    return true;
}


void RandomDriveNode::laserCallback(const sensor_msgs::LaserScan& msg) {
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_min;
    float angle_increment = msg.angle_increment;
    float range_min = msg.range_min;
    float range_max = msg.range_max;
    std::vector<float> ranges = msg.ranges;

    std::vector<int> pillars = {90,91,92,93,
                                176,177,178,179,180,181,182,183,184,185,186,187,
                                305,306,307,308,309,310,311,
                                399,400,401,402,403,404};

    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
        map_.at("local_costmap", *iterator) = 0.0;
    }


    for (int i = 0; i < ranges.size(); ++i) {
        if (std::binary_search(pillars.begin(), pillars.end(), i))
            continue;

        if (ranges[i] < range_min)
            continue;

        if (ranges[i] > 0.49 || ranges[i] > range_max)
            ranges[i] = 0.49;
        
        // compute 3D point
        float angle = angle_min + angle_increment * i;
        float x = ranges[i] * std::cos(angle);
        float y = ranges[i] * std::sin(angle);
        float z = 0.0f;

        //we assume laser is directly above base link, only upside down
        x = -x;

        // ray trace to end point
        grid_map::Position start(x, y);

        grid_map::Position end(-std::cos(angle) * 0.49, std::sin(angle) * 0.49);

        for (grid_map::LineIterator iterator(map_, start, end);
            !iterator.isPastEnd(); ++iterator) {
            map_.at("local_costmap", *iterator) = 0.1;
        }
    }

    publishGridMap();

    publishCmdVel();  
}

void RandomDriveNode::publishCmdVel() {

    // some temporary parms
    float dt = 0.1f;
    float robot_radius = 0.10f;
    float lin_vel_x = 0.1f;
    float ang_vel_z = 0.6f;

    grid_map::Position center(0.0, 0.0);
    for (grid_map::CircleIterator iterator(map_, center, robot_radius);
        !iterator.isPastEnd(); ++iterator) {
        map_.at("local_costmap", *iterator) = 0.0;
    }

    bool move_left = true;
    bool move_right= true;
    grid_map::Polygon left_polygon;
    left_polygon.setFrameId(map_.getFrameId());
    grid_map::Polygon right_polygon;
    right_polygon.setFrameId(map_.getFrameId());

    // Left polygon
    left_polygon.addVertex(grid_map::Position(0, 0));
    left_polygon.addVertex(grid_map::Position(0.19, 0));
    left_polygon.addVertex(grid_map::Position(0.17, 0.12));
    left_polygon.addVertex(grid_map::Position(0, 0.12));
    for (grid_map::PolygonIterator iterator(map_, left_polygon); !iterator.isPastEnd(); ++iterator) {
        if (map_.at("local_costmap", *iterator) > 0.0) {
            move_left = false;
            break;
        }
    }

    // Right polygon
    right_polygon.addVertex(grid_map::Position(0, 0));
    right_polygon.addVertex(grid_map::Position(0.19, 0));
    right_polygon.addVertex(grid_map::Position(0.17, -0.12));
    right_polygon.addVertex(grid_map::Position(0, -0.12));
    for (grid_map::PolygonIterator iterator(map_, right_polygon); !iterator.isPastEnd(); ++iterator) {
        if (map_.at("local_costmap", *iterator) > 0.0) {
            move_right = false;
            break;
        }
    }

    geometry_msgs::Twist safe_msg;
    if (move_left && move_right) {
        //ROS_INFO("MOVE FORWARD.");
        safe_msg.linear.x = lin_vel_x;
    } else if (move_left) {
        //ROS_INFO("TURNING");
        safe_msg.angular.z = ang_vel_z;
    } else if (move_right) {
        safe_msg.angular.z = -ang_vel_z;
    } else {
        safe_msg.angular.z = (rand()%1? -1 : 1) * ang_vel_z;
    }

    // publish safe vel
    safe_cmd_vel_pub_.publish(safe_msg);
}

void RandomDriveNode::publishGridMap() {
    map_.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map_, message);
    grid_map_pub_.publish(message);
    ROS_DEBUG("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
}   

} // namespace kjq_navigation


int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_drive");
    ros::NodeHandle nodeHandle;

    kjq_navigation::RandomDriveNode RandomDriveNode(nodeHandle);

    ros::spin();
    return 0;
}
