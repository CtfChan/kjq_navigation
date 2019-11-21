#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "kjq_navigation/GaussSmoothen.hpp"


void scanCallback(sensor_msgs::LaserScan::ConstPtr msg) {
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_max;
    float angle_increment = msg.angle_increment;
    float range_min = msg.range_min;
    float range_max = msg.range_max;
    std::vector<double> ranges = msg.ranges;

	
	
	std::vector<int> pillars = {90,91,92,93,
                                176,177,178,179,180,181,182,183,184,185,186,187,
                                305,306,307,308,309,310,311,
                                399,400,401,402,403,404};


	/////////////////////////
	//bool is_wall = true;
	double sigma = 2;
	double samples = 10
	ranges = gaussSmoothen(ranges, sigma, samples)
	float min_radius = inf;
	int min_index = -1;
	float disparity_threshold = 0.071842049;
    float block_width = 0.071842049;
	float block_points [2][2] = {};
    float block_center [2][1] = {};
	/////////////////////////
    std::vector<int> far_to_close_indices;
    std::vector<int> close_to_far_indices; 
	
    for (int i = 0; i < ranges.size(); ++i) {
        if (std::binary_search(pillars.begin(), pillars.end(), i))
            continue;

        if (ranges[i] < range_min)
            continue;

        if (ranges[i] > 0.49 || ranges[i] > range_max)
            ranges[i] = 0.49;
		//////////////////////		
		
			
		float disparity = ranges[i] - ranges[i-1];
        
        if (disparity < -1 * disparity_threshold){
            far_to_close_indices.push_back(i)
        }

        if (disparity > disparity_threshold){
            close_to_far__indices.push_back(i)
        }

        // if (std::abs(disparity) > disparity_threshold){
		// 	if (is_wall == true){
        //         is_wall = false;
		// 		block_points[0][0] = ranges[i]*std::cos(angle);
		// 		block_points[1][0] = ranges[i]*std::sin(angle);
		// 	}
		// 	else{
		// 		is_wall = true;
		// 		block_points[0][2] = ranges[i]*std::cos(angle);
		// 		block_points[1][2] = ranges[i]*std::sin(angle);
		// 	}
						
		// }
		//////////
    }

    for 
		///////////
		//float anglemin = angle_min + angle_increment*min_index;
		//block_points[0][1] = ranges[min_index] * std::cos(anglemin);
		//block_points[1][1] = ranges[min_index] * std::sin(anglemin);
			
		//block_points[0][3] = block_points[0][0] + block_points[0][2] - block_points[0][1];
		//block_points[1][3] = block_points[1][0] + block_points[1][2] - block_points[1][1];
		///////////
    
}
    


int main(int argc, char  **argv) {
    ros::init(argc, argv, "pi_bridge_node");
    ros::NodeHandle n;

    ros::Subscriber laser_sub = n.subscribe("/scan", 1000, scanCallback);




    return 0;
}





