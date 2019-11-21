#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "kjq_navigation/GaussSmoothen.hpp"
#include <math.h>


float [] scanCallback(sensor_msgs::LaserScan::ConstPtr msg) {
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
    int block_angle_subtension = 10;
    int block_found = 0;
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
            far_to_close_indices.push_back(i);
        }

        if (disparity > disparity_threshold){
            close_to_far__indices.push_back(i);
        }

        int index_one = 0;
        int index_two = -1;
        for(int i = 0; i < close_to_far__indices.size(); ++i){
            int index_one = close_to_far__indices[i]
                        
            for(int j = 0; j < far_to_close_indices.size(); ++j){
                if(std::abs(index_one - far_to_close_indices[j]) <= block_angle_subtension ){
                    block_found = 1;
                    index_two = far_to_close_indices[j]
                    break;
                }
            if(block_found) break;
        }

        if (!block_found) return null;

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

		/////////
		float angle_one = angle_min + angle_increment*index_one;
		block_points[0][0] = ranges[index_one] * std::cos(angle_one);
		block_points[1][0] = ranges[index_one] * std::sin(angle_one);
			
		float angle_two = angle_min + angle_increment*index_two;
		block_points[0][1] = ranges[index_two] * std::cos(angle_two);
		block_points[1][1] = ranges[index_two] * std::sin(angle_two);

        block_center[0] = (block_points[0][0] + block_points[0][1])/2.0;
        block_center[1] = (block_points[1][0] + block_points[1][1])/2.0;
		/////////
        float distance = pow(block_center[0]*block_center[0] + block_center[1]*block_center[1], 0.5) + pow(
            2.0, 0.5);

        block_center[0] = distance * std::cos((angle_one + angle_two)/2.0);
        block_center[1] = distance * std::sin((angle_one + angle_two)/2.0);

    return block_center;
    
}
    


int main(int argc, char  **argv) {
    ros::init(argc, argv, "pi_bridge_node");
    ros::NodeHandle n;

    ros::Subscriber laser_sub = n.subscribe("/scan", 1000, scanCallback);




    return 0;
}





