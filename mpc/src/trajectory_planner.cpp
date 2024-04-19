#include "trajectory_planner.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <fstream>

#define M_PI 3.14159265358979323846

Trajectory::Trajectory(){
	std::ifstream data("./src/MMPRV_F1Tenth/mpc/Monza_MPC_ref.csv", std::fstream::in);
	if(data.fail())
      		perror("./src/MMPRV_F1Tenth/mpc/Monza_MPC_ref.csv");
	std::string line = "";
	State state_temp;
	std::string cell_temp = "";
	while(getline(data, line)){
		std::istringstream lineStream(line);
		getline(lineStream, cell_temp, ',');
		state_temp.setX(std::stod(cell_temp));
		getline(lineStream, cell_temp, ',');
		state_temp.setY(std::stod(cell_temp));		
		getline(lineStream, cell_temp, '\n');
		double yaw = std::stod(cell_temp);
		state_temp.setYaw(yaw);
		ref_global_trajectory.push_back(state_temp);
	}
	init = false;
	start_index = 0;
}

void Trajectory::get_xy_from_map_to_grid(double map_x, double map_y, int* grid_x, int* grid_y){
	double x = map_x - input_map.info.origin.position.x;
	double y = map_y - input_map.info.origin.position.y;
	*grid_x = static_cast<int>((x*cos(input_map_yaw)+y*sin(input_map_yaw))/input_map.info.resolution) - 1;
     	*grid_y = static_cast<int>((-1*x*sin(input_map_yaw)+y*cos(input_map_yaw))/input_map.info.resolution) - 1;
}

void Trajectory::get_xy_from_grid_to_map(int grid_x, int grid_y, double* map_x, double* map_y){
	*map_x = (input_map.info.resolution * (1.0 + (grid_x*cos(input_map_yaw)-grid_y*sin(input_map_yaw)))) + 
		 input_map.info.origin.position.x;
	*map_y = (input_map.info.resolution * (1.0 + (grid_x*sin(input_map_yaw)+grid_y*cos(input_map_yaw)))) +
		 input_map.info.origin.position.y;
}

void Trajectory::setGlobalMap(nav_msgs::msg::OccupancyGrid map){
	if(!init){
		init = true;
    		input_map = map;
    		tf2::Quaternion q(input_map.info.origin.orientation.x, 
           		input_map.info.origin.orientation.y, 
           		input_map.info.origin.orientation.z, 
   			input_map.info.origin.orientation.w);
		double roll, pitch;
    		tf2::Matrix3x3(q).getRPY(roll, pitch, input_map_yaw);
    	}
}

std::vector<State> Trajectory::getRefTrajectory(){
	return ref_global_trajectory;
}

std::vector<State> Trajectory::findRefTrajectoryPurePursuit(State curr_state, int num_steps){
	int index = findStartPoint(curr_state);
	if(index == -1)
		return ref_local_trajectory;
	ref_local_trajectory.clear();
	for(int i = 0; i < num_steps*3; i+=3){
		if(i+index >= ref_global_trajectory.size()){
			ref_local_trajectory.emplace_back(ref_global_trajectory[i+index-ref_global_trajectory.size()]);
		}
		else{
			ref_local_trajectory.emplace_back(ref_global_trajectory[i+index]);
		}
	}
	return ref_local_trajectory;
}

int Trajectory::findStartPoint(State curr_state){
	int curr_index = start_index;
	int end_index = ref_global_trajectory.size() - 1;
	double distance, angle, diff_angle;
	int min_dist_index = 0;
	double min_distance = 3.0;
	bool valid = false;
	while(curr_index <= end_index){
		distance = sqrt(pow(ref_global_trajectory[curr_index].getX() - curr_state.getX(), 2) + 
				pow(ref_global_trajectory[curr_index].getY() - curr_state.getY(), 2));
		angle = atan2(ref_global_trajectory[curr_index].getY() - curr_state.getY(), ref_global_trajectory[curr_index].getX() - curr_state.getX());
		diff_angle = curr_state.getYaw() - angle;
		// Make diff_angle between -180 and 180 degrees
		if(diff_angle < (-1*M_PI))
			diff_angle += 2*M_PI;
		else if(diff_angle > M_PI)
			diff_angle -= 2*M_PI;
		if((diff_angle < 1.5708) && (diff_angle > -1.5708)){ // Waypoint is not behind car (+-90 degree window)
			if(distance < min_distance){
				min_dist_index = curr_index;
				min_distance = distance;
				valid = true;
			}
		}
		if((curr_index == end_index) && (start_index != 0) && (!valid)){
			end_index = start_index - 1;
			curr_index = 0;
			start_index = 0;
		}
		curr_index++;
	}
	if(!valid)
		return -1;
	start_index = min_dist_index;
	return min_dist_index;
}

void Trajectory::UpdateLaserScan(sensor_msgs::msg::LaserScan scan_msg_){
	scan_msg = scan_msg_;
}
