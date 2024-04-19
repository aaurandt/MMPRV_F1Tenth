#include "goal_point_locator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <fstream>

#define M_PI 3.14159265358979323846

GoalPoint::GoalPoint(){
	goal_lookahead_dist = 4.0;
	std::ifstream data("./src/f1tenth_racetracks/Monza/Monza_centerline.csv", std::fstream::in);
	if(data.fail())
      		perror("./src/f1tenth_racetracks/Monza/Monza_centerline.csv");
	std::string line = "";
	Position position_temp;
	std::string cell_temp = "";
	for(int i = 0; i < 1; i++) // Skip header lines
		getline(data, line);
	while(getline(data, line)){
		std::istringstream lineStream(line);
		getline(lineStream, cell_temp, ',');
		position_temp.setX(std::stod(cell_temp));
		getline(lineStream, cell_temp, ',');
		position_temp.setY(std::stod(cell_temp));		
		waypoints.push_back(position_temp);
	}
}

Position GoalPoint::findRefGoalPointPurePursuit(State curr_state){
	int curr_index = 0;
	int end_index = waypoints.size() - 1;
	bool inside_circle = false;
	double distance, angle, diff_angle;
	int min_dist_index = 0;
	double min_distance = 20.0;
	while(curr_index <= end_index){
		distance = sqrt(pow(waypoints[curr_index].getX() - curr_state.getX(), 2) + 
				pow(waypoints[curr_index].getY() - curr_state.getY(), 2));
		angle = atan2(waypoints[curr_index].getY() - curr_state.getY(), waypoints[curr_index].getX() - curr_state.getX());
		diff_angle = curr_state.getYaw() - angle;
		// Make diff_angle between -180 and 180 degrees
		if(diff_angle < (-1*M_PI))
			diff_angle += 2*M_PI;
		else if(diff_angle > M_PI)
			diff_angle -= 2*M_PI;
		if((diff_angle < 2.35619) && (diff_angle > -2.35619)){ // Waypoint is not behind car (+-135 degree window)
			if(distance < min_distance){
				min_dist_index = curr_index;
				min_distance = distance;
			}
			if(distance < goal_lookahead_dist){
				inside_circle = true;
			}
			else if((distance > goal_lookahead_dist) && inside_circle){
				return waypoints[curr_index];
			}
		}
		curr_index++;
	}
	return waypoints[min_dist_index];
}
