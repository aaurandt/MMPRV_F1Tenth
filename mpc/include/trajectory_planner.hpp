#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <Eigen/Core>
#include "state.hpp"
#include "input.hpp"


class Trajectory{
public:
	Trajectory();
	void get_xy_from_map_to_grid(double map_x, double map_y, int* grid_x, int* grid_y);
	void get_xy_from_grid_to_map(int grid_x, int grid_y, double* map_x, double* map_y);
	void setGlobalMap(nav_msgs::msg::OccupancyGrid map);
	std::vector<State> getRefTrajectory();
	std::vector<State> findRefTrajectoryPurePursuit(State curr_state, int num_steps);
	//std::vector<State> findRefTrajectoryGap(tf2::Stamped<tf2::Transform> tf_laser_to_map, State curr_state, std::vector<geometry_msgs::msg::Point> &points, int num_steps);
	void UpdateLaserScan(sensor_msgs::msg::LaserScan scan_msg_);
	//void findGlobalReferenceTrajectory(std::vector<geometry_msgs::msg::Point> &points);
private:
	int findStartPoint(State curr_state);
	std::vector<State> ref_global_trajectory;
	std::vector<State> ref_local_trajectory;
	int start_index;
	bool init;
	sensor_msgs::msg::LaserScan scan_msg;
	nav_msgs::msg::OccupancyGrid input_map;
	double input_map_yaw;
};

#endif
