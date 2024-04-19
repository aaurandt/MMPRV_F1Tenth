#ifndef GOALPOINT_H
#define GOAL_POINT_H

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "position.hpp"
#include "state.hpp"


class GoalPoint{
public:
	GoalPoint();
	Position findRefGoalPointPurePursuit(State curr_state);
private:
	double goal_lookahead_dist;
	std::vector<Position> waypoints;
};

#endif
