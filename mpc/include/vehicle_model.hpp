#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include "state.hpp"
#include "input.hpp"


class Model{
public:
	Model();
	Eigen::Matrix<double, 3, 3> getA();
	Eigen::Matrix<double, 3, 2> getB();
	Eigen::Matrix<double, 3, 1> getC();
	void update_state_matrices(State curr_state, Input curr_input);
	State get_next_state(State curr_state, Input curr_input);
private:
	Eigen::Matrix<double, 3, 3> a_matrix;
	Eigen::Matrix<double, 3, 2> b_matrix;
	Eigen::Matrix<double, 3, 1> c_matrix;
	const double lf = .15875;
	const double lr = .17145;
	const double dt = 0.05;
};

#endif
