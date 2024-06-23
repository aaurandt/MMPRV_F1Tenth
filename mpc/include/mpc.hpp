#ifndef MPC_H
#define MPC_H

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <Eigen/Core>
#include <OsqpEigen/OsqpEigen.h>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "state.hpp"
#include "input.hpp"
#include "vehicle_model.hpp"
#include "constraints.hpp"


class MPC{
public:
	MPC();
	void setHorizon(int horizon);
	std::vector<State> getPredictedTrajectory();
	std::vector<Input> getPredictedInputs();
	Input getNextInput();
	bool UpdateMPC(State curr_state, Input curr_input, std::vector<State> reference_trajectory);
private:
	int prediction_horizon;
	bool horizon_init;
	bool solver_init;
	int state_size;
	int input_size;
	int num_states;
	int num_inputs;
	int num_variables;
	int num_constraints;
	OsqpEigen::Solver mpc_solver;
	Model model;
	Constraints constraints;
	Eigen::SparseMatrix<double> H;
	Eigen::VectorXd g;
	Eigen::SparseMatrix<double> A;
	Eigen::VectorXd lb;
	Eigen::VectorXd ub;
	Eigen::DiagonalMatrix<double, 3> q;
   	Eigen::DiagonalMatrix<double, 2> r;
   	Eigen::VectorXd QPSolution;
   	void setGradientVector(std::vector<State> state_refs);
	void setConstraintMatrix();
	void setBoundVectors(State curr_state, Input curr_input);
};

#endif
