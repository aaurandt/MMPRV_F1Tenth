#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "mpc.hpp"
#include "math.h"

MPC::MPC(){
	solver_init = false;
	horizon_init = false;
}

void MPC::setHorizon(int horizon){
	if(!horizon_init){
		prediction_horizon = horizon;
		state_size = 3;
		input_size = 2;
		num_states = state_size * (prediction_horizon + 1);
		num_inputs = input_size * prediction_horizon;
		num_variables = num_states + num_inputs;
		num_constraints = num_states + num_inputs;
		
		//Initialize cost matrices
		q.resize(state_size);
		r.resize(input_size);
	      	//q.diagonal() << 10, 10, 0;
	      	//r.diagonal() << .1, 1;
	      	q.diagonal() << 5, 5, .5;
	      	r.diagonal() << 10, .1;
	      	
		//Initialize and populate H matrix
	   	H.resize(num_variables, num_variables);
	   	for(int i = 0; i < num_variables; i++){
	   		if(i < num_states){
	   			int pos_q = i%q.rows();
	   			double value = q.diagonal()[pos_q];
	   			if(value != 0)
	   				H.insert(i,i) = value;
	   		}
	   		else{
	   			int pos_r = i%r.rows();
	   			double value = r.diagonal()[pos_r];
	   			if(value != 0)
	   				H.insert(i,i) = value;
			}
	   	}
	   	
	   	
	   	//Initialize and populate constraint matrix
	   	A.resize(num_constraints, num_variables);
	   	for(int i = 0; i < num_states; i++){
	   		A.insert(i,i) = -1.0;
		}
		for(int i = 0; i < num_inputs; i++){
			A.insert(i+num_states, i+num_states) = 1.0;
		}
		horizon_init = true;
	}
}

std::vector<State> MPC::getPredictedTrajectory(){
	std::vector<State> states;
	State temp;
	for(int i = 0; i < num_states; i=i+3){
	 	temp.setX(QPSolution[i]);
	 	temp.setY(QPSolution[i+1]);
	 	temp.setYaw(QPSolution[i+2]);
	 	states.emplace_back(temp);
	}
	return states;
}

std::vector<Input> MPC::getPredictedInputs(){
	std::vector<Input> inputs;
	Input temp;
	for(int i = 0; i < num_inputs; i=i+2){
		temp.setVelocity(QPSolution[num_states+i]);
		temp.setSteeringAngle(QPSolution[num_states+i+1]);
	 	inputs.emplace_back(temp);
	}
	return inputs;
}

Input MPC::getNextInput(){
	Input temp;
	temp.setVelocity(QPSolution[num_states]);
	temp.setSteeringAngle(QPSolution[num_states+1]);
	//std::cout << "Commanded velocity and steering angle: " << temp.getVelocity() << ", " << temp.getSteeringAngle() << std::endl;
	return temp;
}

bool MPC::UpdateMPC(State curr_state, Input curr_input, std::vector<State> reference_trajectory){
 	if(reference_trajectory.size() != prediction_horizon+1){
 		return false;
 	}
 	/*std::cout << "Reference trajectory: " << std::endl;
 	for(State s : reference_trajectory){
 		std::cout << s.getX() << ", " << s.getY() << std::endl;
 	}*/
 	
   	//std::cout << "Initialization has begun..." << std::endl;
   	//Eigen::MatrixXd temp_hessian(H);
	//std::cout << "Hessian matrix: " << std::endl << temp_hessian << std::endl; 
	setGradientVector(reference_trajectory);
	//std::cout << "Set gradient vector!" << std::endl << g << std::endl;
	model.update_state_matrices(curr_state, curr_input);
	//std::cout << "Matrix A: " << std::endl << model.getA() << std::endl << "Matrix B: " << std::endl << model.getB() << std::endl;
	setConstraintMatrix();
	//Eigen::MatrixXd temp(A);
	//std::cout << "Set constraint matrix!" << std::endl << temp << std::endl;
	setBoundVectors(curr_state, curr_input);
	//std::cout << "Current input is: " << std::endl << curr_input.getVector() << std::endl;
	//std::cout << "Lower bound is: " << std::endl << lb << std::endl;
	//std::cout << "Upper bound is: " << std::endl << ub << std::endl;
	
	if(!solver_init){
		mpc_solver.settings()->setWarmStart(false);
		mpc_solver.settings()->setVerbosity(false);
		mpc_solver.data()->setNumberOfVariables(num_variables);
		mpc_solver.data()->setNumberOfConstraints(num_constraints);
		mpc_solver.data()->clearHessianMatrix();
		mpc_solver.data()->clearLinearConstraintsMatrix();
		if(!mpc_solver.data()->setHessianMatrix(H)){
			std::cout << "Hessian matrix initialization failed!" << std::endl;
			std::cout << "Hessian is " << H.rows() << " x " << H.cols() << ", but needs to be " << num_variables << " x " << num_variables << std::endl;
			return false;
		}
		if(!mpc_solver.data()->setGradient(g)){
			std::cout << "Gradient vector initialization failed!" << std::endl;
			return false;
		}
		if(!mpc_solver.data()->setLinearConstraintsMatrix(A)){
			std::cout << "Contraints matrix initialization failed!" << std::endl;
			return false;
		}
		if(!mpc_solver.data()->setLowerBound(lb)){
			std::cout << "Lower bound constraint vector initialization failed!" << std::endl;
			return false;
		}
		if(!mpc_solver.data()->setUpperBound(ub)){
			std::cout << "Upper bound constraint vector initialization failed!" << std::endl;
			return false;
		}
		if(!mpc_solver.initSolver()){
			std::cout << "Couldn't initialize solver!" << std::endl;
			return false;
		}
		solver_init = true;
	}
	else{
		if(!mpc_solver.updateGradient(g)){
			std::cout << "Gradient vector update failed!" << std::endl;
			return false;
		}	
		if(!mpc_solver.updateLinearConstraintsMatrix(A)){
			std::cout << "Constraint matrix update failed!" << std::endl;
			return false;
		}
		if(!mpc_solver.updateBounds(lb, ub)){
			std::cout << "Lower and Upper bound constraint vector update failed!" << std::endl;
			std::cout << "Lower bound is: " << std::endl << lb << std::endl;
			std::cout << "Upper bound is: " << std::endl << ub << std::endl;
			return false;
		}	
	}
	if(mpc_solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError){
		mpc_solver.clearSolver();
		solver_init = false;
		std::cout << "Couldn't solve the QP problem" << std::endl;
		return false;
	}
	QPSolution = mpc_solver.getSolution();
	//std::cout << "Solution: " << std::endl << QPSolution << std::endl;
	//std::cout << "Solution: " << std::endl << QPSolution.block(num_states, 0, num_inputs, 1) << std::endl;
	if(mpc_solver.getStatus() != OsqpEigen::Status::Solved){
		mpc_solver.clearSolver();
		solver_init = false;
		std::cout << "OSQP did not converge!" << std::endl;
		return false;
	}
	return true;
}

void MPC::setGradientVector(std::vector<State> state_refs){
	g = Eigen::VectorXd::Zero(num_variables);
	for(int i = 0; i < prediction_horizon + 1; i++){
		g.block(i*state_size, 0, state_size, 1) = q * (-state_refs[i].getVector());
	}
}

void MPC::setConstraintMatrix(){
	for(int i = 1; i < prediction_horizon + 1; i++){
		for(int j = 0; j < state_size; j++){
			for(int k = 0; k < state_size; k++){
				A.coeffRef((i*state_size)+j,(i*state_size)-state_size+k) = model.getA().coeff(j,k);
			}
		}
		for(int j = 0; j < state_size; j++){
			for(int k = 0; k < input_size; k++){
				A.coeffRef((i*state_size)+j,(i*input_size)-input_size+num_states+k) = model.getB().coeff(j,k);
			}
		}
	}
}

void MPC::setBoundVectors(State curr_state, Input curr_input){
	lb = Eigen::VectorXd::Zero(num_constraints);
	lb.block(0, 0, state_size, 1) = -1 * curr_state.getVector();
	for(int i = 0; i < prediction_horizon; i++){
		lb.block((i*state_size)+state_size, 0, state_size, 1) = -1.0*model.getC();
	}
	for(int i = 0; i < prediction_horizon; i++){
		lb.block((i*input_size)+num_states, 0, input_size, 1) = constraints.getUMin().getVector();
	}

	ub = Eigen::VectorXd::Zero(num_constraints);
	ub.block(0, 0, state_size, 1) = -1 * curr_state.getVector();
	for(int i = 0; i < prediction_horizon; i++){
		ub.block((i*state_size)+state_size, 0, state_size, 1) = -1.0*model.getC();
	}
	for(int i = 0; i < prediction_horizon; i++){
		ub.block((i*input_size)+num_states, 0, input_size, 1) = constraints.getUMax().getVector();
	}
}
