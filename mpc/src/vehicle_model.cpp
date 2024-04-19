#include "vehicle_model.hpp"

Model::Model(){
}

Eigen::Matrix<double, 3, 3> Model::getA(){
	return a_matrix;
}

Eigen::Matrix<double, 3, 2> Model::getB(){
	return b_matrix;
}

Eigen::Matrix<double, 3, 1> Model::getC(){
	return c_matrix;
}

void Model::update_state_matrices(State curr_state, Input curr_input){
	double slip_angle = atan((lr*tan(curr_input.getSteeringAngle()))/(lf+lr));
	a_matrix << 1, 0, -1.0*curr_input.getVelocity()*sin(curr_state.getYaw()+slip_angle)*dt,
		    0, 1, curr_input.getVelocity()*cos(curr_state.getYaw()+slip_angle)*dt,
		    0, 0, 1;
	b_matrix << cos(curr_state.getYaw()+slip_angle)*dt, 0,
		    sin(curr_state.getYaw()+slip_angle)*dt, 0,
		    (cos(slip_angle)/(lf+lr))*tan(curr_input.getSteeringAngle())*dt,
		    (curr_input.getVelocity()*cos(slip_angle)*pow(cos(curr_input.getSteeringAngle()),-2)*dt)/(lf+lr);
	c_matrix << curr_input.getVelocity()*curr_state.getYaw()*sin(curr_state.getYaw()+slip_angle)*dt,
		    -1.0*curr_input.getVelocity()*curr_state.getYaw()*cos(curr_state.getYaw()+slip_angle)*dt,
		    (-1.0*curr_input.getVelocity()*curr_input.getSteeringAngle()*cos(slip_angle)*
		    pow(cos(curr_input.getSteeringAngle()),-2)*dt)/(lf+lr);  
}

State Model::get_next_state(State curr_state, Input curr_input){
	State next_state;
	double slip_angle = atan((lr*tan(curr_input.getSteeringAngle()))/(lf+lr));
	next_state.setX(curr_state.getX() - (curr_input.getVelocity()*sin(curr_state.getYaw()+slip_angle)*dt*curr_state.getYaw())
				+(cos(curr_state.getYaw()+slip_angle)*dt*curr_input.getVelocity())
				+(curr_input.getVelocity()*curr_state.getYaw()*sin(curr_state.getYaw()+slip_angle)*dt));
	next_state.setY(curr_state.getY() + (curr_input.getVelocity()*cos(curr_state.getYaw()+slip_angle)*dt*curr_state.getYaw())
				+(sin(curr_state.getYaw()+slip_angle)*dt*curr_input.getVelocity())
				-(curr_input.getVelocity()*curr_state.getYaw()*cos(curr_state.getYaw()+slip_angle)*dt));
	next_state.setYaw(curr_state.getYaw() + ((cos(slip_angle)/(lf+lr))*tan(curr_input.getSteeringAngle())*dt*curr_input.getVelocity())
				+ ((curr_input.getVelocity()*cos(slip_angle)*pow(cos(curr_input.getSteeringAngle()),-2)*
					dt*curr_input.getSteeringAngle())/(lf+lr))
				+ ((-1.0*curr_input.getVelocity()*curr_input.getSteeringAngle()*cos(slip_angle)*
		    			pow(cos(curr_input.getSteeringAngle()),-2)*dt)/(lf+lr)));
	return next_state;
}
